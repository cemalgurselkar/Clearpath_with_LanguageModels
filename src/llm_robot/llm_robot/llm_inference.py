#!/usr/bin/env python3
"""
llm_interface.py - LLM Interface Node
- Takes English command from terminal
- Sends to Ollama llama3.2:2b
- Parses JSON response
- Publishes object name to /llm_task topic
"""

import json
import re
import ollama
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from llm_robot.prompt import Config


def extract_json(text: str) -> Optional[dict]:
    match = re.search(r"```(?:json)?\s*(\{.*?\})\s*```", text, re.DOTALL)
    if match:
        candidate = match.group(1)
    else:
        match = re.search(r"\{.*\}", text, re.DOTALL)
        if not match:
            return None
        candidate = match.group(0)
    try:
        return json.loads(candidate)
    except json.JSONDecodeError:
        return None


class LLMInterface(Node):
    def __init__(self):
        super().__init__("llm_interface")

        # Publisher → /llm_task
        self.task_pub = self.create_publisher(String, "/llm_task", 10)

        self._check_ollama()
        self.get_logger().info("Type a command and press Enter. 'q' to quit.")

        # Terminal input timer
        self.create_timer(0.1, self._input_loop)
        self._waiting_input = True

    def _check_ollama(self):
        try:
            models = [m.model for m in ollama.list().models]
            if not any(Config.model_name in m for m in models):
                self.get_logger().warn(f"Model not found: {Config.model_name}")
            else:
                self.get_logger().info(f"Ollama ready. Model: {Config.model_name}")
        except Exception as e:
            self.get_logger().error(f"Ollama connection error: {e}")
            raise

    def _query(self, command: str) -> str:
        response = ollama.generate(
            model=Config.model_name,
            prompt=command,
            system=Config.system_prompt,
            options=Config.options,
        )
        return response.response

    def _input_loop(self):
        if not self._waiting_input:
            return
        try:
            command = input("Command: ").strip()
        except (EOFError, KeyboardInterrupt):
            return

        if not command:
            return

        if command.lower() in ("q", "quit", "exit"):
            self.get_logger().info("Exiting...")
            rclpy.shutdown()
            return

        self.get_logger().info(f"Sending: '{command}'")
        raw  = self._query(command)
        self.get_logger().info(f"Response: {raw}")

        plan = extract_json(raw)
        if plan is None:
            self.get_logger().error("JSON parse failed.")
            return

        task   = plan.get("task", "")
        object_name = plan.get("object", "")

        self.get_logger().info(f"Task: {task} | Object: {object_name}")

        # Publish object name to /llm_task
        msg = String()
        msg.data = object_name
        self.task_pub.publish(msg)
        self.get_logger().info(f"Published to /llm_task: {object_name}")


def main():
    rclpy.init()
    node = LLMInterface()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
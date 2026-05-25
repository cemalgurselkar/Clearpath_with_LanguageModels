# Proje Sürecinde Karşılaşılan Sorunlar ve Çözümler

## 1. Gazebo Ignition — Depth Kamera NaN Sorunu

**Sorun:** `/depth/image` topic'i tüm piksel değerlerini NaN (0x7F800000) olarak yayınlıyordu. Arm controller depth tabanlı kavrama yapamıyordu.

**Kök neden:** `intel_realsense.urdf.xacro` dosyasındaki Gazebo sensör eklentisi `type="rgbd_camera"` olarak tanımlanmıştı ancak `ros_gz_bridge` bu topic'i ROS2'ye doğru köprüleyemiyordu.

**Çözüm:** `my_world.sdf` dosyasındaki tüm plugin `filename` değerlerine `lib` prefix'i ve `.so` uzantısı eklendi (`libignition-gazebo-sensors-system.so`). Ayrıca `Imu` ve `NavSat` sistem plugin'leri eklendi. World dosyası Clearpath'in `warehouse.sdf` base'i üzerine kuruldu.

---

## 2. Gazebo Ignition — Depth Kamera Minimum Mesafe Kısıtı

**Sorun:** Kol nesneye `~0.3m`'den yaklaşınca depth değerleri tekrar NaN dönmeye başladı. Gripper kavrama mesafesine (`0.20-0.25m`) ulaşılamıyor.

**Kök neden:** `clearpath_sensors_description` paketindeki `intel_realsense.urdf.xacro` dosyasında `<near>0.3</near>` clip değeri sabit olarak yazılmış. Bu parametre dışarıdan geçirilemiyor.

**Mevcut durum:** Çözümsüz. Sistem paketi olduğundan değişiklik yapılamamıştır. `<near>` değerinin `0.1`'e düşürülmesi sorunu çözecektir.

---

## 3. ROS2 TF Namespace Sorunu

**Sorun:** `arm_controller` node'u TF dönüşümü yaparken `arm_0_base_link` frame'ini bulamıyordu. `tf2_echo` komutu "frame does not exist" hatası veriyordu.

**Kök neden:** Clearpath robotu `/a200_0000` namespace'i altında çalışıyor. TF topic'leri `/a200_0000/tf` ve `/a200_0000/tf_static` olarak yayınlanıyor. Standart `tf2_ros TransformListener` ise `/tf` ve `/tf_static` topic'lerini dinliyor — namespace uyuşmazlığı yaşanıyordu.

**Çözüm:** `arm_controller` node'una launch dosyasında topic remap eklendi:
```python
remappings=[
    ('/tf', '/a200_0000/tf'),
    ('/tf_static', '/a200_0000/tf_static'),
]
```

---

## 4. YOLOv8 — Yakın Mesafe Detection Kaybı

**Sorun:** Robot nesneye yaklaştıkça YOLO modeli nesneyi tespit edemez hale geldi. Kamera çok yakın mesafede nesnenin tamamını göremiyor, detection confidence düşüyor veya tamamen kayboluyor.

**Kök neden:** Model yalnızca uzak mesafe görüntüleri ile eğitilmişti. Yakın mesafe (`<0.5m`) görüntüleri eğitim setinde yoktu.

**Çözüm:** Gazebo simülasyonundan yakın mesafe kareler toplanarak dataset genişletildi. Model Colab üzerinde A100 GPU ile yeniden eğitildi (50 epoch, YOLOv8n).

---

## 5. Detector Node — Sürekli Detection Yayını

**Sorun:** `detector.py` her frame'de detection yapıp `cx, cy` yayınlıyordu. `arm_controller` bu değerleri her 0.2 saniyede okuyunca kol sürekli küçük komutlar alıp titreşiyordu, hizalama tamamlanamıyordu.

**Kök neden:** Detection topic'i sürekli yayın yapıyor, arm controller bunu durduramıyordu.

**Çözüm:** `detector.py`'a `/detector/freeze` topic'i eklendi. `arm_controller` PUSHING aşamasına geçince freeze sinyali gönderiyor, detector son detection değerini dondurarak tekrar yayınlıyor.

---

## 6. Arm Controller — shoulder_lift Yön Hatası

**Sorun:** `shoulder_lift` eklemine pozitif değer eklenince kol geriye/yukarı gidiyordu, öne uzanmıyordu. Kod yanlış yönde hareket komutları gönderiyordu.

**Kök neden:** UR5e'de `shoulder_lift` ekseninde pozitif yön yukarı/geri, negatif yön öne/aşağı olarak tanımlanmış. Simülasyonda test edilerek doğrulandı.

**Çözüm:** `joints[1] += KP_LIFT` yerine `joints[1] -= KP_LIFT` kullanıldı. Simülasyonda `-1.30 rad` komutu verilince kolun yukarı kalktığı, pozitif artışın öne uzattığı doğrulandı.

---

## 7. Gazebo World — Boş Alanda Depth NaN

**Sorun:** Özel `my_world.sdf` dosyasında depth kamerası NaN dönüyordu, ancak Clearpath'in `warehouse.sdf` dosyasında çalışıyordu.

**Kök neden:** Depth kamerası ışın gönderir, ışın bir yüzeye çarparsa mesafe döner. Boş dünya ortamında (duvar/zemin yok) ışınlar hiçbir şeye çarpmadan gidiyordu. Ayrıca `my_world.sdf`'deki plugin `filename` değerlerinde `lib` prefix'i ve `.so` uzantısı eksikti.

**Çözüm:** `my_world.sdf` Clearpath'in `warehouse.sdf` dosyası base alınarak yeniden oluşturuldu. Özel modeller (cafe_table, coke_can) bu yeni dosyaya eklendi.

---

## 8. ROS2 Namespace — arm_controller Topic Uyuşmazlığı

**Sorun:** `arm_controller` node'u `/a200_0000` namespace'ine alınınca `/navigator/status` topic'ini dinleyemedi. Node `/a200_0000/navigator/status` olarak arıyordu, navigator ise `/navigator/status` yayınlıyordu.

**Kök neden:** ROS2'de namespace altında çalışan bir node, tüm topic isimlerini o namespace ile prefix'liyor.

**Çözüm:** Launch dosyasına topic remap eklendi veya namespace kaldırılarak sadece TF remap bırakıldı.

---

## 9. IK Hesaplama Hatası

**Sorun:** TF + 2-joint IK yaklaşımında `shoulder_lift` için hesaplanan açı (`q1 = 0.9 rad = 51 derece`) çok büyük çıkıyordu, kol geriye/yukarı gidiyordu.

**Kök neden:** Hedef nokta (`r = 0.97m`) UR5e'nin maksimum erişim mesafesini (`L1+L2 = 0.817m`) aşıyordu. IK clamp yapıyor, yanlış açı üretiyordu. Kamera frame'i ile arm frame'i arasındaki karmaşık rotasyon da hesabı zorlaştırıyordu.

**Çözüm:** TF + IK yaklaşımı tamamen kaldırıldı. Yerine depth tabanlı basit close-loop konuldu: depth değerine göre `shoulder_lift` sabit adımlarla artırılıyor.

---

## 10. Gazebo Ignition — ros_gz_bridge Depth Topic Uyuşmazlığı

**Sorun:** Mevcut `camera_0_gz_depth_bridge` node'u ROS2'ye depth yayınlıyordu ancak Ignition tarafında hiçbir topic'e subscribe olmuyordu. Bridge boş veri yayınlıyordu.

**Kök neden:** Ignition tarafındaki topic adı `/a200_0000/sensors/camera_0/depth_image` iken bridge `/a200_0000/sensors/camera_0/depth/image` olarak aranıyordu — isim uyuşmazlığı vardı.

**Çözüm:** World dosyasının `warehouse.sdf` base alınarak yeniden yapılandırılmasıyla çözüldü.

---

## 11. Gripper Fiziksel Kavrama Başarısızlığı

**Sorun:** Arm controller PUSHING → GRIPPING geçişi yapıyor, gripper kapanıyor ancak cola kutusunu tutamıyor. Kol home pozisyonuna dönerken kutu düşüyor.

**Kök neden:** Depth kamerasının minimum mesafe kısıtı (`0.3m`) nedeniyle gripper nesneye tam temas etmeden kapanıyor. Ayrıca gripper ekseninin nesne eksenine tam hizalanmaması da kavramayı zorlaştırıyor.

**Mevcut durum:** Kısmen çözümlü. `GRIP_DEPTH` parametresi ayarlanarak daha iyi sonuçlar alınabilir ancak kalıcı çözüm için minimum clip mesafesinin düşürülmesi gerekiyor.
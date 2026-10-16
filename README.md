# VADAS (Vietnam Advanced Driver Assistance System)

Hệ thống Trợ lái Nâng cao (ADAS) mã nguồn mở tuân thủ quy chuẩn an toàn giao thông đường bộ Việt Nam (**Thông tư 31/2019/TT-BGTVT**).

---

## 🏗️ Kiến trúc Hệ thống (Multi-Module Architecture)

Hệ thống được thiết kế theo kiến trúc module hoá chuẩn Android Automotive:

- **`:core-camera`**: Module thu nhận video từ CameraX trên điện thoại và luồng RTSP từ Camera hành trình (Dashcam).
- **`:core-vision`**: Động cơ thị giác máy tính on-device (UFLD Lane Detection, Vehicle Object Detection, Pinhole 3D Distance Estimator, Dynamic Horizon Line).
- **`:core-safety-engine`**: Động cơ tính toán khoảng cách an toàn luật định theo Thông tư 31/2019/TT-BGTVT, cảnh báo chệch làn (LDW - Lane Departure Warning) và Time-To-Collision (TTC).
- **`:core-vietmap`**: Tầng tích hợp thông tin biển báo, giới hạn tốc độ và dữ liệu điều hướng giao thông.
- **`:feature-auto`**: Tích hợp màn hình xe hơi thông qua Android for Cars App Library (`androidx.car.app`).
- **`:app`**: Ứng dụng Android chạy Foreground Service, HUD AR Overlay thời gian thực và quản lý pin / nhiệt độ phần cứng.

---

## 🚦 Quy chuẩn Cự ly An toàn (Thông tư 31/2019/TT-BGTVT)

| Vận tốc thực tế | Cự ly an toàn tối thiểu (Đường khô ráo) | Hệ số thời tiết mưa / trơn trượt |
| :--- | :---: | :---: |
| $v < 60\text{ km/h}$ | Quy tắc $TTC \le 2.0\text{s}$ (tối thiểu 5m) | $+30\%$ |
| $v = 60\text{ km/h}$ | $35\text{ mét}$ | $+30\%$ |
| $60 < v \le 80\text{ km/h}$ | $55\text{ mét}$ | $+30\%$ |
| $80 < v \le 100\text{ km/h}$ | $70\text{ mét}$ | $+30\%$ |
| $100 < v \le 120\text{ km/h}$ | $100\text{ mét}$ | $+30\%$ |
| $v > 120\text{ km/h}$ | $100\text{m} + 1.25\text{m} \times (v - 120)$ | $+30\%$ |

---

## 👥 Phân công Đội ngũ & Trách nhiệm

1. **Thành viên 1**: Android Lead & Kiến trúc Pipeline Video (`:core-camera`)
2. **Thành viên 2**: AI Phương tiện & Đo khoảng cách Pinhole 3D (`:core-vision`)
3. **Thành viên 3**: AI Làn đường & Đèn tín hiệu giao thông (`:core-vision`)
4. **Thành viên 4**: Logic An toàn Thông tư 31 & Cảm biến Vận tốc (`:core-safety-engine`)
5. **Thành viên 5**: Android Auto, Giao diện HUD AR & Đảm bảo Chất lượng (`:feature-auto`, `:app`, QA)

---

## 🛠️ Yêu cầu Môi trường & Biên dịch

- **Android SDK**: Compile SDK 34, Min SDK 26 (Android 8.0+)
- **Java Version**: OpenJDK 17
- **Gradle Version**: 8.x
- **Git LFS**: Cần cài đặt Git Large File Storage (`git lfs install`) để tải các model TFLite.

```bash
# Kiểm thử đơn vị toàn bộ module
./gradlew testDebugUnitTest

# Biên dịch gói APK gỡ lỗi
./gradlew assembleDebug
```

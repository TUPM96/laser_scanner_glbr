# 3D Scanner Project

Hệ thống quét 3D sử dụng cảm biến LiDAR TF-Luna và 2 động cơ bước.

## Cấu trúc dự án

```
quet3d/
├── firmware/              # Firmware cho Arduino (PlatformIO)
│   ├── src/
│   │   └── main.cpp      # Code chính
│   ├── platformio.ini    # Cấu hình PlatformIO
│   └── README.md
├── scanner_gui/          # Ứng dụng Python GUI
│   ├── main.py           # Giao diện chính
│   ├── stl_generator.py  # Tạo file STL
│   ├── requirements.txt  # Thư viện Python cần thiết
│   └── README.md
└── README.md             # File này
```

## Phần cứng

### Cảm biến
- **TF-Luna LiDAR**: Cảm biến đo khoảng cách ToF (Time of Flight)
  - Khoảng cách: 0.2m - 8m
  - Giao tiếp: I2C (địa chỉ 0x10)
  - Kết nối: SDA (A4), SCL (A5)

### Động cơ bước
- **Motor X (Quay/Theta)**: Điều khiển quay bàn quay
  - STEP: Pin 2
  - DIR: Pin 5
  - ENABLE: Pin 8

- **Motor Y (Trục Z)**: Điều khiển di chuyển dọc
  - STEP: Pin 3
  - DIR: Pin 6
  - ENABLE: Pin 8

## Cài đặt

### Firmware (Arduino)

1. Cài đặt PlatformIO:
   - VS Code: Cài extension PlatformIO IDE
   - Hoặc dùng PlatformIO CLI

2. Build và upload:
```bash
cd firmware
pio run -t upload
```

3. Monitor serial:
```bash
pio device monitor
```

### Python GUI

1. Cài đặt Python 3.7 trở lên

2. Cài đặt thư viện:
```bash
cd scanner_gui
pip install -r requirements.txt
```

3. Chạy ứng dụng:
```bash
python main.py
```

## Sử dụng

### 1. Kết nối phần cứng
- Kết nối TF-Luna vào I2C (A4, A5)
- Kết nối 2 động cơ bước theo cấu hình CNC shield
- Kết nối Arduino qua USB

### 2. Upload firmware
- Upload firmware lên Arduino bằng PlatformIO
- Kiểm tra serial monitor để xác nhận TF-Luna được phát hiện

### 3. Chạy GUI
- Mở ứng dụng Python GUI
- Chọn cổng serial từ dropdown
- Click "Connect" để kết nối
- Click "Start Scan" để bắt đầu quét
- Xem dữ liệu 3D real-time
- Export sang STL khi hoàn thành

## Tính năng

### Firmware
- Điều khiển 2 động cơ bước (X và Y)
- Đọc dữ liệu từ TF-Luna qua I2C
- Gửi dữ liệu quét qua Serial
- Hỗ trợ lệnh START, STOP, HOME

### Python GUI
- Chọn cổng serial và kết nối
- Điều khiển quét từ xa
- Hiển thị point cloud 3D real-time
- Theo dõi tiến độ quét
- Export dữ liệu sang file STL

## Lưu ý

- Đảm bảo cấp đúng điện áp cho TF-Luna (5V)
- Kiểm tra kết nối I2C trước khi quét
- Điều chỉnh tham số quét trong code firmware nếu cần
- Khoảng cách đo phụ thuộc vào độ phản xạ của vật thể

## Tài liệu tham khảo

- [TF-Luna Datasheet](https://hshop.vn/cam-bien-khoang-cach-dfrobot-tf-luna-tof-micro-single-point-ranging-lidar)
- PlatformIO Documentation: https://platformio.org/
- Python Serial: https://pyserial.readthedocs.io/

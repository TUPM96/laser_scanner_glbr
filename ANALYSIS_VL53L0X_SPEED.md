# Phân tích: 33 điểm/giây có đủ cho góc 1 độ trong GRBL không?

## Thông số hệ thống

### Calibration GRBL:
- **0.1mm GRBL = 45° motor rotation**
- **1° = 0.1/45 = 0.00222mm GRBL units**
- **360° = 0.8mm GRBL units**

### Feed Rate:
- **F1 = 1 mm/min** (tốc độ quét trong code)
- Thời gian quay 1 độ: **0.00222mm / 1mm/min × 60 = 0.133 giây = 133ms**

### VL53L0X:
- **Tốc độ đo: ~30ms/lần đo** (tốt nhất)
- **Tốc độ đọc: 33 điểm/giây = 1 điểm mỗi 30.3ms**

## Phân tích kịch bản

### Kịch bản 1: Quay từng bước (Step-by-step) - Code hiện tại

**Cách hoạt động:**
1. Quay 1 độ (133ms với F1)
2. DỪNG motor (wait 0.5s = 500ms)
3. Đọc sensor (30-500ms tùy điều kiện)
4. Lặp lại

**Thời gian cho 1 điểm:**
- Tốt nhất: 133ms (quay) + 30ms (đọc) = **163ms/điểm**
- Xấu nhất: 133ms (quay) + 500ms (timeout) = **633ms/điểm**

**Số điểm/giây:**
- Tốt nhất: 1000/163 = **6.1 điểm/giây**
- Xấu nhất: 1000/633 = **1.6 điểm/giây**

**Kết luận cho 1 vòng (360 điểm):**
- Tốt nhất: 360 × 163ms = **58.7 giây/vòng**
- Xấu nhất: 360 × 633ms = **227 giây/vòng (3.8 phút)**

**→ 33 điểm/giây là ĐỦ cho góc 1 độ** (vì code chỉ đạt 1.6-6.1 điểm/giây)

### Kịch bản 2: Quay liên tục (Continuous) - Lý thuyết

**Nếu quay liên tục 1 độ/giây:**
- 1 độ/giây = 360 độ trong 360 giây = **6 phút/vòng**
- Trong 1 giây quay 1 độ, có thể đọc **33 điểm**
- **→ 33 điểm/giây là ĐỦ cho 1 độ/giây**

**Nếu quay nhanh hơn, ví dụ 10 độ/giây:**
- Trong 1 giây quay 10 độ, vẫn chỉ đọc được 33 điểm
- Mỗi độ chỉ có **3.3 điểm** → KHÔNG ĐỦ chi tiết

## Giới hạn thực tế

### Giới hạn của VL53L0X:
- **Tối đa: ~33 điểm/giây** (do thời gian đo ~30ms)
- **Không phải giới hạn I2C** (I2C có thể nhanh hơn nhiều)

### Giới hạn của GRBL với F1:
- **Tốc độ quay: 1 mm/min = 0.0167 mm/s**
- **Quay 1 độ: 0.00222mm / 0.0167mm/s = 0.133 giây**
- **Tốc độ quay tối đa: 1/0.133 = 7.5 độ/giây**

### Kết luận:

**Với góc 1 độ và tốc độ quay ≤ 7.5 độ/giây:**
- ✅ **33 điểm/giây là ĐỦ** (có thể đọc 33 điểm trong 1 giây)
- ✅ Mỗi độ sẽ có **≥ 4.4 điểm** (33/7.5)

**Với góc nhỏ hơn (ví dụ 0.5 độ):**
- Nếu quay 15 độ/giây: 33 điểm/giây → **2.2 điểm/độ** → VẪN ĐỦ
- Nếu quay 30 độ/giây: 33 điểm/giây → **1.1 điểm/độ** → KHÔNG ĐỦ

## Khuyến nghị

1. **Với góc 1 độ:** 33 điểm/giây là **ĐỦ** cho tốc độ quay ≤ 7.5 độ/giây
2. **Với góc nhỏ hơn:** Cần giảm tốc độ quay hoặc chấp nhận ít điểm hơn
3. **Code hiện tại (step-by-step):** Không bị giới hạn bởi 33 điểm/giây, mà bị giới hạn bởi thời gian chờ motor dừng (0.5s)

## Cải thiện có thể

1. **Giảm delay sau khi quay:** Từ 0.5s xuống 0.2s (nếu motor đủ nhanh)
2. **Tăng feed rate:** Từ F1 lên F10-F50 (nếu cần quay nhanh hơn)
3. **Tối ưu timeout sensor:** Đã tăng từ 100ms lên 500ms → giảm mất điểm


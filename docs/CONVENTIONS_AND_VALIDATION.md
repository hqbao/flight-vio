# Conventions (FC ↔ VIO), Gold tests & UI validation

> Tài liệu kiểm chứng tính đúng đắn của chuỗi VIO → dblink → FC ESKF. Mọi khẳng định
> đều trích dẫn file:line từ code thật (không phỏng đoán). 2026-06-19.

---

## A. QUY ƯỚC TRỤC & DẤU (sign conventions) — toàn chuỗi

### A.0 Kết luận trước
**Wire (dblink) khớp 2 đầu, và VIO ĐÃ convert sang NED thật trước khi gửi.** Frame
"optical-world" gốc của VIO (gravity-aligned, yaw tuỳ ý vì không có la bàn) được đổi sang
NED tại MỘT chỗ duy nhất (SSOT) `sky/fc/fc_earth_pose.py` — UI và sender dùng chung nên
không thể lệch nhau. Heading là TƯƠNG ĐỐI (không la bàn) → FC tự xử bằng anchor.

### A.1 Earth frame = NED (North, East, Down)
| | Quy ước | Bằng chứng |
|---|---|---|
| FC | NED, gravity **+Z (xuống)**, `a_earth[2]+=g`, g=+9.80665 | `robotkit/fusion6.c:313`, `fusion6.h:27` |
| VIO (sau convert) | NED, fwd→N, right→E, down→D | `sky/fc/fc_earth_pose.py:62-64` `_M_OPT_TO_NED=[[0,0,1],[1,0,0],[0,1,0]]` |
| Lúc nghỉ accel đọc | ~(0,0,−g) (specific force ngược gravity) | `fusion6.c:206-213` |

### A.2 Body frame = FRD (Forward, Right, Down)
| | Quy ước | Bằng chứng |
|---|---|---|
| FC | X=trước, Y=phải, Z=xuống; Euler ZYX (yaw-pitch-roll) | `state_estimation/earth2body.h:12-16`, `quat.c:205-222` |
| NED→body | `fwd= cψ·vN+sψ·vE; right=−sψ·vN+cψ·vE; down=vD` | `earth2body.c:31-36` |

### A.3 Quaternion = Hamilton, (w,x,y,z), **body→earth/NED**
| | Bằng chứng |
|---|---|
| FC: lưu (w,x,y,z), body→earth, Hamilton, lỗi right-mult `q←q⊗Exp(δθ)` | `quat.h:7-10`, `fusion6.h:27`, `fusion6.c:567-570` |
| VIO: (w,x,y,z), body→world, Hamilton (cùng công thức quat→rot) | `sky/math/quat.py:10-28` |
| Wire: `q_w,q_x,q_y,q_z` = **body→NED**, w-first | `messages.h:597`, `sky/fc/dblink.py:35-39` |

### A.4 VIO frame gốc và phép đổi sang NED (chỗ DỄ SAI nhất — đã kiểm)
- VIO native = **gravity-aligned OPTICAL world** (cam optical: X=phải, Y=xuống, Z=trước),
  yaw = hướng cam lúc init (KHÔNG có la bàn → "North" tương đối). `sky/imu/imu.py:257-291` (`gravity_aligned_R0`).
- Đổi sang NED (SSOT, dùng chung UI + sender): `sky/fc/fc_earth_pose.py:99-106`
  ```
  pos_ned = M @ pos_opt                      # fwd→N, right→E, down→D
  R_ned   = M @ R_opt @ P @ R_body_cam.T     # P = opencv-cam → FRD; R_body_cam = lệch mount (mặc định I)
  q_ned   = rot_to_quat(R_ned)
  ```
  Gọi tại `fc/main.py:388` TRƯỚC `pack_vision_pose` → **trên dây là NED thật + quat body→NED.**
- **Heading TƯƠNG ĐỐI** (no mag): FC neo lại bằng `ψ0 = yaw_FC − yaw_VIO`, fuse
  `fused = Rz(ψ0)·(vio_pos − anchor) + offset`, D đi thẳng. `vision_pose_rx/vision_pose_math.h:43-69`.
  - **SE1**: chỉ fuse VỊ TRÍ (heading do la bàn FC sở hữu) → relative-North không hại.
  - **SE2**: dùng ĐẠO HÀM vị trí (vận tốc) → bất biến gốc, không cần anchor; xoay NED→body bằng yaw fusion3 (có mag). `vio_body_vel_math.h:45-51`.

### A.5 IMU axis map — THEO TỪNG BOARD (cần validate vật lý)
| Board | raw→body | Bằng chứng |
|---|---|---|
| **h7v1** (board đang HIL) | body=(−raw_y, −raw_x, −raw_z) | `h7v1/modules/icm42688p/icm42688p.c:10-11,32-35,48-50` |
| h7v2 | body=(raw_x, raw_y, raw_z) | `h7v2/.../icm42688p.c:10-11,24-27,36-39` |

⚠️ **Hai board map KHÁC nhau** (PCB mount khác). Không phải bug, nhưng **phải validate
tilt-test cho đúng board mình bay** (mục C). Comment "sensor X=Right…" giống nhau ở 2 file
nhưng map khác → đừng tin comment, tin tilt-test.

### A.6 OAK-D Lite BMI270 — extrinsic EEPROM SAI (đã có fix)
EEPROM trả `Rx(90°)` sai → lật roll ~180°. Đã khắc phục bằng wizard calib per-device
(Kabsch/Wahba) `sky/sensors/imu_cam_extrinsic.py`. **Phải đảm bảo calib đã áp** trước khi bay Lite.

### A.7 Z-sign tại biên publish của SE2 (đã chú thích sẵn, CRITICAL)
fusion5_z chạy POSITIVE-UP nội bộ → **negate** ở biên publish để ra NED-down:
`state_estimation2.c:335,338` `pos_body.z=-g_pos_z.pos_final`. Quên = altitude-hold dương-hồi-tiếp.

---

## B. GOLD TESTS — xác định đúng đắn tới đâu?

### B.1 Tối ưu ESKF em vừa làm → CÓ gold test, CHẶT (machine precision)
- `robotkit/test/fusion6_equiv.c` — 200k case ngẫu nhiên, optimized == naive tới ~1e-14.
- `robotkit/test/fusion6_traj.c` — 200k bước, parity vs git-reference (P-trace 3e-14).
→ **Chứng minh chắc chắn: tối ưu KHÔNG đổi thuật toán.** (Đây là "regression-correctness".)

### B.2 Đúng đắn TỪNG THÀNH PHẦN vs sự thật vật lý → CÓ (mạnh)
| Test | Chứng minh | File |
|---|---|---|
| IMU dead-reckon + ZUPT | tích phân ra dịch chuyển đúng; đứng yên không trôi | `vio/tests/imu_propagate_selftest.py` |
| Wahba extrinsic IMU→cam | giải lại R đã biết tới 1e-9, det=+1, bền nhiễu | `imu_camera/tests/imu_cam_extrinsic_selftest.py` |
| Gravity 6-mặt | accel calib bám \|g\|=9.81 | `imu_camera/tests/gravity_sphere_selftest.py` |
| Preint covariance | Σ giải tích == Monte-Carlo | `vio/tests/imu_preint_cov_selftest.py` |
| **fc_earth_pose SSOT** (optical→NED) | **trục đã biết + pitch-90 → đúng** | `verification/fc_earth_pose_selftest.py` ✅ |
| anchor + body-vel (FC) | yaw, transform, gate đúng (hand-checked) | `tools/vision_pose_rx/test_vision_pose_math.c`, `test_vio_body_vel.c` |

### B.3 Regression gold (khớp tham chiếu đông cứng) → CÓ
- gap=0 oracle byte-parity vs Basalt: `verification/oracle_replay_selftest.py` (TOL 1e-6 mm).
- 12 gold sessions `sessions/gold/` (lab_static/straight/loop, push, shake, yaw…).

### B.4 LỖ HỔNG: chưa có gold test ĐẦU-CUỐI vs ground-truth
KHÔNG có test: "chuyển động THẬT đã biết (đo bằng thước/mocap) → VIO → FC ESKF → so estimate
với sự thật". gap=0 chỉ chứng minh "giống Basalt" (regression), không phải "đúng vật lý".
→ **Cách đóng lỗ hổng NGAY (thủ công, ground-truth = thước dây):** mục C dưới đây. Có thể
dựng thêm harness e2e tự động sau (replay session đã biết → so FC estimate).

---

## C. CÁC BƯỚC VALIDATE BẰNG UI (vật lý, làm được ngay)

> Mục tiêu: mắt thấy DẤU từng trục đúng, đầu-cuối. Cần FC bật + AP `/dev/cu.usbmodem2101`,
> và (cho VIO) Pi chạy stack VIO (320x200, no --tight/BA/SLAM).

### C.1 Validate IMU→body (FC, board h7v1) — `state_estimation_view.py` / `attitude_control_view.py`
```bash
cd /Users/bao/skydev/flight-controller && python3 tools/state_estimation_view.py
```
Cầm FC, làm từng động tác, xác nhận DẤU:
| Động tác vật lý | Kỳ vọng | Ý nghĩa |
|---|---|---|
| Chúi mũi XUỐNG | **pitch +** (hoặc theo quy ước anh chốt) | trục Y/forward đúng |
| Nghiêng phải | **roll +** | trục X/right đúng |
| Xoay mũi sang phải (nhìn từ trên) | **yaw +** | trục Z/down đúng |
| Để yên thăng bằng | roll≈pitch≈0, gravity dồn trục Z | accel Z-down đúng |
→ Sai dấu bất kỳ = IMU axis map của board (A.5) cần sửa. **Đây là cửa quan trọng nhất.**

### C.2 Validate VIO→NED (Pi VIO, UI 3D) — `ui/main.py`
```bash
cd /Users/bao/skydev/flight-vio && ./run-ui-remote.sh    # hoặc ./run.sh ... rồi mở UI
```
⚠️ **VIO KHÔNG có la bàn → "North" của VIO = hướng cam LÚC INIT** (gravity-aligned, code:
`_M_OPT_TO_NED` ⇒ NED-North = init-forward). Nên phải test theo 3 nhóm:

**(a) Down — HEADING-FREE (gravity), test cái này trước, chắc chắn nhất:**
| Di chuyển | Kỳ vọng |
|---|---|
| Hạ rig XUỐNG ~0.5 m | **pos_d +0.5** (luôn đúng, không cần biết heading) |

**(b) N/E — PHỤ THUỘC heading init → phải KHÔNG XOAY rig sau khi init:**
| Di chuyển (giữ nguyên hướng, chỉ tịnh tiến) | Kỳ vọng |
|---|---|
| Theo đúng hướng cam đang nhìn (= init-forward) | **pos_n +**, pos_e ≈ 0 |
| Sang phải của init-forward | **pos_e +**, pos_n ≈ 0 |

**(c) Bất biến HEADING-FREE (kiểm cấu trúc trục mà KHÔNG cần biết North):**
- Di chuyển THUẦN dọc → chỉ `pos_d` đổi; THUẦN ngang → chỉ `pos_n/pos_e` đổi (`pos_d≈0`).
- Đi ra rồi VỀ chỗ cũ → pos về ~0 (drift nhỏ).
- Đi 1 m ngang bất kỳ → `√(pos_n²+pos_e²) ≈ 1 m` (kiểm SCALE, không cần hướng).

→ Lệch ở (a)/(c) = phép đổi optical→NED hoặc extrinsic cam-IMU (A.6) SAI. (b) chỉ đúng khi
không xoay; nếu xoay, N/E phân rã theo trục init-fixed (đúng về mặt toán, không phải lỗi).
**North TUYỆT ĐỐI: VIO không cho được — đúng thiết kế, la bàn FC sở hữu yaw tuyệt đối.**

### C.3 Validate khâu nối VIO→FC (0x34) — `tools/vision_pose_rx/vision_pose_rx_view.py`
```bash
cd /Users/bao/skydev/flight-controller && python3 tools/vision_pose_rx/vision_pose_rx_view.py
```
Pi gửi VIO thật qua `--fc /dev/ttyAMA0`. Di chuyển rig → xem `rx (VIO world)` N/E/D
chạy ĐÚNG hướng + DẤU như C.2. Xác nhận FC nhận đúng những gì VIO gửi.

### C.4 Đối chiếu đầu-cuối (ground-truth thước dây)
- Đặt rig ở mốc 0, **không xoay**, di chuyển đoạn ĐO ĐƯỢC theo init-forward (vd thước: 2.00 m).
- Xem `state_estimation_position_earth.py` (FC fused) **và** UI 3D (VIO): cả hai ra ~+2.00 m
  North, cùng dấu, sai số vài cm. (Hoặc kiểm HEADING-FREE: magnitude ngang `√(N²+E²)≈2.00 m`
  — không cần đi đúng hướng North, chỉ cần đo đúng quãng đường.)
```bash
python3 tools/state_estimation_position_earth.py    # vị trí NED fused của FC
```
- Đây chính là **gold test đầu-cuối thủ công** (đóng lỗ hổng B.4): chuyển động biết trước →
  estimate khớp tới sai số thước.

### C.5 Heading tương đối (nhắc)
VIO yaw KHÔNG tuyệt đối (no mag) → sẽ trôi chậm; **la bàn FC sở hữu yaw tuyệt đối**. Đừng kỳ
vọng VIO-North = North thật. Khi GPS/​mag khoẻ, FC ưu tiên chúng; VIO bù khi mất GPS.

---

## D. Tóm tắt 1 dòng
Quy ước 2 đầu NHẤT QUÁN (NED + FRD + Hamilton body→NED), VIO convert sang NED tại SSOT đã
selftest, heading tương đối được anchor xử lý. Đúng-đắn: thành phần + regression ĐÃ phủ chặt;
còn thiếu gold test đầu-cuối ground-truth → dùng C.4 (thước dây) để chốt trước khi bay.

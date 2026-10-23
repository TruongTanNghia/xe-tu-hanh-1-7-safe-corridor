# Handoff cho Codex trên Raspberry Pi 5 cũ

**Mục đích:** đưa context từ Pi 5 hiện tại sang Codex trên Pi 5 cũ để kiểm tra
và sửa ROS 2 Humble native còn thiếu package/dependency, đặc biệt là
`rosbag2` và pipeline ghi/đọc CSV.

**Ngày lập:** 2026-07-17  
**Repo:** `autonomous_car_hardware_platform_v2`  
**Pi tham chiếu:** Raspberry Pi 5 8 GB, Debian 12/Raspberry Pi OS Bookworm,
arm64, Python 3.11, ROS 2 Humble build từ source.

> Codex phải chạy baseline trên chính Pi cũ trước khi sửa. Không được giả định
> Pi cũ giống Pi tham chiếu.

---

## 1. Trạng thái đã xác minh trên Pi 5 hiện tại

~~~text
OS: Debian 12 Bookworm, arm64
Python: 3.11
ROS_ROOT: /home/ac/ros2_humble
ROS 2: Humble, native source build
ROS middleware: rmw_cyclonedds_cpp
ROS workspace: /home/ac/main/1_projects/autonomous_car_hardware_platform_v2/ros2_ws
~~~

Tool/dependency quan trọng đã có:

~~~text
ros-dev-tools
python3-colcon-common-extensions
python3-vcstool
python3-rosdep
python3-numpy
python3-scipy
python3-yaml
python3-opencv
python3-picamera2
libopencv-dev
libsqlite3-dev
libzstd-dev
~~~

Underlay hiện tại có đủ:

~~~text
ros2cli
rclpy
sensor_msgs
rosbag2_storage
rosbag2_storage_default_plugins
rosbag2_storage_mcap
rosbag2_compression
rosbag2_compression_zstd
rosbag2_cpp
rosbag2_transport
rosbag2_py
ros2bag
~~~

Runtime checks đã PASS:

~~~text
command -v ros2                  -> /home/ac/ros2_humble/install/ros2cli/bin/ros2
ros2 bag --help                  -> PASS
ros2 bag record/info (MCAP)      -> PASS, temporary String bag readable
import rclpy                     -> PASS
import rosbag2_py                -> PASS
import numpy, scipy, yaml, cv2   -> PASS
import picamera2                 -> PASS
~~~

**CSV không phải ROS package riêng.** Module `csv` có sẵn trong Python standard
library. Phần fit model của repo cần `numpy` và `scipy`; `pandas` không được
dùng và không cần cài nếu không có tool nào import nó.

---

## 2. Nguyên nhân Pi cũ bị thiếu

### 2.1. Guidebook camera cũ đã cố ý skip rosbag2

[`rpi5_ros2_humble_camera_setup_guidebook_vi.md`](rpi5_ros2_humble_camera_setup_guidebook_vi.md)
chỉ nhằm chứng minh camera bring-up. STEP 10 skip nhóm:

~~~text
mcap_vendor
rosbag2_storage_mcap
rosbag2_cpp
rosbag2
ros2bag
rosbag2_py
rosbag2_transport
rosbag2_compression
rosbag2_compression_zstd
rqt_bag
~~~

Do đó “camera chạy được” không đồng nghĩa với “full ROS 2 Humble có rosbag2”.

### 2.2. MCAP là dependency chain, không thể skip một package riêng lẻ

Log build cũ có lỗi:

~~~text
Failed to find mcap_vendor
Failed to find rosbag2_storage_mcap
Failed <<< rosbag2_cpp
~~~

Sau đó `rqt_bag` và `launch_testing_examples` fail theo vì rosbag2
downstream không tồn tại. Cách đúng là build vendor + storage + MCAP +
compression + cpp + transport + Python + CLI cùng nhóm.

### 2.3. Bookworm/Python 3.11 cần native source build

Humble binary route phổ biến nhắm Ubuntu 22.04/Python 3.10, còn Pi dùng
Debian 12/Python 3.11. Route đang hoạt động:

~~~text
ROS apt repository -> chỉ lấy build tools/rosdep
ROS 2 Humble -> source build native bằng Python 3.11
~~~

Không trộn full `ros-humble-*` binary hoặc `/opt/ros/humble` vào source
underlay trên Bookworm.

Các rosdep key đã skip trên Pi tham chiếu:

~~~text
fastcdr
rti-connext-dds-6.0.1
urdfdom_headers
python3-vcstool
libpython3.10
libpython3.10-dev
libcyclonedds-dev
~~~

`rosdep_install_humble_step9.log` kết thúc bằng:

~~~text
#All required rosdeps installed successfully
~~~

### 2.4. Lỗi network trước đây không phải lỗi rosbag2

Một lượt build fail ở `shared_queues_vendor` do GitHub/DNS. Sửa network/DNS
rồi build lại; không sửa source ROS 2 để né lỗi mạng.

---

## 3. Dấu vết và script trên Pi tham chiếu

~~~text
/home/ac/ros2_humble/log/build_2026-06-29_07-05-26/
/home/ac/ros2_build_humble_step12*.log
/home/ac/rosdep_install_humble_step9.log
/home/ac/ros2_source_check_step13.log
~~~

Script repo dành cho nhóm rosbag2:

~~~text
scripts/bootstrap_rosbag2.sh
~~~

Script không dùng sudo, không sửa source, không build lại toàn bộ ROS 2. Nó
source setup với `set +u`, build:

~~~bash
colcon build --symlink-install --packages-select \
  rosbag2_storage \
  rosbag2_storage_default_plugins \
  rosbag2_storage_mcap \
  rosbag2_compression \
  rosbag2_compression_zstd \
  rosbag2_cpp \
  rosbag2_transport \
  rosbag2_py \
  ros2bag \
  --event-handlers console_cohesion+
~~~

---

## 4. Quy tắc an toàn

- Trước test phần cứng: ESC main power OFF, motor không cấp nguồn, xe được giữ
  cố định.
- Không xóa toàn bộ `~/ros2_humble`, `build`, `install` hoặc source tree
  trước khi lưu log.
- Không copy `install/` hoặc `build/` từ Pi hiện tại sang Pi cũ.
- Mỗi lần source setup phải source trước khi bật nounset:

~~~bash
set +u
source "$ROS_ROOT/install/setup.bash"
set -u
~~~

Lỗi `setup.bash: COLCON_TRACE: unbound variable` là lỗi thứ tự `set -u`,
không phải lỗi ROS.

---

## 5. Quy trình bắt buộc trên Pi cũ

### Phase A — Baseline không thay đổi hệ thống

~~~bash
mkdir -p "$HOME/old_pi_ros2_handoff"
exec > >(tee "$HOME/old_pi_ros2_handoff/baseline.log") 2>&1

hostname
whoami
cat /etc/os-release
uname -a
dpkg --print-architecture
python3 --version
free -h
swapon --show
df -h /
vcgencmd measure_temp 2>/dev/null || true
ip -br addr
ip route
getent hosts github.com || true
getent hosts raw.githubusercontent.com || true
command -v colcon || true
command -v vcs || true
command -v rosdep || true
command -v ros2 || true
find "$HOME" /opt -maxdepth 4 -path '*/install/setup.bash' \
  -print 2>/dev/null | sort -u
apt-cache policy ros-dev-tools python3-colcon-common-extensions \
  python3-vcstool python3-rosdep python3-numpy python3-scipy \
  python3-yaml python3-opencv python3-picamera2 \
  libsqlite3-dev libzstd-dev || true
~~~

Phải xác định rõ: OS, arm64, Python version, ROS_ROOT, source rosbag2 có tồn tại,
disk còn đủ và GitHub/DNS có hoạt động. Nếu user/path khác `ac`, dùng path
thật của Pi cũ.

### Phase B — Baseline ROS/package

~~~bash
export ROS_ROOT="$HOME/ros2_humble"
test -f "$ROS_ROOT/install/setup.bash" \
  && echo UNDERLAY_SETUP=FOUND || echo UNDERLAY_SETUP=MISSING
test -d "$ROS_ROOT/src/ros2/rosbag2" \
  && echo ROSBAG_SOURCE=FOUND || echo ROSBAG_SOURCE=MISSING

set +u
source "$ROS_ROOT/install/setup.bash" 2>&1 | \
  tee "$HOME/old_pi_ros2_handoff/source.log"
set -u

printenv ROS_DISTRO || true
printenv ROS_VERSION || true
command -v ros2 || true
ros2 --help >/dev/null 2>&1 && echo ROS2_CLI=PASS || echo ROS2_CLI=FAIL
ros2 bag --help >/dev/null 2>&1 && echo ROS2_BAG=PASS || echo ROS2_BAG=FAIL

for p in ros2cli rclpy sensor_msgs rosbag2_storage \
         rosbag2_storage_mcap rosbag2_cpp rosbag2_transport \
         rosbag2_py ros2bag; do
  printf '%-35s' "$p"
  ros2 pkg prefix "$p" 2>/dev/null || echo MISSING
done

python3 - <<'PY'
import importlib
for name in ("rclpy", "rosbag2_py", "numpy", "scipy",
             "yaml", "cv2", "picamera2", "csv"):
    try:
        module = importlib.import_module(name)
        print(f"PYTHON {name}: PASS {getattr(module, '__file__', '')}")
    except Exception as exc:
        print(f"PYTHON {name}: FAIL {type(exc).__name__}: {exc}")
PY
~~~

Phân loại lỗi trước khi sửa:

~~~text
MISSING_SOURCE       -> source tree chưa đủ
MISSING_DEPENDENCY   -> rosdep/apt dependency thiếu
MISSING_BUILD        -> source có nhưng install artifact thiếu
BAD_ENVIRONMENT      -> source sai thứ tự hoặc set -u làm hỏng setup
NETWORK_FAILURE      -> DNS/GitHub/apt
~~~

### Phase C — Cài dependency còn thiếu

Chỉ cài package còn thiếu sau baseline. Bộ cơ bản:

~~~bash
sudo apt update
sudo apt install -y \
  build-essential cmake git wget curl gnupg lsb-release \
  python3-pip python3-dev python3-setuptools \
  python3-colcon-common-extensions python3-vcstool python3-rosdep \
  python3-argcomplete python3-numpy python3-scipy python3-yaml \
  python3-opencv python3-pytest \
  libboost-dev libboost-python-dev libeigen3-dev libopencv-dev \
  libsqlite3-dev libzstd-dev
~~~

Camera nếu cần:

~~~bash
sudo apt install -y rpicam-apps python3-picamera2 \
  python3-libcamera libcamera-tools
~~~

Rosdep:

~~~bash
sudo rosdep init 2>&1 || true
rosdep update
cd "$ROS_ROOT"
rosdep install \
  --from-paths src \
  --ignore-src \
  --rosdistro humble \
  -y \
  --skip-keys 'fastcdr rti-connext-dds-6.0.1 urdfdom_headers python3-vcstool libpython3.10 libpython3.10-dev libcyclonedds-dev' \
  2>&1 | tee "$HOME/old_pi_ros2_handoff/rosdep.log"
~~~

Nếu `ros-dev-tools` không có candidate, sửa apt source/key trước theo
STEP 1–3 của guidebook camera.

### Phase D — Core source build nếu còn thiếu

~~~bash
cd "$ROS_ROOT"
set +u
source "$ROS_ROOT/install/setup.bash" 2>/dev/null || true
set -u

colcon build \
  --symlink-install \
  --parallel-workers 2 \
  --packages-skip rqt_bag launch_testing_examples rviz_default_plugins test_rclcpp \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DPYTHON_EXECUTABLE=/usr/bin/python3.11 \
    -DPython3_EXECUTABLE=/usr/bin/python3.11 \
  2>&1 | tee "$HOME/old_pi_ros2_handoff/core_build.log"
~~~

Nếu fail vì GitHub/DNS, sửa network rồi build lại. Nếu core đã có nhưng rosbag2
thiếu, chuyển Phase E; không rebuild toàn bộ nếu không cần.

### Phase E — Build rosbag2/MCAP cùng nhóm

Từ repo project:

~~~bash
cd /path/to/autonomous_car_hardware_platform_v2
ROS_ROOT="$ROS_ROOT" bash scripts/bootstrap_rosbag2.sh \
  2>&1 | tee "$HOME/old_pi_ros2_handoff/rosbag2_build.log"
~~~

Nếu báo thiếu vendor:

~~~bash
find "$ROS_ROOT/src" -maxdepth 5 -type d \
  \( -name mcap_vendor -o -name zstd_vendor -o -name rosbag2 \) -print
for p in mcap_vendor zstd_vendor rosbag2_storage_mcap; do
  test -d "$ROS_ROOT/install/$p" \
    && echo "FOUND install/$p" || echo "MISSING install/$p"
done
~~~

Nếu vendor chưa có trong install:

~~~bash
cd "$ROS_ROOT"
set +u
source "$ROS_ROOT/install/setup.bash" 2>/dev/null || true
set -u
colcon build --symlink-install --parallel-workers 2 \
  --packages-select mcap_vendor zstd_vendor \
  --event-handlers console_cohesion+ \
  2>&1 | tee "$HOME/old_pi_ros2_handoff/vendor_build.log"

ROS_ROOT="$ROS_ROOT" bash \
  /path/to/autonomous_car_hardware_platform_v2/scripts/bootstrap_rosbag2.sh \
  2>&1 | tee "$HOME/old_pi_ros2_handoff/rosbag2_build_retry.log"
~~~

Không skip `mcap_vendor` nếu mục tiêu là MCAP. Không chỉ build `rosbag2_py`
đơn lẻ vì dependency chain sẽ tạo lỗi package hook.

### Phase F — Verify artifact/runtime

~~~bash
set +u
source "$ROS_ROOT/install/setup.bash"
set -u

command -v ros2
ros2 --help >/dev/null
ros2 bag --help >/dev/null

for p in rosbag2_storage rosbag2_storage_default_plugins \
         rosbag2_storage_mcap rosbag2_compression \
         rosbag2_compression_zstd rosbag2_cpp \
         rosbag2_transport rosbag2_py ros2bag; do
  ros2 pkg prefix "$p"
done

python3 - <<'PY'
import csv
import numpy
import scipy
import rosbag2_py
from std_msgs.msg import String
from scipy.optimize import least_squares
print("CSV + numpy + scipy + rosbag2_py runtime: PASS")
PY
~~~

### Phase G — Functional rosbag test

Test không cần motor/camera:

~~~bash
TEST_DIR="$(mktemp -d /tmp/rosbag2_handoff.XXXXXX)"
ros2 topic pub /handoff_test std_msgs/msg/String '{data: hello}' \
  >"$TEST_DIR/publisher.log" 2>&1 &
PUB_PID=$!
sleep 2

set +e
timeout 8 ros2 bag record --storage mcap \
  --output "$TEST_DIR/bag" /handoff_test \
  >"$TEST_DIR/record.log" 2>&1
RECORD_RC=$?
set -e

kill "$PUB_PID" 2>/dev/null || true
wait "$PUB_PID" 2>/dev/null || true
echo "RECORD_RC=$RECORD_RC"
cat "$TEST_DIR/record.log"
ros2 bag info "$TEST_DIR/bag" | tee "$TEST_DIR/info.log"
find "$TEST_DIR" -maxdepth 3 -type f -printf '%p %s bytes\n' | sort
~~~

`timeout` có thể trả mã 124. PASS nghĩa là bag được tạo, `ros2 bag info`
đọc được bag và có topic `/handoff_test`. Chỉ khi MCAP fail mới thử SQLite3
để phân biệt storage plugin:

~~~bash
timeout 8 ros2 bag record --storage sqlite3 \
  --output "$TEST_DIR/bag_sqlite" /handoff_test
ros2 bag info "$TEST_DIR/bag_sqlite"
~~~

SQLite PASS/MCAP FAIL chưa phải rosbag2 hoàn chỉnh.

### Phase H — Verify CSV/fitting

~~~bash
python3 - <<'PY'
import csv
import numpy as np
import scipy
from scipy.optimize import least_squares
print("csv stdlib: PASS")
print("numpy:", np.__version__)
print("scipy:", scipy.__version__)
print("least_squares: PASS")
PY
~~~

Source repo dùng:

~~~text
ros2_ws/src/rc_validation/rc_validation/longitudinal_recorder.py
ros2_ws/src/rc_validation/rc_validation/longitudinal_fit.py
ros2_ws/src/rc_validation/rc_validation/velocity_bag_evaluator.py
~~~

Import thực tế:

~~~python
import csv
import numpy as np
from scipy.optimize import least_squares
import rosbag2_py
~~~

Không cài `pandas` chỉ vì thấy file `.csv`.

---

## 6. Verify workspace project sau khi underlay PASS

Mỗi terminal runtime source theo thứ tự:

~~~bash
source "$ROS_ROOT/install/setup.bash"
source ros2_ws/install/setup.bash
~~~

Từ repo project:

~~~bash
bash scripts/build.sh
bash scripts/test.sh
bash scripts/preflight.sh
~~~

`preflight.sh` phải PASS tối thiểu:

~~~text
ROS 2 CLI
ros2 bag CLI
acados native + Python
package rc_validation
package rc_bringup
package hardware_camera
package image_processing
~~~

Nếu user/path khác `ac`, Codex phải chỉnh path/biến phù hợp trước khi chạy,
không tạo PASS giả bằng cách bỏ qua check.

---

## 7. Tiêu chí kết luận

### PASS ROS 2 Humble + rosbag2

~~~text
[ ] Debian 12/Bookworm arm64, Python 3.11 đã xác nhận
[ ] rosdep install hoàn tất hoặc failure còn lại đã giải thích
[ ] rclpy và sensor_msgs import PASS
[ ] ros2 CLI chạy từ source underlay đúng
[ ] storage, MCAP, compression, cpp, transport, py và ros2bag đều có
[ ] ros2 bag --help PASS
[ ] functional record + ros2 bag info PASS
[ ] không còn process tạm thời ngoài ý muốn
~~~

### PASS CSV/fitting

~~~text
[ ] Python stdlib csv import PASS
[ ] numpy import PASS
[ ] scipy.optimize.least_squares import PASS
[ ] rosbag2_py import PASS
[ ] package rc_validation build/test PASS
~~~

Không ghi “full ROS 2 fixed” nếu chỉ có camera topic PASS hoặc chỉ có
`ros2 bag --help` mà functional record/read chưa PASS.

---

## 8. Report bắt buộc

Lưu:

~~~text
~/old_pi_ros2_handoff/baseline.log
~/old_pi_ros2_handoff/source.log
~/old_pi_ros2_handoff/rosdep.log
~/old_pi_ros2_handoff/core_build.log
~/old_pi_ros2_handoff/rosbag2_build.log
~/old_pi_ros2_handoff/vendor_build.log
~/old_pi_ros2_handoff/rosbag2_build_retry.log
~~~

File nào không dùng ghi rõ “not applicable”. Report cần có:

~~~text
1. ROS_ROOT và workspace path thực tế.
2. Baseline trước sửa: package nào PASS/MISSING.
3. Lỗi đầu tiên và nguyên nhân.
4. Lệnh/package đã cài hoặc build.
5. Kết quả ros2 bag --help.
6. Functional record/info và storage format (mcap/sqlite3).
7. Kết quả import csv/numpy/scipy/rosbag2_py.
8. Kết quả build/test/preflight của project.
9. Mọi failure còn lại; không đổi FAIL thành PASS.
~~~

---

## 9. File repo nên đọc trước khi hành động

~~~text
project_state.md
README.md
docs/vi/rpi5_ros2_humble_camera_setup_guidebook_vi.md
scripts/bootstrap_rosbag2.sh
scripts/preflight.sh
scripts/build.sh
scripts/test.sh
ros2_ws/src/rc_validation/package.xml
ros2_ws/src/rc_validation/rc_validation/longitudinal_fit.py
ros2_ws/src/rc_validation/rc_validation/velocity_bag_evaluator.py
~~~

**Thông điệp cuối cho Codex Pi cũ:** tái tạo trạng thái từ source và dependency
trên chính Pi cũ, build nhóm MCAP/rosbag2 cùng nhau, verify bằng functional bag
test, rồi mới tiếp tục sửa workspace xe. Đừng đánh đồng camera bring-up cũ với
full dependency completion.

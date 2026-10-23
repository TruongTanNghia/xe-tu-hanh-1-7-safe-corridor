# Guidebook: Setup Raspberry Pi 5 + ROS 2 Humble Core + Pi Camera Module 3 Wide

**Ngôn ngữ:** Tiếng Việt  
**Mục tiêu:** Hướng dẫn người mới bắt đầu setup Raspberry Pi 5 từ gần như zero để chạy được ROS 2 Humble core và publish ảnh từ Pi Camera Module 3 Wide lên topic ROS 2 `/camera/image_raw/compressed`.  
**Trạng thái đã kiểm chứng trong phiên bring-up:** `PASS_OPERATIONAL` cho ROS 2 core + camera compressed ROS topic.  
**Nền tảng đã kiểm chứng:** Raspberry Pi 5, Raspberry Pi OS / Debian 12 Bookworm, Python 3.11, kernel `rpt-rpi-2712`, ROS 2 Humble build từ source.

---

## 0. Tóm tắt kết quả cuối cùng

Sau khi hoàn thành guidebook này, hệ thống kỳ vọng đạt được:

```text
Raspberry Pi 5 + ROS 2 Humble core + Pi Camera Module 3 Wide compressed ROS topic: PASS_OPERATIONAL
```

Các kết quả chính cần đạt:

| Hạng mục | Kết quả mong muốn |
|---|---|
| OS | Debian 12 Bookworm / Raspberry Pi OS Bookworm |
| Python | Python 3.11.x |
| ROS 2 | Humble, build từ source |
| ROS command | `ros2` chạy được |
| ROS graph baseline | `/parameter_events`, `/rosout` |
| Camera system tool | `rpicam-still`, `rpicam-hello` chạy được |
| Camera still capture | tạo được ảnh `.jpg` |
| ROS camera node | chạy được `camera_node.py` |
| Camera topic | `/camera/image_raw/compressed` |
| Message type | `sensor_msgs/msg/CompressedImage` |
| FPS kỳ vọng | khoảng `29–30 FPS` |
| Cleanup | không còn process dư |

Giới hạn cần ghi rõ:

```text
Full ROS 2 source build is not clean-complete. rosbag2/rqt_bag/test/example packages may be skipped or fail. This does not block the current camera bring-up path.
```

---

## 1. Phạm vi và nguyên tắc an toàn

### 1.1. Phạm vi guidebook

Guidebook này chỉ xử lý:

- Raspberry Pi 5 setup cơ bản.
- ROS 2 Humble core build từ source trên Debian 12 Bookworm.
- Pi Camera Module 3 Wide kiểm tra bằng `rpicam-still`.
- ROS 2 Python camera node publish compressed image topic.
- Runtime evidence và cleanup.

Guidebook này **không** xử lý:

- Servo/ESC actuation.
- STM32/Nucleo firmware.
- micro-ROS Agent.
- RPLIDAR A1M8.
- BNO055 IMU.
- Rolling test hoặc dynamic driving.
- Lane following closed-loop.
- Full ROS 2 desktop/RViz/rosbag2 validation.

### 1.2. Trạng thái an toàn bắt buộc

Nếu Raspberry Pi được gắn trên xe RC, trước mọi bước setup:

```text
ESC main power: OFF
Motor: not powered
Servo: not tested
No actuation path active
Wheels lifted or vehicle restrained if any actuator power is connected
```

Guidebook này không gửi lệnh điều khiển motor hoặc servo. Tuy nhiên vẫn cần giữ ESC main power OFF để tránh rủi ro do dây tín hiệu, nguồn hoặc node khác chạy nhầm.

---

## 2. Tại sao phải build ROS 2 Humble từ source?

Trên Raspberry Pi OS Bookworm / Debian 12:

- Python mặc định thường là Python 3.11.
- ROS 2 Humble binary package chính thống được thiết kế chủ yếu cho Ubuntu 22.04 / Python 3.10.
- Vì vậy trên Raspberry Pi 5 Bookworm, route thực tế ổn hơn là:

```text
Install ROS build tools from ROS apt repository
Download ROS 2 Humble source
Build core packages from source with Python 3.11
```

Lưu ý: Ta dùng ROS apt repository chủ yếu để cài tool như `ros-dev-tools`, `colcon`, `vcs`, `rosdep`. Không cài full ROS 2 binary Humble lên Bookworm.

---

## 3. Quy ước trong guidebook

### 3.1. User và hostname mẫu

Guidebook dùng ví dụ đã kiểm chứng:

```text
user: ac
hostname: picar-new
workspace: /home/ac/ros2_humble
camera node: /home/ac/ros2_ws/camera_node.py
```

Nếu user khác `ac`, thay `/home/ac` bằng `/home/<user_của_bạn>`.

### 3.2. Cách chạy command

Mỗi bước có:

- **Mục tiêu**
- **Lệnh cần chạy**
- **Kết quả mong muốn**
- **Lỗi có thể gặp và cách sửa**
- **Điều kiện để chuyển bước tiếp theo**

Nên chạy từng bước một. Không nên copy toàn bộ guidebook và chạy một lần.

### 3.3. Runtime classification

Các nhãn dùng trong guidebook:

```text
PASS_OPERATIONAL      # chạy đúng, có evidence và cleanup sạch
PASS_WITH_NOTES       # chạy được nhưng có cảnh báo không chặn
FAIL_RUNTIME          # lỗi runtime, cần sửa trước khi đi tiếp
INCOMPLETE_EXPORT     # thiếu evidence/log/export
INCOMPLETE_CLEANUP    # chưa xác nhận cleanup
SAFETY_BLOCKED        # bị chặn vì trạng thái an toàn chưa đạt
```

---

# STEP 0 — Kiểm tra an toàn, OS, network và tài nguyên ban đầu

## Mục tiêu

Xác nhận Pi đang chạy đúng OS, có mạng, đủ RAM/disk, chưa có process runtime dư.

## Lệnh cần chạy

```bash
# STEP 0: baseline system, network, resource, and safety check

echo "===== STEP 0: BASELINE SYSTEM CHECK ====="

echo "===== SAFETY STATE ====="
echo "ESC main power: OFF"
echo "Motor: not powered"
echo "Servo: not tested"
echo "No actuation path active"

echo "===== SYSTEM IDENTITY ====="
hostname
whoami
cat /etc/os-release
uname -a
dpkg --print-architecture
python3 --version

echo "===== NETWORK CHECK ====="
ip -br addr
ip route
ping -c 3 8.8.8.8 || true
ping -c 3 github.com || true
getent hosts github.com || true
getent hosts raw.githubusercontent.com || true

echo "===== RESOURCE CHECK ====="
free -h
df -h /
vcgencmd measure_temp 2>/dev/null || true

echo "===== RESIDUAL PROCESS CHECK ====="
pgrep -af 'ros2|camera_node|rpicam|libcamera|micro_ros_agent|colcon build|vcs import|rosdep install' || true
```

## Kết quả mong muốn

Ví dụ output đạt:

```text
PRETTY_NAME="Debian GNU/Linux 12 (bookworm)"
arm64
Python 3.11.2
ping github.com: PASS
RAM available: nhiều hơn 4GB
Disk available: nên còn ít nhất 30GB
Không có process ROS/camera/build dư
```

## Lỗi có thể gặp và cách sửa

| Lỗi | Nguyên nhân | Cách sửa |
|---|---|---|
| Không ping được `8.8.8.8` | Chưa có network route | Kiểm tra Wi-Fi/Ethernet, router, `nmcli` |
| Ping được `8.8.8.8` nhưng không ping được `github.com` | DNS lỗi | Kiểm tra `/etc/resolv.conf`, đổi DNS hoặc reboot network |
| Disk còn dưới 25–30GB | Không đủ cho source build | Dọn file, dùng thẻ nhớ lớn hơn |
| Có process `colcon build` hoặc `camera_node` dư | Phiên cũ chưa cleanup | `pkill -f ...` theo process cụ thể |

## Điều kiện chuyển bước

Chỉ chuyển bước nếu:

```text
OS = Bookworm/Debian 12
Architecture = arm64
Python = 3.11.x
Network OK
Disk còn khoảng 30GB trở lên
Không có runtime process dư
```

Runtime classification: `PASS_OPERATIONAL` cho baseline nếu các điều kiện trên đạt.

---

# STEP 1 — Kiểm tra apt source hiện tại

## Mục tiêu

Xác định Pi đã có ROS 2 apt repository chưa. Nếu chưa có, các package như `ros-dev-tools` sẽ báo `Unable to locate package`.

## Lệnh cần chạy

```bash
# STEP 1: inspect apt sources and ROS build tool availability

echo "===== STEP 1: APT SOURCE INSPECTION ====="

echo "===== APT SOURCE FILES ====="
ls -l /etc/apt/sources.list /etc/apt/sources.list.d 2>/dev/null || true

echo "===== SOURCE CONTENT ====="
cat /etc/apt/sources.list 2>/dev/null || true
for f in /etc/apt/sources.list.d/*.list; do
  echo "----- $f -----"
  cat "$f" 2>/dev/null || true
done

echo "===== CHECK ROS KEYRING ====="
ls -lh /usr/share/keyrings/ros-archive-keyring.gpg 2>/dev/null || true

echo "===== APT POLICY CHECK ====="
apt-cache policy ros-dev-tools python3-colcon-common-extensions python3-vcstool python3-rosdep python3-argcomplete || true

echo "===== CAMERA PACKAGE POLICY CHECK ====="
apt-cache policy rpicam-apps python3-picamera2 python3-opencv libcamera-tools python3-libcamera || true
```

## Kết quả mong muốn

Nếu ROS repository đã có, sẽ thấy policy cho:

```text
ros-dev-tools
python3-colcon-common-extensions
python3-vcstool
python3-rosdep
```

Nếu chưa có, output có thể chỉ thấy `python3-argcomplete`, còn các package ROS không có candidate.

## Lỗi có thể gặp và cách sửa

| Lỗi | Ý nghĩa | Cách sửa |
|---|---|---|
| `Unable to locate package ros-dev-tools` | Chưa có ROS apt source | Làm STEP 2 |
| Không có `/usr/share/keyrings/ros-archive-keyring.gpg` | Chưa có ROS key | Làm STEP 2 |
| Không có file `ros2.list` | Chưa add ROS repo | Làm STEP 2 |

## Điều kiện chuyển bước

- Nếu apt policy đã có `ros-dev-tools`: chuyển STEP 3.
- Nếu chưa có: làm STEP 2.

---

# STEP 2 — Thêm ROS 2 apt repository cho build tools

## Mục tiêu

Thêm ROS 2 apt repository để cài `ros-dev-tools`, `colcon`, `vcs`, `rosdep`.

## Lệnh cần chạy

```bash
# STEP 2: add ROS 2 apt repository for build tools

echo "===== STEP 2: ADD ROS 2 APT REPOSITORY ====="

sudo apt update
sudo apt install -y curl gnupg lsb-release

# Download ROS apt key
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add ROS 2 Jammy repository for arm64 build tools
# Note: used for tools; ROS 2 itself will be built from source.
echo "deb [arch=arm64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" | \
  sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update

echo "===== VERIFY ROS APT SOURCE ====="
cat /etc/apt/sources.list.d/ros2.list
ls -lh /usr/share/keyrings/ros-archive-keyring.gpg
apt-cache policy ros-dev-tools python3-colcon-common-extensions python3-vcstool python3-rosdep python3-argcomplete || true
```

## Kết quả mong muốn

Cần thấy:

```text
/etc/apt/sources.list.d/ros2.list
ros-dev-tools candidate ... packages.ros.org ... jammy/main arm64
python3-colcon-common-extensions candidate ...
python3-vcstool candidate ...
python3-rosdep candidate ...
```

## Lỗi có thể gặp và cách sửa

| Lỗi | Nguyên nhân | Cách sửa |
|---|---|---|
| `Could not resolve host: raw.githubusercontent.com` | DNS/network lỗi | Quay lại STEP 0 network check; thử reboot network/router |
| `NO_PUBKEY` hoặc lỗi signature | Keyring sai hoặc thiếu | Xóa key cũ, tải lại `ros.key` |
| `packages.ros.org` không fetch được | Mạng chặn hoặc DNS lỗi | Kiểm tra DNS, thử mạng khác |

## Điều kiện chuyển bước

Chuyển STEP 3 nếu `apt-cache policy ros-dev-tools` có candidate.

---

# STEP 3 — Cài ROS development tools

## Mục tiêu

Cài các tool cần cho source build:

- `colcon`
- `vcs`
- `rosdep`
- các extension liên quan

## Lệnh cần chạy

```bash
# STEP 3: install ROS development tools

echo "===== STEP 3: INSTALL ROS DEVELOPMENT TOOLS ====="

sudo apt install -y ros-dev-tools

echo "===== VERIFY TOOL PATHS ====="
which colcon || true
which vcs || true
which rosdep || true
which ros2 || true

echo "===== VERIFY TOOL VERSIONS ====="
colcon --help | head -n 5 || true
vcs --version || true
rosdep --version || true

echo "===== VERIFY INSTALLED PACKAGES ====="
dpkg -l | grep -E 'ros-dev-tools|python3-colcon|python3-vcstool|python3-rosdep|python3-argcomplete' || true

echo "===== RESIDUAL PROCESS CHECK ====="
pgrep -af 'colcon build|vcs import|rosdep install' || true
```

## Kết quả mong muốn

Cần thấy:

```text
/usr/bin/colcon
/usr/bin/vcs
/usr/bin/rosdep
```

`ros2` chưa có ở bước này là bình thường vì ROS 2 chưa build.

## Lỗi có thể gặp và cách sửa

| Lỗi | Nguyên nhân | Cách sửa |
|---|---|---|
| `Unable to locate package ros-dev-tools` | Chưa add ROS repo | Quay lại STEP 2 |
| `colcon --version` không hợp lệ | Một số bản colcon không có option version | Không chặn, dùng `colcon --help` |
| `ros2` command not found | Chưa build/source ROS | Bình thường ở bước này |

## Điều kiện chuyển bước

Chuyển STEP 4 nếu `colcon`, `vcs`, `rosdep` đều có path.

---

# STEP 4 — Cài build dependencies nền tảng

## Mục tiêu

Cài compiler, CMake, Python dev headers và một số thư viện nền.

## Lệnh cần chạy

```bash
# STEP 4: install base build dependencies

echo "===== STEP 4: INSTALL BASE BUILD DEPENDENCIES ====="

sudo apt install -y build-essential cmake git wget python3-pip python3-dev \
  python3-setuptools libbullet-dev libpython3-dev libcunit1-dev \
  libasio-dev libtinyxml2-dev

echo "===== VERIFY BUILD TOOLS ====="
which gcc || true
which g++ || true
which cmake || true
which git || true
which wget || true
which python3 || true
which pip3 || true

echo "===== VERIFY VERSIONS ====="
gcc --version | head -n 1 || true
g++ --version | head -n 1 || true
cmake --version | head -n 1 || true
git --version || true
wget --version | head -n 1 || true
python3 --version || true
pip3 --version || true

echo "===== RESOURCE CHECK ====="
free -h
df -h /
vcgencmd measure_temp 2>/dev/null || true
```

## Kết quả mong muốn

Cần thấy:

```text
/usr/bin/gcc
/usr/bin/g++
/usr/bin/cmake
/usr/bin/git
/usr/bin/wget
Python 3.11.x
```

## Lỗi có thể gặp và cách sửa

| Lỗi | Nguyên nhân | Cách sửa |
|---|---|---|
| Package not found | apt source chưa update | `sudo apt update`, kiểm tra source |
| Disk gần đầy | Source build sẽ fail | Dọn disk trước khi đi tiếp |
| Nhiệt độ cao | Pi thiếu tản nhiệt | Dừng build, thêm fan/tản nhiệt |

## Điều kiện chuyển bước

Chuyển STEP 5 nếu compiler, CMake, Git, Python đều có.

---

# STEP 5 — Tạo workspace và tải ROS 2 Humble source

## Mục tiêu

Tải toàn bộ source tree ROS 2 Humble vào `~/ros2_humble/src`.

## Lệnh cần chạy

```bash
# STEP 5: create ROS 2 Humble source workspace and import repositories

echo "===== STEP 5: DOWNLOAD ROS 2 HUMBLE SOURCE ====="

mkdir -p ~/ros2_humble/src
cd ~/ros2_humble

# Download Humble repo manifest
wget -O ros2.repos https://raw.githubusercontent.com/ros2/ros2/humble/ros2.repos

# Import source repositories
vcs import --input ros2.repos src

echo "===== VERIFY SOURCE TREE ====="
pwd
ls -lh ros2.repos
find src -mindepth 2 -maxdepth 3 -name .git | wc -l
find src -name package.xml | wc -l

# Key source directories expected for camera-route ROS core
for p in \
  src/ros2/rclpy \
  src/ros2/rclcpp \
  src/ros2/ros2cli \
  src/ros2/common_interfaces \
  src/ros2/launch_ros \
  src/ros2/rmw_fastrtps \
  src/eclipse-cyclonedds/cyclonedds \
  src/eProsima/Fast-DDS
do
  echo "----- CHECK $p -----"
  ls -ld "$p" 2>/dev/null || true
done

echo "===== RESIDUAL PROCESS CHECK ====="
pgrep -af 'vcs import|colcon build|rosdep install' || true
```

## Kết quả mong muốn

Ví dụ đã kiểm chứng:

```text
find .git count: khoảng 100+ repos
find package.xml count: khoảng 300+ packages
rclpy, rclcpp, ros2cli, common_interfaces tồn tại
```

## Lỗi có thể gặp và cách sửa

| Lỗi | Nguyên nhân | Cách sửa |
|---|---|---|
| `Could not resolve host: github.com` | DNS/network lỗi | Quay lại STEP 0 network check |
| Clone repo bị timeout | Mạng yếu | Chạy lại `vcs import --input ros2.repos src`; nó thường tiếp tục repo còn thiếu |
| Repo count quá thấp | Import chưa xong | Chạy lại `vcs import` |

## Điều kiện chuyển bước

Chuyển STEP 6 nếu source tree có đủ repo và các thư mục core tồn tại.

---

# STEP 6 — Khởi tạo và cập nhật rosdep

## Mục tiêu

Tạo database rosdep để tự động cài dependency cho source build.

## Lệnh cần chạy

```bash
# STEP 6: initialize and update rosdep

echo "===== STEP 6: ROSDEP INIT/UPDATE ====="

sudo rosdep init 2>&1 || true
rosdep update

echo "===== VERIFY ROSDEP ====="
rosdep --version
ls -l /etc/ros/rosdep/sources.list.d/20-default.list 2>/dev/null || true
ls -ld ~/.ros/rosdep/sources.cache 2>/dev/null || true

echo "===== RESIDUAL PROCESS CHECK ====="
pgrep -af 'rosdep install|colcon build|vcs import' || true
```

## Kết quả mong muốn

Cần thấy:

```text
Wrote /etc/ros/rosdep/sources.list.d/20-default.list
updated cache in ~/.ros/rosdep/sources.cache
rosdep version ...
```

Nếu `sudo rosdep init` báo file đã tồn tại, không sao.

## Lỗi có thể gặp và cách sửa

| Lỗi | Nguyên nhân | Cách sửa |
|---|---|---|
| `ERROR: default sources list file already exists` | Đã init trước đó | Không chặn, tiếp tục `rosdep update` |
| Không tải được rosdep yaml | Mạng/DNS lỗi | Quay lại STEP 0 |
| Permission denied | Sai quyền khi init | Dùng `sudo rosdep init` |

## Điều kiện chuyển bước

Chuyển STEP 7 nếu `rosdep update` hoàn tất.

---

# STEP 7 — Cài dependencies bằng rosdep

## Mục tiêu

Cài các dependency hệ thống cần thiết cho ROS 2 Humble source build.

## Lệnh cần chạy

```bash
# STEP 7: install ROS 2 Humble dependencies with rosdep

echo "===== STEP 7: ROSDEP INSTALL HUMBLE DEPENDENCIES ====="

cd ~/ros2_humble

rosdep install \
  --from-paths src \
  --ignore-src \
  --rosdistro humble \
  -y \
  --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers python3-vcstool libpython3.10 libpython3.10-dev libcyclonedds-dev" \
  2>&1 | tee ~/rosdep_install_humble_step7.log

echo "===== VERIFY ROSDEP LOG ====="
tail -n 40 ~/rosdep_install_humble_step7.log
ls -lh ~/rosdep_install_humble_step7.log

echo "===== RESOURCE CHECK ====="
free -h
df -h /
vcgencmd measure_temp 2>/dev/null || true

echo "===== RESIDUAL PROCESS CHECK ====="
pgrep -af 'rosdep install|colcon build|vcs import' || true
```

## Kết quả mong muốn

Cần thấy cuối log:

```text
#All required rosdeps installed successfully
```

## Lỗi có thể gặp và cách sửa

| Lỗi | Nguyên nhân | Cách sửa |
|---|---|---|
| rosdep không resolve được `libpython3.10` | Bookworm dùng Python 3.11 | Đã skip bằng `--skip-keys` |
| Package apt install fail | Network/apt lock | Kiểm tra `sudo apt update`, chờ apt process khác xong |
| Disk đầy | Dependency nhiều | Dọn disk hoặc dùng thẻ nhớ lớn hơn |

## Điều kiện chuyển bước

Chuyển STEP 8 nếu rosdep báo all required dependencies installed successfully.

---

# STEP 8 — Kiểm tra tài nguyên trước khi build

## Mục tiêu

Xác nhận RAM/swap/disk/nhiệt độ trước khi chạy `colcon build`.

## Lệnh cần chạy

```bash
# STEP 8: pre-build resource and swap check

echo "===== STEP 8: PRE-BUILD RESOURCE CHECK ====="

free -h
swapon --show
df -h /
vcgencmd measure_temp 2>/dev/null || true

echo "===== SWAP CONFIG CHECK ====="
dpkg -l | grep dphys-swapfile || true
grep -n '^CONF_SWAPSIZE' /etc/dphys-swapfile 2>/dev/null || true

echo "===== WORKSPACE SIZE ====="
du -sh ~/ros2_humble 2>/dev/null || true
```

## Kết quả mong muốn

Khuyến nghị:

```text
RAM: Pi 5 8GB tốt hơn
Swap: nên tăng lên 2048MB
Disk available: khoảng 30GB trở lên
Temp: nên dưới ~65°C trước khi build
```

## Lỗi có thể gặp và cách sửa

| Lỗi | Nguyên nhân | Cách sửa |
|---|---|---|
| Swap chỉ 512MB | Dễ fail khi build dài | Làm STEP 9 |
| Disk còn ít | Build fail giữa chừng | Dọn disk hoặc thẻ nhớ lớn hơn |
| Temp cao | Tản nhiệt kém | Thêm fan, nghỉ máy |

## Điều kiện chuyển bước

Nếu swap nhỏ hơn 2GB, làm STEP 9. Nếu đã có 2GB swap, có thể chuyển STEP 10.

---

# STEP 9 — Tăng swap lên 2048MB

## Mục tiêu

Tăng swap để giảm rủi ro build fail do thiếu memory.

## Lệnh cần chạy

```bash
# STEP 9: increase swap to 2048MB

echo "===== STEP 9: INCREASE SWAP TO 2048MB ====="

sudo apt install -y dphys-swapfile

# Backup current swap config
sudo cp /etc/dphys-swapfile /etc/dphys-swapfile.bak.$(date +%Y%m%d_%H%M%S)

# Set swap size to 2048MB
sudo sed -i 's/^CONF_SWAPSIZE=.*/CONF_SWAPSIZE=2048/' /etc/dphys-swapfile

# If CONF_SWAPSIZE line does not exist, append it
grep -q '^CONF_SWAPSIZE=' /etc/dphys-swapfile || echo 'CONF_SWAPSIZE=2048' | sudo tee -a /etc/dphys-swapfile

# Restart swap service
sudo systemctl restart dphys-swapfile
sleep 3

echo "===== VERIFY SWAP ====="
grep -n '^CONF_SWAPSIZE' /etc/dphys-swapfile
swapon --show
free -h

echo "===== RESOURCE CHECK ====="
df -h /
vcgencmd measure_temp 2>/dev/null || true
```

## Kết quả mong muốn

Cần thấy:

```text
CONF_SWAPSIZE=2048
Swap: 2.0Gi
```

## Lỗi có thể gặp và cách sửa

| Lỗi | Nguyên nhân | Cách sửa |
|---|---|---|
| `dphys-swapfile` không restart | Service lỗi | `sudo systemctl status dphys-swapfile` để xem lỗi |
| Swap vẫn 512MB | Config chưa được sửa | Kiểm tra `/etc/dphys-swapfile` |
| Disk quá ít sau khi tạo swap | Không đủ storage | Giảm swap hoặc dùng thẻ nhớ lớn hơn |

## Điều kiện chuyển bước

Chuyển STEP 10 nếu `free -h` hiển thị swap khoảng `2.0Gi`.

---

# STEP 10 — Build ROS 2 Humble core từ source

## Mục tiêu

Build ROS 2 Humble source đủ cho camera route:

- `ros2cli`
- `rclpy`
- `rclcpp`
- `sensor_msgs`
- `std_msgs`
- `launch_ros`

Không yêu cầu full desktop, RViz, rosbag2, rqt, test/example packages.

## Lệnh khuyến nghị

Lệnh dưới đây ưu tiên camera bring-up route. Nó skip các gói dễ gây lỗi hoặc không cần cho camera node.

```bash
# STEP 10: build ROS 2 Humble core from source for camera route

echo "===== STEP 10: BUILD ROS 2 HUMBLE CORE CAMERA ROUTE ====="

cd ~/ros2_humble

# Optional cleanup for known failed/partial packages from previous attempts
rm -rf build/mcap_vendor install/mcap_vendor log/latest_build/mcap_vendor 2>/dev/null || true
rm -rf build/rosbag2_storage_mcap install/rosbag2_storage_mcap log/latest_build/rosbag2_storage_mcap 2>/dev/null || true
rm -rf build/rosbag2_cpp install/rosbag2_cpp log/latest_build/rosbag2_cpp 2>/dev/null || true
rm -rf build/rqt_bag install/rqt_bag log/latest_build/rqt_bag 2>/dev/null || true
rm -rf build/launch_testing_examples install/launch_testing_examples log/latest_build/launch_testing_examples 2>/dev/null || true

# Build core packages. This can take 40-90+ minutes depending on cooling/storage.
colcon build \
  --symlink-install \
  --parallel-workers 3 \
  --packages-skip \
    mcap_vendor \
    rosbag2_storage_mcap \
    rosbag2_cpp \
    rosbag2 \
    ros2bag \
    rosbag2_py \
    rosbag2_transport \
    rosbag2_tests \
    rosbag2_performance_benchmarking \
    rosbag2_compression \
    rosbag2_compression_zstd \
    rqt_bag \
    launch_testing_examples \
    rviz_default_plugins \
    test_rclcpp \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DPYTHON_EXECUTABLE=/usr/bin/python3.11 \
    -DPython3_EXECUTABLE=/usr/bin/python3.11 \
  2>&1 | tee ~/ros2_build_humble_step10.log

BUILD_EXIT_CODE=${PIPESTATUS[0]}
echo "BUILD_EXIT_CODE=${BUILD_EXIT_CODE}" | tee -a ~/ros2_build_humble_step10.log

echo "===== BUILD SUMMARY TAIL ====="
tail -n 80 ~/ros2_build_humble_step10.log

echo "===== RESOURCE CHECK AFTER BUILD ====="
free -h
df -h /
vcgencmd measure_temp 2>/dev/null || true

echo "===== RESIDUAL PROCESS CHECK ====="
pgrep -af 'colcon build|vcs import|rosdep install' || true
```

## Kết quả mong muốn

Tốt nhất là:

```text
BUILD_EXIT_CODE=0
```

Tuy nhiên trên route đã kiểm chứng, build có thể không sạch hoàn toàn nếu một số gói optional/test/example bị kéo theo. Với camera route, điều quan trọng là core packages phải build được. Ta sẽ xác nhận ở STEP 11 và STEP 12.

## Lỗi có thể gặp và cách sửa

### Lỗi 1: DNS/GitHub trong lúc build

Dấu hiệu:

```text
Could not resolve host: github.com
shared_queues_vendor failed
```

Cách sửa:

```bash
# Diagnose network/DNS
ping -c 3 8.8.8.8 || true
ping -c 3 github.com || true
getent hosts github.com || true
curl -I https://github.com || true

# Nếu network OK lại, xóa package fail rồi build lại
cd ~/ros2_humble
rm -rf build/shared_queues_vendor install/shared_queues_vendor log/latest_build/shared_queues_vendor 2>/dev/null || true
```

Sau đó chạy lại STEP 10.

### Lỗi 2: `rosbag2_cpp` đòi `mcap_vendor`

Dấu hiệu:

```text
Failed to find:
/home/ac/ros2_humble/install/mcap_vendor/share/mcap_vendor/package.sh
/home/ac/ros2_humble/install/rosbag2_storage_mcap/share/rosbag2_storage_mcap/package.sh
```

Cách sửa:

- Không chỉ skip `mcap_vendor` và `rosbag2_storage_mcap` riêng lẻ.
- Skip cả nhóm `rosbag2` như trong lệnh STEP 10.

### Lỗi 3: `rqt_bag` fail vì rosbag2 bị skip

Dấu hiệu:

```text
Failed <<< rqt_bag
```

Cách sửa:

- Thêm `rqt_bag` vào `--packages-skip`.

### Lỗi 4: `launch_testing_examples` fail vì rosbag2 bị skip

Dấu hiệu:

```text
Failed <<< launch_testing_examples
```

Cách sửa:

- Thêm `launch_testing_examples` vào `--packages-skip`.
- Package này không cần cho camera runtime.

### Lỗi 5: nhiệt độ cao

Dấu hiệu:

```text
temp > 70°C
build chậm hoặc crash
```

Cách sửa:

- Dùng fan/tản nhiệt.
- Giảm `--parallel-workers 3` xuống `--parallel-workers 2`.

## Điều kiện chuyển bước

Chuyển STEP 11 dù `BUILD_EXIT_CODE` không bằng 0, miễn là build đã chạy xong và không còn process `colcon build`. STEP 11 sẽ xác nhận core package có đủ không.

---

# STEP 11 — Kiểm tra core packages sau build

## Mục tiêu

Xác nhận các gói cần cho camera route đã có trong `~/ros2_humble/install`.

## Lệnh cần chạy

```bash
# STEP 11: verify ROS 2 core package artifacts

echo "===== STEP 11: VERIFY ROS 2 CORE PACKAGE ARTIFACTS ====="

cd ~/ros2_humble

for p in \
  launch_ros \
  rclcpp \
  rclpy \
  ros2cli \
  sensor_msgs \
  std_msgs \
  ros2component
do
  echo "----- CHECK install/$p -----"
  ls -ld "install/$p" 2>/dev/null || true
done

echo "===== CHECK SETUP FILES ====="
ls -lh install/setup.bash install/local_setup.bash 2>/dev/null || true

echo "===== CHECK RECENT BUILD ERRORS ====="
grep -nE 'Failed <<<|Aborted <<<|BUILD_EXIT_CODE|Failed to find|Could not resolve host|Unknown CMake command|Traceback' ~/ros2_build_humble_step10.log 2>/dev/null | tail -n 120 || true

echo "===== RESOURCE CHECK ====="
free -h
df -h /
vcgencmd measure_temp 2>/dev/null || true

echo "===== RESIDUAL PROCESS CHECK ====="
pgrep -af 'colcon build|vcs import|rosdep install' || true
```

## Kết quả mong muốn

Cần thấy các thư mục này tồn tại:

```text
install/launch_ros
install/rclcpp
install/rclpy
install/ros2cli
install/sensor_msgs
install/std_msgs
```

Nếu các gói trên có đủ, có thể đi tiếp STEP 12 để verify runtime.

## Lỗi có thể gặp và cách sửa

| Lỗi | Ý nghĩa | Cách sửa |
|---|---|---|
| Thiếu `install/ros2cli` | Build core chưa đủ | Chạy lại STEP 10, xem log lỗi |
| Thiếu `install/rclpy` | Python ROS client chưa build | Xem log `rclpy`, kiểm tra Python dev |
| Thiếu `sensor_msgs` | Common interfaces chưa build | Xem lỗi `common_interfaces` |
| `install/setup.bash` không có | Build chưa tạo install space | Build fail quá sớm, quay STEP 10 |

## Điều kiện chuyển bước

Chuyển STEP 12 nếu core package artifacts có đủ.

---

# STEP 12 — Verify ROS 2 core runtime

## Mục tiêu

Source ROS 2 environment và xác nhận:

- `ros2` command chạy được.
- Python import `rclpy`, `CompressedImage`, `String` chạy được.
- ROS graph baseline có `/parameter_events`, `/rosout`.
- Không có setup warning chặn runtime.

## Lệnh cần chạy

```bash
# STEP 12: verify ROS 2 core runtime

echo "===== STEP 12: VERIFY ROS 2 CORE RUNTIME ====="

source ~/ros2_humble/install/setup.bash

echo "===== VERIFY ROS ENV ====="
echo "ROS_DISTRO=$ROS_DISTRO"
echo "ROS_VERSION=$ROS_VERSION"
echo "ROS_PYTHON_VERSION=$ROS_PYTHON_VERSION"

echo "===== VERIFY ROS2 COMMAND ====="
which ros2 || true
ros2 --help | head -n 20 || true

echo "===== VERIFY PYTHON IMPORTS ====="
python3 - <<'PY'
import rclpy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
print("rclpy import: PASS")
print("sensor_msgs CompressedImage import: PASS")
print("std_msgs String import: PASS")
PY

echo "===== VERIFY ROS GRAPH BASELINE ====="
ros2 topic list 2>&1 | head -n 30 || true

echo "===== CHECK SETUP WARNINGS IN FRESH SHELL ====="
bash -lc 'source ~/ros2_humble/install/setup.bash && echo "fresh shell source: PASS"' 2>&1 | tee ~/ros2_source_check_step12.log

echo "===== RESIDUAL PROCESS CHECK ====="
ros2 daemon stop 2>/dev/null || true
sleep 1
pgrep -af 'ros2cli.daemon.daemonize|camera_node|micro_ros_agent|colcon build|vcs import|rosdep install' || true
```

## Kết quả mong muốn

Cần thấy:

```text
ROS_DISTRO=humble
ROS_VERSION=2
ROS_PYTHON_VERSION=3
/home/ac/ros2_humble/install/ros2cli/bin/ros2
rclpy import: PASS
sensor_msgs CompressedImage import: PASS
std_msgs String import: PASS
/parameter_events
/rosout
fresh shell source: PASS
```

## Lỗi có thể gặp và cách sửa

| Lỗi | Nguyên nhân | Cách sửa |
|---|---|---|
| `ros2: command not found` sau khi source | `setup.bash` không đúng hoặc build thiếu `ros2cli` | Quay STEP 11/10 |
| Import `rclpy` fail | `rclpy` chưa build hoặc PYTHONPATH sai | Source lại setup; nếu vẫn fail quay STEP 10 |
| `ros2 topic list` fail | ROS core runtime chưa ổn | Xem lỗi cụ thể, kiểm tra `rclpy`, daemon |
| `not found: mcap_vendor ...` khi source | Skip package nhưng hook còn tham chiếu | Kiểm tra build/skip; nếu chỉ warning và core chạy vẫn có thể đi tiếp, nhưng nên ghi chú |

## Điều kiện chuyển bước

Chuyển STEP 13 nếu STEP 12 đạt. Runtime classification: `PASS_OPERATIONAL` cho ROS 2 core runtime.

---

# STEP 13 — Persist ROS 2 environment vào `.bashrc`

## Mục tiêu

Tự động source ROS 2 khi mở terminal mới.

## Lệnh cần chạy

```bash
# STEP 13: persist ROS 2 environment in ~/.bashrc

echo "===== STEP 13: PERSIST ROS 2 ENVIRONMENT ====="

echo "===== CHECK EXISTING BASHRC ROS LINES ====="
grep -nE 'ros2_humble|setup.bash|local_setup.bash' ~/.bashrc || true

echo "===== ADD ROS 2 SOURCE LINE IF MISSING ====="
grep -q 'source ~/ros2_humble/install/setup.bash' ~/.bashrc || \
  echo 'source ~/ros2_humble/install/setup.bash' >> ~/.bashrc

echo "===== VERIFY BASHRC ROS LINES ====="
grep -nE 'ros2_humble|setup.bash|local_setup.bash' ~/.bashrc || true

echo "===== VERIFY CURRENT SHELL STILL HAS ROS ENV ====="
source ~/.bashrc
echo "ROS_DISTRO=$ROS_DISTRO"
which ros2 || true
ros2 topic list | head -n 10 || true

echo "===== RESIDUAL PROCESS CHECK ====="
ros2 daemon stop 2>/dev/null || true
sleep 1
pgrep -af 'ros2cli.daemon.daemonize|camera_node|micro_ros_agent|colcon build|vcs import|rosdep install' || true
```

## Kết quả mong muốn

Cần thấy `.bashrc` có dòng:

```bash
source ~/ros2_humble/install/setup.bash
```

Và current shell có:

```text
ROS_DISTRO=humble
/home/ac/ros2_humble/install/ros2cli/bin/ros2
```

## Lỗi có thể gặp và cách sửa

| Lỗi | Nguyên nhân | Cách sửa |
|---|---|---|
| Dòng source bị thêm nhiều lần | Chạy lặp command thủ công sai | Xóa dòng trùng trong `nano ~/.bashrc` |
| `bash -lc` không thấy `ros2` | Login shell không load `.bashrc` như interactive shell | Làm STEP 14 |
| `.bashrc` lỗi syntax | Paste hỏng file | Mở `nano ~/.bashrc`, sửa dòng lỗi |

## Điều kiện chuyển bước

Chuyển STEP 14 để verify terminal interactive mới.

---

# STEP 14 — Verify interactive shell ROS environment

## Mục tiêu

Xác nhận terminal SSH/interactive mới tự nhận ROS 2.

## Lệnh cần chạy

```bash
# STEP 14: verify interactive shell ROS environment

echo "===== STEP 14: VERIFY INTERACTIVE SHELL ROS ENV ====="

echo "===== CURRENT BASHRC ROS LINE ====="
grep -nE 'ros2_humble|setup.bash|local_setup.bash' ~/.bashrc || true

echo "===== VERIFY WITH INTERACTIVE BASH ====="
bash -ic 'echo "ROS_DISTRO=$ROS_DISTRO"; which ros2; ros2 topic list | head -n 10'

echo "===== VERIFY PROFILE SOURCES BASHRC ====="
grep -nE 'bashrc|BASH_VERSION' ~/.profile ~/.bash_profile ~/.bash_login 2>/dev/null || true

echo "===== RESIDUAL PROCESS CHECK ====="
ros2 daemon stop 2>/dev/null || true
sleep 1
pgrep -af 'ros2cli.daemon.daemonize|camera_node|micro_ros_agent|colcon build|vcs import|rosdep install' || true
```

## Kết quả mong muốn

Cần thấy:

```text
ROS_DISTRO=humble
/home/ac/ros2_humble/install/ros2cli/bin/ros2
/parameter_events
/rosout
```

Và `.profile` thường có đoạn source `.bashrc`:

```text
if [ -f "$HOME/.bashrc" ]; then
    . "$HOME/.bashrc"
fi
```

## Lỗi có thể gặp và cách sửa

| Lỗi | Nguyên nhân | Cách sửa |
|---|---|---|
| `bash -ic` vẫn không thấy ROS | `.bashrc` chưa có source line hoặc source fail | Quay STEP 13 |
| `.profile` không source `.bashrc` | Shell login không load `.bashrc` | Thêm source `.bashrc` vào `.profile` nếu cần |
| ROS daemon còn chạy sau test | `ros2 topic list` khởi động daemon | `ros2 daemon stop` là bình thường |

## Điều kiện chuyển bước

Chuyển STEP 15 nếu interactive shell tự thấy `ROS_DISTRO=humble` và `ros2` đúng path.

---

# STEP 15 — Cài và verify camera dependencies

## Mục tiêu

Cài các package cần cho Raspberry Pi Camera Module 3 Wide:

- `rpicam-apps`
- `python3-picamera2`
- `python3-opencv`

## Lệnh cần chạy

```bash
# STEP 15: verify/install camera dependencies

echo "===== STEP 15: VERIFY/INSTALL CAMERA DEPENDENCIES ====="

echo "===== INSTALL CAMERA DEPENDENCIES ====="
sudo apt install -y rpicam-apps python3-picamera2 python3-opencv

echo "===== VERIFY CAMERA COMMANDS ====="
which rpicam-still || true
which rpicam-hello || true
which libcamera-still || true

echo "===== VERIFY PYTHON CAMERA IMPORTS ====="
python3 - <<'PY'
from picamera2 import Picamera2
import cv2
print("picamera2 import: PASS")
print("opencv import: PASS")
print("opencv version:", cv2.__version__)
PY

echo "===== VERIFY CAMERA PACKAGES ====="
dpkg -l | grep -E 'rpicam-apps|python3-picamera2|python3-opencv|libcamera-tools|python3-libcamera' || true

echo "===== VERIFY ROS ENV STILL AVAILABLE ====="
echo "ROS_DISTRO=$ROS_DISTRO"
which ros2 || true

echo "===== RESIDUAL PROCESS CHECK ====="
ros2 daemon stop 2>/dev/null || true
sleep 1
pgrep -af 'ros2cli.daemon.daemonize|camera_node|rpicam|libcamera|micro_ros_agent|colcon build|vcs import|rosdep install' || true
```

## Kết quả mong muốn

Cần thấy:

```text
/usr/bin/rpicam-still
/usr/bin/rpicam-hello
picamera2 import: PASS
opencv import: PASS
opencv version: 4.6.0 hoặc tương đương
ROS_DISTRO=humble
```

## Lỗi có thể gặp và cách sửa

| Lỗi | Nguyên nhân | Cách sửa |
|---|---|---|
| `rpicam-still` không có | Thiếu Raspberry Pi camera apps repo/package | `sudo apt update`, kiểm tra Raspberry Pi repo |
| `from picamera2 import Picamera2` fail | Thiếu `python3-picamera2` | `sudo apt install python3-picamera2` |
| `import cv2` fail | Thiếu OpenCV Python | `sudo apt install python3-opencv` |
| ROS env mất | Terminal chưa source `.bashrc` | `source ~/.bashrc` |

## Điều kiện chuyển bước

Chuyển STEP 16 nếu camera commands và Python imports đều PASS.

---

# STEP 16 — Test Pi Camera bằng `rpicam-still`

## Mục tiêu

Xác nhận camera vật lý hoạt động bằng công cụ hệ thống trước khi đưa vào ROS 2.

## Lệnh cần chạy

```bash
# STEP 16: test Pi Camera with rpicam-still

echo "===== STEP 16: TEST PI CAMERA WITH RPICAM-STILL ====="

echo "===== CLEAN OLD TEST IMAGE ====="
rm -f ~/camera_test_step16.jpg

echo "===== CAMERA STILL CAPTURE ====="
rpicam-still -o ~/camera_test_step16.jpg --nopreview --timeout 1000

echo "===== VERIFY IMAGE FILE ====="
ls -lh ~/camera_test_step16.jpg

echo "===== VERIFY IMAGE READ WITH OPENCV ====="
python3 - <<'PY'
import cv2
path = "/home/ac/camera_test_step16.jpg"
img = cv2.imread(path)
if img is None:
    raise SystemExit("OpenCV image read: FAIL")
print("OpenCV image read: PASS")
print("Image shape:", img.shape)
PY

echo "===== CAMERA PROCESS CLEANUP CHECK ====="
pgrep -af 'rpicam|libcamera|camera_node' || true

echo "===== ROS RESIDUAL PROCESS CHECK ====="
ros2 daemon stop 2>/dev/null || true
sleep 1
pgrep -af 'ros2cli.daemon.daemonize|camera_node|micro_ros_agent|colcon build|vcs import|rosdep install' || true
```

> Nếu user của bạn không phải `ac`, sửa path `/home/ac/camera_test_step16.jpg` thành `/home/<user>/camera_test_step16.jpg`.

## Kết quả mong muốn

Ví dụ đã kiểm chứng:

```text
Still capture image received
-rw-r--r-- 1 ac ac 1.1M ... /home/ac/camera_test_step16.jpg
OpenCV image read: PASS
Image shape: (2592, 4608, 3)
```

Các warning sau có thể xuất hiện nhưng không chặn nếu ảnh vẫn capture được:

```text
No static properties available for 'imx708_wide'
No sensor delays found in static properties. Assuming unverified defaults.
```

## Lỗi có thể gặp và cách sửa

| Lỗi | Nguyên nhân | Cách sửa |
|---|---|---|
| `no cameras available` | Cáp camera lỏng/sai cổng/camera chưa nhận | Tắt Pi, kiểm tra cáp CSI, bật lại |
| File ảnh không tạo | Camera capture fail | Xem log `rpicam-still`, kiểm tra cáp/nguồn |
| OpenCV đọc ảnh fail | File hỏng hoặc path sai | Kiểm tra `ls -lh`, chạy lại capture |
| Camera process còn dư | App chưa thoát sạch | `pkill -f 'rpicam|libcamera'` |

## Điều kiện chuyển bước

Chuyển STEP 17 nếu ảnh được tạo, OpenCV đọc được và không còn process camera dư.

Runtime classification: `PASS_OPERATIONAL` cho camera still capture.

---

# STEP 17 — Tạo ROS 2 camera node file

## Mục tiêu

Tạo Python node publish ảnh camera lên topic:

```text
/camera/image_raw/compressed
```

Message type:

```text
sensor_msgs/msg/CompressedImage
```

## Lệnh cần chạy

```bash
# STEP 17: create ROS 2 camera node file

echo "===== STEP 17: CREATE ROS 2 CAMERA NODE FILE ====="

echo "===== CREATE ROS WORKSPACE FOLDER ====="
mkdir -p ~/ros2_ws

echo "===== WRITE CAMERA NODE ====="
cat > ~/ros2_ws/camera_node.py <<'PY'
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from picamera2 import Picamera2
import threading
import cv2


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        self.publisher = self.create_publisher(
            CompressedImage,
            '/camera/image_raw/compressed',
            10
        )

        self.picam = Picamera2()
        config = self.picam.create_video_configuration(
            main={"size": (640, 480), "format": "RGB888"},
            controls={"FrameRate": 30.0}
        )
        self.picam.configure(config)
        self.picam.start()

        self.frame = None
        self.lock = threading.Lock()

        self.capture_thread = threading.Thread(
            target=self.capture_loop,
            daemon=True
        )
        self.capture_thread.start()

        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)

        self.get_logger().info(
            'Camera node started. Publishing /camera/image_raw/compressed'
        )

    def capture_loop(self):
        while rclpy.ok():
            frame = self.picam.capture_array("main")
            with self.lock:
                self.frame = frame

    def timer_callback(self):
        with self.lock:
            if self.frame is None:
                return
            frame = self.frame.copy()

        ok, jpeg = cv2.imencode(
            '.jpg',
            cv2.cvtColor(frame, cv2.COLOR_RGB2BGR),
            [cv2.IMWRITE_JPEG_QUALITY, 85]
        )

        if not ok:
            self.get_logger().warning('JPEG encode failed')
            return

        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera'
        msg.format = 'jpeg'
        msg.data = jpeg.tobytes()
        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = CameraNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.picam.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
PY

echo "===== MAKE FILE EXECUTABLE ====="
chmod +x ~/ros2_ws/camera_node.py

echo "===== VERIFY FILE ====="
ls -lh ~/ros2_ws/camera_node.py
head -n 20 ~/ros2_ws/camera_node.py

echo "===== VERIFY PYTHON SYNTAX ====="
python3 -m py_compile ~/ros2_ws/camera_node.py

echo "===== VERIFY IMPORTS WITHOUT STARTING CAMERA NODE ====="
python3 - <<'PY'
import rclpy
from sensor_msgs.msg import CompressedImage
from picamera2 import Picamera2
import cv2
print("rclpy import: PASS")
print("CompressedImage import: PASS")
print("Picamera2 import: PASS")
print("cv2 import: PASS")
PY

echo "===== ROS ENV CHECK ====="
echo "ROS_DISTRO=$ROS_DISTRO"
which ros2 || true

echo "===== RESIDUAL PROCESS CHECK ====="
ros2 daemon stop 2>/dev/null || true
sleep 1
pgrep -af 'ros2cli.daemon.daemonize|camera_node|rpicam|libcamera|micro_ros_agent|colcon build|vcs import|rosdep install' || true
```

## Kết quả mong muốn

Cần thấy:

```text
-rwxr-xr-x ... /home/ac/ros2_ws/camera_node.py
rclpy import: PASS
CompressedImage import: PASS
Picamera2 import: PASS
cv2 import: PASS
ROS_DISTRO=humble
```

`python3 -m py_compile` không in gì là PASS.

## Lỗi có thể gặp và cách sửa

| Lỗi | Nguyên nhân | Cách sửa |
|---|---|---|
| Syntax error | Paste thiếu dòng hoặc mất dấu nháy | Xóa file và chạy lại block STEP 17 |
| `rclpy import` fail | ROS env chưa source hoặc build lỗi | `source ~/.bashrc`, nếu vẫn fail quay STEP 12 |
| `Picamera2 import` fail | Thiếu dependency | Quay STEP 15 |
| File không executable | `chmod` chưa chạy | `chmod +x ~/ros2_ws/camera_node.py` |

## Điều kiện chuyển bước

Chuyển STEP 18 nếu file tồn tại, compile không lỗi, import PASS và cleanup sạch.

---

# STEP 18 — Chạy ROS 2 camera node và verify topic/FPS

## Mục tiêu

Chạy camera node, kiểm tra:

- Topic `/camera/image_raw/compressed` xuất hiện.
- Message type đúng.
- FPS khoảng 29–30.
- Node cleanup sạch.

## Lệnh cần chạy

```bash
# STEP 18: run ROS 2 camera node and verify topic/FPS

echo "===== STEP 18: RUN ROS 2 CAMERA NODE AND VERIFY TOPIC/FPS ====="

echo "===== SAFETY STATE ====="
echo "ESC main power: OFF"
echo "Motor: not powered"
echo "Servo: not tested"
echo "No actuation path active"

echo "===== SOURCE ROS 2 ENVIRONMENT ====="
source ~/ros2_humble/install/setup.bash

echo "===== PRE-RUN CLEANUP ====="
pkill -f '/home/ac/ros2_ws/camera_node.py' 2>/dev/null || true
ros2 daemon stop 2>/dev/null || true
sleep 1

echo "===== START CAMERA NODE IN BACKGROUND ====="
rm -f ~/camera_node_step18.log
python3 ~/ros2_ws/camera_node.py > ~/camera_node_step18.log 2>&1 &
CAMERA_PID=$!
echo "CAMERA_PID=${CAMERA_PID}"

echo "===== WAIT FOR CAMERA NODE STARTUP ====="
sleep 8

echo "===== CAMERA NODE LOG HEAD ====="
head -n 80 ~/camera_node_step18.log || true

echo "===== ROS TOPIC LIST ====="
ros2 topic list

echo "===== ROS TOPIC INFO ====="
ros2 topic info /camera/image_raw/compressed || true

echo "===== ROS TOPIC FPS CHECK ====="
timeout 20s ros2 topic hz /camera/image_raw/compressed || true

echo "===== CAMERA NODE PROCESS CHECK ====="
ps -p ${CAMERA_PID} -o pid,cmd || true

echo "===== CLEANUP CAMERA NODE ====="
kill ${CAMERA_PID} 2>/dev/null || true
sleep 2
pkill -f '/home/ac/ros2_ws/camera_node.py' 2>/dev/null || true

echo "===== POST-CLEANUP CAMERA PROCESS CHECK ====="
pgrep -af 'camera_node|rpicam|libcamera' || true

echo "===== ROS DAEMON CLEANUP ====="
ros2 daemon stop 2>/dev/null || true
sleep 1

echo "===== FINAL RESIDUAL PROCESS CHECK ====="
pgrep -af 'ros2cli.daemon.daemonize|camera_node|rpicam|libcamera|micro_ros_agent|colcon build|vcs import|rosdep install' || true

echo "===== STEP 18 COMPLETE ====="
```

> Nếu user không phải `ac`, sửa `/home/ac/ros2_ws/camera_node.py` thành path đúng.

## Kết quả mong muốn

Ví dụ đã kiểm chứng:

```text
[INFO] [camera_node]: Camera node started. Publishing /camera/image_raw/compressed

/camera/image_raw/compressed
/parameter_events
/rosout

Type: sensor_msgs/msg/CompressedImage
Publisher count: 1

average rate: 30.001
average rate: 29.965
average rate: 29.99...
```

Sau cleanup:

```text
The daemon has been stopped
FINAL RESIDUAL PROCESS CHECK
# không in process dư
```

## Lỗi có thể gặp và cách sửa

| Lỗi | Nguyên nhân | Cách sửa |
|---|---|---|
| Camera node không start | Camera đang bị process khác giữ | `pkill -f 'rpicam|libcamera|camera_node'`, chạy lại |
| Topic không xuất hiện | Node chưa start đủ lâu hoặc crash | Xem `~/camera_node_step18.log` |
| FPS thấp | CPU/nhiệt/storage/camera config | Kiểm tra temp, giảm workload, reboot nếu cần |
| `ros2 topic info` báo `Unknown topic` nhưng `ros2 topic hz` đo được | ROS discovery/cache tạm thời | Không chặn nếu topic list có topic và hz đo được |
| `failed to initialize wait set...` sau `timeout` | `timeout` ngắt `ros2 topic hz` | Không chặn nếu FPS đã được đo và cleanup sạch |
| `[1]+ Exit 1 python3 ...` sau kill | Do chủ động kill node | Không chặn nếu final residual process sạch |

## Điều kiện PASS

Chốt `PASS_OPERATIONAL` nếu:

```text
Camera node started
Topic /camera/image_raw/compressed xuất hiện
FPS khoảng 29–30
Cleanup sạch
Final residual process check sạch
```

---

# STEP 19 — Lưu final evidence snapshot

## Mục tiêu

Lưu trạng thái hệ thống, ROS, camera dependency, camera node và runtime evidence vào file log.

## Lệnh cần chạy

```bash
# STEP 19: final ROS 2 camera setup evidence snapshot

echo "===== STEP 19: FINAL ROS 2 CAMERA SETUP EVIDENCE SNAPSHOT =====" | tee ~/setup_evidence_step19.log

{
echo "===== SYSTEM ====="
hostname
whoami
cat /etc/os-release
uname -a
dpkg --print-architecture
python3 --version

echo "===== ROS ENV ====="
echo "ROS_DISTRO=$ROS_DISTRO"
echo "ROS_VERSION=$ROS_VERSION"
echo "ROS_PYTHON_VERSION=$ROS_PYTHON_VERSION"
which ros2 || true
ros2 topic list 2>&1 | head -n 20 || true

echo "===== ROS CORE IMPORT CHECK ====="
python3 - <<'PY'
import rclpy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
print("rclpy import: PASS")
print("CompressedImage import: PASS")
print("String import: PASS")
PY

echo "===== CAMERA DEPENDENCY CHECK ====="
which rpicam-still || true
which rpicam-hello || true
python3 - <<'PY'
from picamera2 import Picamera2
import cv2
print("picamera2 import: PASS")
print("opencv import: PASS")
print("opencv version:", cv2.__version__)
PY

echo "===== CAMERA NODE FILE CHECK ====="
ls -lh ~/ros2_ws/camera_node.py
python3 -m py_compile ~/ros2_ws/camera_node.py && echo "camera_node syntax: PASS"

echo "===== STEP 18 CAMERA TOPIC EVIDENCE LOG ====="
grep -E 'Camera node started|average rate|/camera/image_raw/compressed|Type:|Publisher count' ~/camera_node_step18.log ~/camera_runtime_evidence_step19b.log 2>/dev/null || true

echo "===== RESOURCE CHECK ====="
free -h
df -h /
vcgencmd measure_temp 2>/dev/null || true

echo "===== RESIDUAL PROCESS CHECK ====="
ros2 daemon stop 2>/dev/null || true
sleep 1
pgrep -af 'ros2cli.daemon.daemonize|camera_node|rpicam|libcamera|micro_ros_agent|colcon build|vcs import|rosdep install' || true

echo "===== SNAPSHOT COMPLETE ====="
} 2>&1 | tee -a ~/setup_evidence_step19.log

echo "Evidence log saved at:"
ls -lh ~/setup_evidence_step19.log
```

## Kết quả mong muốn

Cần thấy:

```text
rclpy import: PASS
CompressedImage import: PASS
String import: PASS
picamera2 import: PASS
opencv import: PASS
camera_node syntax: PASS
The daemon has been stopped
Evidence log saved at:
-rw-r--r-- ... /home/ac/setup_evidence_step19.log
```

## Bổ sung evidence FPS nếu muốn

Nếu file STEP 19 chưa có FPS, chạy thêm block dưới đây để lưu riêng FPS evidence:

```bash
# Optional: save camera runtime FPS evidence to a file

echo "===== OPTIONAL: SAVE CAMERA TOPIC FPS EVIDENCE ====="

source ~/ros2_humble/install/setup.bash

pkill -f '/home/ac/ros2_ws/camera_node.py' 2>/dev/null || true
ros2 daemon stop 2>/dev/null || true
sleep 1

rm -f ~/camera_runtime_evidence_step19b.log
python3 ~/ros2_ws/camera_node.py > ~/camera_node_step19b.log 2>&1 &
CAMERA_PID=$!
echo "CAMERA_PID=${CAMERA_PID}" | tee ~/camera_runtime_evidence_step19b.log

sleep 8

{
echo "===== CAMERA NODE LOG HEAD ====="
head -n 80 ~/camera_node_step19b.log || true

echo "===== ROS TOPIC LIST ====="
ros2 topic list

echo "===== ROS TOPIC INFO ====="
ros2 topic info /camera/image_raw/compressed || true

echo "===== ROS TOPIC FPS CHECK ====="
timeout 20s ros2 topic hz /camera/image_raw/compressed || true

echo "===== CAMERA NODE PROCESS CHECK ====="
ps -p ${CAMERA_PID} -o pid,cmd || true
} 2>&1 | tee -a ~/camera_runtime_evidence_step19b.log

kill ${CAMERA_PID} 2>/dev/null || true
sleep 2
pkill -f '/home/ac/ros2_ws/camera_node.py' 2>/dev/null || true
ros2 daemon stop 2>/dev/null || true
sleep 1

{
echo "===== FINAL RESIDUAL PROCESS CHECK ====="
pgrep -af 'ros2cli.daemon.daemonize|camera_node|rpicam|libcamera|micro_ros_agent|colcon build|vcs import|rosdep install' || true

echo "===== EVIDENCE FILE ====="
ls -lh ~/camera_runtime_evidence_step19b.log
} 2>&1 | tee -a ~/camera_runtime_evidence_step19b.log
```

## Lỗi có thể gặp và cách sửa

| Lỗi | Nguyên nhân | Cách sửa |
|---|---|---|
| Evidence thiếu FPS | `ros2 topic hz` không được ghi vào file | Chạy optional FPS evidence block |
| Topic type lookup không ổn định | ROS CLI discovery/cache | Không chặn nếu STEP 18 đã có topic info và FPS |
| Final residual có process camera | Node chưa kill sạch | `pkill -f '/home/ac/ros2_ws/camera_node.py'` và kiểm tra lại |

## Điều kiện hoàn tất

Chốt final nếu:

```text
setup_evidence_step19.log tồn tại
camera_runtime_evidence_step19b.log tồn tại nếu cần FPS log riêng
Final residual process check sạch
```

Runtime classification cuối:

```text
PASS_OPERATIONAL
```

---

# 4. Bảng lỗi tổng hợp và cách khắc phục

## 4.1. APT và package source

| Triệu chứng | Khả năng cao | Cách xử lý |
|---|---|---|
| `Unable to locate package ros-dev-tools` | Chưa có ROS 2 apt repo | Làm STEP 2 |
| `NO_PUBKEY` | Keyring ROS lỗi | Tải lại `/usr/share/keyrings/ros-archive-keyring.gpg` |
| apt update chậm/fail | Network/DNS | Kiểm tra `ping`, `getent hosts`, đổi mạng |
| `159 not upgraded` hoặc nhiều package not upgraded | Bình thường nếu không cần upgrade | Không cần `apt upgrade` trong guide này |

## 4.2. Network/GitHub

| Triệu chứng | Khả năng cao | Cách xử lý |
|---|---|---|
| `Could not resolve host: github.com` | DNS fail tạm thời | Kiểm tra `/etc/resolv.conf`, `ping`, reboot network |
| `vcs import` dừng giữa chừng | Mạng yếu | Chạy lại `vcs import --input ros2.repos src` |
| `curl -I github.com` fail | HTTPS/network blocked | Thử mạng khác hoặc DNS khác |

## 4.3. ROS 2 build

| Triệu chứng | Khả năng cao | Cách xử lý |
|---|---|---|
| `shared_queues_vendor` fail download | DNS/GitHub fail | Fix network, xóa package build/install, build lại |
| `rosbag2_cpp` đòi `mcap_vendor` | Skip MCAP chưa skip downstream | Skip cả nhóm rosbag2 |
| `rqt_bag` fail | Phụ thuộc rosbag2 | Skip `rqt_bag` |
| `launch_testing_examples` fail | Phụ thuộc rosbag2/test stack | Skip `launch_testing_examples` |
| Build quá nóng | Pi throttling | Fan/tản nhiệt, giảm workers |
| Build fail nhưng core packages có đủ | Optional packages fail | Verify STEP 11/12; nếu core runtime PASS thì camera route vẫn đi tiếp được |

## 4.4. ROS runtime

| Triệu chứng | Khả năng cao | Cách xử lý |
|---|---|---|
| `ros2: command not found` | Chưa source setup | `source ~/ros2_humble/install/setup.bash` |
| Terminal mới không thấy ROS | `.bashrc` chưa source | Làm STEP 13/14 |
| `bash -lc` không thấy ROS nhưng `bash -ic` thấy | Non-interactive shell behavior | Không chặn, dùng interactive verify |
| `rclpy import` fail | Build/source lỗi | Source lại; nếu không được quay STEP 10/12 |

## 4.5. Camera

| Triệu chứng | Khả năng cao | Cách xử lý |
|---|---|---|
| `no cameras available` | Cáp camera/cổng/camera detection | Tắt Pi, kiểm tra cáp, bật lại |
| `imx708_wide` static properties warning | Camera DB chưa có đầy đủ static properties | Không chặn nếu capture OK |
| `picamera2 import` fail | Thiếu package | `sudo apt install python3-picamera2` |
| `cv2 import` fail | Thiếu OpenCV | `sudo apt install python3-opencv` |
| ROS camera node không start | Camera bị process khác giữ | `pkill -f 'rpicam|libcamera|camera_node'` |
| FPS thấp | CPU/nhiệt/config | Kiểm tra temp, giảm resolution/FPS nếu cần |

## 4.6. ROS topic và CLI

| Triệu chứng | Khả năng cao | Cách xử lý |
|---|---|---|
| `ros2 topic info` báo `Unknown topic` nhưng `topic hz` đo được | Discovery/cache CLI tạm thời | Không chặn nếu topic list có topic và FPS OK |
| `failed to initialize wait set` sau `timeout` | `timeout` ngắt `ros2 topic hz` | Không chặn nếu đã đo FPS |
| `[1]+ Exit 1` sau `kill` camera node | Node bị kill chủ động | Không chặn nếu cleanup sạch |
| Residual process còn `camera_node` | Cleanup chưa sạch | `pkill -f '/home/ac/ros2_ws/camera_node.py'` |

---

# 5. Checklist hoàn tất

Dùng checklist này để xác nhận trước khi chuyển sang RPLIDAR/BNO055/micro-ROS.

```text
[ ] STEP 0: OS/network/resource/safety baseline PASS
[ ] STEP 2: ROS apt repository configured
[ ] STEP 3: colcon/vcs/rosdep installed
[ ] STEP 5: ROS 2 Humble source imported
[ ] STEP 7: rosdep install successful
[ ] STEP 9: swap set to about 2GB
[ ] STEP 10: ROS 2 source build completed enough for core route
[ ] STEP 11: core packages exist in install space
[ ] STEP 12: ROS 2 core runtime PASS
[ ] STEP 13/14: terminal auto-source ROS env PASS
[ ] STEP 15: camera dependencies PASS
[ ] STEP 16: rpicam-still capture PASS
[ ] STEP 17: camera_node.py created and syntax/import PASS
[ ] STEP 18: /camera/image_raw/compressed topic PASS, FPS about 29–30
[ ] STEP 19: evidence snapshot saved
[ ] Cleanup: no residual ros2/camera/build process
[ ] Hardware safety: ESC main power OFF, motor not powered, servo not tested
```

---

# 6. Final evidence statement mẫu

Có thể ghi vào project log như sau:

```text
Raspberry Pi 5 running Debian 12 Bookworm was configured with ROS 2 Humble core built from source under Python 3.11. The ROS 2 core runtime was verified with rclpy, sensor_msgs, std_msgs imports and baseline ROS graph topics. Pi Camera Module 3 Wide was verified with rpicam-still and then integrated through a Python ROS 2 camera node publishing sensor_msgs/msg/CompressedImage on /camera/image_raw/compressed. Runtime validation showed stable camera publishing at approximately 29.99–30.00 FPS. Cleanup and residual-process checks passed. Classification: PASS_OPERATIONAL for ROS 2 core + Pi Camera compressed image topic.
```

Giới hạn cần đi kèm:

```text
The full ROS 2 source build is not claimed as clean-complete because rosbag2/rqt_bag/test/example packages were skipped or failed during the camera-focused bring-up route. This limitation does not block the validated camera topic path.
```

---

# 7. Bước tiếp theo sau guidebook này

Thứ tự bring-up an toàn đề xuất sau khi camera PASS:

1. **RPLIDAR A1M8 static bring-up**: chỉ verify `/scan`, không actuation.
2. **BNO055 IMU bring-up**: verify orientation/imu topic, mount orientation, magnetic disturbance.
3. **Sensor timestamp/frame convention**: thống nhất `camera`, `laser`, `imu_link`, `base_link`.
4. **Nucleo/micro-ROS transport**: chỉ heartbeat/status trước, chưa điều khiển servo/ESC.
5. **Static actuator safety validation**: neutral steering, ESC neutral, stale timeout, clamps.
6. **Rolling validation**: chỉ sau khi static safety PASS.

Không chuyển sang rolling/dynamic test khi chưa có static safety evidence.

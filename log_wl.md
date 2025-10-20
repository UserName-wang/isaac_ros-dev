容器内运行安装命令遇到无法访问代理的问题：
sudo apt-get -o Acquire::http::proxy="http://192.168.0.57:12346" \
             -o Acquire::https::proxy="http://192.168.0.57:12346" \
             install -y ros-humble-isaac-ros-visual-slam ros-humble-isaac-ros-examples

sudo apt-get -o Acquire::http::proxy="http://192.168.0.57:12346" \
             -o Acquire::https::proxy="http://192.168.0.57:12346" \
             install -y ros-humble-isaac-ros-realsense



             sudo apt-get install -y ros-humble-isaac-ros-examples

ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam_hawk.launch.py save_map_folder_path:=/workspaces/isaac_ros-dev/map/
ros2 launch isaac_ros_examples isaac_ros_examples.launch.py launch_fragments:=visual_slam \
interface_specs_file:=${ISAAC_ROS_WS}/isaac_ros_assets/isaac_ros_visual_slam/quickstart_interface_specs.json \
rectified_images:=false \
save_map_folder_path:=/workspaces/isaac_ros-dev/map/

cd ${ISAAC_ROS_WS}/src/isaac_ros_common && \
./scripts/run_dev.sh -d ${ISAAC_ROS_WS} -a "--build-arg http_proxy=http://192.168.0.57:12346" -a "--build-arg https_proxy=http://192.168.0.57:12346"

wget https://github.com/triton-inference-server/server/releases/download/v2.49.0/tritonserver2.49.0-igpu.tar.gz 

cd /home/orin/study/ros/isaac_ros/isaac_ros_ws/src/isaac_ros_common/scripts
./run_dev.sh --build_arg "http_proxy=http://192.168.0.57:12346" --build_arg "https_proxy=http://192.168.0.57:12346" --build_arg "HTTP_PROXY=http://192.168.0.57:12346" --build_arg "HTTPS_PROXY=http://192.168.0.57:12346"


sudo apt-get -o Acquire::http::proxy="http://192.168.0.57:12346" update
sudo apt-get -o Acquire::http::proxy="http://192.168.0.57:12346" install -y ros-humble-isaac-common


admin@ubuntu:/workspaces/isaac_ros-dev$ ^C
admin@ubuntu:/workspaces/isaac_ros-dev$ cd ${ISAAC_ROS_WS} && rm -rf build/isaac_ros_visual_slam_interfaces install/isaac_ros_visual_slam_interfaces build/isaac_ros_visual_slam install/isaac_ros_visual_slam
admin@ubuntu:/workspaces/isaac_ros-dev$ 

ros2 bag play -l ${ISAAC_ROS_WS}/isaac_ros_assets/isaac_ros_visual_slam/quickstart_bag --remap \
/front_stereo_camera/left/image_raw:=/left/image_rect \
/front_stereo_camera/left/camera_info:=/left/camera_info_rect \
/front_stereo_camera/right/image_raw:=/right/image_rect \
/front_stereo_camera/right/camera_info:=/right/camera_info_rect \
/back_stereo_camera/left/image_raw:=/rear_left/image_rect \
/back_stereo_camera/left/camera_info:=/rear_left/camera_info_rect \
/back_stereo_camera/right/image_raw:=/rear_right/image_rect \
/back_stereo_camera/right/camera_info:=/rear_right/camera_info_rect

sudo apt-get -o Acquire::http::proxy="http://192.168.0.57:12346" \
             -o Acquire::https::proxy="http://192.168.0.57:12346" \
             install -y --fix-missing ros-humble-isaac-ros-examples ros-humble-isaac-ros-apriltag-interfaces


sudo apt-get -o Acquire::http::proxy="http://192.168.0.57:12346" \
             -o Acquire::https::proxy="http://192.168.0.57:12346" update

export http_proxy=http://192.168.0.57:12346
export https_proxy=http://192.168.0.57:12346
export HTTP_PROXY=http://192.168.0.57:12346
export HTTPS_PROXY=http://192.168.0.57:12346

rosdep update && rosdep install -i -r --from-paths ${ISAAC_ROS_WS}/src/isaac_ros_nvblox/ --rosdistro humble -y \
--rosdep-install-options="-o Acquire::http::proxy=http://192.168.0.57:12346 -o Acquire::https::proxy=http://192.168.0.57:12346"

rosdep update && rosdep install -i -r --from-paths ${ISAAC_ROS_WS}/src/isaac_ros_nvblox/ --rosdistro humble -y

rosdep update && ROSDEP_APT_PROXY=http://192.168.0.57:12346 rosdep install -i -r --from-paths ${ISAAC_ROS_WS}/src/isaac_ros_nvblox/ --rosdistro humble -y

ros2 bag play -l ${ISAAC_ROS_WS}/isaac_ros_assets/isaac_ros_apriltag/quickstart.bag --remap image:=image_rect camera_info:=camera_info_rect

cd ${ISAAC_ROS_WS}/ && \
   colcon build --symlink-install --packages-up-to isaac_ros_apriltag --base-paths ${ISAAC_ROS_WS}/src/isaac_ros_apriltag/isaac_ros_apriltag


ros2 bag play -l ${ISAAC_ROS_WS}/rosbag2_2025_07_27-20_31_30 \
navigation:=False

docker commit -m "ISAAC_ROS with nvblox support" 5150a1dfef3e isaac_ros_dev-aarch64:5.0
export LIBREALSENSE_SOURCE_VERSION=v2.55.1
export REALSENSE_ROS_GIT_URL=https://github.com/NVIDIA-ISAAC-ROS/realsense-ros.git
export REALSENSE_ROS_VERSION=release/4.51.1-isaac

/etc/udev/rules.d/99-realsense-libusb.rules (your custom rules)
/lib/udev/rules.d/60-librealsense2-udev-rules.rules (system package rules)


07:42:08 [Warn] /root/librealsense/common/notifications.cpp:511 - RealSense UDEV-Rules are missing!
UDEV-Rules permissions configuration 
 for RealSense devices.`
Missing/outdated UDEV-Rules will cause 'Permissions Denied' errors
unless the application is running under 'sudo' (not recommended)
In case of Debians use: 
sudo apt-get upgrade/install librealsense2-udev-rules
To manually install UDEV-Rules in terminal run:
$ sudo cp ~/.99-realsense-libusb.rules /etc/udev/rules.d/99-realsense-libusb.rules && sudo udevadm control --reload-rules && udevadm trigger

# Copy to the correct location
sudo cp /workspaces/isaac_ros-dev/.devcontainer/udev_rules/99-realsense-libusb-custom.rules /etc/udev/rules.d/99-realsense-libusb.rules
sudo cp /workspaces/isaac_ros-dev/.devcontainer/udev_rules/99-realsense-libusb-custom.rules /etc/udev/rules.d/rules.d/99-realsense-libusb.rules

play rosbag command:
ros2 launch nvblox_examples_bringup realsense_example.launch.py \
rosbag:=rosbag2_2025_07_27-20_31_30

try to save map:
ros2 service call /nvblox_node/save_map nvblox_msgs/srv/FilePath "{file_path: '/workspaces/isaac_ros-dev/isaac_ros_ws/map/nvblox_map'}"

docker save isaac_ros_dev-aarch64:5.0 | gzip > isaac_ros_dev-aarch64_5.0.tar.gz


cd /workspaces/isaac_ros-dev
git clone https://github.com/AprilRobotics/apriltag-generation.git apriltag
cd apriltag
mkdir build
cd build
cmake ..
make -j$(nproc)

cd /workspaces/isaac_ros-dev/apriltag
mkdir -p build
cd build
cmake ..
make -j$(nproc)


08:44:30 [Info]  - librealsense version: 2.55.1

08:44:30 [Warn] /root/librealsense/common/notifications.cpp:511 - RealSense UDEV-Rules file:
 /etc/udev/rules.d/99-realsense-libusb.rules
 is not up-to date! Version 1.1 can be applied
UDEV-Rules permissions configuration 
 for RealSense devices.`
Missing/outdated UDEV-Rules will cause 'Permissions Denied' errors
unless the application is running under 'sudo' (not recommended)
In case of Debians use: 
sudo apt-get upgrade/install librealsense2-udev-rules
To manually install UDEV-Rules in terminal run:
$ sudo cp ~/.99-realsense-libusb.rules /etc/udev/rules.d/99-realsense-libusb.rules && sudo udevadm control --reload-rules && udevadm trigger

# Install PAL SDK
sudo wget -qO - https://dreamvu.github.io/ppa/KEY.gpg | sudo apt-key add -
sudo wget -qO /etc/apt/sources.list.d/dreamvu.list https://dreamvu.github.io/ppa/dreamvu.list
sudo apt-get update && sudo apt-get upgrade -y && sudo apt-get dist-upgrade -y
sudo apt install ppa-pal -y
sudo apt install pal -y
sudo apt install pal-melodic-navigation -y
sudo apt install ros-melodic-ddynamic-reconfigure -y


# Install RealSense SDK
sudo apt-get install libssl-dev libusb-1.0-0-dev libudev-dev pkg-config libgtk-3-dev -y
sudo apt-get install git wget cmake build-essential -y
sudo apt-get install libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev at -y
sudo mkdir -p /etc/apt/keyrings
curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null
echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | \
sudo tee /etc/apt/sources.list.d/librealsense.list
sudo apt-get update
sudo apt-get install librealsense2-dkms librealsense2-utils librealsense2-dev librealsense2-dbg -y


# Install Velodyne ROS dependencies
sudo apt-get install ros-melodic-velodyne -y


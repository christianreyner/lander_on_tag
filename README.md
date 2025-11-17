# Lander Setup

Works on:
- Raspberry Pi 5 (8GB)
- Raspberry Pi OS 12 (Bookworm)

## Dependencies
```bash
sudo apt update
sudo apt install -y libusb-1.0-0-dev libopencv-dev python3-pip
# MAVProxy (required)
pip3 install --user MAVProxy
```

## Installation
1) DepthAI installation (v2.x)
```bash
git clone https://github.com/luxonis/depthai-core
cd depthai-core
git checkout v2.29.0
git submodule update --init --recursive
mkdir build && cd build
sudo cmake ..
sudo cmake --build . --target install
```
2) MAVLink (inside lander)
```bash
cd ../lander
git clone https://github.com/mavlink/c_library_v2.git third_party/mavlink
```
3) Install MAVProxy (if not already installed)
```bash
pip3 install --user MAVProxy
```
4) Build
```bash
From the lander folder:

mkdir -p build
cd build
cmake -Ddepthai_DIR=../depthai-core/build/install/lib/cmake/depthai ..
make -j$(nproc)
```
5) Make launcher executable
```bash
sudo chmod +x run_lander.sh
```
6) Run
```bash
./run_lander.sh
```

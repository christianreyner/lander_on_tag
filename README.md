Work on Raspberry Pi 5 8GB -- Raspberry OS: 12 Bookworm

sudo apt install libusb-1.0-0-dev
sudo apt install libopencv-dev

1. Depth AI installation (ver 2.29.0)
git clone https://github.com/luxonis/depthai-core
cd depthai-core
git checkout v2.25.0
git submodule update --init --recursive
mkdir build
cd build
sudo cmake ..
sudo cmake --build . --target install

2. Mavlink (inside lander)
cd ../lander
git clone https://github.com/mavlink/c_library_v2.git /third_party/mavlink

3. Install also the mavproxy!

4. Build
Navigate to lander folder
mkdir -p build
cd build
cmake -Ddepthai_DIR=../depthai-core/build/install/lib/cmake/depthai ..
make -j$(nproc)

5. sudo chmod +x run_lander.sh

6. ./run_lander.sh

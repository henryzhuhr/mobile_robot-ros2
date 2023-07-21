BASE_DIR=$(pwd)/ros
cd $BASE_DIR
tar xf jetson-utils.tar.bz2
cd jetson-utils
mkdir build
cd build

# sudo apt update 
# sudo apt install -y --no-install-recommends \
#     dialog \
#     libglew-dev \
#     glew-utils \
#     gstreamer1.0-libav \
#     gstreamer1.0-nice \
#     libgstreamer1.0-dev \
#     libgstreamer-plugins-base1.0-dev \
#     libgstreamer-plugins-good1.0-dev \
#     libgstreamer-plugins-bad1.0-dev \
#     libgstrtspserver-1.0-dev \
#     libsoup2.4-dev \
#     libjson-glib-dev \
#     qtbase5-dev
# sudo apt autoremove -y

# rm CMakeCache.txt

cmake .. \
    -DCMAKE_INSTALL_PREFIX="$HOME/program/jetson-utils"

make -j$(nproc)
make install

# python -c 'import site; print(site.getsitepackages()[0])'

cd $BASE_DIR
rm -rf jetson-utils
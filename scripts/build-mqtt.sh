PWD=`pwd`

PACKAGE_NAME=paho-mqtt-c
SOURCE_DIR=$PWD/downloads
INSTALL_DIR=$PWD/libs


cd $SOURCE_DIR
if [ ! -d $PACKAGE_NAME ]; then
  git clone git@github.com:eclipse/paho.mqtt.c.git $PACKAGE_NAME
  cd $PACKAGE_NAME
  git checkout v1.3.8
  cd ..
fi

rm -rf $PACKAGE_NAME-build
mkdir $PACKAGE_NAME-build
cd $PACKAGE_NAME-build
cmake ../$PACKAGE_NAME \
    -DCMAKE_INSTALL_PREFIX=$INSTALL_DIR/paho-mqtt \
    -DPAHO_WITH_SSL=TRUE \
    -DPAHO_BUILD_SHARED=FALSE -DPAHO_BUILD_STATIC=TRUE \
    -DPAHO_HIGH_PERFORMANCE=TRUE \
    -DPAHO_WITH_SSL=TRUE \
    -DPAHO_BUILD_DOCUMENTATION=FALSE -DPAHO_BUILD_SAMPLES=FALSE -DPAHO_ENABLE_TESTING=FALSE
make
make install

export CMAKE_PREFIX_PATH=$INSTALL_DIR/paho-mqtt:$CMAKE_PREFIX_PATH

PACKAGE_NAME=paho-mqtt-cpp

cd $SOURCE_DIR
if [ ! -d $PACKAGE_NAME ]; then
  git clone git@github.com:eclipse/paho.mqtt.cpp.git $PACKAGE_NAME
fi

rm -rf $PACKAGE_NAME-build
mkdir $PACKAGE_NAME-build
cd $PACKAGE_NAME-build
cmake ../$PACKAGE_NAME \
    -DCMAKE_INSTALL_PREFIX=$INSTALL_DIR/paho-mqtt \
    -DPAHO_BUILD_SHARED=TRUE -DPAHO_BUILD_STATIC=TRUE \
    -DPAHO_BUILD_DOCUMENTATION=FALSE -DPAHO_BUILD_SAMPLES=FALSE -DPAHO_BUILD_TESTS=FALSE
make
make install


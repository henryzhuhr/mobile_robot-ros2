PWD=`pwd`

PACKAGE_NAME=mqttclient
SOURCE_DIR=$PWD/downloads
INSTALL_DIR=$PWD/libs


cd $SOURCE_DIR
if [ ! -d $PACKAGE_NAME ]; then
  git clone git@github.com:jiejieTop/mqttclient.git
fi

mkdir $PACKAGE_NAME-build
cd $PACKAGE_NAME-build
cmake ../$PACKAGE_NAME \
    -DCMAKE_INSTALL_PREFIX=$INSTALL_DIR/$PACKAGE_NAME \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_EXAMPLES=OFF \
    -DBUILD_SHARED_LIBS=ON
make
make install

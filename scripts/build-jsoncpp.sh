PWD=`pwd`
INSTALL_DIR=$PWD/libs


cd downloads
if [ ! -d jsoncpp ]; then
  git clone git@github.com:open-source-parsers/jsoncpp.git
fi

mkdir jsoncpp-build
cd jsoncpp-build
cmake ../jsoncpp \
    -DCMAKE_INSTALL_PREFIX=$INSTALL_DIR/jsoncpp \
    -DBUILD_STATIC_LIBS=ON \
    -DBUILD_SHARED_LIBS=ON
make
make install
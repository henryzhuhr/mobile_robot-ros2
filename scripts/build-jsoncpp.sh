PWD=`pwd`

PACKAGE_NAME=jsoncpp
SOURCE_DIR=$PWD/downloads
INSTALL_DIR=$PWD/libs

if [ ! -d $SOURCE_DIR ]; then
  mkdir -p $SOURCE_DIR
fi

cd $SOURCE_DIR

if [ ! -d $PACKAGE_NAME ]; then
  git clone git@github.com:open-source-parsers/jsoncpp.git $PACKAGE_NAME  
fi

mkdir $PACKAGE_NAME-build
cd $PACKAGE_NAME-build
cmake ../$PACKAGE_NAME \
    -DCMAKE_INSTALL_PREFIX=$INSTALL_DIR/$PACKAGE_NAME \
    -DBUILD_STATIC_LIBS=ON \
    -DBUILD_SHARED_LIBS=ON
make
make install
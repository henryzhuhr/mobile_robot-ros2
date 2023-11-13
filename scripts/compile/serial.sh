PWD=`pwd`

PACKAGE_NAME=serial
SOURCE_DIR=$PWD/downloads
INSTALL_DIR=$PWD/libs

source ~/.bashrc
source .env/ros2/bin/activate

if [ ! -d $SOURCE_DIR ]; then
    mkdir -p $SOURCE_DIR
fi

cd $SOURCE_DIR
if [ ! -d $PACKAGE_NAME ]; then
    # https://github.com/wjwwood/serial/issues/288
    git clone git@github.com:HenryZhuHR/serial-ros2.git $PACKAGE_NAME
fi


 

# if [ -d $PACKAGE_NAME-build ]; then
#     rm -rf $PACKAGE_NAME-build
# fi
# mkdir -p $PACKAGE_NAME-build
# cd $PACKAGE_NAME-build

# cmake ../$PACKAGE_NAME \
#     -DCMAKE_INSTALL_PREFIX=$INSTALL_DIR/$PACKAGE_NAME

# make
# make install
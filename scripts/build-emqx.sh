PWD=`pwd`

PACKAGE_NAME=emqx
SOURCE_DIR=$PWD/downloads
INSTALL_DIR=$PWD/libs

cd $SOURCE_DIR
if [ ! -d $PACKAGE_NAME ]; then
    # https://www.emqx.io/docs/en/v5.1/deploy/install-ubuntu.html#install-with-deb
    wget https://www.emqx.com/en/downloads/broker/5.1.3/emqx-5.1.3-ubuntu22.04-arm64.tar.gz
    mkdir -p $PACKAGE_NAME
    tar -zxvf emqx-5.1.3-ubuntu22.04-arm64.tar.gz -C $PACKAGE_NAME
fi

./emqx/bin/emqx start
# ./emqx/bin/emqx stop



# http://localhost:18083/
# http://192.168.64.22:18083/
# admin
# public

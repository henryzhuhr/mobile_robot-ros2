source scripts/env/constant.sh
if [ ! -d $ENV_DIR ];then
    python3 -m venv $ENV_DIR
fi
source $ENV_DIR/bin/activate
python3 -m pip install --upgrade pip
python3 -m pip install -r requirements.txt

python3 -m pip install pyserial
python3 -m pip install setuptools # ==58.2.0

if [ ! -z $TENSORRT_HOME ];then
    python3 -m pip install  $TENSORRT_HOME/python/tensorrt-8.2.1.8-cp38-none-linux_x86_64.whl
fi



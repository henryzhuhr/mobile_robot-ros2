source scripts/constant.sh

USER_DEVICE=" "
# 查找存在的 /dev/ttyUSB* 并挂载
ls /dev/ttyUSB* > /tmp/ttyUSB
if [ -s /tmp/ttyUSB ]; then
    echo "ttyUSB exist"
    while read line
    do
        echo "${LGREEN}Find Serial port${DEFAULT}: $line"
        USER_DEVICE="$USER_DEVICE --device=$line "
    done < /tmp/ttyUSB
fi

echo $USER_DEVICE
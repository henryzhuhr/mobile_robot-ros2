

class SystemState:
    class services:
        update_state="__server__update_system_state"

    class topics:
        system_state = "system_state"
        set_speed = "topic__set_speed"

    GROUP_NUM_MAX= 255
    ID_NUM_MAX=255

    class StateGroup:
        NONE=0
        TASK=1
        CONTROLLER=2
        SENSOR=3
        VISION=4
        

    class StateID:
        class Task:
            IDLE=0
            JOY_CONTROL=1

        class Sensor:
            IDLE=0
            JOY=1
            USB_RGB_CAMERA=2
            SCI_RGB_CAMERA=2
        class Controller:
            NONE=0
            MECANUM_WHEEL_CAR=1

        class Vison:
            IDLE=0
            JOY=1
            OBJECT_DETECTION = 1
            LANE_DETECTION = 4
    
    class State:
        NONE = 0
        IDLE=1
        RUNNING=2
        PAUSE=3
        ERROR=4
        REJECT=5
        
    class ErrorCode:
        NO_ERROR = 0
        STATE_UPDATE_GROUP_OVERFLOW=1
        STATE_UPDATE_GROUP_NOT_FOUND=2
        STATE_UPDATE_ID_OVERFLOW=3
        STATE_UPDATE_ID_NOT_FOUND=4

        FILE_NOT_FOUND=3

    class Color:
        DEFAULT="\033[0m"
        GREEN="\033[0;32m"
        LGREEN="\033[1;32m"
        CYAN="\033[0;36m"
        LCYAN="\033[1;36m"
        RED="\033[0;31m"
        LRED="\033[1;31m"
        BLUE="\033[0;34m"
        LBLUE="\033[1;34m"
        YELLOW="\033[0;33m"
        LYELLOW="\033[1;33m"
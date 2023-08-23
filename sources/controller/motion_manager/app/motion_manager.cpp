#include "motion_manager/motion_manager.hpp"
#include "serial/serial.h"

int main()
{

    serial::Serial ser;

    try
    {
        ser.setPort("/dev/ttyUSB0");
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException &e)
    {
        printf("Unable to open port \n");
        return -1;
    }

    if (ser.isOpen())
    {
        printf("Serial Port initialized\n");
    }
    else
    {
        printf("Serial Port not open\n");
        return -1;
    }

    return 0;
}

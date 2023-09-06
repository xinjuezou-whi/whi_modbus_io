/******************************************************************
class to handle modbus IO

Features:
- modbus
- xxx

Dependencies:
- sudo apt install ros-<ros distro>-serial
- sudo usermod -a -G dialout <user name>, the reboot

Written by Xinjue Zou, xinjue.zou@outlook.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

******************************************************************/
#include "whi_modbus_io/modbus_io.h"

namespace whi_modbus_io
{
    ModbusIo::ModbusIo(std::shared_ptr<ros::NodeHandle>& NodeHandle)
        : node_handle_(NodeHandle)
    {
        init();
    }

    ModbusIo::~ModbusIo()
    {

    }

    void ModbusIo::init()
    {
        // params
        node_handle_->param("hardware_interface/module", module_, std::string());
        node_handle_->param("hardware_interface/port", serial_port_, std::string("/dev/ttyUSB0"));
        node_handle_->param("hardware_interface/baudrate", baudrate_, 9600);

        // serial
	    try
	    {
		    serial_inst_ = std::make_unique<serial::Serial>(serial_port_, baudrate_, serial::Timeout::simpleTimeout(500));
	    }
	    catch (serial::IOException& e)
	    {
		    ROS_FATAL_STREAM_NAMED("failed to open serial %s", serial_port_.c_str());
	    }
    }
} // namespace whi_modbus_io

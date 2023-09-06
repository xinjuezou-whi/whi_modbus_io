/******************************************************************
class to handle modbus IO

Features:
- modbus
- xxx

Dependencies:
- sudo apt install ros-<ros distro>-serial
- sudo usermod -a -G dialout <user name>, the reboot
- CRC++, https://github.com/d-bahr/CRCpp

Written by Xinjue Zou, xinjue.zou@outlook.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2023-09-06: Initial version
2022-xx-xx: xxx
******************************************************************/
#pragma once
#include "whi_interfaces/WhiIo.h"

#include <ros/ros.h>
#include <serial/serial.h>

#include <memory>

namespace whi_modbus_io
{
	class ModbusIo
	{
    public:
        ModbusIo() = delete;
        ModbusIo(std::shared_ptr<ros::NodeHandle>& NodeHandle);
        ~ModbusIo();

    protected:
        void init();
        bool onServiceIo(whi_interfaces::WhiIo::Request& Request,
            whi_interfaces::WhiIo::Response& Response);

    protected:
        std::shared_ptr<ros::NodeHandle> node_handle_{ nullptr };
        std::string module_;
	    std::string serial_port_;
	    int baudrate_{ 9600 };
        std::unique_ptr<serial::Serial> serial_inst_{ nullptr };
        std::unique_ptr<ros::ServiceServer> service_{ nullptr };
	};
} // namespace whi_modbus_io

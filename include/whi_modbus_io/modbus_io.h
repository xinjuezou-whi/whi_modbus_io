/******************************************************************
class to handle modbus IO

Features:
- modbus
- xxx

Written by Xinjue Zou, xinjue.zou@outlook.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2023-09-06: Initial version
2022-xx-xx: xxx
******************************************************************/
#pragma once
#include <ros/ros.h>

#include <memory>

namespace whi_modbus_io
{
	class ModbusIo
	{
    public:
        ModbusIo(std::shared_ptr<ros::NodeHandle>& NodeHandle);
        ~ModbusIo();

    protected:
        void init();

    protected:
        std::shared_ptr<ros::NodeHandle> node_handle_{ nullptr };
	};
} // namespace whi_modbus_io

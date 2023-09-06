/******************************************************************
class to handle modbus IO

Features:
- modbus
- xxx

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

    }
} // namespace whi_modbus_io

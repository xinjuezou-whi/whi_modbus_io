/******************************************************************
class to handle modbus IO

Features:
- modbus
- xxx

Dependencies:
- sudo apt install ros-<ros distro>-serial
- sudo usermod -a -G dialout <user name>, then reboot

Written by Xinjue Zou, xinjue.zou@outlook.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

******************************************************************/
#include "whi_modbus_io/modbus_io.h"

#include <yaml-cpp/yaml.h>

#include <thread>

namespace whi_modbus_io
{
    static uint16_t crc16(const uint8_t* Data, size_t Length)
    {
        uint16_t crc = 0xffff;
        uint16_t polynomial = 0xa001;

        for (size_t i = 0; i < Length; ++i)
        {
            crc ^= Data[i];
            for (int j = 0; j < 8; ++j)
            {
                if ((crc & 0x0001))
                {
                    crc = (crc >> 1) ^ polynomial;
                }
                else
                {
                    crc >>= 1;
                }
            }
        }

        return crc;
    }

    ModbusIo::ModbusIo(std::shared_ptr<ros::NodeHandle>& NodeHandle)
        : node_handle_(NodeHandle)
    {
        init();
    }

    ModbusIo::~ModbusIo()
    {
        // reset to init level
        for (const auto& it : init_levels_map_)
        {
            whi_interfaces::WhiSrvIo::Request request;
            request.reg = it.first;
            request.level = it.second;
            request.operation = whi_interfaces::WhiSrvIo::Request::OPER_WRITE;
            whi_interfaces::WhiSrvIo::Response response;
            onServiceIo(request, response);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(800));

	    if (serial_inst_)
	    {
		    serial_inst_->close();
	    }
    }

    void ModbusIo::init()
    {
        // params
        std::string service;
        node_handle_->param("service", service, std::string("io_request"));
        std::string levelConfig;
        if (node_handle_->param("init_levels", levelConfig, std::string("init_levels.yaml")))
        {
            readInitLevels(levelConfig);
        }
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
		    ROS_FATAL_STREAM("failed to open serial " << serial_port_);
	    }

        if (serial_inst_)
        {
            service_ = std::make_unique<ros::ServiceServer>(
                node_handle_->advertiseService(service, &ModbusIo::onServiceIo, this));
        }
    }

    bool ModbusIo::readInitLevels(const std::string& Config)
    {
        try
        {
            YAML::Node node = YAML::LoadFile(Config);
            const auto& levels = node["init_levels"];
            if (levels)
            {
                for (const auto& it : levels)
                {
                    init_levels_map_.emplace(std::make_pair(it.first.as<int>(), it.second.as<int>()));
                }
            }

            return true;
        }
        catch (const std::exception& e)
        {
            std::cout << "failed to load init levels config file " << Config << std::endl;
            return false;
        }
    }

    bool ModbusIo::onServiceIo(whi_interfaces::WhiSrvIo::Request& Request,
        whi_interfaces::WhiSrvIo::Response& Response)
    {
        std::array<uint8_t, 8> data;
        if (Request.operation == whi_interfaces::WhiSrvIo::Request::OPER_READ)
        {
            data[0] = 0x01;
            if (Request.reg < 17)
            {
                data[1] = 0x02;
            }
            else
            {
                data[1] = 0x01;
            }
            data[2] = 0;
            data[3] = Request.reg - 1;
            data[4] = 0;
            data[5] = 0x01;
            uint16_t crc = crc16(data.data(), data.size() - 2);
            data[6] = uint8_t(crc);
            data[7] = uint8_t(crc >> 8);
            serial_inst_->write(data.data(), data.size());
        }
        else if (Request.operation == whi_interfaces::WhiSrvIo::Request::OPER_WRITE)
        {
            data[0] = 0x01;
            data[1] = 0x05;
            data[2] = 0;
            data[3] = Request.reg - 1;
            data[4] = 0;
            data[5] = Request.level;
            uint16_t crc = crc16(data.data(), data.size() - 2);
            data[6] = uint8_t(crc);
            data[7] = uint8_t(crc >> 8);
            serial_inst_->write(data.data(), data.size());
        }
        
        int tryCount = 0;
        const int MAX_TRY_COUNT = 3;
        size_t count = 0;
        while ((count = serial_inst_->available()) <= 0 && tryCount++ < MAX_TRY_COUNT)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }

        Response.level = 0;
        Response.result = false;
        if (tryCount < MAX_TRY_COUNT)
        {
            unsigned char rbuff[count];
		    size_t readNum = serial_inst_->read(rbuff, count);
            uint16_t crc = crc16(rbuff, readNum - 2);
            uint16_t readCrc = rbuff[readNum - 2] | uint16_t(rbuff[readNum - 1] << 8);
            if (crc == readCrc)
            {
                Response.level = rbuff[3];
                Response.result = true;
            }
        }

        return Response.result;
    }
} // namespace whi_modbus_io

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

    ModbusIo::ModbusIo(std::shared_ptr<rclcpp::Node>& NodeHandle)
        : node_handle_(NodeHandle)
    {
        init();
    }

    ModbusIo::~ModbusIo()
    {
        // reset to init level
        for (const auto& it : init_levels_map_)
        {
            auto request = std::make_shared<whi_interfaces::srv::WhiSrvIo::Request>();
            request->addr = it.first;
            request->level = it.second;
            request->operation = whi_interfaces::srv::WhiSrvIo::Request::OPER_WRITE;
            auto response = std::make_shared<whi_interfaces::srv::WhiSrvIo::Response>();
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
        std::string service = node_handle_->declare_parameter("service", "io_request");
        std::string levelConfig = node_handle_->declare_parameter("init_levels", "init_levels.yaml");
        if (!levelConfig.empty())
        {
            readInitLevels(levelConfig);
        }
        module_ = node_handle_->declare_parameter("hardware_interface.module", "");
        device_addr_ = node_handle_->declare_parameter("hardware_interface.device_addr", 0x01);
        serial_port_ = node_handle_->declare_parameter("hardware_interface.port", "/dev/ttyUSB0");
        baudrate_ = node_handle_->declare_parameter("hardware_interface.baudrate", 9600);

        // serial
	    try
	    {
		    serial_inst_ = std::make_unique<serial::Serial>(serial_port_, baudrate_, serial::Timeout::simpleTimeout(500));
	    }
	    catch (serial::IOException& e)
	    {
		    RCLCPP_FATAL_STREAM(node_handle_->get_logger(), "\033[1;31" << "failed to open serial " <<
                serial_port_ << "\033[0m");
	    }

        if (serial_inst_)
        {
            service_ = node_handle_->create_service<whi_interfaces::srv::WhiSrvIo>(service, 
                std::bind(&ModbusIo::onServiceIo, this, std::placeholders::_1, std::placeholders::_2));
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

    void ModbusIo::onServiceIo(const std::shared_ptr<whi_interfaces::srv::WhiSrvIo::Request> request,
        std::shared_ptr<whi_interfaces::srv::WhiSrvIo::Response> response)
    {
        std::array<uint8_t, 8> data;
        if (request->operation == whi_interfaces::srv::WhiSrvIo::Request::OPER_READ)
        {
            data[0] = uint8_t(device_addr_);
            if (request->addr < 17)
            {
                data[1] = 0x02;
            }
            else
            {
                data[1] = 0x01;
            }
            data[2] = 0;
            data[3] = uint8_t(request->addr - 1);
            data[4] = 0;
            data[5] = 0x01;
            uint16_t crc = crc16(data.data(), data.size() - 2);
            data[6] = uint8_t(crc);
            data[7] = uint8_t(crc >> 8);
            serial_inst_->write(data.data(), data.size());
        }
        else if (request->operation == whi_interfaces::srv::WhiSrvIo::Request::OPER_WRITE)
        {
            data[0] = uint8_t(device_addr_);
            data[1] = 0x05;
            data[2] = 0;
            data[3] = uint8_t(request->addr - 1);
            data[4] = 0;
            data[5] = request->level;
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

        response->level = 0;
        response->result = false;
        if (tryCount < MAX_TRY_COUNT)
        {
            unsigned char rbuff[count];
		    size_t readNum = serial_inst_->read(rbuff, count);
            uint16_t crc = crc16(rbuff, readNum - 2);
            uint16_t readCrc = rbuff[readNum - 2] | uint16_t(rbuff[readNum - 1] << 8);
            if (crc == readCrc)
            {
                response->level = rbuff[3];
                response->result = true;
            }
        }
    }
} // namespace whi_modbus_io

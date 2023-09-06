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

******************************************************************/
#include "whi_modbus_io/modbus_io.h"
#include "CRC.h"

#include <thread>

namespace whi_modbus_io
{
    ModbusIo::ModbusIo(std::shared_ptr<ros::NodeHandle>& NodeHandle)
        : node_handle_(NodeHandle)
    {
        init();
    }

    ModbusIo::~ModbusIo()
    {
	    if (serial_inst_)
	    {
		    serial_inst_->close();
	    }
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

        if (serial_inst_)
        {
            service_ = std::make_unique<ros::ServiceServer>(
                node_handle_->advertiseService("io_request", &ModbusIo::onServiceIo, this));
        }

//////////////////////////////////////////////////
        std::array<uint8_t, 6> data;
        data[0] = 0x01;
        data[1] = 0x01;
        data[2] = 0;
        data[3] = 0x10;
        data[4] = 0;
        data[5] = 0x08;
        int offset = 0;
        uint16_t crcArc = CRC::Calculate(data.data() + offset, data.size() - offset, CRC::CRC_16_ARC());
        std::cout << "crc arcccccccccccccccccccccc " << std::hex << crcArc << std::endl;
        uint16_t crcBuypass = CRC::Calculate(data.data() + offset, data.size() - offset, CRC::CRC_16_BUYPASS());
        std::cout << "crc buypasssssssssssssssssss " << std::hex << crcBuypass << std::endl;
        uint16_t crcCcitt = CRC::Calculate(data.data() + offset, data.size() - offset, CRC::CRC_16_CCITTFALSE());
        std::cout << "crc ccittttttttttttttttttttt " << std::hex << crcCcitt << std::endl;
        uint16_t crcMcrf4 = CRC::Calculate(data.data() + offset, data.size() - offset, CRC::CRC_16_MCRF4XX());
        std::cout << "crc mcrf44444444444444444444 " << std::hex << crcMcrf4 << std::endl;
        uint16_t crcGenibus = CRC::Calculate(data.data() + offset, data.size() - offset, CRC::CRC_16_GENIBUS());
        std::cout << "crc genibussssssssssssssssss " << std::hex << crcGenibus << std::endl;
        uint16_t crcKermit = CRC::Calculate(data.data() + offset, data.size() - offset, CRC::CRC_16_KERMIT());
        std::cout << "crc kermittttttttttttttttttt " << std::hex << crcKermit << std::endl;
        uint16_t crcX25 = CRC::Calculate(data.data() + offset, data.size() - offset, CRC::CRC_16_X25());
        std::cout << "crc x25555555555555555555555 " << std::hex << crcX25 << std::endl;
        uint16_t crcXmodem = CRC::Calculate(data.data() + offset, data.size() - offset, CRC::CRC_16_XMODEM());
        std::cout << "crc xmodemmmmmmmmmmmmmmmmmmm " << std::hex << crcXmodem << std::endl;
    }

    bool ModbusIo::onServiceIo(whi_interfaces::WhiIo::Request& Request,
        whi_interfaces::WhiIo::Response& Response)
    {
        std::array<uint8_t, 8> data;
        if (Request.operation == whi_interfaces::WhiIo::Request::OPER_READ)
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
            uint16_t crc = CRC::Calculate(data.data(), data.size() - 2, CRC::CRC_16_ARC());
            data[6] = uint8_t(crc);
            data[7] = uint8_t(crc >> 8);
            serial_inst_->write(data.data(), data.size());
        }
        else if (Request.operation == whi_interfaces::WhiIo::Request::OPER_WRITE)
        {
            data[0] = 0x01;
            data[1] = 0x05;
            data[2] = 0;
            data[3] = Request.reg - 1;
            data[4] = 0;
            data[5] = Request.level;
            uint16_t crc = CRC::Calculate(data.data(), data.size() - 2, CRC::CRC_16_ARC());
            data[6] = uint8_t(crc);
            data[7] = uint8_t(crc >> 8);
            serial_inst_->write(data.data(), data.size());
        }
        
        int tryCount = 0;
        const int MAX_TRY_COUNT = 3;
        size_t count = 0;
        while ((count = serial_inst_->available()) <= 0 && tryCount++ < MAX_TRY_COUNT)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        if (tryCount < MAX_TRY_COUNT)
        {
            Response.result = true;

            unsigned char rbuff[count];
		    size_t readNum = serial_inst_->read(rbuff, count);
            uint16_t crc = CRC::Calculate(rbuff, readNum - 2, CRC::CRC_16_ARC());
            uint16_t readCrc = rbuff[readNum - 2] | uint16_t(rbuff[readNum - 1] << 8);
            if (crc == readCrc)
            {
                Response.level = rbuff[3];
            }
            else
            {
                Response.result = false;
            }
        }
        else
        {
            Response.result = false;
        }

        return Response.result;
    }
} // namespace whi_modbus_io

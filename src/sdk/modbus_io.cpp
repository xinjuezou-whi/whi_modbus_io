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

    ModbusIo::ModbusIo(const std::string& NodeName/* = "whi_modbus_io"*/,
        const rclcpp::NodeOptions& Options/* = rclcpp::NodeOptions()*/)
        : rclcpp_lifecycle::LifecycleNode(NodeName, "", Options)
    {
        // params
        declare_parameter("init_levels", "init_levels.yaml");
        declare_parameter("hardware_interface.module", "");
        declare_parameter("hardware_interface.device_addr", 0x01);
        declare_parameter("hardware_interface.modbus_instance", "stand_alone");
        declare_parameter("hardware_interface.stand_alone.port", "/dev/ttyUSB0");
        declare_parameter("hardware_interface.stand_alone.baudrate", 9600);
        declare_parameter("hardware_interface.server_depend.modbus_service", "modbus_request");
        declare_parameter("debug.print_comm", false);
        declare_parameter("with_bond", true);
        declare_parameter("heart_beat_period", 0.1);
        declare_parameter("heart_beat_timeout", 4.0);
    }

    ModbusIo::~ModbusIo()
    {
        // reset to init level
        resetToInitLevel();

	    if (serial_inst_)
	    {
		    serial_inst_->close();
	    }
    }

    void ModbusIo::createBond()
    {
        if (with_bond_)
        {
            RCLCPP_INFO(get_logger(), "Creating bond (%s) to lifecycle manager.", get_name());

            bond_ = std::make_shared<bond::Bond>(std::string("bond"), get_name(), shared_from_this());

            bond_->setHeartbeatPeriod(heart_beat_period_);
            bond_->setHeartbeatTimeout(heart_beat_timeout_);
            bond_->start();
        }
    }

    void ModbusIo::destroyBond()
    {
        if (with_bond_)
        {
            RCLCPP_INFO(get_logger(), "Destroying bond (%s) to lifecycle manager.", get_name());

            if (bond_)
            {
                bond_.reset();
            }
        }
    }

    CallbackReturn ModbusIo::on_configure(const rclcpp_lifecycle::State&)
    {
        RCLCPP_INFO(get_logger(), "Configuring");

        init();
        
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn ModbusIo::on_activate(const rclcpp_lifecycle::State&)
    {
        RCLCPP_INFO(get_logger(), "Activating");

        createBond();

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn ModbusIo::on_deactivate(const rclcpp_lifecycle::State&)
    {
        RCLCPP_INFO(get_logger(), "Deactivating");

        // reset to init level
        resetToInitLevel();

        if (serial_inst_)
	    {
		    serial_inst_->close();
	    }

        destroyBond();

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn ModbusIo::on_cleanup(const rclcpp_lifecycle::State&)
    {
        RCLCPP_INFO(get_logger(), "Cleaning up");

        subscriber_.reset();
        service_.reset();

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn ModbusIo::on_shutdown(const rclcpp_lifecycle::State&)
    {
        RCLCPP_INFO(get_logger(), "Shutting down");

        return CallbackReturn::SUCCESS;
    }

    void ModbusIo::init()
    {
        // params
        std::string levelConfig = get_parameter("init_levels").as_string();
        if (!levelConfig.empty())
        {
            readInitLevels(levelConfig);
        }
        module_ = get_parameter("hardware_interface.module").as_string();
        device_addr_ = get_parameter("hardware_interface.device_addr").as_int();

        std::string modbusInstance = get_parameter("hardware_interface.modbus_instance").as_string();
        if (modbusInstance == "stand_alone")
        {
            serial_port_ = get_parameter("hardware_interface.stand_alone.port").as_string();
            baudrate_ = get_parameter("hardware_interface.stand_alone.baudrate").as_int();

            // serial
            try
            {
                serial_inst_ = std::make_unique<serial::Serial>(serial_port_, baudrate_, serial::Timeout::simpleTimeout(500));
            }
            catch (serial::IOException& e)
            {
                RCLCPP_FATAL_STREAM(get_logger(), "\033[1;31" << "failed to open serial " <<
                    serial_port_ << "\033[0m");
            }
        }
        else
        {
            std::string serviceName = get_parameter("hardware_interface.server_depend.modbus_service").as_string();
            modbus_client_ = create_client<whi_interfaces::srv::WhiSrvModBus>(serviceName);
        }

        std::string name("modbus_io_request");
        service_ = create_service<whi_interfaces::srv::WhiSrvIo>(name, 
            std::bind(&ModbusIo::onServiceIo, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
        subscriber_ = create_subscription<whi_interfaces::msg::WhiIo>(
            name, 10, std::bind(&ModbusIo::callbackSub, this, std::placeholders::_1));

        debug_print_comm_ = get_parameter("debug.print_comm").as_bool();
        with_bond_ = get_parameter("with_bond").as_bool();
        if (with_bond_)
        {
            heart_beat_period_ = get_parameter("heart_beat_period").as_double();
            heart_beat_timeout_ = get_parameter("heart_beat_timeout").as_double();
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

    void ModbusIo::composeData(const whi_interfaces::msg::WhiIo& Msg, std::array<uint8_t, 8>& Data)
    {
        Data[0] = uint8_t(device_addr_);
        if (Msg.operation == whi_interfaces::msg::WhiIo::OPER_READ)
        {
            if (Msg.addr < 17)
            {
                Data[1] = 0x02;
            }
            else
            {
                Data[1] = 0x01;
            }
            Data[2] = 0;
            Data[3] = uint8_t(Msg.addr - 1);
            Data[4] = 0;
            Data[5] = 0x01;
        }
        else if (Msg.operation == whi_interfaces::msg::WhiIo::OPER_WRITE)
        {
            Data[1] = 0x05;
            Data[2] = 0;
            Data[3] = uint8_t(Msg.addr - 1);
            Data[4] = 0;
            Data[5] = Msg.level;
        }
        uint16_t crc = crc16(Data.data(), Data.size() - 2);
        Data[6] = uint8_t(crc);
        Data[7] = uint8_t(crc >> 8);
    }

    void ModbusIo::onServiceIo(std::shared_ptr<rclcpp::Service<whi_interfaces::srv::WhiSrvIo>> Service,
        const std::shared_ptr<rmw_request_id_t> RequestHeader,
        const std::shared_ptr<whi_interfaces::srv::WhiSrvIo::Request> Request)
    {
        std::array<uint8_t, 8> data;
        composeData(Request->io, data);

        if (serial_inst_)
        {
            try
            {
                serial_inst_->write(data.data(), data.size());
                if (debug_print_comm_)
                {
                    std::cout << "write ";
                    for (const auto& it : data)
                    {
                        std::cout << std::dec << int(it) << ",";
                    }
                    std::cout << std::endl;
                }

                whi_interfaces::srv::WhiSrvIo::Response response;
                if (Request->io.operation == whi_interfaces::msg::WhiIo::OPER_READ ||
                    Request->io.operation == whi_interfaces::msg::WhiIo::OPER_WRITE_WITH_FEEDBACK)
                {
                    int tryCount = 0;
                    const int MAX_TRY_COUNT = 3;
                    size_t count = 0;
                    while ((count = serial_inst_->available()) < 4 && tryCount++ < MAX_TRY_COUNT)
                    {
                        std::this_thread::sleep_for(std::chrono::milliseconds(50));
                        if (debug_print_comm_)
                        {
                            std::cout << "waiting for response, try count: " << tryCount << std::endl;
                        }
                    }

                    response.level = 0;
                    response.result = false;
                    if (tryCount < MAX_TRY_COUNT)
                    {
                        unsigned char rbuff[count];
                        size_t readNum = serial_inst_->read(rbuff, count);
                        uint16_t crc = crc16(rbuff, readNum - 2);
                        uint16_t readCrc = rbuff[readNum - 2] | uint16_t(rbuff[readNum - 1] << 8);
                        if (crc == readCrc)
                        {
                            response.level = rbuff[3];
                            response.result = true;
                        }

                        if (debug_print_comm_)
                        {
                            std::cout << "read " << readNum << std::endl;
                            for (int i = 0; i < readNum; ++i)
                            {
                                std::cout << std::hex << int(rbuff[i]) << ",";
                            }
                            std::cout << std::endl;
                        }
                    }
                }
                else
                {
                    response.level = Request->io.level;
                    response.result = true;
                }

                if (Service)
                {
                    Service->send_response(*RequestHeader, response);
                }
            }
            catch (const serial::IOException& e) 
            {
                RCLCPP_FATAL_STREAM(get_logger(), "\033[1;31" << "ModBUS IO Exception: " <<
                    e.what() << "\033[0m");
            }
            catch (const serial::SerialException& e) 
            {
                RCLCPP_FATAL_STREAM(get_logger(), "\033[1;31" << "ModBUS Serial Exception: " <<
                    e.what() << "\033[0m");
            }
        }
        else if (modbus_client_)
        {
            auto request = std::make_shared<whi_interfaces::srv::WhiSrvModBus::Request>();
            request->instance.device = data[0];
            request->instance.func = data[1];
            request->instance.crc_size = 2;
            request->instance.data = std::vector<uint8_t>(data.begin() + 2, data.end());
            modbus_client_->async_send_request(
                request,
                [this, Service, RequestHeader](rclcpp::Client<whi_interfaces::srv::WhiSrvModBus>::SharedFuture future)
                {
                    whi_interfaces::srv::WhiSrvIo::Response response;

                    if (future.get()->result)
                    {
                        auto returnSize = future.get()->data.size();
                        uint16_t crc = crc16(future.get()->data.data(), returnSize - 2);
                        uint16_t readCrc = future.get()->data[returnSize - 2] | uint16_t(future.get()->data[returnSize - 1] << 8);
                        if (crc == readCrc)
                        {
                            response.level = future.get()->data[3];
                            response.result = true;
                        }
                        else
                        {
                            response.level = 0;
                            response.result = false;
                        }
                    }
                    else
                    {
                        response.level = 0;
                        response.result = false;

                        RCLCPP_ERROR(get_logger(), "ModBUS service failed");
                    }

                    if (Service)
                    {
                        Service->send_response(*RequestHeader, response);
                    }
                });
        }
    }

    void ModbusIo::callbackSub(const whi_interfaces::msg::WhiIo::SharedPtr Msg)
    {
        std::array<uint8_t, 8> data;
        composeData(*Msg, data);

        if (serial_inst_)
        {
            serial_inst_->write(data.data(), data.size());
        }
        else if (modbus_client_)
        {
            auto request = std::make_shared<whi_interfaces::srv::WhiSrvModBus::Request>();
            request->instance.device = data[0];
            request->instance.func = data[1];
            request->instance.data = std::vector<uint8_t>(data.begin() + 2, data.end());
            auto result = modbus_client_->async_send_request(request);
        }
    }

    void ModbusIo::resetToInitLevel()
    {
        for (const auto& it : init_levels_map_)
        {
            auto request = std::make_shared<whi_interfaces::srv::WhiSrvIo::Request>();
            request->io.addr = it.first;
            request->io.level = it.second;
            request->io.operation = whi_interfaces::msg::WhiIo::OPER_WRITE;

            onServiceIo(nullptr, nullptr, request);
        }
    }
} // namespace whi_modbus_io

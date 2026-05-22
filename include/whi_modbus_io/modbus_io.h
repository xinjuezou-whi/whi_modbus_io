/******************************************************************
class to handle modbus IO

Features:
- modbus
- xxx

Dependencies:
- sudo apt install ros-<ros distro>-serial
- sudo usermod -a -G dialout <user name>, then reboot

Written by Xinjue Zou, xinjue.zou@outlook.com
           Yuhang Shang

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2023-09-06: Initial version
2025-07-24：Migrate from ROS 1
2025-xx-xx: xxx
******************************************************************/
#pragma once
#include "whi_interfaces/srv/whi_srv_io.hpp"
#include <whi_interfaces/msg/whi_io.hpp>
#include <whi_interfaces/srv/whi_srv_mod_bus.hpp>

#include <rclcpp/rclcpp.hpp>
#include <serial/serial.h>

#include <map>
#include <memory>
#include <thread>
#include <condition_variable>
#include <mutex>

namespace whi_modbus_io
{
	class ModbusIo
	{
    public:
        ModbusIo() = delete;
        ModbusIo(std::shared_ptr<rclcpp::Node>& NodeHandle);
        ~ModbusIo();

    protected:
        void init();
        bool readInitLevels(const std::string& Config);
        void composeData(const whi_interfaces::msg::WhiIo& Msg, std::array<uint8_t, 8>& Data);
        void onServiceIo(std::shared_ptr<rclcpp::Service<whi_interfaces::srv::WhiSrvIo>> Service,
            const std::shared_ptr<rmw_request_id_t> RequestHeader,
            const std::shared_ptr<whi_interfaces::srv::WhiSrvIo::Request> Request); // nested service call type
        void callbackSub(const whi_interfaces::msg::WhiIo::SharedPtr Msg);
        void request(const std::array<uint8_t, 8>& Data);

    protected:
        std::shared_ptr<rclcpp::Node> node_handle_{ nullptr };
        std::string module_;
        int device_addr_{ 0x01 };
	    std::string serial_port_;
	    int baudrate_{ 9600 };
        std::unique_ptr<serial::Serial> serial_inst_{ nullptr };
        rclcpp::Client<whi_interfaces::srv::WhiSrvModBus>::SharedPtr modbus_client_{ nullptr };
        rclcpp::Service<whi_interfaces::srv::WhiSrvIo>::SharedPtr service_{ nullptr};
        rclcpp::Subscription<whi_interfaces::msg::WhiIo>::SharedPtr subscriber_{ nullptr };
        std::map<int, int> init_levels_map_;
        bool debug_print_comm_{ false };
	};
} // namespace whi_modbus_io

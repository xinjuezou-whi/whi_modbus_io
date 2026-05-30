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
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <bondcpp/bond.hpp>
#include <serial/serial.h>

#include <map>
#include <memory>
#include <thread>
#include <condition_variable>
#include <mutex>

namespace whi_modbus_io
{
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

	class ModbusIo : public rclcpp_lifecycle::LifecycleNode
	{
    public:
        ModbusIo() = delete;
        ModbusIo(const std::string& NodeName = "whi_modbus_io",
            const rclcpp::NodeOptions& Options = rclcpp::NodeOptions());
        ~ModbusIo();

    public:
        // Create bond connection for nav2 lifecycle manager
        void createBond();
        // Destroy bond connection for nav2 lifecycle manager
        void destroyBond();
        CallbackReturn on_configure(const rclcpp_lifecycle::State&) override;
        CallbackReturn on_activate(const rclcpp_lifecycle::State&) override;
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override;
        CallbackReturn on_cleanup(const rclcpp_lifecycle::State&) override;
        CallbackReturn on_shutdown(const rclcpp_lifecycle::State&) override;

    protected:
        void init();
        bool readInitLevels(const std::string& Config);
        void composeData(const whi_interfaces::msg::WhiIo& Msg, std::array<uint8_t, 8>& Data);
        void onServiceIo(std::shared_ptr<rclcpp::Service<whi_interfaces::srv::WhiSrvIo>> Service,
            const std::shared_ptr<rmw_request_id_t> RequestHeader,
            const std::shared_ptr<whi_interfaces::srv::WhiSrvIo::Request> Request); // nested service call type
        void callbackSub(const whi_interfaces::msg::WhiIo::SharedPtr Msg);
        void resetToInitLevel();

    protected:
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

        bool with_bond_{ true };
        double heart_beat_period_{ 0.1 };
        double heart_beat_timeout_{ 4.0 };
        // Connection to tell that server is still up
        std::shared_ptr<bond::Bond> bond_{nullptr};
	};
} // namespace whi_modbus_io

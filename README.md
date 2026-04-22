# whi_modbus_io
Handle the ModBus IO device while advertising the service of IO setting

## Supported devices
Currently, only the DAQM-43xx is supported; other devices will be introduced following the requirements change of the project

### DAQM-43xx
![image](https://github.com/xinjuezou-whi/whi_modbus_io/assets/72239958/4559a32d-8cd6-460f-b29d-a676bea59959)

## Dependencies
```
git clone https://github.com/xinjuezou-whi/whi_interfaces.git
```

## Parameters

This node can run stand-alone, which occupies the serial resource. Under multiple nodes or device drivers scenario, please configure as the `server_depend` mode, which depends the ModBUS server and does not occupy the serial resource
```
init_levels: /home/nvidia/ros2_ws/src/whi_modbus_io/config/init_levels.yaml
hardware_interface:
  module: DAQM-43xx
  device_addr: 0x07
  modbus_instance: server_depend
  server_depend:
    modbus_service: modbus_request
  stand_alone:
    port: /dev/ttyUART_485_1
    baudrate: 115200
```

## Usage
Read
```
ros2 service call /modbus_io_request whi_interfaces/srv/WhiSrvIo "{io: {addr: <register address>, operation: 0}}"
```

Write
```
ros2 service call /modbus_io_request whi_interfaces/srv/WhiSrvIo "{io: {addr: <register address>, operation: 1, level: <0/1>}}"
```

```
ros2 topic pub -1 /modbus_io_request whi_interfaces/msg/WhiIo "{addr: <register address>, operation: 1, level: <0/1>}"
```

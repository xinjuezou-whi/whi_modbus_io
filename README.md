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
```
whi_modbus_io:
  ros__parameters:
    service: io_request
    init_levels: /home/nvidia/ros2_ws/src/whi_modbus_io/config/init_levels.yaml
    hardware_interface:
      module: DAQM-43xx
      port: /dev/ttyUART_485_2
      baudrate: 9600
```

## Usage
Read
```
ros2 service call /modbus_io_request whi_interfaces/srv/WhiSrvIo "{addr: <register address>, operation: 0}"
```

Write
```
ros2 service call /modbus_io_request whi_interfaces/srv/WhiSrvIo "{addr: <register address>, operation: 1, level: <0/1>}"
```

> NOTE: This node has the default namespace "whi_modbus_io"; therefore, its advertised service would be with this namespace if the service is configured as relative. For absolute service name, please set the service to absolute, like "/modbus_io_request"

# whi_modbus_io
handle the modbus io device

## Dependencies
```
git clone https://github.com/xinjuezou-whi/whi_interfaces.git
```

## Parameters
```
whi_modbus_io:
  service: io_request
  hardware_interface:
    module: DAQM-43xx
    port: /dev/ttyUSB0
    baudrate: 9600
```

## Usage
Read
```
rosservice call /whi_modbus_io/io_request "{reg: <register address>, operation: 0}"
```

Write
```
rosservice call /whi_modbus_io/io_request "{reg: <register address>, operation: 1, level: <0/1>}"
```

> NOTE: this node has the default namespace "whi_modbus_io", therefore its advertised service would with such namespace if the service is configured as relative. For absolute service name please set the service to absolute, like "/io_request"

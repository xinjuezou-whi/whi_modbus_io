# whi_modbus_io
handle the modbus io device

## Dependencies
```
git clone https://github.com/xinjuezou-whi/whi_interfaces.git
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

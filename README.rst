MPRLS Pressure Telemetry (BLE)
##############################

Reads pressure from an MPRLS sensor over I2C and streams measurements over Bluetooth Low Energy (BLE).
This firmware is being developed for integration into a Phantom system.

Features
********
- I2C interface to MPRLS pressure sensor
- BLE GATT service to transmit live pressure samples (notify)
- Serial logging for bring-up/debug
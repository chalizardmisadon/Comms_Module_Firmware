# Firmware_Comms

### to upload code to board
pio run --target upload

### to start a gdb debug server
platformio debug --interface=gdb -x .pioinit

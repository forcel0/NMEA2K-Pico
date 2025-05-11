# Generated Cmake Pico project file

# debuggin on
add_definitions(-DDEBUG_MCP_CAN_SPEED)
add_definitions(-DPICO_OPTIMIZED_DEBUG=1)

project(pico-nmea2k C CXX ASM)

# Add executable. Default name is the project name, version 0.1
add_executable(pico-nmea2k
    # Library executables
    ./NMEA2000_PICO/mcp2515.cpp
    ./NMEA2000_PICO/NMEA2000_PICO.cpp
    ./LCD/lcd_1602_i2c.cpp
    ../NMEA/src/N2kMsg.cpp
    ../NMEA/src/N2kStream.cpp
    ../NMEA/src/N2kMessages.cpp
    ../NMEA/src/N2kTimer.cpp
    ../NMEA/src/Seasmart.cpp
    ../NMEA/src/N2kGroupFunction.cpp
    ../NMEA/src/N2kDeviceList.cpp
    ../NMEA/src/N2kMaretron.cpp
    ../NMEA/src/N2kGroupFunctionDefaultHandlers.cpp
    ../NMEA/src/NMEA2000.cpp


    # Main executalbes
    ./nmea2k.cpp
)

# Add NMEA Lib
#target_include_directories(pico-nmea2k PUBLIC include/)
target_include_directories(pico-nmea2k PUBLIC ./CAN/)
target_include_directories(pico-nmea2k PUBLIC ../NMEA/src/)
target_include_directories(pico-nmea2k PUBLIC ./LCD/)

pico_set_program_name(pico-nmea2k "pico-nmea2k")
pico_set_program_version(pico-nmea2k "0.2")

#pico_enable_stdio_uart(pico-nmea2k 0)
pico_enable_stdio_usb(pico-nmea2k 1)

# Add any user requested libraries
target_link_libraries(pico-nmea2k
    pico_stdlib
    hardware_spi
    hardware_i2c
    hardware_pwm
)

pico_add_extra_outputs(pico-nmea2k)


cmake_minimum_required(VERSION 3.9)
project(ROBOCON2K18)

set(CMAKE_CXX_STANDARD 11)

add_executable(ROBOCON2K18
        "Closed Loop Motor Driver/I2C_anything_Master_2Slaves/motorDriver.cpp"
        "Closed Loop Motor Driver/I2C_anything_Master_2Slaves/motorDriver.h"
        "Closed Loop Motor Driver/I2C_anything_Master_2Slaves_lib/motorDriver.cpp"
        "Closed Loop Motor Driver/I2C_anything_Master_2Slaves_lib/motorDriver.h"
        "Closed Loop Motor Driver/PID_motor_001/AutoPID.cpp"
        "Closed Loop Motor Driver/PID_motor_001/AutoPID.h"
        "Closed Loop Motor Driver/PID_motor_004/AutoPID.cpp"
        "Closed Loop Motor Driver/PID_motor_004/AutoPID.h"
        Libraries/Adafruit_Unified_Sensor/Adafruit_Sensor.h
        Libraries/I2C_Anything/I2C_Anything.h
        Libraries/motorDriver/I2C_Anything.h
        Libraries/motorDriver/motorDriver.cpp
        Libraries/motorDriver/motorDriver.h)

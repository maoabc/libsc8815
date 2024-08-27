cmake_minimum_required(VERSION 3.22)
project(sc8815)


#Hardware related config

#BAT and BUS current sense resistor. 5mR or 10mR
add_definitions(
        -DCONFIG_BUS_SENSE_RESISTOR=5
        -DCONFIG_BAT_SENSE_RESISTOR=5
)

#Battery config
add_definitions(
        #0(1S), 1(2S), 2(3S), 3(4s)
        -DCONFIG_BATTERY_CELL_COUNT=3
        # 0(4.1V), 1(4.2V), 2(4.25V), 3(4.3V), 4(4.35V), 4(4.4V), 6(4.45V), 7(4.5V),
        -DCONFIG_BATTERY_VOLTAGE=1
)
#Ratio config
add_definitions(
        #0(12.5x), 1(5x)
        -DCONFIG_BUS_VOLTAGE_RATIO=0
        #0(12.5x), 1(5x)
        -DCONFIG_BAT_VOLTAGE_RATIO=0
        #0(not allowed), 1(6x), 2(3x), 3(not allowed)
        -DCONFIG_BUS_CURRENT_RATIO=2
        #0(6x), 1(12x)
        -DCONFIG_BAT_CURRENT_RATIO=0

        #0(100x), 1(40x)
        -DCONFIG_VINREG_RATIO=0
)

add_definitions(
        -DCONFIG_EXTERNAL_VBUS=0

        #external vbus use fb pin. vbus_ratio=(1+Rup/Rdown)
        #-DCONFIG_EXTERNAL_VBUS=1
        #-DCONFIG_FB_RESISTOR_UP=1000
        #-DCONFIG_FB_RESISTOR_DOWN=1000
)


set(SC8815_SRC sc8815.c)
set(INCLUDE_DIRS .)

add_library(sc8815 STATIC
        ${SC8815_SRC}
        )

target_include_directories(sc8815 PUBLIC ${INCLUDE_DIRS})
target_link_libraries(sc8815)


add_executable(sc8815_test main.c)
target_link_libraries(sc8815_test sc8815)

# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

# Include additional local components
set(EXTRA_COMPONENT_DIRS "./button")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(esp32-xbee)

if(CONFIG_APP_BUILD_TYPE_APP_2NDBOOT)
    # Erstelle merged binary nach dem Build
    add_custom_command(TARGET app POST_BUILD
        COMMAND ${PYTHON} -m esptool 
        --chip ${IDF_TARGET} 
        merge_bin 
        -o ${CMAKE_BINARY_DIR}/merged_firmware.bin 
        --flash_mode ${CONFIG_ESPTOOLPY_FLASHMODE}
        --flash_freq ${CONFIG_ESPTOOLPY_FLASHFREQ}
        --flash_size ${CONFIG_ESPTOOLPY_FLASHSIZE}
        0x1000 ${CMAKE_BINARY_DIR}/bootloader/bootloader.bin
        0x8000 ${CMAKE_BINARY_DIR}/partition_table/partition-table.bin  
        0x10000 ${CMAKE_BINARY_DIR}/${CMAKE_PROJECT_NAME}.bin
        COMMENT "Creating merged firmware binary"
    )
endif()

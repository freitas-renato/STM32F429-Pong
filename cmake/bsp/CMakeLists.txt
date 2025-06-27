cmake_minimum_required(VERSION 3.22)

set(BSP_Include_Dirs
    ${CMAKE_SOURCE_DIR}/BSP/Inc
)

set(BSP_Src
    ${CMAKE_SOURCE_DIR}/BSP/Src/buzzer.c
    ${CMAKE_SOURCE_DIR}/BSP/Src/cap1206.c
    ${CMAKE_SOURCE_DIR}/BSP/Src/st7789v.c
    ${CMAKE_SOURCE_DIR}/BSP/Src/touch_interface.c
)

# Add include dirs to the "stm32cubemx" interface library 
target_include_directories(stm32cubemx INTERFACE ${BSP_Include_Dirs})

# Create BSP static library
add_library(BSP OBJECT)
target_sources(BSP PRIVATE ${BSP_Src})
target_link_libraries(BSP PUBLIC stm32cubemx)


# Link BSP library to the main project
target_link_libraries(${CMAKE_PROJECT_NAME} BSP)

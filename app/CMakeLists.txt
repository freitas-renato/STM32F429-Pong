cmake_minimum_required(VERSION 3.20.0)

set(ZEPHYR_EXTRA_MODULES ${CMAKE_CURRENT_SOURCE_DIR}/..)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(stm32pong_zephyr)

# set(APP_INCLUDE_DIRECTORIES
#     ${CMAKE_CURRENT_SOURCE_DIR}/inc
# )

target_include_directories(app PRIVATE ${APP_INCLUDE_DIRECTORIES})

set(APP_SOURCES
    src/common/rtos.c
    src/controller/gui.c
    src/controller/touch.c
    src/gui/components/paddle.c
    src/gui/components/ball.c
    src/gui/components/scoreboard.c
    src/gui/views/homepage.c
    src/gui/views/homepage_controller.c
    src/main.c
)

target_sources(app PRIVATE ${APP_SOURCES})

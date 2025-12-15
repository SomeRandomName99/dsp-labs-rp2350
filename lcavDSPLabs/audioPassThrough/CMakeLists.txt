# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico2_w CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(firmware_dsp C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Pull in CMSIS-DSP
set(CMSISCORE "${PICO_SDK_PATH}/src/rp2_common/cmsis/stub/CMSIS/Core")
set(DISABLEFLOAT16 ON)
set(CMSISDSP "${CMAKE_CURRENT_LIST_DIR}/../CMSIS-DSP")
add_subdirectory(${CMSISDSP}/Source bin_dsp)
target_compile_options(CMSISDSP PRIVATE -Ofast -ffunction-sections -fdata-sections)

# Choose the lab to build
set(LAB_ID 5 CACHE STRING "Selects the lab to build (1=PassThrough, 2=RobotEffects)")

if(LAB_ID EQUAL 1)
    set(LAB_SRC lab1_audioPassThrough/audio_proc.c)
elseif(LAB_ID EQUAL 2)
    set(LAB_SRC lab2_robotEffects/audio_proc.c)
elseif(LAB_ID EQUAL 3)
    set(LAB_SRC lab3_digitalFilterDesign/audio_proc.c)
elseif(LAB_ID EQUAL 4)
    set(LAB_SRC lab4_granularSynthesis/audio_proc.c)
elseif(LAB_ID EQUAL 5)
    set(LAB_SRC lab5_linearPredictiveCoding/audio_proc.c)
else()
    message(FATAL_ERROR "Invalid LAB_ID: ${LAB_ID}")
endif()

# A library for all common code shared by all the labs
add_executable(firmware_dsp
    firmware_core/main.c
    firmware_core/usb_descriptors.c
    firmware_core/audio_usb.c
    firmware_core/audio_i2s.c 
    firmware_core/audio_bus.c
    ${LAB_SRC}
)

# Generate PIO header
pico_generate_pio_header(firmware_dsp ${CMAKE_CURRENT_LIST_DIR}/firmware_core/firmware_dsp.pio)

target_link_libraries(firmware_dsp 
    pico_stdlib
    hardware_dma
    hardware_pio
    pico_cyw43_arch_none
    tinyusb_device
    tinyusb_board
    CMSISDSP
)

target_compile_definitions(firmware_dsp PRIVATE LAB_ID=${LAB_ID})
target_include_directories(firmware_dsp PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/firmware_core
    ${CMAKE_CURRENT_LIST_DIR}/../CMSIS-DSP/Include
    ${CMSISCORE}/Include
)

pico_set_program_name(firmware_dsp "firmware_dsp Lab${LAB_ID}")
pico_enable_stdio_uart(firmware_dsp 1)
pico_enable_stdio_usb(firmware_dsp 0)
pico_add_extra_outputs(firmware_dsp)
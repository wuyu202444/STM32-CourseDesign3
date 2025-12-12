cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME CourseDesign3)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)


set(MY_BSP_PATH
        "${CMAKE_CURRENT_SOURCE_DIR}/USER/BSP"
        "${CMAKE_CURRENT_SOURCE_DIR}/USER/App"
)

# 循环遍历路径列表，把每个目录下的 .c 都找出来放入 MY_BSP_SOURCES
foreach(dir ${MY_BSP_PATH})
    file(GLOB temp_sources "${dir}/*.c")
    list(APPEND MY_BSP_SOURCES ${temp_sources})
endforeach()

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE ${MY_BSP_PATH})
target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${MY_BSP_SOURCES})


# 设置浮点运算模式（根据MCU选择）
# 对于没有硬件FPU的MCU使用soft
#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mfloat-abi=soft")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mfloat-abi=soft")

# 对于有硬件FPU的MCU（如F4系列）使用
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mfloat-abi=hard -mfpu=fpv4-sp-d16")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mfloat-abi=hard -mfpu=fpv4-sp-d16")

# 使能printf浮点数支持
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE -u_printf_float)

# 可选：使能scanf浮点数支持
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE -u_scanf_float)

# 可选：使用nano库减小代码体积
# target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
#     --specs=nano.specs
#     --specs=nosys.specs
# )
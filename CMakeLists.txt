cmake_minimum_required(VERSION 3.14)
if (DEFINED MCU)
  include(FetchContent)
  FetchContent_Declare(
    mcu_support
    GIT_REPOSITORY https://github.com/bolderflight/mcu-support.git
    GIT_TAG v1.0.0
  )
  FetchContent_MakeAvailable(mcu_support)
  # Setting up the toolchain
  set(CMAKE_TOOLCHAIN_FILE "${mcu_support_SOURCE_DIR}/cmake/cortex.cmake")
  # Project information
  project(EasyCan
    VERSION 1.0.0
    DESCRIPTION "Easy to use CAN driver"
    LANGUAGES CXX
  )
  # Grab the processor and set up definitions and compile options
  include(${mcu_support_SOURCE_DIR}/cmake/config_mcu.cmake)
  configMcu(${MCU} ${mcu_support_SOURCE_DIR})
  # Fetch core
  FetchContent_Declare(
    core
    GIT_REPOSITORY https://github.com/bolderflight/core.git
    GIT_TAG v3.0.2
  )
  FetchContent_MakeAvailable(core)
  # Fetch flexcan
  FetchContent_Declare(
    flexcan
    GIT_REPOSITORY https://github.com/bolderflight/flexcan.git
    GIT_TAG v1.0.0
  )
  FetchContent_MakeAvailable(flexcan)
  # Fetch circular buffer
  FetchContent_Declare(
    circle_buf
    GIT_REPOSITORY https://github.com/bolderflight/circle_buf.git
    GIT_TAG v4.0.2
  )
  FetchContent_MakeAvailable(circle_buf)
  # Add the library target
  add_library(easycan
    src/easycan.h
  )
  # Link libraries
  target_link_libraries(easycan
    PUBLIC
      core
      flexcan
      circle_buf
  )
  # Setup include directories 
  target_include_directories(easycan PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:include>
  )
  # Example and test if this project is built separately
  if (PROJECT_NAME STREQUAL CMAKE_PROJECT_NAME)
    # Add the example target
    add_executable(loopback_example examples/cmake/loopback.cc)
    # Add the includes
    target_include_directories(loopback_example PUBLIC 
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
    )
    # Link libraries to the example target
    target_link_libraries(loopback_example
      PRIVATE 
        easycan
    )
    # Add hex and upload targets
    include(${mcu_support_SOURCE_DIR}/cmake/flash_mcu.cmake)
    FlashMcu(loopback_example ${MCU} ${mcu_support_SOURCE_DIR})
  endif()
endif()

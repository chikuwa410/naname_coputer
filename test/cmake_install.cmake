# Install script for directory: /home/shu/pico/pico-sdk/test

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "TRUE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/arm-none-eabi-objdump")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/shu/pico/pico-sdk/Benzen_new/test/pico_test/cmake_install.cmake")
  include("/home/shu/pico/pico-sdk/Benzen_new/test/pico_stdlib_test/cmake_install.cmake")
  include("/home/shu/pico/pico-sdk/Benzen_new/test/pico_stdio_test/cmake_install.cmake")
  include("/home/shu/pico/pico-sdk/Benzen_new/test/pico_time_test/cmake_install.cmake")
  include("/home/shu/pico/pico-sdk/Benzen_new/test/pico_divider_test/cmake_install.cmake")
  include("/home/shu/pico/pico-sdk/Benzen_new/test/pico_float_test/cmake_install.cmake")
  include("/home/shu/pico/pico-sdk/Benzen_new/test/kitchen_sink/cmake_install.cmake")
  include("/home/shu/pico/pico-sdk/Benzen_new/test/hardware_irq_test/cmake_install.cmake")
  include("/home/shu/pico/pico-sdk/Benzen_new/test/hardware_pwm_test/cmake_install.cmake")
  include("/home/shu/pico/pico-sdk/Benzen_new/test/cmsis_test/cmake_install.cmake")
  include("/home/shu/pico/pico-sdk/Benzen_new/test/pico_sem_test/cmake_install.cmake")

endif()


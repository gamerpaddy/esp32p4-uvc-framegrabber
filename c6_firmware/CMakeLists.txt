# ESP-Hosted coprocessor firmware for the on-module ESP32-C6.
#
# Separate project from the P4 app: different target, different toolchain
# invocation. Build it FIRST - the P4 app embeds the binaries this produces
# and flashes them into the C6 over UART. See README.md.

cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(c6_hosted_cp)

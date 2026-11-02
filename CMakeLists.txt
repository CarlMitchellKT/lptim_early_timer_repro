# Minimal reproducer for early k_timer expiry / kernel-time jumps with the
# STM32 LPTIM tickless system timer (drivers/timer/stm32_lptim_timer.c).
cmake_minimum_required(VERSION 3.20)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(lptim_early_timer_repro LANGUAGES C)

target_sources(app PRIVATE src/main.c)
target_compile_options(app PRIVATE -Wall -Wextra)

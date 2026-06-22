# AeroDrag Wheel — Zephyr application.
# Firmware SEPARATO (nRF52840) — NON fa parte del build ESP-IDF della repo madre.
cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(aerodrag_wheel)

target_sources(app PRIVATE
	src/main.c
	src/wheel_ble.c
	src/wheel_battery.c
	src/wheel_store.c
)

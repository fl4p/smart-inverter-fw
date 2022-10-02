https://docs.espressif.com/projects/esp-idf/en/release-v3.3/get-started-cmake/index.html#get-started-get-esp-idf-cmake

idf.py menuconfig
idf.py build

idf.py -p COM3 flash


# Realtime
is the mcpwm real-time safe?
.https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/mcpwm.html#iram-safe


https://esp32.com/viewtopic.php?t=15856

..\esp-idf-v5.0\export.bat
idf.py build && idf.py -p COM3 flash && idf.py -p COM3 monitor
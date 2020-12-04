@echo off
:start
set /p address=Adresse: 
C:\Users\Nik\.platformio\packages\toolchain-xtensa32\bin\xtensa-esp32-elf-gdb "%cd%\.pio\build\nodemcu-32s\firmware.elf" -silent -ex "l *%address%" -ex "q"
REM /home/ruben/.platformio/packages/toolchain-xtensa32/bin/xtensa-esp32-elf-gdb .pio/build/nodemcu-32s/firmware.elf
goto start
pause
exit
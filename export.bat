SETLOCAL ENABLEEXTENSIONS
@echo off
set SENSOR_VERISON=2020061101
set INSIDE_SENSORS=3C71BFAA8054 CC50E399AF7C
set OUTSIDE_SENSORS=246F282E895C

echo exporting FW images with Version %SENSOR_VERISON%
del .\export\*.* /Q
copy .\.pio\build\inside_sensor\firmware.bin .\export\3C71BFAA8054.bin
echo Inside Sensors:
(for %%a in (%INSIDE_SENSORS%) do ( 
   echo %%a %SENSOR_VERISON%
   copy .\.pio\build\inside_sensor\firmware.bin .\export\%%a.bin
   echo %SENSOR_VERISON% > .\export\%%a.version
))
echo Outside Sensors:
(for %%b in (%OUTSIDE_SENSORS%) do ( 
   echo %%b %SENSOR_VERISON%
   copy .\.pio\build\outside_sensor\firmware.bin .\export\%%b.bin
   echo %SENSOR_VERISON% > .\export\%%b.version
))

del Z:\ESPOTA\*.* /Q
copy .\export\*.* Z:\ESPOTA\ 

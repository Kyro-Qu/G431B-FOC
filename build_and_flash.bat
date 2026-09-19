@echo off
"E:\Keil_v5\UV4\UV4.exe" -b MDK-ARM\FOC_G431.uvprojx -t "FOC_G431" -o "build.log"
type MDK-ARM\build.log
"E:\Keil_v5\UV4\UV4.exe" -f MDK-ARM\FOC_G431.uvprojx -t "FOC_G431" -o "flash.log"
type MDK-ARM\flash.log

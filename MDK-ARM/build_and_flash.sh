#!/bin/bash
/e/Keil_v5/UV4/UV4.exe -b FOC_G431.uvprojx -o build_out.txt
cat build_out.txt
/e/Keil_v5/UV4/UV4.exe -f FOC_G431.uvprojx -o flash_out.txt
cat flash_out.txt

# AGENTS.md - Keil MDK / STM32 Project Guide

This file is a reusable guide for AI agents working on Keil MDK STM32/CubeMX style projects.
Keep it generic enough to copy into similar projects. Put project-specific notes near the bottom.

## First Steps

1. Start from the repository/project root.
2. Look for a Keil project file:
   ```powershell
   Get-ChildItem -Recurse -Filter *.uvprojx
   ```
3. Look for a CubeMX project file:
   ```powershell
   Get-ChildItem -Recurse -Filter *.ioc
   ```
4. Read the project tree before editing:
   ```powershell
   rg --files
   ```

Typical layout:

```text
Core/                 CubeMX generated application code
Drivers/              STM32 HAL/CMSIS drivers
MDK-ARM/              Keil project, build outputs, custom source folders
MDK-ARM/Code/         User/application modules, if present
```

## Build With Keil

Find `UV4.exe`:

```powershell
where UV4.exe
Test-Path 'E:\Keil_v5\UV4\UV4.exe'
Test-Path 'C:\Keil_v5\UV4\UV4.exe'
Get-ChildItem -Path C:\,D:\,E:\ -Recurse -Filter UV4.exe -ErrorAction SilentlyContinue | Select-Object -First 10 -ExpandProperty FullName
```

Build from the project root:

```powershell
& '<path-to-UV4.exe>' -b '<path-to-project.uvprojx>' -j0
```

Rebuild all:

```powershell
& '<path-to-UV4.exe>' -r '<path-to-project.uvprojx>' -j0
```

Example:

```powershell
& 'E:\Keil_v5\UV4\UV4.exe' -b 'MDK-ARM\Project.uvprojx' -j0
```

Important: do not trust the process exit code alone. UV4 can return success even when the build has errors. Always inspect the generated build log.

Common build log locations:

```text
MDK-ARM/<TargetName>/<TargetName>.build_log.htm
MDK-ARM/build/<TargetName>/*.log
```

Search the log:

```powershell
Select-String -Path 'MDK-ARM\**\*.build_log.htm' -Pattern 'error|warning|0 Error|Target not created|Program Size'
```

Build is clean only when the log contains something like:

```text
"TargetName.axf" - 0 Error(s), 0 Warning(s).
```

## Adding Source Files To A Keil Project

Creating a `.c` file is not enough. Keil compiles only files listed in the `.uvprojx`.

Open the `.uvprojx` and find:

```xml
<Groups>
  <Group>
    <GroupName>...</GroupName>
    <Files>
```

Add a C source file under the appropriate group:

```xml
<File>
  <FileName>module_name.c</FileName>
  <FileType>1</FileType>
  <FilePath>.\relative\path\module_name.c</FilePath>
</File>
```

Common `FileType` values:

```text
1 = C source
2 = assembly source
```

Headers usually do not need to be added to `.uvprojx`; they only need to be reachable through include paths.

## Include Paths

Include paths live in `.uvprojx` under:

```xml
<VariousControls>
  <IncludePath>...</IncludePath>
</VariousControls>
```

Paths are semicolon-separated, often relative to the Keil project directory:

```text
../Core/Inc;../Drivers/...;./Code/app;./Code/core
```

If a new header is placed in an already listed directory, no include path change is needed.
If a new directory is introduced, add it to `IncludePath`.

Prefer existing project include style. If both are used, keep local code consistent:

```c
#include "module.h"
#include "../folder/module.h"
```

## CubeMX Boundaries

CubeMX generated files often contain protected regions:

```c
/* USER CODE BEGIN ... */
/* USER CODE END ... */
```

Prefer putting edits inside USER CODE blocks when editing CubeMX-managed files such as:

```text
Core/Src/main.c
Core/Src/stm32xx_it.c
Core/Src/stm32xx_hal_msp.c
Core/Inc/main.h
```

Avoid editing generated initialization code outside USER CODE blocks unless necessary, because CubeMX may overwrite it.

## Encoding Notes

Many older Keil/CubeMX projects contain non-UTF-8 files, especially when comments include Chinese text.

When `apply_patch` fails with an invalid UTF-8 error:

1. Prefer small ASCII-only edits.
2. Use PowerShell with `[System.Text.Encoding]::Default` for precise replacements.
3. Avoid rewriting unrelated comments.
4. If a header is badly corrupted by encoding/comment issues, consider rewriting only that header as clean ASCII.

PowerShell pattern:

```powershell
$path = 'path\to\file.c'
$enc = [System.Text.Encoding]::Default
$full = (Resolve-Path -LiteralPath $path).Path
$text = [System.IO.File]::ReadAllText($full, $enc)
# modify $text carefully
[System.IO.File]::WriteAllText($full, $text, $enc)
```

## Verification Checklist

After edits:

1. Confirm source references:
   ```powershell
   rg -n "new_function|new_file|new_symbol" Core MDK-ARM
   ```
2. If a new `.c` file was added, confirm it is in `.uvprojx`.
3. Run Keil build.
4. Inspect build log for errors and warnings.
5. Report the exact build result.

## Embedded/FOC Safety Notes

For motor-control firmware:

1. Keep startup behavior conservative.
2. Use low voltage/current values for calibration by default.
3. Add timeout/fault paths for motion-producing routines.
4. Do not let debug/main-loop commands overwrite calibration commands while calibration is active.
5. Make telemetry channels explicit so hardware behavior can be verified.

## Project-Local Notes

Update this section when copying the file to another project.

Current project example:

```text
Keil project: MDK-ARM\FOC_G431.uvprojx
Known UV4 path: E:\Keil_v5\UV4\UV4.exe
Build command:
  & 'E:\Keil_v5\UV4\UV4.exe' -b 'MDK-ARM\FOC_G431.uvprojx' -j0
Build log:
  MDK-ARM\FOC_G431\FOC_G431.build_log.htm
Custom FOC source group:
  MDK-ARM\Code\foc
```

Current telemetry example:

```text
VOFA fdata[0] = open-loop electrical angle
VOFA fdata[1] = encoder mechanical angle
VOFA fdata[2] = encoder velocity rpm
VOFA fdata[3] = ABZ Z/index interrupt count
VOFA fdata[4] = calibration state
```

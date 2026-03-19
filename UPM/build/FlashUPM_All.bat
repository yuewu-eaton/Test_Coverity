for /F "tokens=3" %%A in ('findstr /C:"#define CONST_FirmwareVersion" ..\Source\Version.h') do if NOT %%A=="" (set FW_VERSION=%%A)
for /F "tokens=3 delims=(	 " %%A in ('findstr /C:"#define CONST_FirmwareBuildNum" ..\Source\Version.h') do if NOT %%A=="" (set FW_BUILD=%%A)

eFlash.exe ..\Release\Essential_UPM_ALL_%FW_VERSION%_%FW_BUILD%.ROM -t60 -aCOM2 -c57600

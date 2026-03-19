set FW_VERSION=001
set FW_BUILD=001

%1 --intel --outfile=%3 -order=MS --romwidth=16 %2

del /F *.ROM 

for /F "tokens=3" %%A in ('findstr /C:"#define CONST_FirmwareVersion" ..\Source\Version.h') do if NOT %%A=="" (set FW_VERSION=%%A)
for /F "tokens=3 delims=(	 " %%A in ('findstr /C:"#define CONST_FirmwareBuildNum" ..\Source\Version.h') do if NOT %%A=="" (set FW_BUILD=%%A)

"../build/IntelHex2Rom.exe" 590 %FW_VERSION% %3 /CSE /K /I 5

move Essential_UPM.ROM Hobbit_3C3_UPM1_%FW_VERSION%_%FW_BUILD%.ROM

"../build/IntelHex2Rom.exe" 590 %FW_VERSION% %3 /CSE /K /I 8

move Essential_UPM.ROM Hobbit_3C3_UPM2_%FW_VERSION%_%FW_BUILD%.ROM

"../build/IntelHex2Rom.exe" 590 %FW_VERSION% %3 /CSE /K /I 1D

move Essential_UPM.ROM Hobbit_3C3_UPM_ALL_%FW_VERSION%_%FW_BUILD%.ROM

"../build/IntelHex2Rom.exe" 591 %FW_VERSION% %3 /CSE /K /I 5

move Essential_UPM.ROM Oriondo_Tower_UPM1_%FW_VERSION%_%FW_BUILD%.ROM

"../build/IntelHex2Rom.exe" 591 %FW_VERSION% %3 /CSE /K /I 8

move Essential_UPM.ROM Oriondo_Tower_UPM2_%FW_VERSION%_%FW_BUILD%.ROM

"../build/IntelHex2Rom.exe" 591 %FW_VERSION% %3 /CSE /K /I 1D

move Essential_UPM.ROM Oriondo_Tower_UPM_ALL_%FW_VERSION%_%FW_BUILD%.ROM

"../build/IntelHex2Rom.exe" 720 %FW_VERSION% %3 /CSE /K /I 5

move Essential_UPM.ROM Hobbit_93T_UPM1_%FW_VERSION%_%FW_BUILD%.ROM

"../build/IntelHex2Rom.exe" 720 %FW_VERSION% %3 /CSE /K /I 8

move Essential_UPM.ROM Hobbit_93T_UPM2_%FW_VERSION%_%FW_BUILD%.ROM

"../build/IntelHex2Rom.exe" 720 %FW_VERSION% %3 /CSE /K /I 1D

move Essential_UPM.ROM Hobbit_93T_UPM_ALL_%FW_VERSION%_%FW_BUILD%.ROM
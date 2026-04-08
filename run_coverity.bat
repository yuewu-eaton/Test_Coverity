@echo off
echo === Coverity Wrapper Start ===

set REAL_SCRIPT=C:\Temp\Coverity\run_coverity.bat

if not exist "%REAL_SCRIPT%" (
  echo [ERROR] Real Coverity script not found:
  echo %REAL_SCRIPT%
  exit /b 100
)

call "%REAL_SCRIPT%"
set RET=%ERRORLEVEL%

echo === Coverity Wrapper End (exit=%RET%) ===
exit /b %RET%

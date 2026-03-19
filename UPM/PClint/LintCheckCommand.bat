:: Batch file to merge Com and Core s19 files into a single one, adding an offset to
:: all Com addresses

:: 1st argument must be destination file path

@echo Eaton PC Lint Check
if %1 == all goto :all
if %1 == incremental goto :incremental
goto :ERROR1

:all
@echo Search All file
"%~dp0LintFileCheck.vbs" "%~dp0" all
goto :END

:incremental
@echo Incremental Search file
"%~dp0LintFileCheck.vbs" "%~dp0" incremental
goto :END

:END
@copy "%~dp0LintHeaderFolder.txt" "%~dp0LintHeaderFolder.lnt"
@copy "%~dp0LintSourceFile.txt" "%~dp0LintSourceFile.lnt"
@del "%~dp0LintHeaderFolder.txt"
@del "%~dp0LintSourceFile.txt"
C:\lint\lint-nt.exe compiler-config.lnt project-config.lnt LintSourceFile.lnt -vf -w3
@echo Eaton PC Lint Check Completed
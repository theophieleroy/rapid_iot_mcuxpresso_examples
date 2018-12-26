@echo off
cls
setlocal EnableDelayedExpansion
set curfldr=%cd%
set timestmp=%date:~0,2%%time:~0,2%%time:~3,2%%time:~6,2%%time:~9,2%
cd %curfldr%

if not exist "%curfldr%\_OUTPUT_\" (
  mkdir "%curfldr%\_OUTPUT_\"
  if "!errorlevel!" EQU "0" (
    echo Folder created successfully
  ) else (
    echo Error while creating folder
  )
) else (
  echo Folder already exists
)

set curfldr=%cd%
cd %curfldr%
echo AutoAPIDoc solution helps you generate multiple formats of the API reference document, using latest source code files.
echo.
echo Generating PDF Help file format.....
doxygen Doxyfile_lib_PDF_ChangeLog_RAPIDIOT
set src=%cd%
REM copy %src%\template\doxygen-post-process.pl %src%\latex\doxygen-post-process.pl
REM copy %src%\template\doxygen.sty %src%\latex\doxygen.sty
REM perl fix_latex_issues.pl
cd latex
REM perl doxygen-post-process.pl
echo Please confirm with the inclusion order in .\latex\refman.tex, then press any key to continue...
pause
call make.bat
echo Generation of PDF Help file format complete
echo.
echo Copying generated files to the release folder
move refman.pdf %curfldr%/_OUTPUT_/refman.pdf
cd..
del output.txt /q
move html %curfldr%/_OUTPUT_/html_%timestmp% 
rd latex /s /q 



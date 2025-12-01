@echo off
REM Build helper script for Windows - compiles CMSIS-DSP functions before tests
ruby "%~dp0build_helpers\compile_cmsis.rb"
exit /b %errorlevel%

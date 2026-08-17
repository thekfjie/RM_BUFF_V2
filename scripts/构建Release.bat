@echo off
setlocal
cd /d "%~dp0.."
call "%~dp0build_release.bat"
exit /b %ERRORLEVEL%

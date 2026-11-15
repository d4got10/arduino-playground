@echo off
rem Запуск микрофона для Discord: звук уходит в VB-Cable.
rem В Discord выбери устройство ввода "CABLE Output".
chcp 65001 >nul
python "%~dp0host\usb_mic.py" --device "CABLE Input" --gain 3 %*
pause

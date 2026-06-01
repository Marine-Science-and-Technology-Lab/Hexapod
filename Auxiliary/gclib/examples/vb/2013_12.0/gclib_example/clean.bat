@echo off
cd gclib_example
if exist bin (rmdir /s /q bin)
if exist obj (rmdir /s /q obj)
cd ..
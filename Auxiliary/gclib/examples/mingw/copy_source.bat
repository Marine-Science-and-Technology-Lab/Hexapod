@echo off
REM Copies source from default installation of gclib to the project directory.
REM First, be sure to copy the project directory somewhere writeable.
if not exist "\Program Files (x86)\Galil\gclib\examples\cpp" (
echo Cannot find default installation
pause
exit
)
copy "\Program Files (x86)\Galil\gclib\examples\cpp\*.*" .
copy "\Program Files (x86)\Galil\gclib\include\*.*" .
copy "\Program Files (x86)\Galil\gclib\lib\dynamic\x86\*.lib" .
copy "\Program Files (x86)\Galil\gclib\dll\x86\*.dll" .
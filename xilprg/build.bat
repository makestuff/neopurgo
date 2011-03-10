@echo off
vcbuild /nologo xilprg.vcproj
copy xilprg.conf obj\win32\Debug\
copy xilprg.conf obj\win32\Release\

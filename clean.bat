@ECHO OFF

del /q main.hex >NUL 2>NUL
del /q main.ihx >NUL 2>NUL

cd stdperiphlib\src
del /q *.asm >NUL 2>NUL
del /q *.rel >NUL 2>NUL
del /q *.lk >NUL 2>NUL
del /q *.lst >NUL 2>NUL
del /q *.rst >NUL 2>NUL
del /q *.sym >NUL 2>NUL
del /q *.cdb >NUL 2>NUL
del /q *.map >NUL 2>NUL
del /q *.elf >NUL 2>NUL
del /q *.bin >NUL 2>NUL
cd..
cd..

@ECHO ON
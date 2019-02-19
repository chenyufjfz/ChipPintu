C:\chenyu\Software\GnuWin32\bin\flex -olex.yy.cpp .\spice.l
del spice.tab.*
copy .\spice.ypp C:\chenyu\Software\GnuWin32\bin\
cd C:\chenyu\Software\GnuWin32\bin\
bison -vd  spice.ypp
copy spice.tab.* C:\chenyu\work\ChipPintu\NetAnalysis
pause
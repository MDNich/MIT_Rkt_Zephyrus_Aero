echo off
set LOCALHOST=%COMPUTERNAME%
set KILL_CMD="C:\PROGRA~1\ANSYSI~1\v242\fluent/ntbin/win64/winkill.exe"

start "tell.exe" /B "C:\PROGRA~1\ANSYSI~1\v242\fluent\ntbin\win64\tell.exe" MARCDNICHITBF25 49839 CLEANUP_EXITING
timeout /t 1
"C:\PROGRA~1\ANSYSI~1\v242\fluent\ntbin\win64\kill.exe" tell.exe
if /i "%LOCALHOST%"=="MARCDNICHITBF25" (%KILL_CMD% 5664) 
if /i "%LOCALHOST%"=="MARCDNICHITBF25" (%KILL_CMD% 5544) 
if /i "%LOCALHOST%"=="MARCDNICHITBF25" (%KILL_CMD% 6012) 
if /i "%LOCALHOST%"=="MARCDNICHITBF25" (%KILL_CMD% 7000) 
if /i "%LOCALHOST%"=="MARCDNICHITBF25" (%KILL_CMD% 3976) 
if /i "%LOCALHOST%"=="MARCDNICHITBF25" (%KILL_CMD% 8680)
del "Z:\Developer\MIT_Rkt_Team\MIT_Rkt_Aurora_Aero\MIT_Rkt_Aurora_Aero\AeroShell_proj\Old\ansys_wd\cleanup-fluent-MARCDNICHITBF25-3976.bat"

echo off
set LOCALHOST=%COMPUTERNAME%
set KILL_CMD="C:\PROGRA~1\ANSYSI~1\v242\fluent/ntbin/win64/winkill.exe"

start "tell.exe" /B "C:\PROGRA~1\ANSYSI~1\v242\fluent\ntbin\win64\tell.exe" MARCDNICHITBF25 50441 CLEANUP_EXITING
timeout /t 1
"C:\PROGRA~1\ANSYSI~1\v242\fluent\ntbin\win64\kill.exe" tell.exe
if /i "%LOCALHOST%"=="MARCDNICHITBF25" (%KILL_CMD% 8900) 
if /i "%LOCALHOST%"=="MARCDNICHITBF25" (%KILL_CMD% 4464) 
if /i "%LOCALHOST%"=="MARCDNICHITBF25" (%KILL_CMD% 8264) 
if /i "%LOCALHOST%"=="MARCDNICHITBF25" (%KILL_CMD% 1172) 
if /i "%LOCALHOST%"=="MARCDNICHITBF25" (%KILL_CMD% 5412) 
if /i "%LOCALHOST%"=="MARCDNICHITBF25" (%KILL_CMD% 5452)
del "Z:\Developer\MIT_Rkt_Team\MIT_Rkt_Aurora_Aero\MIT_Rkt_Aurora_Aero\AeroShell_proj\Old\ansys_wd\cleanup-fluent-MARCDNICHITBF25-5412.bat"

echo off
set LOCALHOST=%COMPUTERNAME%
set KILL_CMD="C:\PROGRA~1\ANSYSI~1\v242\fluent/ntbin/win64/winkill.exe"

start "tell.exe" /B "C:\PROGRA~1\ANSYSI~1\v242\fluent\ntbin\win64\tell.exe" MARCDNICHITBF25 50701 CLEANUP_EXITING
timeout /t 1
"C:\PROGRA~1\ANSYSI~1\v242\fluent\ntbin\win64\kill.exe" tell.exe
if /i "%LOCALHOST%"=="MARCDNICHITBF25" (%KILL_CMD% 7920) 
if /i "%LOCALHOST%"=="MARCDNICHITBF25" (%KILL_CMD% 10840) 
if /i "%LOCALHOST%"=="MARCDNICHITBF25" (%KILL_CMD% 12440) 
if /i "%LOCALHOST%"=="MARCDNICHITBF25" (%KILL_CMD% 9068) 
if /i "%LOCALHOST%"=="MARCDNICHITBF25" (%KILL_CMD% 6640) 
if /i "%LOCALHOST%"=="MARCDNICHITBF25" (%KILL_CMD% 9340)
del "Z:\Developer\MIT_Rkt_Team\MIT_Rkt_Aurora_Aero\MIT_Rkt_Aurora_Aero\AeroShell_proj\Old\ansys_wd\cleanup-fluent-MARCDNICHITBF25-6640.bat"

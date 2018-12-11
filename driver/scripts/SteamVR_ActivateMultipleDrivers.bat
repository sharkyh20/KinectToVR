@ECHO off
SET HOME=%~dp0

ECHO "Setting activateMultipleDrivers to true in steamvr.vrsettings..."
SET "ActivateMultipleDriversPath=%HOME%SteamVR_ActivateMultipleDrivers.ps1"
powershell -NoProfile -ExecutionPolicy Bypass "& '%ActivateMultipleDriversPath%'";
IF %ERRORLEVEL% NEQ 0 (goto failure)

:: Pause before exit if pause isn't suppressed
ECHO "Successfuly set activateMultipleDrivers in steamvr.vrsettings."
IF NOT DEFINED suppressPause (
  PAUSE
)
GOTO exit

:failure
ECHO "Error occured setting activateMultipleDrivers in steamvr.vrsettings!"
IF NOT DEFINED suppressPause (
  PAUSE
)
EXIT /B %ERRORLEVEL%
GOTO exit

:exit
ENDLOCAL
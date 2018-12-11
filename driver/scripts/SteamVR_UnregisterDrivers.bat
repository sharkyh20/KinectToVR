@ECHO off
CALL SteamVR_SetDriverVars.bat

SET "DEPRECATED_INSTALL_DIR=%STEAMVR_RUNTIME_DIR%drivers\psmove"
IF EXIST %DEPRECATED_INSTALL_DIR% (
  ECHO "Unregistering deprecated K2VR driver in SteamVR folder..."
  "%STEAMVR_RUNTIME_DIR%\bin\win64\vrpathreg" removedriver "%DEPRECATED_INSTALL_DIR%"
  
  ECHO "Deleting deprecated K2VR driver in SteamVR folder..."
  RMDIR /s /q "%DEPRECATED_INSTALL_DIR%"
)

ECHO "Unregistering K2VR SteamVR driver..."
"%STEAMVR_RUNTIME_DIR%\bin\win64\vrpathreg" removedriver "%INSTALL_DIR%"

IF NOT DEFINED suppressPause (
  ECHO "Done"
  PAUSE
)
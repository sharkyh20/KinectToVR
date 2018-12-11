@ECHO off
CALL SteamVR_SetDriverVars.bat

ECHO "Registering KinectToVR SteamVR driver..."
"%STEAMVR_RUNTIME_DIR%\bin\win64\vrpathreg" adddriver "%INSTALL_DIR%"

IF NOT DEFINED suppressPause (
  ECHO "Done"
  PAUSE
)
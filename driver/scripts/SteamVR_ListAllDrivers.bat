@ECHO off
CALL SteamVR_SetDriverVars.bat

ECHO "Listing registered SteamVR drivers..."
"%STEAMVR_RUNTIME_DIR%\bin\win64\vrpathreg" show

ECHO "Done"
PAUSE
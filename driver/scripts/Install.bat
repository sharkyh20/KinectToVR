:: These scripts have been taken and modified from PSMoveSteamVRBridge - thank you to HipsterSloth!
@ECHO off
SETLOCAL
SET HOME=%~dp0

:: Don't pause at the end of the script if requested not to
SET suppressPause=0
IF "%1"=="-nopause" (
    SET /A suppressPause=suppressPause+1
)

::Try to find the SteamVR runtime path from %LOCALAPPDATA%\openvr\openvrpaths.vrpath
SET "FindRuntimeScriptPath=%HOME%SteamVR_FindRuntimePath.ps1"
FOR /f "delims=" %%a in ('powershell -NoProfile -ExecutionPolicy Bypass "& '%FindRuntimeScriptPath%'"') DO SET "STEAMVR_RUNTIME_DIR=%%a"
IF %ERRORLEVEL% NEQ 0 (goto browse_for_steam_folder)
IF DEFINED STEAMVR_RUNTIME_DIR (goto write_set_drivers_script)

::Fall back to user specified selection of the Steam folder
:browse_for_steam_folder
SET "findSteamVRCommand="(new-object -COM 'Shell.Application').BrowseForFolder(0,'Please select the root folder for steam (ex: c:\Program Files (x86)\steam).',0,0).self.path""
FOR /f "usebackq delims=" %%I in (`powershell %findSteamVRCommand%`) DO SET "STEAM_ROOT_PATH=%%I"
IF NOT DEFINED STEAM_ROOT_PATH (goto failure)

IF EXIST "%STEAM_ROOT_PATH%\steamapps\common\OpenVR" GOTO use_openvr
IF EXIST "%STEAM_ROOT_PATH%\steamapps\common\SteamVR" GOTO use_steamvr
GOTO no_steamvr_installed

:no_steamvr_installed
ECHO "No steamvr folder found at %STEAM_ROOT_PATH%! Please install steamvr."
GOTO failure

:use_openvr
SET STEAMVR_RUNTIME_DIR=%STEAM_ROOT_PATH%\steamapps\common\OpenVR
GOTO write_set_drivers_script

:use_steamvr
SET STEAMVR_RUNTIME_DIR=%STEAM_ROOT_PATH%\steamapps\common\SteamVR
GOTO write_set_drivers_script

:write_set_drivers_script
ECHO "Found SteamVR Runtime Dir: %STEAMVR_RUNTIME_DIR%"

PUSHD %~dp0..
SET DRIVER_ABS_PATH=%CD%
POPD

:: Write out the paths to a config batch file
DEL SteamVR_SetDriverVars.bat 2>NUL
ECHO @echo off >> SteamVR_SetDriverVars.bat
ECHO SET INSTALL_DIR=%DRIVER_ABS_PATH%>> SteamVR_SetDriverVars.bat
ECHO SET STEAMVR_RUNTIME_DIR=%STEAMVR_RUNTIME_DIR%>> SteamVR_SetDriverVars.bat

:: Suppress pause for child scripts
SET /A suppressPause=suppressPause+1

:: Unregister any existing K2VR drivers
CALL SteamVR_UnregisterDrivers.bat
IF %ERRORLEVEL% NEQ 0 (goto failure)

:: Register the new K2VR drivers
CALL SteamVR_RegisterDrivers.bat
IF %ERRORLEVEL% NEQ 0 (goto failure)

:: Enabled the activateMultipleDrivers flag in steamvr.vrsettings
CALL SteamVR_ActivateMultipleDrivers.bat
IF %ERRORLEVEL% NEQ 0 (goto failure)

:: Un-Suppress pause for child scripts
SET /A suppressPause=suppressPause-1

:: Pause before exit if pause isn't suppressed
ECHO "Installation successful."
IF %suppressPause% EQU 0 (
  PAUSE
)
GOTO exit

:failure
ECHO "Error occured during installation!"
IF %suppressPause% EQU 0 (
  PAUSE
)
EXIT /B %ERRORLEVEL%
GOTO exit

:exit
ENDLOCAL
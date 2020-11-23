## <ins>__[Discord server](https://discord.gg/YBQCRDG)__</ins> | <ins>__[Site](https://k2vr.tech/)__</ins>

![Build](https://github.com/KimihikoAkayasaki/KinectToVR/workflows/Build/badge.svg)

Automatic calibration scripts are written by **[コレヂャン](https://github.com/korejan)**<br>
(Also general help with just everything related to real programming)<br>
KinectToVR base is **[Sharky's](https://github.com/sharkyh20/)**<br>
Rest is probably written by **[公彦赤屋先](https://github.com/KimihikoAkayasaki)**<br>
**[Triping](https://github.com/TripingPC)** organizes the whole project.<br>

## License
This project is licensed under the GNU GPL v3 License 

## Build
You'll need:
 - Visual Studio 2019 (with: C++, v142 tools, ATL)<br>or just build tools for same (see GitHub Actions script)
 - Kinect SDK 1.8 & 2.0 installed and visible in PATH
 - Working installation of SteamVR for testing

Follow **[GitHub Actions script](https://github.com/KimihikoAkayasaki/KinectToVR/blob/master/.github/workflows/main.yml)**, or:<br>

- Clone Valve's ```OpenVR``` to ```external/``` (eventually remove ```-master``` from folder name)<br>
- Restore NuGet packages for ```VRInputEmulator``` and ```KinectToVR```
- Build ```lib_vrinputemulator``` (another solution in ```external/```) in x64/Release
- Build all in ```KinectToVR``` in x64/Release

## Deploy
Grab all needed files from your current KinecToVR installation folder.<br>
This also applies to OpenVR driver folders structure and files.

[![alt text](https://raytracing-benchmarks.are-really.cool/2BPWpjH.png "Go to k2vr.tech")](https://k2vr.tech)
## [Join the Discord](https://discord.gg/YBGCRDQ)<br><br>
# KinectToVR SFML Version
This repository contains the original source of KinectToVR from Version 0.1 to 0.8 prior to the Qt rewrite.

KinectToVR is an application for emulative the functionality of Vive trackers using skeleton tracking from a Kinect for Xbox 360 or Xbox One, or using tracked devices from PSMoveService.

![main window](https://raytracing-benchmarks.are-really.cool/3g4jtCf.png)

## Build
You'll need:
 - Visual Studio 2019 (with: C++, v142 tools, ATL)<br>or just build tools for same (see GitHub Actions script)
 - Kinect SDK 1.8 & 2.0 installed and visible in PATH

 Follow **[GitHub Actions script](https://github.com/KimihikoAkayasaki/KinectToVR/blob/master/.github/workflows/main.yml)**, or:<br>

- Clone Valve's [```OpenVR```](https://github.com/valvesoftware/openvr) to ```external/``` (eventually remove ```-master``` from folder name)<br>
- Restore NuGet packages for ```VRInputEmulator``` and ```KinectToVR```
- Build ```lib_vrinputemulator``` (another solution in ```external/```) in ```x64/Release``` (It's set to Debug by default)
- Build all in ```KinectToVR``` in ```x64/Release```

## Deploy
Retrieve the needed libraries either from your own KinectToVR installation folder or from the latest release<br>
This also applies to OpenVR driver folders structure and files.

## Credits
- [Sharkyh20](https://github.com/sharkyh20) Original developer
- [公彦赤屋先](https://github.com/KimihikoAkayasaki) Maintenance and updates after 0.6.0r2
- [TripingPC](https://github.com/TripingPC) Old installer, website and support
- [Himbeersaft](https://github.com/Himbeersaft) Installer and support
- [コレヂャン](https://github.com/korejan) Help with calibration maths

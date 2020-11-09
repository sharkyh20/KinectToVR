## __**[Discord server](https://discord.gg/YBQCRDG)**__

[![Build Status](https://travis-ci.com/KimihikoAkayasaki/KinectToVR.svg?branch=master)](https://travis-ci.com/KimihikoAkayasaki/KinectToVR)

Automatic calibration scripts are written by **[コレヂャン](https://github.com/korejan)**<br>
(Also general help with just everything related to real programming)<br>
KinectToVR base is **[Sharky's](https://github.com/sharkyh20/)**<br>
Rest is probably written by **[公彦赤屋先](https://github.com/KimihikoAkayasaki)**<br>
**[Triping](https://github.com/TripingPC)** organizes the whole project.<br>

## License
This project is licensed under the GNU GPL v3 License 

## Build
Follow **[travis script](https://github.com/KimihikoAkayasaki/KinectToVR/blob/master/.travis.yml)**, or:<br>

- Download boost 1.74 and extract it to ```/external/```, making path like ```/external/boost_1_74_0/boost/any.hpp``` available.<br>
- Download OpenVR and put it to ```/external/```, making path like ```/external/openvr/headers/openvr.h``` available.<br>
- Download cereal and extract it to ```/external/```, making path similar to ```/external/cereal/include/cereal/``` available.<br>
- Build in Release mode for x64 architecture.

## Deploy
Copy ```openvr_api.dll```, ```opencv_world341.dll``` to deploy directory,<br>
or grab pack of dlls from program directory created by the installer.

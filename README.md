# KinectToVR
An open-source hook for VRInputEmulator to enable foot and hip tracking with the Kinect.
It currently allows for the hips, left foot and right foot to be tracked with the skeletal positions from the Kinect. Their positions are updated to a virtual tracker from the OpenVR Input Emulator in order to translate it into VR games.


## Getting Started - Users
There is no installation required by this project itself, however, it requires the Kinect Runtime from Microsoft, and the InputEmulator from matzman666.

### Prerequisites

[The Kinect Runtime v1.8](https://www.microsoft.com/en-au/download/details.aspx?id=40277)

[The OpenVR InputEmulator .exe - (Latest tested version 1.3)](https://github.com/matzman666/OpenVR-InputEmulator/releases)

### Running the program
- You can use the Xbox 360 Kinect with an adapter [such as this one](https://www.amazon.com/Adapter-Kinect-360-HandHelditems-Sketch-Universal/dp/B005EIXVAE), or a regular Kinect for Windows.

1. Ensure that the runtime is installed, and the Kinect is plugged into your PC
  * If Windows won't detect it, you can try looking through Microsoft's troubleshooting [here](https://support.xbox.com/en-AU/xbox-on-windows/accessories/kinect-for-windows-v2-known-issues)
2. Point your Kinect facing away from the 'front' of your VR space (Usually facing you like your monitor.) *As of writing this, KinectToVR does not support a different Kinect orientation.*
1. Open SteamVR with your headset plugged in, and install InputEmulator
1. Run KinectToVR.exe, and 3 tracker devices should appear in the SteamVR device list
1. Put on your headset and stand in front of the Kinect. The optimal distance microsoft reccommends is about 1.5-2.5m away.
1. You may see a bunch of trackers floating behind you! Don't worry, you can initialise them to your current position by pressing down the right grip button.
1. Although the Kinect tries it's best to find your exact position, it's not always correct, so you can use the left stick to offset the trackers laterally on the ground, and the right stick to move them up and down to your body.
1. When you have set it to your desired position click down on the right trigger.
-NOTE: If you hit the trigger on accident before it reached your desired position, then you can press 'Q' on your keyboard to enable tracker adjusting again

Unfortunately, due to the limitations of the Kinect, it can only detect a skeleton head-on and it may jitter or get occluded fairly easily as it is only one sensor. This means that with tracking enabled, you're going to have to stand facing the Kinect, like the old-fashioned 2-sensor Oculus configuration. I can't really do anything about this limitation.

### Some useful keys while I get a UI to work
'A' - If the Kinect is working, it toggles the display of its camera

'S' - If the skeleton tracking is working, it will draw the tracked bones

# Known Issues/Fixes

### Greyed out trackers in SteamVR panel
Restart SteamVR, as far as I know I can't remove them from the panel, only disable them while its open.

### SteamVR error 308 "A component of SteamVR isn't working properly
1. Quit SteamVR

1. Use task manager to search for and kill *VR Server* background process

1. Restart SteamVR

## Getting Started - Developers

If you wish to compile the project yourself, then you'll need:

### SFML

[SFML v2.4.2](https://www.sfml-dev.org/download/sfml/2.4.2/)

### OpenVR

[OpenVR v1.0.12](https://github.com/ValveSoftware/openvr)

### Kinect Runtime  

[The Kinect Runtime v1.8](https://www.microsoft.com/en-au/download/details.aspx?id=40277)

### InputEmulator

[The OpenVR InputEmulator .exe - (Latest tested version 1.3)](https://github.com/matzman666/OpenVR-InputEmulator/releases)

[The OpenVR InputEmulator source - (Tested v1.3)](https://github.com/matzman666/OpenVR-InputEmulator)

### Kinect SDK:

[Kinect SDK v1.8](https://www.microsoft.com/en-us/download/details.aspx?id=40278)

The InputEmulator requires the boost library to use their library, so boost is also required.


### Boost(From the IE page)
1. Goto https://sourceforge.net/projects/boost/files/boost-binaries/1.63.0/
1. Download Boost 1.63 Binaries (boost_1_63_0-msvc-14.0-64.exe)
1. Install Boost into `OpenVR-InputEmulator/third-party/boost_1_63_0`
 
### Installing for the environment

The project was compiled in Visual Studio 2017.

To get the required headers for the InputEmulator, build the 'lib_vrinputemulator' project in 64-bit mode, and transfer the headers and .lib from:

```
(FILEPATH)OpenVR-InputEmulator-1.3\Release\lib\x64 -- Copy the contents
(FILEPATH)OpenVR-InputEmulator-1.3\Debug\lib\x64 -- Copy the contents

(FILEPATH)OpenVR-InputEmulator-1.3\lib_vrinputemulator\include\ - Copy the contents
```

To here:

```
(FILEPATH)\SFMLProject\SFMLProject\InputEmulator
```
#### NOTE: The debug library will need to be renamed in the new folder to 'libvrinputemulator_d.lib'

The OpenVR libraries and headers also need to be copied from their folder:

```
(FILEPATH)\openvr-master\bin\win64\Debug\ -- Copy the contents

(FILEPATH)\openvr-master\headers -- Copy the folder
```
And then pasted into the project folder:

```
(FILEPATH)\SFMLProject\SFMLProject\openvr\ -- paste the lib files and 'headers' folder here
```

The final contents of these two folders should look like this:

![Image](readmeimg/iefolder.PNG?raw=true)
![Image](readmeimg/ovrfolder.PNG?raw=true)

## Building

The project is meant to be built for 64bit computers only, so if you're using Visual Studio to compile, then near the top left of the screen change the mode to 'Release' or 'Debug', and set the 'solution platforms' to 'x64'.

![Image](readmeimg/buildmode.png?raw=true)

### NOTE

The compiled .exe will need to have the dll's for the dependencies included in it's output folder for it to work.

These include:

```
openal32.dll -- found at \SFML-2.4.2\extlibs\bin\x64

openvr_api64.dll -- found at \openvr-master\bin\win64\

-- All below found in \SFML-2.4.2\lib\Release\

sfml-audio-2.dll

sfml-graphics-2.dll

sfml-system-2.dll

sfml-window-2.dll

```

## Versioning

We use [SemVer](http://semver.org/) for versioning. For the versions available, see the [tags on this repository](https://github.com/your/project/tags). 

## Authors

* **sharkyh20** - *Initial work* - [sharkyh20](https://github.com/sharkyh20/)

## License

This project is licensed under the GPL v3 License - see the [LICENSE.md](LICENSE.md) file for details

## Acknowledgments

* matzman666 - for the VR InputEmulator that makes this possible
* zecbmo - [for his SkyrimVR project that helped me to understand VR inputs, and provided a code base](https://github.com/zecbmo/ViveSkyrim)
* Omnifinity - [for his project showing how to get HMD positions, and matrix math](https://github.com/Omnifinity/OpenVR-Tracking-Example/)
* Tons of stackoverflow posts and steam community pages that helped me learn how to glue this together

## Author's note
This was my first actual project and I've had a lot of 'fun' and fun getting it to work. I'm still learning C++ and programming concepts, so I feel that this has helped a great deal where reading from a book really can't. Thank you for being patient enough to read this all the way to the end and I hope my code is not too atrocious.

As always, feedback is appreciated so that I can learn and improve my skills.

Thanks, and have fun in VR!

# KinectToVR
An open-source hook for VRInputEmulator to enable foot and hip tracking with the Kinect.
It currently allows for the hips, left foot and right foot to be tracked with the skeletal positions from the Kinect. Their positions are updated to a virtual tracker from the OpenVR Input Emulator in order to translate it into VR games.

## Supporting the Project
After much urging from others, I now have a [donation jar here](https://digitaltipjar.com/sharkyh20), and a [paypal link here](https://www.paypal.me/sharkyh20)

If you'd prefer to donate to charity in my stead, I recommend [Doctors Without Borders](https://donate.doctorswithoutborders.org/onetime.cfm)

## For those who want video instructions [go here](https://www.youtube.com/playlist?list=PL9kBn2ECbDU_NFxcJRx7XXfJavCsVol_Y)

## *The Official Discord is [here](https://discord.gg/Mu28W4N) (Australian timezone, though)*

## If you're unfamiliar with github: [Go here to download](https://github.com/sharkyh20/KinectToVR/releases)

<img src="readmeimg/SkeletonDrawing.png?raw=true" width = 100%><img src="readmeimg/Weebadoo.jpg?raw=true"  width=50%><img src="readmeimg/WeebadooSitting.jpg?raw=true" width = 50%>



## Getting Started - Users
There is no installation required by this project itself, however, it requires the Kinect Runtime from Microsoft, and the InputEmulator from matzman666.

## Kinect Compatibility

| VR System | Xbox 360/V1 | Xbox One/V2|
|-------------|------------|------------|
| Oculus Rift | Yes | Yes|
| HTC Vive | Yes | With Adjustment* |
| Windows MR | Yes | Yes |

\* The Xbone sensor can't be facing directly into a base station, but it otherwise works flawlessly 
### Prerequisites

[Visual Studio Redist 2010 x64](https://www.microsoft.com/en-us/download/details.aspx?id=14632)

[Visual Studio Redist 2017 x64](https://go.microsoft.com/fwlink/?LinkId=746572)

SteamVR - NOT the beta branch, just the regular branch

#### IMPORTANT: If your Kinect is a 360/Xbone version, instead of a Windows version, then you will also need the corresponding SDK for it to detect it. As Microsoft prevents these from working outside of a 'development environment'.

For the Xbox 360 Kinect:
[The Kinect SDK v1.8](https://www.microsoft.com/en-us/download/details.aspx?id=40278)
	
For the Xbox One Kinect:
[The Kinect SDK v2.0](https://www.microsoft.com/en-au/download/details.aspx?id=44561)

[The OpenVR InputEmulator .exe - (Latest tested version 1.3)](https://github.com/matzman666/OpenVR-InputEmulator/releases)


### Running the program
 You can use the Xbox 360 Kinect with an adapter [such as this one](https://www.amazon.com/Adapter-Kinect-360-HandHelditems-Sketch-Universal/dp/B005EIXVAE), or a regular Kinect for Windows.

1. Ensure that the runtime is installed, and the Kinect is plugged into your PC.
  	* If Windows won't detect it, you can try looking through Microsoft's troubleshooting [here](https://support.xbox.com/en-AU/xbox-on-windows/accessories/kinect-for-windows-v2-known-issues)
2. Point your Kinect into your VR area
	* I recommend turning on the Draw Skeleton checkbox while adjusting your sensor position, so that you can visualise where it's tracking. Remember to turn it off, as it may cause a little lag.
1. Open SteamVR with your headset plugged in, and install InputEmulator. Additionally, make sure to install https://github.com/sharkyh20/OpenVR-InputEmulator/releases/tag/SteamVR-Fix as after November 2018, InputEmulator broke a little (A lot).
1. SteamVR Home blocks the trackers from appearing - you need to exit it so that you are in the Steam Grey void area to see them.
1. Run the corresponding process in the KinectToVR folder

	* Xbox 360 - KinectV1Process.exe
	
	* Xbox One - KinectV2Process.exe
	
1. Put on your headset and stand in front of the Kinect. The optimal distance Microsoft recommends is about 1.5-2.5m away.
1. You may see a bunch of trackers floating behind you! Don't worry, this means that the Kinect has detected your skeleton, but it's not at your current position.

	* If the trackers are stuck at the center position without moving, your Kinect is not detecting you. Stand further back, or do a little crouch. (You only need to do this to get it to recognise you, not while its running)

1. Although the Kinect tries it's best to find your exact position, it's not always correct.
	* You should see an arrow in the SteamVR space
	* Back in the K2VR process you should see a checkbox saying 'Enable Kinect Position Calibration', click it.
	* Use the thumbsticks/trackpad to move the arrow to where the Kinect is in real life.
	* Press the trigger to confirm
1. The trackers may not fit your position still, so you need to adjust the rotation, follow the same process with the rotation checkbox.

Unfortunately, due to the limitations of the Kinect, it can only detect a skeleton head-on and it may jitter or get occluded fairly easily as it is only one sensor. This means that with tracking enabled, you're going to have to stand facing the Kinect, like the old-fashioned 2-sensor Oculus configuration. I can't really do anything about this limitation. But the Xbox One suffers a lot less from this than the Xbox 360.

## If you are after the PSMoveService Instructions

# KinectToVR PSMove Beta Test Instructions (As of 0.6.0)

https://github.com/sharkyh20/KinectToVR/releases/tag/a0.6.0

### The PSEye camera is currently the only supported camera, due to this being the PSMoveService test. Kinect support will come when it is ready

### Again, just to reiterate: this is not a stable version, and is purely for testing - the process will likely be frustrating to setup if you aren't savvy with PSMoveService, and there are many tiny things that you need to ensure are working correctly. But when they *do* work correctly, 360 degree tracking baybee.

# Notes:

* Make sure you've applied the Input Emulator DLL fix from my fork of InputEmulator
* Use PSMoveService version https://github.com/cboulay/PSMoveService/releases/tag/v0.9-alpha9.0.1
* Follow through with the Setup of the PSMove stuff here: https://github.com/cboulay/PSMoveService/wiki
* Additionally, to spawn Virtual Trackers (Using coloured ping pong balls tracked by PSEyes): https://github.com/HipsterSloth/PSMoveSteamVRBridge/wiki/Virtual-Controller-Setup

Once all that is setup, and your SteamVR doesn't boot into Safe Mode and crash from other drivers (OVR Advanced Settings is the likely other culprit: updated version here: https://github.com/ykeara/OpenVR-AdvancedSettings/releases/tag/v2.7) Follow on to the next steps.

# To use the PSMoves with K2VR:
1. Open up the K2VR/driver/scripts directory
1. Run Install.bat - make sure it says it installed successfully
    * If you move the location of K2VR, make sure to Uninstall and reinstall it with the batch scripts
1. Have PSMoveService running, and all of your PSMove's connected with the cameras
1. Open KinectToVR's 'KinectLessProcess.exe (Doesn't require any kinect connected at all, and actually recommended if not using skeletal tracking)
1. Go to the Tracking Methods Tab, click 'Run PSMove Handler'
1. If it ran successfully, go to the Adv. Trackers tab, and select the PSMove's corresponding to your left foot position and rotation (The same PSMove ID), set it's role to 'Left Foot', and click Add.
1. Repeat this process for the right foot, and hip trackers respectively.
1. Go back to the main menu of K2VR and click the spawn tracker button.
1. Wear your headset. Hold any PSMove to the bottom of the front of your headset, with the buttons facing you, and look directly forwards in SteamVR. Hold the START + SELECT buttons, and it should be calibrated. Retry it as many times as you want until it feels right.

### VRChat

I'd assume the majority of people that want full body tracking are going to be using it in VRChat, so here is a list of steps to get it to work.

1. Make sure you've turned this program on, and the trackers are calibrated and set up in SteamVR.
1. Make sure both controllers are connected before launching VRChat, as sometimes it may not detect them if they are turned on after the game has opened.
1. While the game is loading, make sure you are facing your Kinect head-on, and keep your head facing that direction until the body spawns, if you wish to calibrate your last used avatar
	* VRChat spawns the avatar facing the direction you're looking at the time, but you cannot adjust it after you have spawned. If you don't remember to do this: face the right direction, select a different avatar and switch back.
1. If you want to use another avatar, once you are in the world, open the menu and go to 'Avatars'. Face forwards as in the previous step
1. From here you can select an avatar for your character, then click change in the bottom left.
1. Your character should be stuck in a T-Pose.
1. Move your feet to match with the position of the characters feet, hips with the hips, and your controllers to each of their hands.
1. When you are ready, hold down the trigger's and grips on both controllers and the tracking should be activated.

Full-body tracking is still fairly uncommon, and as such there's not much help or support if something goes wrong. Many character models may not work, or glitch out when full-body is activated. 

If the joints are weird, try crouching a little bit during calibration of the avatar, or lower your player height. There are way too many variables for me to list here, but that's usually the problem, unless the Kinect is improperly calibrated - in which case you repeat the 'Enable' checkboxes and align the arrow until your feet move 1:1 in VR.

#### NOTE: Sometimes this process may not work, whether it be because VRChat didn't recognise the trackers, or just didn't activate the T-Pose, if this happens, close VRChat and SteamVR and begin the process again.


#### If your character's knees are bent but the tracking still works, then you can adjust the player height in the 'System' menu to compensate, usually this means make it smaller.

#### [Here's a useful guide/reference for tracking related issues](https://www.reddit.com/r/VRchat/comments/7y879f/tutorial_a_guide_to_full_body_tracking_for_vive/)

# Known Issues/Fixes

### Vive tracking bugs out

The Kinect for Xbox One causes interference with the Vive, so it's recommended to not have the Kinect facing into a lighthouse, or at the height of the Vive headset. This isn't as bad as it sounds - as the Xbone Kinect can track at a much shorter distance and more reliably than the X360. 

### Greyed out trackers in SteamVR panel
Restart SteamVR, as far as I know I can't remove them from the panel, only disable them while its open.

### SteamVR error 308 "A component of SteamVR isn't working properly"
1. Quit SteamVR

1. Use task manager to search for and kill *VR Server* background process

1. Restart SteamVR

# Getting Started - Developers

If you wish to compile the project yourself, then you'll need:

### InputEmulator

[The OpenVR InputEmulator .exe - (Latest tested version 1.3)](https://github.com/matzman666/OpenVR-InputEmulator/releases)

### OpenCV (For the color tracking stuff)

[OpenCV 3.4.1](https://sourceforge.net/projects/opencvlibrary/files/opencv-win/3.4.1/opencv-3.4.1-vc14_vc15.exe/)

### Kinect SDK's:

[Kinect SDK v1.8](https://www.microsoft.com/en-us/download/details.aspx?id=40278)

[The Kinect SDK v2.0](https://www.microsoft.com/en-au/download/details.aspx?id=44561)

The InputEmulator requires the boost library to use their library, so boost is also required.


### Boost(From the IE page)
1. Goto https://sourceforge.net/projects/boost/files/boost-binaries/1.63.0/
1. Download Boost 1.63 Binaries (boost_1_63_0-msvc-14.0-64.exe)
1. Boost also needs to be added to the directory
```
Copy the whole boost_1_63_0\ to (PROJECTDIR)\external\boost_1_63_0\
```
### Installing for the environment

The project was compiled in Visual Studio 2017.

First off, the git commands:

`git clone -b master https://github.com/sharkyh20/KinectToVR/`

`git submodule update --init --recursive --progress`



## Building

The project is meant to be built for 64bit computers only, so if you're using Visual Studio to compile, then near the top left of the screen change the mode to 'Release' or 'Debug', and set the 'solution platforms' to 'x64'.

![Image](readmeimg/buildmode.png?raw=true)

### NOTE

The compiled .exe will need to have the dll's for the dependencies included in it's output folder for it to work.

Most are included in the dll's folder, and copied to the build directory automatically after building.

The large ones will have to be copied over into `external/dlls/`, in their respective `Debug` or `Release` folder depending on what the dll is intended for, these include:

```
// The opencv dll's:

opencv_world341.dll

opencv_ffmpeg341_64.dll

```

## Versioning

We use [SemVer](http://semver.org/) for versioning. For the versions available, see the [tags on this repository](https://github.com/sharkyh20/KinectToVR/tags). 

## Authors

* **sharkyh20** - *Initial work* - [sharkyh20](https://github.com/sharkyh20/)

* **naelstrof** - *Playspace Movement, General Improvements* - [naelstrof](https://github.com/naelstrof/)
* **DJ Lukis.LT** - *Color Tracking Help, Project Management* - [lukis101](https://github.com/lukis101)

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

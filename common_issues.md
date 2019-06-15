# Common Issues with KinectToVR (WIP)

Common questions/worries are listed below (in no particular order). References to common fixes/explanations will be explained away as SUB'letter' where letter is however many letters we get up to. See the Sub-answers section at the bottom of the questions to see what this note means.

A really common fix for a lot of issues with the Kinect V2 (as well as a few neato added features like using it as a native windows webcam) can be found in a driver update. Now, this is actually *extremely* obscure, and I could not find any info about it online. I only lucked into it by chance.
1. Go to device manager
1. Expand the drop down for Kinect Sensor Devices
1. Right click the WDF KinectSensor Interface -> Properties
1. Go to the Driver tab and click the Update Driver button. It should download and install it.
1. Restart your PC.

Your kinect should now be on the latest set of drivers - it should hopefully be more stable, and as a bonus you can now use it as a webcam! (This driver version may have already been installed, but it's unlikely, as mine wasn't, and I haven't heard much talk about the fact you can even update your Kinect's drivers online)

### Q: My Kinect keeps connecting and disconnecting
Ensure your kinect has an adequate power supply (1A is not enough, 1.5 *might* be okay, 2-3A should work), and is turned on and plugged into a USB port. The Kinect V2 *requires* a USB 3 port, with adequate bandwidth. (See *SUB-A*))

Right click the volume icon in the bottom right of your screen -> Sounds.

Go to the recording tab.

Right click your kinect microphone array and make sure it is *enabled*. Yes, it's weird. No, I can't do anything about it. Microsoft seems to have forced this behaviour.
### Q: My skeleton isn't showing in the window
Make sure that your kinect is connected, and isn't disconnecting and reconnecting.
It should be placed up high - around chest/neck height - and angled slightly downwards.
Turn on the Draw Skeleton option.
Stand as far back as you can from the kinect.
For your skeleton to latch on to you, it needs to see your entire body in the frame, often (especially for the Kinect V1) this means squatting down and waggling about until your skeleton shows up.

Other factors which can effect your skeleton include the information in *SUB-B.*


### Q: My controller input won't work in SteamVR after spawning trackers

### Q: My kinect doesn't connect at all

### Q: I keep getting missing Kinect20/Kinect10.dll files!

### Q: When I try to run K2VR it says sfml-graphics.dll is missing!

### Q: My avatar looks weird

### Q: No tracking balls are showing up in VRChat

### Q: My calibration arrow won't move!

### Q: I'm not in the discord server!

### Q: My calibration has gone weird, how do I reset it and start from scratch?

### Q: Input Emulator isn't connecting - "System file not specified or something"?

### Q: My headset is glitching out with the Kinect running

### Q: My tracking is super laggy

### Q: One of my questions isn't answered!

## Sub-Answers to avoid repetition
### SUB-A
So, it's possible that your PC can't support the Kinect right now.

But it's also possible there are many things going wrong.

This is more so a bandwidth problem and in 99% of cases this is going to be an issue with the Kinect V2 - as it has really high bandwidth requirements and Microsoft imposed restrictions (which make it even more intensive to run than Oculus cameras).

If you are using the V2, and you have the SDK installed, run 'SDK Browser v2.0' from your search bar, and then within that application, open the 'Kinect Configuration Verifier', which should tell you what's going wrong.

If you think you are having a USB issue do a simple isolation test: disconnect as many devices as possible, close all VR applications, and just see if the Kinect works. Then slowly add in your important USB devices (Headset, sensors if using a Rift), until it breaks.

If it doesn't work even just by itself, try all the other USB ports. And reinstall the SDK.

### SUB-B
Factors which may influence your Kinect tracking include:
* Sunlight/IR Interference - close all the blinds/curtains
* Reflective Surfaces - mirrors, photo frames, monitors - cover them
* Lack of USB bandwidth
* Vive Base Stations - don't point the kinect directly towards a base station
## Resource Links

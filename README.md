# KinectToVR
An open-source PARTIAL hook for VRInputEmulator to enable foot and hip tracking with the Kinect.   
It currently allows for the hips, left foot and right foot to be tracked with the skeletal positions from the Kinect.   
Their positions are updated to a driver, that run soft-trackers: it's somehow better solution that IE, 
but it still needs some work and general improvments.   
Also, now K2VR can track your head! (using mod inputemulator dll)

# Where has the dev been?
[->Watching this video shall explain all.<-](https://www.youtube.com/watch?v=6ZwE7Bl7hbc)

## Getting Started - Users
I think 'normal' users will not use exactly this release,    
but if you like, unpack everything and run register batch file.   
Then you need to replace your ie dll with that provided in release.    
(ONLY needed when you plan to use head tracking)

## *The Official Discord is [here](https://discord.gg/Mu28W4N)

## Supporting the Project
If you'd prefer to donate to charity in my stead, I recommend [Doctors Without Borders](https://donate.doctorswithoutborders.org/onetime.cfm)

## (OUTDATED) ~~For those who want video instructions [go here](https://www.youtube.com/playlist?list=PL9kBn2ECbDU_NFxcJRx7XXfJavCsVol_Y)~~

<img src="readmeimg/SkeletonDrawing.png?raw=true" width = 100%><img src="readmeimg/Weebadoo.jpg?raw=true"  width=50%><img src="readmeimg/WeebadooSitting.jpg?raw=true" width = 50%>

## Versioning

We use [SemVer](http://semver.org/) for versioning. For the versions available, see the [tags on this repository](https://github.com/sharkyh20/KinectToVR/tags). 

## Authors

* **sharkyh20** - *Initial work* - [sharkyh20](https://github.com/sharkyh20/)

* **naelstrof** - *Playspace Movement, General Improvements* - [naelstrof](https://github.com/naelstrof/)
* **DJ Lukis.LT** - *Color Tracking Help, Project Management* - [lukis101](https://github.com/lukis101)

## License

~~This project is licensed under the GPL v3 License - see the [LICENSE.md](LICENSE.md) file for details~~     
KinectToVR is based on GPL v3, so you need to follow it.     
Although, all changes that i made, go to CreativeCommons.    

## Acknowledgments
* TripingPC for running the community in my absence and providing updated guides to get it running in 2020
* matzman666 - for the VR InputEmulator that makes this possible
* zecbmo - [for his SkyrimVR project that helped me to understand VR inputs, and provided a code base](https://github.com/zecbmo/ViveSkyrim)
* Omnifinity - [for his project showing how to get HMD positions, and matrix math](https://github.com/Omnifinity/OpenVR-Tracking-Example/)
* Tons of stackoverflow posts and steam community pages that helped me learn how to glue this together

## First author's note
This was my first actual project and I've had a lot of 'fun' and fun getting it to work. I'm still learning C++ and programming concepts, so I feel that this has helped a great deal where reading from a book really can't. Thank you for being patient enough to read this all the way to the end and I hope my code is not too atrocious.

As always, feedback is appreciated so that I can learn and improve my skills.

Thanks, and have fun in VR!

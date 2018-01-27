# KinectToVR
An open-source hook for VRInputEmulator to enable foot and hip tracking with the Kinect.
It currently allows for the hips, left foot and right foot to be tracked with the skeletal positions from the Kinect. Their positions are updated to a virtual tracker from the OpenVR Input Emulator in order to translate it into VR games.


## Getting Started - Users
There is no installation required by this project itself, however, it requires the Kinect Runtime from Microsoft, and the InputEmulator from matzman666.

### Prerequisites

[The Kinect Runtime v1.8](https://www.microsoft.com/en-au/download/details.aspx?id=40277)

[The OpenVR InputEmulator - (Latest tested version 1.3)](https://github.com/matzman666/OpenVR-InputEmulator)

## Getting Started - Developers

If you wish to compile the project yourself, then you'll need:

### SFML

[SFML v2.4.2](https://www.sfml-dev.org/download/sfml/2.4.2/)

### OpenVR

[OpenVR v1.0.12](https://github.com/ValveSoftware/openvr)

### Kinect Runtime  

[The Kinect Runtime v1.8](https://www.microsoft.com/en-au/download/details.aspx?id=40277)

### InputEmulator

[The OpenVR InputEmulator - (Tested v1.3)](https://github.com/matzman666/OpenVR-InputEmulator)

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

## Built With

* [Dropwizard](http://www.dropwizard.io/1.0.2/docs/) - The web framework used
* [Maven](https://maven.apache.org/) - Dependency Management
* [ROME](https://rometools.github.io/rome/) - Used to generate RSS Feeds

## Contributing

Please read [CONTRIBUTING.md](https://gist.github.com/PurpleBooth/b24679402957c63ec426) for details on our code of conduct, and the process for submitting pull requests to us.

## Versioning

We use [SemVer](http://semver.org/) for versioning. For the versions available, see the [tags on this repository](https://github.com/your/project/tags). 

## Authors

* **sharkyh20** - *Initial work* - [PurpleBooth](https://github.com/sharkyh20/)

## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details

## Acknowledgments

* matzman666 - for the VR InputEmulator that makes this possible
* zecbmo - [for his SkyrimVR project that helped me to understand VR inputs, and provided a code base](https://github.com/zecbmo/ViveSkyrim)
* Omnifinity - [for his project showing how to get HMD positions, and matrix math](https://github.com/Omnifinity/OpenVR-Tracking-Example/)
* Tons of stackoverflow posts and steam community pages that helped me learn how to glue this together

## Author's note
This was my first actual project and I've had a lot of 'fun' and fun getting it to work. I'm still learning C++ and programming concepts, so I feel that this has helped a great deal where reading from a book really can't. Thank you for being patient enough to read this all the way to the end and I hope my code is not too atrocious.

As always, feedback is appreciated so that I can learn and improve my skills.

Thanks, and have fun in VR!

Tracking Project
================

About
-----

This project provides a generic platform for performing monocular and stereo
tracking of 3D objects in video. It provides interfaces to do video/image loading,
image detection/classification and model fitting within a single framework. Currently
built in are pixel-wise detection methods (Random Forest, Naive Bayes and SVM) and a 
level set based pose estimation method. The system also supports temporal tracking. In all 
cases overloading base classes with your own algorithms for pose estimation/detection etc will 
allow extension of the project. Models are represented as 3D meshes and can be rigid or articulated with 
simple tree models.

Dependencies
------------

In all cases these are not strict minimum versions but merely the oldest version on which they have been tested to work.

* [OpenCV v2.3.1](http://opencv.org/downloads.html) 
* [Boost v1.48.0](http://www.boost.org/users/download/)
* [Cinder v0.8.5](https://github.com/cinder/Cinder)
* [Image](https://github.com/maximilianallan/image)
* [Quaternion](https://github.com/maximilianallan/quaternion)

Install Guide
-------------

Clone the Image and Quaternion header repositories to the deps directory.
Eventually I will get around to properly learning CMake. Until then...

* Linux - Grab the makefile from the build directory and copy to the
root directory. Edit path to dependencies. Build with Make.   

* Windows - Use Visual Studio solution in the build directory.

* OSX - Code is not designed to support OSX. No plans to add support in the future.


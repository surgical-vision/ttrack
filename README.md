Tracking Project
================

About
-----

This project is a monocular/stereo tracking system which detects and
localises 3D objects in video images. The system
incorporates a Random Forest classifier and a level-set based
segmentation system to determine pose in 3D dimenesions. The system is
currently being developed to tracking MIS instruments but can easily
be extended to track any rigid object.

Dependencies
------------

* [OpenCV v2.3.1](http://opencv.org/downloads.html) or higher 
* [Boost v1.48.0](http://www.boost.org/users/download/) but will probably work on older versions of boost too.
* [Image](https://github.com/maximilianallan/image)

Install Guide
-------------

* Linux - Grab the makefile from the build directory and copy to the
root directory. Edit path to boost/opencv if necessary. Build with Make.   

* Windows - Use Visual Studio solution in the build directory.

* OSX - Code is not designed to support OSX. No plans to add support in the future.


Tracking Project
================

About
-----

This project provides a generic platform for performing monocular and stereo
tracking of 3D objects in video. It provides interfaces to do video/image loading,
image detection/classification and model fitting within a single framework. 

Currently built in are pixel-wise detection methods and a level set based pose estimation method. The system 
also supports basic temporal tracking. In all cases overloading base classes with your own algorithms for pose 
estimation/detection etc will allow extension of the project. Models are represented as .obj and can be rigid or articulated with 
simple tree models.

Dependencies
------------

In all cases these are not strict minimum versions but merely the oldest version on which they have been tested to work.

* [OpenCV v2.3.1](http://opencv.org/downloads.html) 
* [Boost v1.48.0](http://www.boost.org/users/download/)
* [Cinder v0.8.5](https://github.com/cinder/Cinder)

Install Guide
-------------

CMake.


Contributors
------------

* Max Allan
* Ping-Lin Chang

If you make use of this code, please cite the following paper

M Allan, S. Thompson, M. Clarkson, S. Ourselin, D. Hawkes, J. Kelly and D. Stoyanov, 2D-3D Pose Tracking of Rigid Instruments in Minimally Invasive Surgery, IPCAI 2014
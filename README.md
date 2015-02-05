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


Usage
-----

Right now the core algorithm works as follows: frames are loaded using any class that inherits from the Handler interface and passed (via the single TTrack manager) to the detector. There features which are to be used for pose estimation are extracted, right now this is a pixel-wise classification for region based pose estimation although any other type of feature is possible. The TTrack manager then passes this detected frame to the tracker (which inherits from the Tracker interface). This defines a custom initialization function which allows the first pose to be roughly estimated before a more precise pose localizer (which inherits from Localizer) focusses on refinement. 

Trackable objects are represented by subclassing the Model class and are loaded via a .json file which defines a possibly articulated structure where the standard tree-like model is used. Each node of the tree (from root to children to children-of-children) represents a single rigid body transform from its parent and possibly also some geometry (in mesh format) and texture (mtl or gl textures). It is useful to not make geometry mandatory as some robotic manipulators define their structure this way. Right now the example file is for a robotic instrument so the transforms are defined with DH paramters (using subclass DenavitHartenbergArticulatedModel) although it should be easy to subclass Model to handle SE3 or another parameterization.

Contributors
------------

* Max Allan
* Ping-Lin Chang

If you make use of this code, please cite the following paper

M Allan, S. Thompson, M. Clarkson, S. Ourselin, D. Hawkes, J. Kelly and D. Stoyanov, 2D-3D Pose Tracking of Rigid Instruments in Minimally Invasive Surgery, IPCAI 2014
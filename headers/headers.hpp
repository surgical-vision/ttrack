#ifndef _HEADERS_H_
#define _HEADERS_H_

#include <cv.h>
#include <cxcore.h>
#include <highgui.h>
#if defined(_WIN32) || defined(_WIN64)
  #define _CRTDBG_MAP_ALLOC
  #include <stdlib.h>
  #include <crtdbg.h>
#endif
#include <boost/shared_ptr.hpp>
#include <boost/timer.hpp>
#include <boost/progress.hpp>
#include <iostream>

#ifndef DEBUG
  #define DEBUG
#endif

#define SAVEDEBUG
  
#endif //_HEADERS_H_

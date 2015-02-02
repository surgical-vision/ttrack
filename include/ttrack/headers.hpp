#ifndef _HEADERS_H_
#define _HEADERS_H_

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui_c.h>

#if defined(_WIN32) || defined(_WIN64)
  #define _CRTDBG_MAP_ALLOC
  #include <stdlib.h>
  #include <crtdbg.h>
#endif

#include <boost/shared_ptr.hpp>
#include <boost/timer.hpp>
#include <boost/progress.hpp>
#include <iostream>
#include <boost/thread/thread.hpp>
#include <boost/filesystem.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>
#include <boost/system/error_code.hpp>
#include <boost/ref.hpp>
#include <boost/math/special_functions/fpclassify.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/functional/hash.hpp>
#include <boost/math/quaternion.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#endif //_HEADERS_H_

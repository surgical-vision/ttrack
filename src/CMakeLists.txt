# HEADERS - contains a list of our application header files
# SOURCES - contains a list of our application src files
# LIB_INC_DIR - list of dirs we have header files in 
# USER_INC - all of the external library header directories
# LINK_LIBS - same as USER_INC but for the built libs

#######################################################
## Library sources

set( INCDIR "../include/ttrack")
set( BINARY_NAME "ttrack")
set( MAIN_FILE "ttrack_app.cpp" )

## Header only includes 
set(
  HEADERS
  ${INCDIR}/constants.hpp 
  ${INCDIR}/headers.hpp
  ${INCDIR}/resources.hpp 
  ${INCDIR}/ttrack.hpp
  ${INCDIR}/utils/camera.hpp 
  ${INCDIR}/ttrack_app.hpp 
  ${INCDIR}/detect/baseclassifier.hpp
  ${INCDIR}/detect/detect.hpp
  ${INCDIR}/detect/histogram.hpp
  ${INCDIR}/detect/im_mask_set.hpp
  ${INCDIR}/detect/multiclass_randomforest.hpp
  ${INCDIR}/detect/online_forest.hpp
  ${INCDIR}/detect/randomforest.hpp
  ${INCDIR}/detect/supportvectormachine.hpp
  ${INCDIR}/utils/config_reader.hpp 
  ${INCDIR}/utils/exceptions.hpp
  ${INCDIR}/utils/handler.hpp 
  ${INCDIR}/utils/helpers.hpp
  ${INCDIR}/utils/nd_image.hpp 
  ${INCDIR}/utils/plotter.hpp
  ${INCDIR}/utils/UI.hpp
  ${INCDIR}/utils/image.hpp
  ${INCDIR}/utils/sub_window.hpp
  ${INCDIR}/track/model/pose.hpp 
  ${INCDIR}/track/model/articulated_model.hpp
  ${INCDIR}/track/model/dh_helpers.hpp
  ${INCDIR}/track/model/node.hpp 
  ${INCDIR}/track/model/model.hpp
  ${INCDIR}/track/tracker/tracker.hpp 
  ${INCDIR}/track/tracker/monocular_tool_tracker.hpp
  ${INCDIR}/track/tracker/stereo_tool_tracker.hpp 
  ${INCDIR}/track/tracker/surgical_tool_tracker.hpp
  ${INCDIR}/track/localizer/localizer.hpp
  ${INCDIR}/track/localizer/levelsets/comp_ls.hpp 
  ${INCDIR}/track/localizer/levelsets/mono_pwp3d.hpp
  ${INCDIR}/track/localizer/levelsets/pwp3d.hpp 
  ${INCDIR}/track/localizer/levelsets/stereo_pwp3d.hpp 
  ${INCDIR}/track/localizer/levelsets/articulated_level_set.hpp
  ${INCDIR}/track/localizer/levelsets/level_set_forest.hpp
  ${INCDIR}/track/localizer/features/feature_localizer.hpp
  ${INCDIR}/track/localizer/features/register_points.hpp
  ${INCDIR}/track/localizer/features/lk_tracker.hpp
  ${INCDIR}/track/localizer/features/descriptor.hpp
  ${INCDIR}/track/temporal/temporal.hpp
  )


## Store list of source files
set( 
  SOURCES 
  ttrack.cpp 
  ttrack_app.cpp 
  detect/baseclassifier.cpp 
  detect/detect.cpp
  detect/histogram.cpp 
  detect/im_mask_set.cpp
  detect/multiclass_randomforest.cpp 
  detect/randomforest.cpp
  detect/supportvectormachine.cpp
  utils/camera.cpp 
  utils/handler.cpp 
  utils/helpers.cpp 
  utils/nd_image.cpp 
  utils/plotter.cpp 
  utils/sub_window.cpp
  track/tracker/tracker.cpp 
  track/tracker/monocular_tool_tracker.cpp 
  track/tracker/stereo_tool_tracker.cpp 
  track/tracker/surgical_tool_tracker.cpp
  track/model/model.cpp 
  track/model/node.cpp 
  track/model/pose.cpp 
  track/model/articulated_model.cpp
  track/localizer/levelsets/comp_ls.cpp    
  track/localizer/levelsets/mono_pwp3d.cpp 
  track/localizer/features/register_points.cpp 
  track/localizer/levelsets/stereo_pwp3d.cpp 
  track/localizer/levelsets/pwp3d.cpp
  track/localizer/features/feature_localizer.cpp
  track/localizer/features/descriptor.cpp
  track/localizer/features/lk_tracker.cpp
  track/localizer/levelsets/articulated_level_set.cpp
  track/localizer/levelsets/level_set_forest.cpp
  track/temporal/temporal.cpp
  
  )


#######################################################
## User build options

# Project headers trump everything 
list(APPEND LIB_INC_DIR  "${PROJECT_SOURCE_DIR}/include;${CMAKE_CURRENT_BINARY_DIR}/include" )

# hack remove later
if("${CMAKE_SIZEOF_VOID_P}" EQUAL "8")  

  list(APPEND LIB_INC_DIR "C:/Users/max/scratch/Python2.7-x86-64/include/")
  list(APPEND LINK_LIBS "C:/Users/max/scratch/Python2.7-x86-64/libs/python27.lib")
  
else("${CMAKE_SIZEOF_VOID_P}" EQUAL "8")

  list(APPEND LIB_INC_DIR "c:/Python27/include/")
  list(APPEND LINK_LIBS "c:/Python27/libs/python27.lib")

endif("${CMAKE_SIZEOF_VOID_P}" EQUAL "8")

list(APPEND SOURCES "C:/sdks/dlib-18.18/dlib-18.18/dlib/all/source.cpp")
list(APPEND LIB_INC_DIR "C:/sdks/dlib-18.18/dlib-18.18/")

#MathGL
option(WITH_MATHGL2 "Use MathGL for graph plotting" OFF)
if(WITH_MATHGL2)
  find_package(MathGL2 REQUIRED)
  if(MATHGL2_FOUND)
    
    add_definitions(-DUSE_MATHGL)
    set(USE_MATHGL 1)
    message(STATUS "MathGL2 Found and Enabled")
        
    list(APPEND USER_INC ${MATHGL2_INCLUDE_DIRS})
   	list(APPEND LINK_LIBS ${MATHGL2_LIBRARIES})
    
  endif()
endif()

#Cuda
option(WITH_CUDA "Use CUDA optimized functions" OFF)
if(WITH_CUDA)
  find_package(CUDA REQUIRED QUIET)
  if(CUDA_FOUND)
    
    add_definitions(-DUSE_CUDA)
  	set(USE_CUDA 1)
	  set(CUDA_NVCC_FLAGS "-arch sm_20")

	  add_definitions(-DCUDA_BUILD)
	  message(STATUS "CUDA Found and Enabled")

    string(FIND "${EXTERNAL_INC_DIRS}" "${CUDA_INCLUDE_DIRS}" HAS_INCLUDED REVERSE)
    if(HAS_INCLUDED LESS 0 OR HAS_INCLUDED EQUAL 0)
    		list(APPEND USER_INC ${CUDA_INCLUDE_DIRS})
   	    list(APPEND LINK_LIBS ${CUDA_LIBRARIES})
  	endif()
    
  endif()
endif()

if(USE_CUDA)
  list(APPEND HEADERS ${INCDIR}/track/localizer/levelsets/pwp3d_cuda.hpp ${INCDIR}/track/localizer/levelsets/comp_ls_cuda.hpp)
	list(APPEND SOURCES track/localizer/levelsets/pwp3d.cu track/localizer/levelsets/comp_ls.cu)
endif()

#Boost
#boost setting must go before find_package
set(Boost_USE_STATIC_LIBS ON) 
set(Boost_USE_MULTITHREADED ON)  
set(Boost_USE_STATIC_RUNTIME OFF)
#set(Boost_BOOST_ALL_DYN_LINK)
find_package(Boost REQUIRED COMPONENTS regex chrono date_time system filesystem thread python)
if(Boost_FOUND)
    list(APPEND USER_INC "${Boost_INCLUDE_DIRS}")
    list(APPEND LINK_LIBS "${Boost_LIBRARIES}")
endif()

find_package(OpenCV REQUIRED)
list(APPEND USER_INC "${OpenCV_INCLUDE_DIRS}" )
list(APPEND LINK_LIBS "${OpenCV_LIBRARIES}" )

find_package(Cinder REQUIRED)
list(APPEND USER_INC "${CINDER_INCLUDE_DIRS}" )
list(APPEND LINK_LIBS "${CINDER_LIBRARIES}" )

option(WITH_CERES "Use Ceres based optimization" OFF)
if(WITH_CERES)
  add_definitions(-DUSE_CERES) 
  find_package(Ceres REQUIRED)
  list(APPEND USER_INC "${CERES_INCLUDE_DIRS}")
  list(APPEND LINK_LIBS "${CERES_LIBRARIES}")
endif()

#######################################################
## Add Libraries / Include Directories / Link directories
include_directories( ${LIB_INC_DIR} )
include_directories( ${USER_INC} )
link_directories( ${USER_LIB} )

if(USE_CUDA)
if(_WIN_)
cuda_add_executable(${BINARY_NAME} "../resources/Resources.rc" ${MAIN_FILE} ${SOURCES} ${HEADERS} )
set_property(TARGET ${BINARY_NAME} PROPERTY WIN32_EXECUTABLE TRUE)
else(_WIN_)
cuda_add_executable(${BINARY_NAME} ${MAIN_FILE} ${SOURCES} ${HEADERS} )
endif()
else(USE_CUDA)
if(_WIN_)
add_executable(${BINARY_NAME} WIN32 "../resources/Resources.rc" ${MAIN_FILE} ${SOURCES} ${HEADERS} )
else(_WIN_)
add_executable(${BINARY_NAME} "../resources/Resources.rc" ${MAIN_FILE} ${SOURCES} ${HEADERS} )
endif(_WIN_)
endif()



target_link_libraries(${BINARY_NAME} ${LINK_LIBS})




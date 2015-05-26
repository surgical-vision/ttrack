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
  ${INCDIR}/track/localizer/levelsets/articulated_component_level_set.hpp  
  ${INCDIR}/track/localizer/levelsets/articulated_solver.hpp
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
  track/localizer/levelsets/articulated_solver.cpp
  track/localizer/features/feature_localizer.cpp
  track/localizer/features/descriptor.cpp
  track/localizer/features/lk_tracker.cpp
  track/localizer/levelsets/articulated_level_set.cpp
  track/localizer/levelsets/articulated_component_level_set.cpp
  track/temporal/temporal.cpp
  
  )


#######################################################
## User build options

# Project headers trump everything 
list(APPEND LIB_INC_DIR  "${PROJECT_SOURCE_DIR}/include;${CMAKE_CURRENT_BINARY_DIR}/include" )

#Boost
set(Boost_USE_STATIC_LIBS OFF) 
set(Boost_USE_MULTITHREADED ON)  
set(Boost_USE_STATIC_RUNTIME OFF) 
find_package(Boost REQUIRED COMPONENTS regex chrono date_time system filesystem thread) 

if(Boost_FOUND)
    list(APPEND USER_INC "${Boost_INCLUDE_DIRS}")
    list(APPEND LINK_LIBS "${Boost_LIBRARIES}")
endif()
#set(Boost_USE_STATIC_LIBS OFF) # only find static libs
#set(Boost_USE_MULTITHREADED ON)
#set(Boost_USE_STATIC_RUNTIME OFF)
#set(BOOST_ALL_DYN_LINK ON)
#find_package(Boost REQUIRED QUIET)
#list(APPEND USER_INC  "${Boost_INCLUDE_DIRS}" )
#list(APPEND LINK_LIBS "${Boost_LIBRARIES}" )
#list(APPEND USER_LIB "${Boost_LIBRARY_DIR}" )

find_package(OpenCV REQUIRED)
list(APPEND USER_INC "${OpenCV_INCLUDE_DIRS}" )
list(APPEND LINK_LIBS "${OpenCV_LIBRARIES}" )

MESSAGE(${OpenCV_LIBRARIES})
MESSAGE(${OpenCV_LIB_DIR})

find_package(Cinder REQUIRED)
list(APPEND USER_INC "${CINDER_INCLUDE_DIRS}" )
list(APPEND LINK_LIBS "${CINDER_LIBRARIES}" )

#find_package(Eigen3 REQUIRED)
#list(APPEND USER_INC "${EIGEN3_INCLUDE_DIR}")

find_package(Ceres REQUIRED)
list(APPEND USER_INC "${CERES_INCLUDE_DIRS}")
list(APPEND LINK_LIBS "${CERES_LIBRARIES}")

#######################################################
## Add Libraries / Include Directories / Link directories

include_directories( ${LIB_INC_DIR} )
include_directories( ${USER_INC} )
link_directories( ${USER_LIB} )

#if(_WIN_)
add_executable(${BINARY_NAME} WIN32 "../resources/Resources.rc" ${MAIN_FILE} ${SOURCES} ${HEADERS} )
#else
#add_executable(${BINARY_NAME} ${MAIN_FILE} ${SOURCES} ${HEADERS} )
#endif


target_link_libraries(${BINARY_NAME} ${LINK_LIBS})




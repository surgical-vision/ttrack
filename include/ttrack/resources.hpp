#ifndef __RES_HPP__
#define __RES_HPP__
#include "cinder/CinderResources.h"

#define RES_SHADER_FRAG		CINDER_RESOURCE( ../resources/, phong_frag.glsl, 128, GLSL )
#define RES_SHADER_VERT		CINDER_RESOURCE( ../resources/, phong_vert.glsl, 129, GLSL )

#define PWP3D_FRONT_DEPTH_FRAG  CINDER_RESOURCE(../resources/, pwp3d_front_depth_frag.glsl, 130, GLSL)
#define PWP3D_FRONT_DEPTH_VERT  CINDER_RESOURCE(../resources/, pwp3d_front_depth_vert.glsl, 131, GLSL)
#define PWP3D_BACK_DEPTH_AND_CONTOUR_FRAG  CINDER_RESOURCE(../resources/, pwp3d_back_depth_contour_frag.glsl, 132, GLSL)
#define PWP3D_BACK_DEPTH_AND_CONTOUR_VERT  CINDER_RESOURCE(../resources/, pwp3d_back_depth_contour_vert.glsl, 133, GLSL)
#define COMP_LS_BACK_DEPTH_AND_CONTOUR_FRAG  CINDER_RESOURCE(../resources/, comp_ls_back_depth_contour_frag.glsl, 134, GLSL)
#define COMP_LS_BACK_DEPTH_AND_CONTOUR_VERT  CINDER_RESOURCE(../resources/, comp_ls_back_depth_contour_vert.glsl, 135, GLSL)
#define HELPER_WIN    CINDER_RESOURCE( ../resources/, start.png, 136, IMAGE )

#endif



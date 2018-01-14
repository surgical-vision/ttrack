#ifndef __GPU_COMP_LS_HPP__
#define __GPU_COMP_LS_HPP__

#include <cuda.h>

namespace ttrk {

  namespace gpu {


    struct CudaImage {


      float *data;
      

    };


    class ComponentLevelSet {

    public:
      
      virtual void TrackTargetInFrame(float *model_parameters, unsigned char *image, float *classification_map);

    protected:

      unsigned char *image_data;
      float *classification_map_data;

    };
  }




}



#endif 
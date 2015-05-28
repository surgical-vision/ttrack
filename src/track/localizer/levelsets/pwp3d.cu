//#include <cuda.h>

#include "../../../../include/ttrack/track/localizer/levelsets/pwp3d_cuda.hpp"

__global__ void testCudaFunction(){




}


void testCudaFunctionCPP(){

  testCudaFunction << <1, 1 >> >();

}
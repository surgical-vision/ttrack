#pragma once

#include <cuda.h>

namespace ttrk {

  namespace gpu{

    bool checkCudaFunctionality();

    void computeJacobiansForEye();

  }

}



//__global__ void testCudaFunction();

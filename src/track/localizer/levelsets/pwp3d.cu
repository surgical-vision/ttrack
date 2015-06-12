#include "../../../../include/ttrack/track/localizer/levelsets/pwp3d_cuda.hpp"

__global__ void testCudaFunction(int *a, int *b, int *c){

  *c = *a + *b;
  //data[0] = 4;

}


bool ttrk::gpu::checkCudaFunctionality(){

  int ct;
  cudaGetDeviceCount(&ct);
  if (ct == 0){
    return false;
  }

  cudaError_t code = cudaGetLastError();
  
  for (int dev = 0; dev < ct; ++dev){
    cudaDeviceProp prop;
    cudaGetDeviceProperties(&prop, dev);
  }

  return true;
  //
  //int a, b, c;
  //a = 4;
  //b = 6;

  //int *a_d, *b_d, *c_d;
  //cudaMalloc((void **)&a_d, sizeof(int));
  //cudaMalloc((void **)&b_d, sizeof(int));
  //cudaMalloc((void **)&c_d, sizeof(int));
  //cudaMemcpy(a_d, &a, sizeof(int), cudaMemcpyHostToDevice);
  //cudaMemcpy(b_d, &b, sizeof(int), cudaMemcpyHostToDevice);

  //testCudaFunction<<<1,1>>>(a_d, b_d, c_d);
  //cudaMemcpy(&c, c_d, sizeof(int), cudaMemcpyDeviceToHost);
  //code = cudaGetLastError();
  //
  //cudaMemcpy(&c, c_d, sizeof(int), cudaMemcpyDeviceToHost);
  //cudaFree(a_d);
  //cudaFree(b_d);
  //cudaFree(c_d);
  ////ci::app::console() << "P = " << p << std::endl;
  //int j = 0;
  //int x = j + 3;

}
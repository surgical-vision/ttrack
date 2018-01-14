#include "../../../../include/ttrack/track/localizer/levelsets/pwp3d_cuda.hpp"


#include <stdint.h>
#include <cinder/app/App.h>


//initialise the float image to INF along the contour and zero everywhere else
__global__ void initImage(float *distance_transform_image, unsigned char *raw_contour_image, const uint32_t width, const uint32_t height){

  uint32_t x = blockIdx.x * blockDim.x + threadIdx.x;
  uint32_t y = blockIdx.y * blockDim.y + threadIdx.y;

  if (x < width && y < height){
    distance_transform_image[x + y*width] = raw_contour_image[x + y*width] == 255 ? 0 : +INF;
  }

}

__global__ void cudaDistanceTransformRows(float *distance_transform, int *v, float *z, float *output, const uint32_t width, const uint32_t height){

  int row_idx = blockIdx.x * blockDim.x + threadIdx.x;

  if (row_idx < height){

    int k = 0;
    v[row_idx*width] = 0;
    z[row_idx*(width + 1)] = -INF;
    z[row_idx*(width + 1) + 1] = +INF;

    float s;

#pragma unroll
    for (int q = 1; q < width; q++){
      //get a row index for this loop
      int q_idx = q + row_idx*width;

      int vk = v[k + (row_idx*width)];

      s = ((distance_transform[q_idx] + q*q) - (distance_transform[vk + (row_idx*width)] + vk*vk)) / ((2 * q) - (2 * vk));

      while (s <= z[k + row_idx*(width + 1)]){ //z is 1 row and 1 col wider

        k--;

        int vk = v[k + row_idx*width];
        s = ((distance_transform[q_idx] + (q*q)) - (distance_transform[vk + (row_idx*width)] + (vk*vk))) / ((2 * q) - (2 * vk));

      }

      k++;
      v[k + row_idx*width] = q;
      z[k + row_idx*(width + 1)] = s;
      z[(k + 1) + (row_idx)*(width + 1)] = +INF;

    }

    k = 0;
#pragma unroll
    for (int q = 0; q < width; q++){

      while (z[(k + 1) + row_idx*(width + 1)] < q)
        k++;

      int vk = v[k + row_idx*width];

      float data = float(q - vk)*float(q - vk) + distance_transform[vk + row_idx*width];

      output[(q)+width * row_idx] = sqrt(data); 
    }
  }
}

__global__ void cudaDistanceTransformCols(float *distance_transform, int *v, float *z, float *output, const uint32_t width, const uint32_t height){

  int col_idx = blockIdx.x * blockDim.x + threadIdx.x;

  if (col_idx < width){

    int k = 0;
    v[col_idx] = 0;
    z[col_idx] = -INF;
    z[col_idx + width + 1] = +INF;

    float s;

#pragma unroll
    for (int q = 1; q < height; q++) {
      //get a row index for this loop
      int q_idx = col_idx + q*width;

      int vk = v[col_idx + k*width];

      s = ((distance_transform[q_idx] + q*q) - (distance_transform[col_idx + vk*width] + vk*vk)) / (2 * q - 2 * vk);

      while (s <= z[col_idx + k*(width + 1)]){ //z is 1 row and 1 col wider
        k--;
        int vk = v[col_idx + k*width];
        s = ((distance_transform[q_idx] + (q*q)) - (distance_transform[col_idx + vk*width] + (vk*vk))) / ((2 * q) - (2 * vk));
      }

      k++;
      v[col_idx + k*width] = q;
      z[col_idx + k*(width + 1)] = s;
      z[col_idx + (k + 1)*(width + 1)] = +INF;

    }

    k = 0;
#pragma unroll
    for (int q = 0; q < height; q++){

      while (z[col_idx + (k + 1)*(width + 1)] < q)
        k++;

      int vk = v[col_idx + k*width];

      float data = float(q - vk)*float(q - vk) + distance_transform[col_idx + vk*width];

      //we do the square root on the row-wise transform
      output[q * width + (col_idx)] = data;
    }
  }
}

void ttrk::gpu::distanceTransform(cv::Mat &image, cv::Mat &distance_transform){

  if (!image.isContinuous()){
    image = image.clone();
  }

  distance_transform = cv::Mat(image.rows, image.cols, CV_32FC1);
  if (!distance_transform.isContinuous()){
    distance_transform = distance_transform.clone();
  }

  const uint32_t width = image.cols;
  const uint32_t height = image.rows;

  float *distance_transform_image;
  unsigned char *contour_image;

  checkCudaErrors(cudaMalloc((void **)&distance_transform_image, sizeof(float)*width*height));
  checkCudaErrors(cudaMalloc((void **)&contour_image, sizeof(unsigned char)*width*height));

  //create and allocate helper images
  int *v_image;
  float *z_image, *output_image;
  checkCudaErrors(cudaMalloc((void**)&z_image, sizeof(float)*(width + 1) * (height + 1)));
  checkCudaErrors(cudaMalloc((void**)&v_image, sizeof(int)*width*height));
  checkCudaErrors(cudaMalloc((void**)&output_image, sizeof(float)*width*height));

  //copy the contour image to the GPU
  checkCudaErrors(cudaMemcpy(contour_image, image.data, sizeof(unsigned char)*width*height, cudaMemcpyHostToDevice));

  checkCudaErrors(cudaMemset(distance_transform_image, 0, sizeof(float)*width*height));

  cudaEvent_t start, stop;
  cudaEventCreate(&start);
  cudaEventCreate(&stop);
  cudaEventRecord(start, 0);

  dim3 dimBlock(16, 16);
  dim3 dimGrid((width + dimBlock.x - 1) / dimBlock.x, (height + dimBlock.y - 1) / dimBlock.y);

  initImage << <dimGrid, dimBlock >> >(distance_transform_image, contour_image, width, height);
  cudaDistanceTransformCols << <(width + 255) / 256, 256 >> >(distance_transform_image, v_image, z_image, output_image, width, height);
  checkCudaErrors(cudaMemcpy(distance_transform_image, output_image, sizeof(float)*width*height, cudaMemcpyDeviceToDevice));
  cudaDistanceTransformRows << <(height + 255) / 256, 256 >> >(distance_transform_image, v_image, z_image, output_image, width, height);
  cudaEventRecord(stop, 0);
  cudaEventSynchronize(stop);
  float time;
  cudaEventElapsedTime(&time, start, stop);
  //cinder::app::console() << "Time to generate:  " << time << "ms \n" << std::endl; 
  checkCudaErrors(cudaMemcpy(distance_transform.data, output_image, sizeof(float)*width*height, cudaMemcpyDeviceToHost));
  checkCudaErrors(cudaFree(distance_transform_image));
  checkCudaErrors(cudaFree(contour_image));
  checkCudaErrors(cudaFree(z_image));
  checkCudaErrors(cudaFree(v_image));
  checkCudaErrors(cudaFree(output_image));
}
//
//void ttrk::gpu::distanceTransform(unsigned char *input, float *output, const size_t width, const size_t height){
//
//  unsigned char *contour_image;
//  float *distance_transform_image;
//
//  checkCudaErrors(cudaMalloc((void **)&distance_transform_image, sizeof(float)*width*height));
//  checkCudaErrors(cudaMalloc((void **)&contour_image, sizeof(unsigned char)*width*height));
//
//  //create and allocate helper images
//  int *v_image;
//  float *z_image, *output_image;
//  checkCudaErrors(cudaMalloc((void**)&z_image, sizeof(float)*(width + 1) * (height + 1)));
//  checkCudaErrors(cudaMalloc((void**)&v_image, sizeof(int)*width*height));
//  checkCudaErrors(cudaMalloc((void**)&output_image, sizeof(float)*width*height));
//
//   //copy the contour image to the GPU
//  checkCudaErrors(cudaMemcpy(contour_image, input, sizeof(unsigned char)*width*height, cudaMemcpyHostToDevice));
//
//  checkCudaErrors(cudaMemset(distance_transform_image, 0, sizeof(float)*width*height));
//
//  cudaEvent_t start, stop;
//  cudaEventCreate(&start);
//  cudaEventCreate(&stop);
//  cudaEventRecord(start, 0);
//
//  dim3 dimBlock(16, 16);
//  dim3 dimGrid((width + dimBlock.x - 1) / dimBlock.x, (height + dimBlock.y - 1) / dimBlock.y);
//
//  initImage <<< dimGrid, dimBlock >> >(distance_transform_image, contour_image, width, height);  
//
//  cudaDistanceTransformCols <<< (width + 255) / 256, 256 >>>(distance_transform_image, v_image, z_image, output_image, width, height);
//
//  cv::Mat tmp(height, width, CV_32FC1);
//  checkCudaErrors(cudaMemcpy(tmp.data, output_image, sizeof(float)*width*height, cudaMemcpyDeviceToHost));
//
//  checkCudaErrors(cudaMemcpy(distance_transform_image, output_image, sizeof(float)*width*height, cudaMemcpyDeviceToDevice));
//  cudaDistanceTransformRows <<< (height + 255) / 256, 256 >> >(distance_transform_image, v_image, z_image, output_image, width, height);
//
//  cudaEventRecord(stop, 0);
//  cudaEventSynchronize(stop);
//  
//  checkCudaErrors(cudaMemcpy(output, distance_transform_image, sizeof(float)*width*height, cudaMemcpyDeviceToHost));
//
//  float time;
//  cudaEventElapsedTime(&time, start, stop);
//    
//  //cinder::app::console() << "Time to generate:  " << time << "ms \n" << std::endl;
//  checkCudaErrors(cudaFree(z_image));
//  checkCudaErrors(cudaFree(v_image));
//  checkCudaErrors(cudaFree(output_image));
//  checkCudaErrors(cudaFree(contour_image));
//  checkCudaErrors(cudaFree(distance_transform_image));
//
//}
//
//void ttrk::gpu::distanceTransform(cv::Mat &image, cv::Mat &distance_transform){
//
//  if (!image.isContinuous()){
//    image = image.clone();
//  }
//
//  if (!distance_transform.isContinuous()){
//    distance_transform = distance_transform.clone();
//  }
//
//  const uint32_t width = image.cols;
//  const uint32_t height = image.rows;
//
//  float *dt_data = (float *)distance_transform.data;
//
//  distanceTransform(image.data, dt_data, width, height);
//  
//}

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
 

}

//void ttrk::gpu::GetCompLSJacobians(boost::shared_ptr<Model> current_model, unsigned char *frame, const float *classification_image, const size_t image_width, const size_t image_height){
//
//  //create a contour, sdf, front intersection and back intersection image
//  CuImage<float, 1> sdf_image(classification_image, image_width, image_height);
//  CuImage<unsigned char, 3> gpu_image(frame, image_width, image_height);
//
//}
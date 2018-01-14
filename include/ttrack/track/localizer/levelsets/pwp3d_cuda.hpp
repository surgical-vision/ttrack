#pragma once

#include <cuda.h>
#include <opencv2/opencv.hpp>
#include <cuda_runtime.h>
//#include <boost/shared_ptr.hpp>

//#include "../../model/model.hpp"
//#include <cinder/app/App.h>
//#include <cinder/gl/Fbo.h>

#define INF 1E20

#define checkCudaErrors(err)  ttrk::gpu::__checkCudaErrors (err, __FILE__, __LINE__)
#define getLastCudaError(msg) ttrk::gpu::__getLastCudaError (msg, __FILE__, __LINE__)

namespace ttrk {

  namespace gpu{

    inline void __checkCudaErrors(cudaError err, const char *file, const int line){
      if (err != cudaSuccess){
        //ci::app::console() << file << "(" << line << ") : CUDA Runtime API error " << (int)err << ": " << cudaGetErrorString(err) << std::endl;
        fprintf(stderr, "%s(%i) : CUDA Runtime API error %d: %s.\n", file, line, (int)err, cudaGetErrorString(err));
        return;
      }
    }

    inline void __getLastCudaError(const char *errorMessage, const char *file, const int line){
      cudaError_t err = cudaGetLastError(); 
      if (err != cudaSuccess){
        //ci::app::console() << file << "(" << line << ") : getLastCudaError() CUDA error : " << (int)err << ": " << cudaGetErrorString(err) << std::endl;
        fprintf(stderr, "%s(%i) : getLastCudaError() CUDA error : %s : (%d) %s.\n", file, line, errorMessage, (int)err, cudaGetErrorString(err));
        return;
      }
    }


    //template<typename TypeName, int Channels>
    //struct CuImage {

    //  CuImage(const TypeName *cpu_data, const size_t width, const size_t height) : data(nullptr) {

    //    checkCudaErrors(cudaMalloc((void**)&data, sizeof(TypeName)*width * height * Channels));

    //  }

    //  ~CuImage(){

    //    if (data)
    //      checkCudaErrors(cudaFree(data));

    //  }

    //  TypeName *data;

    //};

    bool checkCudaFunctionality();

    //void computeJacobiansForEye();

    //using P. Fetzenszwalb method: 'Distance transform of Sampled Functions'.
    void distanceTransform(cv::Mat &image, cv::Mat &distance_transform);
    //void distanceTransform(cv::Mat &image, cv::Mat &distance_transform);
    //void distanceTransform(unsigned char *image, float *sdf, const size_t width, const size_t height);
    
    //void GetCompLSJacobians(boost::shared_ptr<Model> current_model, unsigned char *frame, const float *classification_image, const size_t image_width, const size_t image_height);


  }

}
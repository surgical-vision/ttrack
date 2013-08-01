#ifndef __QUASI_DENSE_STEREO_HPP__
#define __QUASI_DENSE_STEREO_HPP__
#include"../headers.hpp"
//-------------------------------------------------------------------------------------------------
// Class for performing quasi dense stereo matching. The code uses pyramidal LK from OpenCV with
// Shi and Tomasi features to get the initial seed correspondences. Then these are propagated by
// using the simple growing scheme. The CUDA and segmentation style code is not provided here.
//
// If this code is udeful for your work please cite the following papers:
//
// Danail Stoyanov, Marco Visentini Scarzanella, Philip Pratt, Guang-Zhong Yang: Real-Time Stereo 
// Reconstruction in Robotically Assisted Minimally Invasive Surgery. MICCAI (1) 2010: 275-282
//
// also the following article with the original growing scheme idea
//
// Maxime Lhuillier, Long Quan: Robust Dense Matching Using Local and Global Geometric Constraints. 
// ICPR 2000: 1968-1972
//
// Disclaimer - This code is provided as is! It does not guarantee any performance or stability
// cireteria. Any damage to your compueter or data is not the responsibility of the author.
//
// Non-formal Disclaimer - The code is a "bit" messy and I have not checked it for memory leaks 
// or optimised it for very fast performance or removed bugs... 
// Any suggestions, correction and improvements you make would be of interest and I'd be happy 
// if you let me know. Thanks.
//-------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------
// Copyright (C) 2010 - Danail Stoyanov
// Centre for Medical Image Computing
// University College London
//-------------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------------
//
// // EXAMPLE USAGE: generates a disparity map, does not rely on rectified images (though if you
// // use rectified images the search space is smaller and so code is much faster...
//
// QuasiDenseStereo qds;
//
// qds.initialise( cvGetSize(i0) );
// qds.Param.BorderX = 15;				// borders around the image
// qds.Param.BorderY = 15;
// qds.Param.N = 5;						// neighbours
// qds.Param.Ct = 0.5;					// corre threshold for seeds
// qds.Param.Dg = 0.5;					// disparity gradient
// qds.Param.WinSizeX = 3;				// corr window size
// qds.Param.WinSizeY = 3;	
//
// qds.process(g0,g1);
//
// qds.getDisparityImage(d0);
// cvSaveImage("C:\\output\\stereo\\disparity_map.bmp\0",d0);
//
//-------------------------------------------------------------------------------------------------



#include <vector>
#include <queue>
using namespace std;

#define BUFFER_SIZE		 4
#define NUM_MAX_FEATURES 500
#define NO_MATCH 0

// Algorithm properties
struct PropagationParameters
{
	int	WinSizeX;		// similarity window
	int	WinSizeY;

	int aggWinSizeX;
	int aggWinSizeY;

	int BorderX;		// border to ignore
	int BorderY;

	float Ct;			// correlation threshold
	float Tt;			// texture threshold
	float Et;			// epipolar threshold

	int	  N;			// neighbourhood size
	int	  Dg;			// disparity gradient threshold

	int   neighboursX;
	int	  neighboursY;

	bool  aggregate;
	float minCostThreshold;

	bool savePropagationImages;
};

// A basic match structure
struct Match
{
	CvPoint p0;
	CvPoint p1;
	float	corr;

	bool operator < (const Match & rhs) const 
	{
		return corr < rhs.corr;
	}
};


class QuasiDenseStereo
{
public:

	QuasiDenseStereo(void);
	~QuasiDenseStereo(void);

	// init and cleanup
	void initialise(CvSize size);
	void reset();
	void cleanup();

	// the work
	void sparseMatching(IplImage* imgL , IplImage* imgR);
	void quasiDenseMatching(IplImage* imgL , IplImage* imgR,CvPoint2D32f* pointsL,CvPoint2D32f* pointsR,char* featureStatus,int numInputPoints);
	void process(IplImage* imgL , IplImage* imgR);

	// correlation function
	void	buildTextureDescriptor(IplImage* img,int* descriptor);
	float	iZNCC_c1(CvPoint p0, CvPoint p1, int wx=1, int wy=1);

	// access methods
	CvPoint getMatch(int x, int y);					// return the matching point for a (x,y) cooridinate in the ref image
	void	saveDisparityImage(char* name, int numDisparityLevels = -1, bool overlay=false);
	void	getDisparityImage(IplImage* img, int numDisparityLevels = -1, bool overlay=false);

	int				mFeatureCount[BUFFER_SIZE];
	char*			mStatus[BUFFER_SIZE];			// array to hold the status of features
	CvPoint2D32f*	mPoints2D[BUFFER_SIZE];			// 2D points in the image
	CvPoint2D32f*	mTempPoints[BUFFER_SIZE];		// temporary buffer for inefficient processing

	IplImage*		mEigenImage;					// temp image required for detection call
	IplImage*		mTempImage;						// same as above

	IplImage*		mImagesCopy[BUFFER_SIZE];		// local image data kept for tracking

	IplImage*		mImages[BUFFER_SIZE];			// local image data kept for tracking
	IplImage*		mIntegralImage[BUFFER_SIZE];	// intergral images
	IplImage*		mIntegralImageSq[BUFFER_SIZE];	// intergral images^2

	CvPoint*		mRefMap;						// Reference image matching map
	CvPoint*		mMtcMap;						// Match image matching map

	int*			mRefTextureDesc;				// Texture descriptor
	int*			mDstTextureDesc;
	int				MaxTextureVal;					// Texture vals
	int				MinTextureVal;

	unsigned char *data0;
	unsigned char *data1;
#ifdef __linux__
	int32_t *sum0;
	int32_t	*sum1;
#elif defined _WIN32 || _WIN64
  __int32 *sum0;
  __int32 *sum1;
#endif
	double *ssum0;
	double *ssum1;	
	int	Step,Steps;

	float *costSpace,*costSpaceTemp;
	int    costWidth,costHeight,costDepth;

	// buffer swaps
	char curr,past;

	PropagationParameters	Param;
};
#endif

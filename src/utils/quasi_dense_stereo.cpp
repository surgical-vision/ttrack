#include "../../headers/utils/quasi_dense_stereo.hpp"


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
// Non-formal Disclaimer - The code is a bit messy and I have not checked it for memory leaks 
// or optimised it for very fast performance. Any suggestions, correction and improvements you make
// would be of interest and I'd be happy if you let me know. Thanks.
//-------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------
// Copyright (C) 2010 - Danail Stoyanov
// Centre for Medical Image Computing
// University College London
//-------------------------------------------------------------------------------------------------


//-------------------------------------------------------------------
// Helper functions
//-------------------------------------------------------------------


#define clampVal(x,y) {x = x / y; x = 255*x; if(x>255)x=255;}
#define setG2RGB(val,data){ if(data){ data[0]=val; data[1]=val; data[2]=val;} }


// check image border proximity
inline bool CheckBorder(Match m, int bx, int by, int w, int h)
{
	if(m.p0.x<bx || m.p0.x>w-bx || m.p0.y<by || m.p0.y>h-by || m.p1.x<bx || m.p1.x>w-bx || m.p1.y<by || m.p1.y>h-by)
	{
		return false;
	}

	return true;
}

// Used for sorting
bool MatchCompare(Match a, Match b)
{
	if(a.corr<=b.corr)return true;
	return false;
}

//-------------------------------------------------------------------

QuasiDenseStereo::QuasiDenseStereo(void)
{
	curr = 0;
	past = 2;
}

QuasiDenseStereo::~QuasiDenseStereo(void)
{
	cleanup();
}

void QuasiDenseStereo::initialise(CvSize size)
{
	// allocate memory
	for( int i = 0; i < BUFFER_SIZE; i++ )
	{
		mImagesCopy[i] = cvCreateImage(size,IPL_DEPTH_8U,3);
		mImages[i] = cvCreateImage(size,IPL_DEPTH_8U,1);
		mIntegralImage[i] = cvCreateImage(cvSize(size.width+1,size.height+1),IPL_DEPTH_32S,1);
		mIntegralImageSq[i] = cvCreateImage(cvSize(size.width+1,size.height+1),IPL_DEPTH_64F,1);

		mPoints2D[i] = new CvPoint2D32f[NUM_MAX_FEATURES];
		mTempPoints[i] = new CvPoint2D32f[NUM_MAX_FEATURES];
		mStatus[i] = new char[NUM_MAX_FEATURES];
	}	


	mEigenImage = cvCreateImage(size,IPL_DEPTH_32F,1);	
	mTempImage = cvCreateImage(size,IPL_DEPTH_32F,1);	
	
	// memory for maps etc
	mRefMap = new CvPoint[size.width*size.height];
	mMtcMap = new CvPoint[size.width*size.height];
	mRefTextureDesc = new int[size.width*size.height];
	mDstTextureDesc = new int[size.width*size.height];
}

void QuasiDenseStereo::cleanup()
{
	if(mRefMap)
	  delete [] mRefMap;
	if(mMtcMap)
	  delete [] mMtcMap;
	if(mRefTextureDesc)
	  delete [] mRefTextureDesc;
	if(mDstTextureDesc)
	  delete [] mDstTextureDesc;

	// release the images
	for(int i = 0; i < BUFFER_SIZE; i++)
	{
		if(mImagesCopy[i]!=NULL)
			cvReleaseImage(&mImages[i]);
		if(mImages[i]!=NULL)
			cvReleaseImage(&mImages[i]);
		if(mIntegralImage[i]!=NULL)
			cvReleaseImage(&mIntegralImage[i]);
		if(mIntegralImageSq[i]!=NULL)
			cvReleaseImage(&mIntegralImageSq[i]);
	}

	if(mEigenImage!=NULL)
		cvReleaseImage(&mEigenImage);
	if(mTempImage!=NULL)
		cvReleaseImage(&mTempImage);
}

void QuasiDenseStereo::reset()
{

}


void	QuasiDenseStereo::buildTextureDescriptor(IplImage* img,int* descriptor)
{	
	int x,y,cb;

	float minVal = 1e9 , maxVal = -1e9;

	unsigned char* imgData = (unsigned char*)img->imageData,*pixel=0;
	int nb = img->nChannels,s=img->widthStep,w=img->width,h=img->height;

	float a,b,c,d,sb = 1.f/(float)nb;

	memset(descriptor,0,sizeof(int)*w*h);

	for(y=1;y<h-1;y++)
	{	
		for(x=1;x<w-1;x++)
		{
			//current pixel
			pixel = &imgData[y*s+x*nb];

			//difference
			a=0;b=0;c=0;d=0;
			for(cb=0;cb<nb;cb++)
			{
				a += (float)abs(pixel[cb]-pixel[cb-nb]);
				b += (float)abs(pixel[cb]-pixel[cb+nb]);
				c += (float)abs(pixel[cb]-pixel[cb-s]);
				d += (float)abs(pixel[cb]-pixel[cb+s]);
			}

			a *= sb;
			b *= sb;
			c *= sb;
			d *= sb;
						
			int val = std::max(a,std::max(b,std::max(c,d)));

			// clamp
			if(val > 255) val  = 255;

			descriptor[y*w+x] = val;

			// just so we know the data ranges
		}
	}	
}


//------------------------------------------------------------------------------------------------------------------
// Zero mean normalized cross correlation functions using precalculated integral images 
// Note: done for Gray (c1) and RGB (c3) images individually for perfomance
//------------------------------------------------------------------------------------------------------------------

float	QuasiDenseStereo::iZNCC_c1(CvPoint p0, CvPoint p1, int wx, int wy)
{
	int x,y,otl,otr,obl,obr,wx1,wy1;
	float m0=0.f,m1=0.f,s0=0.f,s1=0.f;
	float wa = (float)(2*wy+1)*(2*wx+1);
	float zncc=0.f;

	int boy0=(p0.y-wy)*Step+(p0.x-wx);
	int boy1=(p1.y-wy)*Step+(p1.x-wx);
	int oy0=boy0,oy1=boy1;
	int ox0=0,ox1=0;

	// offsets for corners top-left, top-right, bottom-left, bottom-right
	wx1 = wx+1;
	wy1 = wy+1;

	// offsets for left image
	otl = (p0.y-wy)*Steps+(p0.x-wx);
	otr = (p0.y-wy)*Steps+(p0.x+wx1);
	obl = (p0.y+wy1)*Steps+(p0.x-wx);
	obr = (p0.y+wy1)*Steps+(p0.x+wx1);

	// sum and squared sum for left window
	m0 = ((sum0[obr] + sum0[otl]) - (sum0[obl] + sum0[otr]));
	s0 = (ssum0[obr] + ssum0[otl]) - (ssum0[obl] + ssum0[otr]);

	// offsets for right image
	otl = (p1.y-wy)*Steps+(p1.x-wx);
	otr = (p1.y-wy)*Steps+(p1.x+wx1);
	obl = (p1.y+wy1)*Steps+(p1.x-wx);
	obr = (p1.y+wy1)*Steps+(p1.x+wx1);

	// sum and squares sum for right window
	m1 = (float)((sum1[obr] + sum1[otl]) - (sum1[obl] + sum1[otr]));
	s1 = (ssum1[obr] + ssum1[otl]) - (ssum1[obl] + ssum1[otr]);

	// window means
	m0 /= wa;
	m1 /= wa;

	// standard deviations
	s0 = sqrt(s0-wa*m0*m0);
	s1 = sqrt(s1-wa*m1*m1);

	zncc = 0;
	for(y=-wy;y<=wy;y++,oy1+=Step,oy0+=Step)
	{
		ox0=0,ox1=0;
		unsigned char * line0 = &data0[oy0];
		unsigned char * line1 = &data1[oy1];
		for(x=-wx;x<=wx;x++)
		{	
			zncc += (float)line0[ox0++]*(float)line1[ox1++];
		}	
	}

	// the final result
	zncc = (zncc-wa*m0*m1)/(s0*s1);

	return zncc;
}

// This is just a simple matcher using the OpenCV stereo pyramid LK stuff
void QuasiDenseStereo::sparseMatching(IplImage* imgL , IplImage* imgR)
{
	// some parameters
	CvSize	templateSize = cvSize(3,3);
	int		numPyrLevels = 3;
	double	qualityThreshold = 0.01;
	double	minSeparationDistance = 10;
	
	mFeatureCount[curr] = NUM_MAX_FEATURES;

	// Detect features in the left image
	cvGoodFeaturesToTrack( imgL, mEigenImage, mTempImage,
		&mPoints2D[curr][0],&mFeatureCount[curr],qualityThreshold, minSeparationDistance,NULL );
			
	// match features from the curren left frame to the current right frame
	cvCalcOpticalFlowPyrLK( imgL, imgR, NULL,NULL,
			mPoints2D[curr], mPoints2D[curr+1],mFeatureCount[curr],
			templateSize,numPyrLevels,&mStatus[curr][0],0,
			cvTermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,20,0.0003),0);
}

void QuasiDenseStereo::quasiDenseMatching(IplImage* imgL , IplImage* imgR,CvPoint2D32f* pointsL, CvPoint2D32f* pointsR, char* featureStatus, int numInputPoints)
{
	int x,y,i,wx,wy,off,NumberOfMatchesFound=0;

	// reset memory of maps
	memset(mRefMap,0, imgL->width * imgL->height * sizeof(CvPoint));
	memset(mMtcMap,0, imgL->width * imgL->height * sizeof(CvPoint));

	// build texture homogeneity reference maps.
	buildTextureDescriptor(imgL,mRefTextureDesc);
	buildTextureDescriptor(imgR,mDstTextureDesc);

	// generate the intergal images for fast variable window correlation calculations
	cvIntegral(imgL,mIntegralImage[curr],mIntegralImageSq[curr]);
	cvIntegral(imgR,mIntegralImage[curr+1],mIntegralImageSq[curr+1]);

	// for fast processing initialize some pointers to the data 
	data0 = (unsigned char *)imgL->imageData;
	data1 = (unsigned char *)imgR->imageData;
#ifdef __linux__
	sum0  = (int32_t*)mIntegralImage[curr]->imageData;
	sum1  = (int32_t*)mIntegralImage[curr+1]->imageData;
#elif defined _WIN32 || _WIN64
  sum0  = (__int32*)mIntegralImage[curr]->imageData;
	sum1  = (__int32*)mIntegralImage[curr+1]->imageData;
#endif
	ssum0 = (double*)mIntegralImageSq[curr]->imageData;
	ssum1 = (double*)mIntegralImageSq[curr+1]->imageData;

	Step = imgL->widthStep;
	Steps = mIntegralImage[curr]->width;

	// init these vars
	costWidth = imgL->width;
	costHeight = imgL->height;
	costDepth = 15;

	// Seed list
	priority_queue <Match, vector<Match>, less<Match> > Seed;

	// Build a list of seeds from the starting features
	for(i=0; i < mFeatureCount[curr]; i++)
	{
		if(mStatus[curr][i]==1)
		{
			// Calculate correlation and store match in Seeds.
			Match m;
			m.p0 = cvPoint(pointsL[i].x,pointsL[i].y);
			m.p1 = cvPoint(pointsR[i].x,pointsR[i].y);

			// Check if too close to boundary.
			if(!CheckBorder(m,Param.BorderX,Param.BorderY,imgL->width,imgL->height))
				continue;

			// Calculate the correlation threshold
			m.corr = iZNCC_c1(m.p0,m.p1,Param.WinSizeX,Param.WinSizeY);

			// Can we add it to the list
			if( m.corr > Param.Ct )
			{
				// FIXME: Check if this is unique (or assume it is due to prior supression)
				Seed.push(m);
				mRefMap[m.p0.y*imgL->width + m.p0.x] = m.p1;
				mMtcMap[m.p1.y*imgL->width + m.p1.x] = m.p0;
			}
		}
	}

	// Do the propagation part
	while(!Seed.empty())
	{
		priority_queue <Match, vector<Match>, less<Match> > Local;

		// Get the best seed at the moment
		Match m = Seed.top();
		Seed.pop();

		// Ignore the border
		if(!CheckBorder(m,Param.BorderX,Param.BorderY,imgL->width,imgL->height))
			continue;
		
		// For all neighbours in image 1
		for(y=-Param.N;y<=Param.N;y++)
		{
			for(x=-Param.N;x<=Param.N;x++)
			{
				CvPoint p0 = cvPoint(m.p0.x+x,m.p0.y+y);

				// Check if its unique in ref
				if(mRefMap[p0.y*imgL->width + p0.x].x != NO_MATCH) continue;

				// Check the texture descriptor for a boundary
				if(mRefTextureDesc[p0.y*imgL->width + p0.x] > Param.Tt) continue;

				// For all candidate matches.
				for(wy=-Param.Dg; wy<=Param.Dg; wy++)
				{
					for(wx=-Param.Dg; wx<=Param.Dg; wx++)
					{
						CvPoint p1 = cvPoint(m.p1.x+x+wx,m.p1.y+y+wy);

						// Check if its unique in ref
						if(mMtcMap[p1.y*imgL->width + p1.x].x != NO_MATCH) continue;

						// Check the texture descriptor for a boundary	
						if(mDstTextureDesc[p1.y*imgL->width + p1.x] > Param.Tt) continue;

						// Calculate ZNCC and store local match.
						float corr = iZNCC_c1(p0,p1,Param.WinSizeX,Param.WinSizeY);

						// push back if this is valid match
						if( corr > Param.Ct )
						{
							Match nm;
							nm.p0 = p0;
							nm.p1 = p1;
							nm.corr = corr;
							Local.push(nm);
						}
					}
				}
			}
		}

		// Get seeds from the local
		while( !Local.empty() )
		{
			Match lm = Local.top();
			Local.pop();

			// Check if its unique in both ref and dst.
			if(mRefMap[lm.p0.y*imgL->width + lm.p0.x].x != NO_MATCH) continue;
			if(mMtcMap[lm.p1.y*imgL->width + lm.p1.x].x != NO_MATCH) continue;

			// Unique match
			mRefMap[lm.p0.y*imgL->width + lm.p0.x] = lm.p1;
			mMtcMap[lm.p1.y*imgL->width + lm.p1.x] = lm.p0;

			// Add to the seed list
			Seed.push(lm);
			NumberOfMatchesFound++;
		}
	}
}


void QuasiDenseStereo::process(IplImage* imgL , IplImage* imgR)
{
	bool	drawOnOriginal = false;
	int		i,count;
	int		featureCount = NUM_MAX_FEATURES;
		
#ifdef _WIN32 || _WIN64
  // just for timing
  //DWORD t0 = timeGetTime();
#endif
	// Copy the image data locally or convert to gray if needed
	// note: sub optimal
	if(imgL->nChannels == 1)
	{
		cvCopy(imgL,mImages[curr]);
		cvCopy(imgR,mImages[curr+1]);
	}
	else
	{
		// for local display
		cvCopy(imgL,mImagesCopy[curr]);
		cvCopy(imgR,mImagesCopy[curr+1]);

		cvCvtColor(imgL,mImages[curr],CV_RGB2GRAY);
		cvCvtColor(imgR,mImages[curr+1],CV_RGB2GRAY);
	}

	// run the sparse part
	sparseMatching(mImages[curr],mImages[curr+1]);

	// run the quasi dense part
	quasiDenseMatching(mImages[curr],mImages[curr+1],mPoints2D[curr],mPoints2D[curr+1],mStatus[curr],mFeatureCount[curr]);

	// just for timing
#ifdef _WIN32 || _WIN64
	//DWORD t1 = timeGetTime() - t0;
#endif
}

//------------------------------------------------------------------------
// access methods

CvPoint QuasiDenseStereo::getMatch(int x, int y)
{
	return mRefMap[y*mImages[curr]->width + x];
}


void	QuasiDenseStereo::getDisparityImage(IplImage* img, int numDisparityLevels, bool overlay)
{
	int s = img->widthStep;
	unsigned char* imgData = (unsigned char*)img->imageData;
	float mindsp = 1e9;
	float maxdsp = -1e9;

	float* dsp = new float[img->width*img->height];
	memset(dsp,0,img->width*img->height*sizeof(float));

	for(int y=0;y<img->height;y++)
	{
		for(int x=0;x<img->width;x++)
		{
			if(mRefMap[y*img->width+x].x == NO_MATCH)
			{
				dsp[y*img->width+x] = NO_MATCH;
				continue;
			}

			float dx = x-mRefMap[y*img->width+x].x;
			float dy = y-mRefMap[y*img->width+x].y;

			dsp[y*img->width+x] = sqrt(float(dx*dx+dy*dy));

			if(dsp[y*img->width+x] < mindsp) mindsp = dsp[y*img->width+x];
			if(dsp[y*img->width+x] > maxdsp) maxdsp = dsp[y*img->width+x];
		}
	}

	// change the disparity level scale
	if(numDisparityLevels != -1)
	{
		mindsp = 0;
		maxdsp = numDisparityLevels;
	}

	for(int y=0;y<img->height;y++)
	{
		for(int x=0;x<img->width;x++)
		{

			if(dsp[(y)*img->width+x] == NO_MATCH)
			{
				if(!overlay)
				{
					imgData[y*s+x*3] = 200;
					imgData[y*s+x*3+1] = 0;
					imgData[y*s+x*3+2] = 0;
				}
				continue;
			}

			//float ScaledDisparity = 255 - 255.f*((dsp[(y)*img->width+x]-mindsp)/(maxdsp-mindsp));
			float ScaledDisparity = dsp[(y)*img->width+x];
      unsigned char DisparityImageVal = (unsigned char)ScaledDisparity;

			imgData[(y)*s+x*3] = DisparityImageVal;
			imgData[(y)*s+x*3+1] = DisparityImageVal;
			imgData[(y)*s+x*3+2] = DisparityImageVal;
		}
	}
	delete [] dsp;
}


void	QuasiDenseStereo::saveDisparityImage(char* name, int numDisparityLevels,bool overlay)
{
	IplImage* tempImage;
	if(!overlay)
	{
		tempImage = cvCreateImage(cvSize(costWidth,costHeight),IPL_DEPTH_8U,3);
	}
	else
	{
		tempImage = cvCloneImage(mImagesCopy[curr]);
	}
	
	getDisparityImage(tempImage,numDisparityLevels,overlay);

	cvSaveImage(name,tempImage);
	
	cvReleaseImage(&tempImage);
}

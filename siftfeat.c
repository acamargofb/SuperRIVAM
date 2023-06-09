/*
This program detects image features using SIFT keypoints. For more info,
refer to:

Lowe, D. Distinctive image features from scale-invariant keypoints.
International Journal of Computer Vision, 60, 2 (2004), pp.91--110.

Copyright (C) 2006  Rob Hess <hess@eecs.oregonstate.edu>

Note: The SIFT algorithm is patented in the United States and cannot be
used in commercial products without a license from the University of
British Columbia.  For more information, refer to the file LICENSE.ubc
that accompanied this distribution.

Version: 1.1.1-20070330
*/

#include "sift.h"
#include "imgfeatures.h"
#include "utils.h"

#include <highgui.h>

#include <stdio.h>

/******************************** Globals ************************************/

char* img_file_name = "..\\beaver.png";
char* out_file_name  = "..\\output.sift";
//char* out_file_name  = "output.jpg";
char* out_img_name =  "output.jpg";
int display = 1;
int intvls = SIFT_INTVLS;
double sigma = SIFT_SIGMA;
double contr_thr = SIFT_CONTR_THR;
int curv_thr = SIFT_CURV_THR;
int img_dbl = SIFT_IMG_DBL;
int descr_width = SIFT_DESCR_WIDTH;
int descr_hist_bins = SIFT_DESCR_HIST_BINS;


/********************************** Main *************************************/

int main( int argc, char** argv )
{
	IplImage* img;
	struct feature* features;
	int n = 0;
    

    //char path1[] = "F:\\OpenCV\\SIFT\\sift-1.1.1_20070330_win\\beaver.png";
    //char path1[] = "F:\\OpenCV\\SIFT\\sift-1.1.1_20070330_win\\beaver_xform.png";

    //char path1[] = "F:\\OutsideLectures\\UAV\\UAV_VideoProcessing\\MosaikingCCode\\gim1.png";
    char path2[] = "E:\\Summer2009\\mosaicv10\\debug\\frames_UAS_IR_set1\\frame5.jpg";
	

	fprintf( stderr, "Finding SIFT features...\n" );
	//img = cvLoadImage( img_file_name, 1 );
	img = cvLoadImage( path2, 1);
	if( ! img )
	{
		fprintf( stderr, "unable to load image from %s", img_file_name );
		exit( 1 );
	}
	n = _sift_features( img, &features, intvls, sigma, contr_thr, curv_thr,
						img_dbl, descr_width, descr_hist_bins );
	fprintf( stderr, "Found %d features.\n", n );

	if( display )
	{
		draw_features( img, features, n );
		cvNamedWindow( img_file_name, 1 );
		cvShowImage( img_file_name, img );
		cvWaitKey( 0 );
	}

	if( out_file_name != NULL )
		export_features( out_file_name, features, n );

	if( out_img_name != NULL )
		cvSaveImage( out_img_name, img );
	return 0;
}

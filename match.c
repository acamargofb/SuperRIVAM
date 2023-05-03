/*
Detects SIFT features in two images and finds matches between them.

Copyright (C) 2006  Rob Hess <hess@eecs.oregonstate.edu>

@version 1.1.1-20070330
*/

#include "sift.h"
#include "imgfeatures.h"
#include "kdtree.h"
#include "utils.h"
#include "xform.h"



#include <cv.h>
#include <cxcore.h>
#include <highgui.h>

#include <stdio.h>


/* the maximum number of keypoint NN candidates to check during BBF search */
#define KDTREE_BBF_MAX_NN_CHKS 200

/* threshold on squared ratio of distances between NN and 2nd NN */
#define NN_SQ_DIST_RATIO_THR 0.49

/******************************** Globals ************************************/

//char img1_file[] = "..\\beaver.png";
//char img1_file[] = "F:\\OpenCV\\SIFT\\sift-1.1.1_20070330_win\\beaver.png";
//char img2_file[] = "..\\beaver_xform.png";
//char img2_file[] = "F:\\OpenCV\\SIFT\\sift-1.1.1_20070330_win\\beaver_xform.png";

char img1_file[] = "E:\\Summer2009\\mosaicv10\\debug\\frames_UAS_IR_set1\\frame4.jpg";
char img2_file[] = "E:\\Summer2009\\mosaicv10\\debug\\frames_UAS_IR_set1\\frame5.jpg";

//char img1_file[] = "ikonos_xy00_LR_1.jpg";
//char img2_file[] = "ikonos_xy00_LR_2.jpg";


/********************************** Main *************************************/


//typedef struct {
//	int width;
//	int height;
//	float  **p;
//} PlaneFP;  

//int PixelValue(IplImage *Img, int i,int j)
//{
// 
 //  uchar* ptr = &CV_IMAGE_ELEM(Img,uchar,i,j*1);
//);  
//  return ptr[0]; 
//}

int main( int argc, char** argv )
{
	IplImage* img1, * img2, * stacked;
	struct feature* feat1, * feat2, * feat;
	struct feature** nbrs;
	struct kd_node* kd_root;
//	struct PlaneFP*  img1p;
	//struct CvMat* H;
	CvPoint pt1, pt2;
	double d0, d1;
	int n1, n2, k, i, m = 0, dx, dy;
	//int height, width, step, channels;
	//unsigned char data;
	unsigned char * ptr;
  
   IplImage *newimag;
// Declarations for the Matching
/*	unsigned char val1, val2;
	unsigned char r1, g1, b1, r2, b2, g2;
	int value1, value2;
	int xOffset = 0;
	int yOffset = 0;
	int ULX, ULY, LLX, LLY, URX, URY, LRX, LRY;
	int sizex, sizey;

	IplImage *newImage, *tempImage;
	CvSize newSize;
	CvMat *Translate, *tempXY;
	Translate = cvCreateMat(3,3,CV_32FC1);
	tempXY = cvCreateMat(3,1,CV_32FC1); */
	



	img1 = cvLoadImage( img1_file, 1 );
	if( ! img1 )
		fatal_error( "unable to load image from %s", img1_file );
	img2 = cvLoadImage( img2_file, 1 );
	if( ! img2 )
		fatal_error( "unable to load image from %s", img2_file );
	stacked = stack_imgs( img1, img2 );

	fprintf( stderr, "Finding features in %s...\n", img1_file );
	n1 = sift_features( img1, &feat1 );
	fprintf( stderr, "Finding features in %s...\n", img2_file );
	n2 = sift_features( img2, &feat2 );
	kd_root = kdtree_build( feat2, n2 );
	for( i = 0; i < n1; i++ )
	{
		feat = feat1 + i;
		k = kdtree_bbf_knn( kd_root, feat, 2, &nbrs, KDTREE_BBF_MAX_NN_CHKS );
		if( k == 2 )
		{
			d0 = descr_dist_sq( feat, nbrs[0] );
			d1 = descr_dist_sq( feat, nbrs[1] );
			if( d0 < d1 * NN_SQ_DIST_RATIO_THR )
			{
				pt1 = cvPoint( cvRound( feat->x ), cvRound( feat->y ) );
				pt2 = cvPoint( cvRound( nbrs[0]->x ), cvRound( nbrs[0]->y ) );
				pt2.y += img1->height;
				cvLine( stacked, pt1, pt2, CV_RGB(255,0,255), 1, 8, 0 );
				m++;
				feat1[i].fwd_match = nbrs[0];
			}
		}
		free( nbrs );
	}

	fprintf( stderr, "Found %d total matches\n", m );
	cvNamedWindow( "Matches", 1 );
	cvShowImage( "Matches", stacked );
	cvWaitKey( 0 );


	cvSaveImage( "output_matches.jpg",  stacked );


	/* 
	UNCOMMENT BELOW TO SEE HOW RANSAC FUNCTION WORKS

	Note that this line above:

	feat1[i].fwd_match = nbrs[0];

	is important for the RANSAC function to work.
	*/

	
	{
		CvMat* H;
		CvMat* Hp = cvCreateMat(2,3,CV_32FC1);
		H = ransac_xform( feat1, n1, FEATURE_FWD_MATCH, lsq_homog, 4, 0.01,
			homog_xfer_err, 3.0, NULL, NULL );
      // Output Homography
        printf("\nThe value of the Homography are: \n");
         for (i = 0; i < 3; i ++) {
          for (k = 0; k < 3; k ++) {
			  printf("\n H (%d, %d ) =  %f", i, k, cvmGet(H,i,k));}}      

// values for Hp that will be used with cvWarpAffine

        cvmSet(Hp,0,0,cvmGet(H,0,0));
        cvmSet(Hp,0,1,cvmGet(H,0,1));
		cvmSet(Hp,0,2,cvmGet(H,0,2));
		cvmSet(Hp,1,0,cvmGet(H,1,0));
		cvmSet(Hp,1,1,cvmGet(H,1,1));
		cvmSet(Hp,1,2,cvmGet(H,1,2));

        for (i = 0; i < 2; i ++) {
           for (k = 0; k < 3; k ++) {
		  	  printf("\n Hp (%d, %d ) =  %f", i, k, cvmGet(Hp,i,k));}}      



		if( H )
		{
			CvPoint2D64f ptimg1;
			CvPoint2D64f ptnewimg1;
			IplImage* xformed;
			IplImage* xformed2;
			IplImage* newimg;
			CvMat* ptnewimag = cvCreateMat(img1->height,img1->width,CV_64FC1);
			xformed2 = cvCreateImage( cvGetSize( img2 ), IPL_DEPTH_8U, 3 );
			//xformed = cvCreateImage( cvGetSize( img2 ), IPL_DEPTH_8U, 3 );//I change img2
            xformed = cvCreateImage( cvSize(1000,350), IPL_DEPTH_8U, 3 );
            newimag = cvCreateImage( cvGetSize( img1 ), IPL_DEPTH_8U, 3 );
			
			
		cvWarpPerspective( img1, xformed, H, 
		CV_INTER_LINEAR  + CV_WARP_FILL_OUTLIERS, cvScalarAll( 0 ));//,cvPoint(0,0) );

//			cvWarpPerspective( I1, tempImage, Translate, CV_INTER_LINEAR  + CV_WARP_FILL_OUTLIERS, cvScalarAll( 0 ));

        cvWarpAffine( img1, xformed2, Hp, 
		CV_INTER_LINEAR  + CV_WARP_FILL_OUTLIERS, cvScalarAll( 0 ) );

        for (i=0;i<img1->height;i++){
    		for(k=0;k<img1->width;k++){
			 ptimg1.x = i; 
			 ptimg1.y = k; 
			 ptnewimg1 = persp_xform_pt(ptimg1, H);
			 dx = cvRound(ptnewimg1.x);
			 dy = cvRound(ptnewimg1.y);
     	     ptr = &CV_IMAGE_ELEM(img1,uchar,dx,dy*3);
			 }}


			// Converting to Pointer to work with the warp and points
            
		
/*
			for (i=0;i<img1->height;i++){
				for(k=0;k<img1->width;k++){
					ptimg1.x = i; 
					ptimg1.y = k; 
					ptnewimg1 = (ptimg1, H);
					dx = i;
					dy = k;
     				ptr = &CV_IMAGE_ELEM(img1,uchar,dx,dy*3);
					dx = cvRound(ptnewimg1.x);
					dy = cvRound(ptnewimg1.y);	
	  
				 }} */

         //  cvNamedWindow( "test", 1 );
         //  cvShowImage( "test", img1 );
          // cvWaitKey( 0 );


/*
			// Result according with WarpPerspective
     		cvNamedWindow( "Xformed", 1 );
			cvShowImage( "Xformed", xformed );
			//cvSaveImage("F:\\Research\\Matlab\\Mosaicing\\Images2Test\\Mountain1p.jpg", xformed);
			cvWaitKey( 0 );

			// Results according with WarpAffine
			cvNamedWindow( "Xformed2", 1 );
			cvShowImage( "Xformed2", xformed2 );
			cvWaitKey( 0 );	
            			

     		//cvNamedWindow( "Xformed2", 1 );
			//cvShowImage( "Xformed2", xformed2 );
            //cvSaveImage("F:\\Research\\Matlab\\Mosaicing\\Images2Test\\Mountain1p.jpg", xformed);
			cvWaitKey( 0 );

			//newimag = meshimages(img1,img2,Hp);
		    //cvNamedWindow( "Xformed2", 1 );
			//cvShowImage( "Xformed2", newimag );
*/
			cvReleaseImage( &xformed2 );  
			cvReleaseMat( &H );
            cvReleaseMat( &Hp );
			cvReleaseImage( &xformed );


		}
	} 

    
	cvReleaseImage( &stacked );
	cvReleaseImage( &img1 );
	cvReleaseImage( &img2 );
//    cvReleaseImage( &newimag );
  
//	cvReleaseMat(&Translate);
//	cvReleaseImage(&tempImage);
	kdtree_release( kd_root );
	free( feat1 );
	free( feat2 );
	return 0;
}

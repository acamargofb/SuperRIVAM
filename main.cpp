#include <cv.h>
#include <cxcore.h>
#include <highgui.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/timeb.h>

#include <time.h>




extern "C"{
#include "mosaic.h"
#include "sift.h"
#include "imgfeatures.h"
#include "kdtree.h"
#include "utils.h"
#include "xform.h"
}


#include "Common_DICOM.h"
#include "DICOM_IO.h"

/* the maximum number of keypoint NN candidates to check during BBF search */
#define KDTREE_BBF_MAX_NN_CHKS 200

/* threshold on squared ratio of distances between NN and 2nd NN */
#define NN_SQ_DIST_RATIO_THR 0.49

#define MAX_NUM_ITERATIONS 7

#define ERROR_LIMIT 0.000001

static void allocateOnDemand( IplImage **img, CvSize size, int depth, int channels )
{
	if ( *img != NULL )	return;

	*img = cvCreateImage( size, depth, channels );
	if ( *img == NULL )
	{
		fprintf(stderr, "Error: Couldn't allocate image.  Out of memory?\n");
		exit(-1);
	}
}




/********************************** Main *************************************/

int main( int argc, char** argv )
{
	IplImage* img1=NULL, * img2=NULL, *img3=NULL, *img4=NULL, *img5=NULL;
	IplImage* img1p=NULL, * img2p=NULL, *img3p=NULL, *img4p=NULL, *img5p=NULL;
	IplImage* img1pp=NULL, * img2pp=NULL, *img3pp=NULL, *img4pp=NULL, *img5pp=NULL;
	IplImage* img1ppp=NULL, * img2ppp=NULL, *img3ppp=NULL, *img4ppp=NULL, *img5ppp=NULL;
	IplImage* img1pppp=NULL, * img2pppp=NULL, *img3pppp=NULL, *img4pppp=NULL, *img5pppp=NULL;
	
	
	 vtkImageData *VTKImage1;
	 vtkImageData *VTKImage2, *VTKImage3, *VTKImage4, *VTKImage5;//, *f2;


	IplImage* dimg1=NULL, *dimg2=NULL, *dimg3=NULL, *dimg4=NULL, *dimg5=NULL;
	IplImage* dimg1_2=NULL, *dimg2_2=NULL, *dimg3_2=NULL, *dimg4_2=NULL, *dimg5_2=NULL;
	IplImage* dimg1_3=NULL, *dimg2_3=NULL, *dimg3_3=NULL, *dimg4_3=NULL, *dimg5_3=NULL;

	IplImage* Dimg1=NULL, * Dimg2=NULL, *Dimg3=NULL, *Dimg4=NULL, *Dimg5=NULL;
	//IplImage* Dimg1_2=NULL, * Dimg2_2=NULL, *Dimg3_2=NULL, *Dimg4_2=NULL, *Dimg5_2=NULL;
	IplImage* img1_inv = NULL, *img2_inv = NULL, *img3_inv=NULL, *img4_inv=NULL, *img5_inv=NULL;
	IplImage* dst2_wp;
	//IplImage* img5_inv_test = NULL;

	IplImage* img1_inv_2 = NULL, * img2_inv_2 = NULL, * img3_inv_2 = NULL, * img4_inv_2 = NULL, *img5_inv_2 = NULL;

	IplImage* img1_inv_3 = NULL, * img2_inv_3 = NULL, * img3_inv_3 = NULL, * img4_inv_3 = NULL, *img5_inv_3 = NULL;

	IplImage* Ddimg1=NULL, *Ddimg2=NULL, *Ddimg3=NULL, *Ddimg4=NULL, *Ddimg5=NULL;
	IplImage* Ddimg1_2 = NULL, *Ddimg2_2 = NULL, *Ddimg3_2 = NULL, *Ddimg4_2 = NULL, *Ddimg5_2 = NULL;
	IplImage* Ddimg1_3 = NULL, *Ddimg2_3 = NULL, *Ddimg3_3 = NULL, *Ddimg4_3 = NULL, *Ddimg5_3 = NULL;

	IplImage* ddst1 = NULL, *ddst2 = NULL, *ddst3 = NULL, *ddst4 = NULL, *ddst5=NULL;
	IplImage* ddst1_2 = NULL, *ddst2_2 = NULL, *ddst3_2 = NULL, *ddst4_2 = NULL, *ddst5_2 = NULL;
	IplImage* ddst1_3 = NULL, *ddst2_3 = NULL, *ddst3_3 = NULL, *ddst4_3 = NULL, *ddst5_3 = NULL;


//	IplImage ArryofImgs[20]; // Array of Images to test

	IplImage** ArrayofImgs;

	IplImage* Limg1 = NULL,* Limg1_2 = NULL, * Limg1_3 = NULL;
	IplImage* panorama5LLt = NULL, * panorama5LLt_2 = NULL,* panorama5LLt_3 = NULL;
	IplImage* panorama5padded = NULL;
	IplImage* panorama5_it_2 = NULL, *panorama5_2 = NULL, *panorama5_Delta = NULL, *panorama5_Delta_2 = NULL, *panorama5_Delta_3 = NULL;

	IplImage* panoramaSR = NULL;
	IplImage* panoramaSR_32f = NULL;
	IplImage* prev_panorama = NULL;

	IplImage* X1_dst = NULL, *X2_dst = NULL, *X3_dst = NULL, *X4_dst = NULL, *X5_dst = NULL; 

	CvMat *H12 = cvCreateMat(3,3,CV_32FC1);
	CvMat *H13 = cvCreateMat(3,3,CV_32FC1);
	CvMat *H23 = cvCreateMat(3,3,CV_32FC1);
	CvMat *Htemp = cvCreateMat(3,3,CV_32FC1);
	CvMat *H34 = cvCreateMat(3,3,CV_32FC1);
	CvMat *H14 = cvCreateMat(3,3,CV_32FC1);
	CvMat *H45 = cvCreateMat(3,3,CV_32FC1);
	CvMat *H15 = cvCreateMat(3,3,CV_32FC1);

	CvMat *H21 = cvCreateMat(3,3,CV_32FC1);
	CvMat *H31 = cvCreateMat(3,3,CV_32FC1);
	CvMat *H41 = cvCreateMat(3,3,CV_32FC1);
	CvMat *H51 = cvCreateMat(3,3,CV_32FC1);

	CvMat* kernelLLt = cvCreateMat(3,3,CV_32FC1);

	IplImage* panorama5 = NULL;
	IplImage* panorama5d = NULL;
	IplImage* panorama5d_2 = NULL;
	IplImage* panorama5d_3 = NULL;
	IplImage* panorama5_3 = NULL;

	IplImage* WarpImgvideo = NULL;

	IplImage* Uimg1 = NULL, *Uimg2 = NULL, *Uimg3 = NULL, *Uimg4 = NULL, *Uimg5 = NULL;

	//struct feature *feat1;
    //struct feature *feat2;
	struct feature* feat1, * feat2;
    struct feature* feat1_t = NULL;

	CvMat *BBoxf1; //frame 1
	CvMat *Hf1;
			
	double px_min, px_max,py_min, py_max;
	double p1_x, p2_x, p3_x, p4_x;
	double p1_y, p2_y, p3_y, p4_y;
	double p1_x1, p2_x1, p3_x1, p4_x1;
	double p1_y1, p2_y1, p3_y1, p4_y1;
	
	double P1_x, P1_y, P2_x, P2_y, P3_x, P3_y, P4_x, P4_y;
			
	double Box_width;
	double Box_height;
	double lambda_1, lambda_2, lambda_3, lambda_4, lambda_5;
	double lambda_num, lambda_den;

    double lambdad_1, lambdad_2, lambdad_3, lambdad_4, lambdad_5;
    double lambda_it_1,lambda_it_2,lambda_it_3,lambda_it_4;
			
	struct image_border ROI_frame1, ROI_frame2, ROI_frame3, ROI_frame4, ROI_frame5;
	struct image_border ROI_frame_X;
	struct image_border ROI_frame1_inv;
	struct image_border ROI_frame2_inv, ROI_frame3_inv, ROI_frame4_inv, ROI_frame5_inv;
	struct image_border ROI_mosaic_img1; /// To be used to create synthetic data 
	struct RegionS  S1, S2, S3, S4, S5;

	IplImage* dst1, *dst2, *dst3, *dst4, *dst5;
	//IplImage* dst2_inv, *dst3_inv, *dst4_inv, *dst5_inv;
	//IplImage *dst4_inv, *dst5_inv;

	IplImage* ROI_panorama5_img1 = NULL;

	IplImage* GradientMosaic = NULL, *prev_GradientMosaic = NULL, *prev_pseudoHessianMosaic = NULL, *pseudoHessianMosaic = NULL, *npseudoHessianMosaic = NULL;
	IplImage* SRCG_r = NULL, *SRCG_w = NULL;
	IplImage* SRCG_wf=NULL,*SRCG_rf=NULL,*p=NULL, *p_u8=NULL,*alphap=NULL,*alphaw=NULL,*alphap_u8=NULL,*bethap=NULL,*pc=NULL,*pct=NULL,*wc=NULL;
	IplImage* prev_Pn = NULL, *Pn = NULL;

	CvSize dst2_inv_size, dst3_inv_size, dst4_inv_size, dst5_inv_size;
	CvSize panorama_size, sizep;
	
	CvPoint2D32f srcQuad[4], dstQuad[4];
	CvPoint P1, P2, P3, P4;
	CvPoint offsets;
	CvPoint2D32f P_test, P; // P for the mosaic

	float tetha; // test cropping images
    IplImage* test_crop = NULL;

	CvScalar z;
	CvScalar s;
	CvScalar t;
    CvScalar u;
	CvScalar ave;

	int x_length = 0;
	int y_length = 0;
	int x_offset = 0;
	int y_offset = 0; 
	int it = 0;
		
	CvPoint pt1, pt2;
	double d0, d1;
	int n1, n2, k, i,j, m = 0, dx, dy;
	int x, y;
	int n=0;
	int D = 2;
	int levels, levels2;
	unsigned char * ptr;

	int widthROI;  // To create synthetic data 
    int heightROI;

	int option_SR; //Option to compute SR: 0 -> SD, 1 -> CG, 2 -> LM

	int option_Hist_Eq; // 0 
	
	float lambda;
	
	double v1P1[3], v1P2[3], v1P3[3], v1P4[3];
	double v2P1[3], v2P2[3], v2P3[3], v2P4[3];
	double vR[3], vR2[3];

	CvSize sizeD1, sizeD2, sizeD3, sizeD4, sizeD5;
	CvSize size1_inv, size2_inv, size3_inv, size4_inv, size5_inv;
	CvSize img1_ROI;

	FILE *pFile = NULL; // To save the lambda_it in every iteration
	double* lambda_it = NULL;
	double* errorSRp = NULL;
	double* pmp = NULL;
	double *alpha_nump = NULL;
	double eps, normY, p0, alpha, betha, pm;
	double errorSR_LR;

	time_t startltime;
	time_t endltime;

	struct _timeb startltimeb;
	struct _timeb endltimeb;
	char *startltimebuffer;
	char *endltimebuffer;
	typedef double FP;
	FP hours, minutes, seconds;

	char* DICOM_File = "DICOM_Out";
  

	// Interface with VTK to read DICOM Images

	// vtkImageData *VTKImage1;
	// vtkImageData *VTKImage2, *VTKImage3, *VTKImage4, *VTKImage5;//, *f2;

	 vtkSmartPointer<vtkDICOMImageReader> reader1 =  vtkSmartPointer<vtkDICOMImageReader>::New();
	 vtkSmartPointer<vtkDICOMImageReader> reader2 =  vtkSmartPointer<vtkDICOMImageReader>::New();
	 vtkSmartPointer<vtkDICOMImageReader> reader3 =  vtkSmartPointer<vtkDICOMImageReader>::New();
	 vtkSmartPointer<vtkDICOMImageReader> reader4 =  vtkSmartPointer<vtkDICOMImageReader>::New();
	 vtkSmartPointer<vtkDICOMImageReader> reader5 =  vtkSmartPointer<vtkDICOMImageReader>::New();
	
	
	 reader1->SetFileName(argv[1]);
	 reader2->SetFileName(argv[2]);
	 reader3->SetFileName(argv[3]);
	 reader4->SetFileName(argv[4]);
	 reader5->SetFileName(argv[5]);

  //actualizando
	reader1->Update();
	reader2->Update();
	reader3->Update();
	reader4->Update();
	reader5->Update();


	VTKImage1 = reader1->GetOutput();
	VTKImage2 = reader2->GetOutput();
	VTKImage3 = reader3->GetOutput();
	VTKImage4 = reader4->GetOutput();
	VTKImage5 = reader5->GetOutput();

/*
    img1p = cvLoadImage( argv[1], 1 );
	if( ! img1p )
		fatal_error( "unable to load image from %s", argv[1] );
	img2p = cvLoadImage( argv[2], 1 );
	if( ! img2p )
		fatal_error( "unable to load image from %s", argv[2] );
	//stacked = stack_imgs( img1, img2 );

	img3p = cvLoadImage( argv[3], 1 );
	if( ! img3p )
		fatal_error( "unable to load image from %s", argv[3] );

	img4p = cvLoadImage( argv[4], 1 );
	if( ! img4p )
		fatal_error( "unable to load image from %s", argv[4] );

	img5p = cvLoadImage( argv[5], 1 );
	if( ! img5p )
		fatal_error( "unable to load image from %s", argv[5] );

*/

//	x_offset = atoi(argv[6]); // test warp perspective
//	y_offset = atoi(argv[7]);
		  
	panorama_size.height = atoi(argv[6]); 
	panorama_size.width = atoi(argv[7]);

	x_offset = atoi(argv[8]); 
	y_offset = atoi(argv[9]);

	option_SR = atoi(argv[10]);

	option_Hist_Eq = atoi(argv[11]);

	if ( option_SR >= 3 || option_SR < 0 )  {
		printf("\n ERROR: Option to select the algoritm to use must be 0,1, or 2 \n");
		printf(" 0: Steep Descent (SD) Algorithm \n");
		printf(" 1: Conjugate Gradient (CG)  Algorithm \n");
		printf(" 2: Levenberg Marquardt (LM) Algorithm \n");
		return -1;
	}
	
	if ( option_Hist_Eq != 0 ){
		if (option_Hist_Eq != 1 )  {
		printf(" option_Hist_Eq  = %d \t value:  %d ", option_Hist_Eq, ( option_Hist_Eq != 0 || option_Hist_Eq != 1 ));
		printf("\n ERROR: Option for Histogram Equalization must be 0 or 1\n");
		printf("0 : Histogram Equalization desactivated \n");
		printf("1 : Histogram Equalization activated \n");
		return -1;
		}
	}



	 
	// Start time
	time( &startltime );
	_ftime( &startltimeb );

	  char *out = "salida";
	  int fact = 1;
      Amp_Factor( argv[1], fact, out );

	img1pp = ConvertDicom2OpenCV(VTKImage1, reader1->GetWidth(), reader1->GetHeight());
	img2pp = ConvertDicom2OpenCV(VTKImage2, reader2->GetWidth(), reader2->GetHeight());
	img3pp = ConvertDicom2OpenCV(VTKImage3, reader3->GetWidth(), reader3->GetHeight());
	img4pp = ConvertDicom2OpenCV(VTKImage4, reader4->GetWidth(), reader4->GetHeight());
	img5pp = ConvertDicom2OpenCV(VTKImage5, reader5->GetWidth(), reader5->GetHeight());

	img1ppp = cvCreateImage(cvGetSize(img1pp), IPL_DEPTH_64F, 1);
	img2ppp = cvCreateImage(cvGetSize(img2pp), IPL_DEPTH_64F, 1);
	img3ppp = cvCreateImage(cvGetSize(img3pp), IPL_DEPTH_64F, 1);
	img4ppp = cvCreateImage(cvGetSize(img4pp), IPL_DEPTH_64F, 1);
	img5ppp = cvCreateImage(cvGetSize(img5pp), IPL_DEPTH_64F, 1);
	
	F64to8f(img1pp, img1ppp);
	F64to8f(img2pp, img2ppp);
	F64to8f(img3pp, img3ppp);
	F64to8f(img4pp, img4ppp);
	F64to8f(img5pp, img5ppp);

	img1p = cvCreateImage(cvGetSize(img1pp), IPL_DEPTH_8U, 1);
	img2p = cvCreateImage(cvGetSize(img2pp), IPL_DEPTH_8U, 1);
	img3p = cvCreateImage(cvGetSize(img3pp), IPL_DEPTH_8U, 1);
	img4p = cvCreateImage(cvGetSize(img4pp), IPL_DEPTH_8U, 1);
	img5p = cvCreateImage(cvGetSize(img5pp), IPL_DEPTH_8U, 1);

	
	cvScale(img1ppp,img1p,1,0);
	cvScale(img2ppp,img2p,1,0);
	cvScale(img3ppp,img3p,1,0);
	cvScale(img4ppp,img4p,1,0);
	cvScale(img5ppp,img5p,1,0);



	 /// Convert to grayscale
  //cvtColor( img1p, img1p, CV_BGR2GRAY );

  //img1pppp = cvCreateImage(cvGetSize(img1p), IPL_DEPTH_8U, 1);

 
	if ( option_Hist_Eq == 1 ){
		printf(" Performing Histogram Equaliztion to the input images ...\n");
			cvEqualizeHist(img1p, img1p);
			cvEqualizeHist(img2p, img2p);
			cvEqualizeHist(img3p, img3p);
			cvEqualizeHist(img4p, img4p);
			cvEqualizeHist(img5p, img5p);
	}
  /// Display results

 //   cvNamedWindow( "img1p", 1 );
//	cvShowImage( "img1p", img1p );
//	cvWaitKey(0);


	// Initialization
	sizep.height = img1p->height;
	sizep.width = img1p->width - 10;// padding by 5 pixels in each side (left and right)

    allocateOnDemand( &img1, sizep, img1p->depth, img1p->nChannels );
	allocateOnDemand( &img2, sizep, img1p->depth, img1p->nChannels );
	allocateOnDemand( &img3, sizep, img1p->depth, img1p->nChannels );
	allocateOnDemand( &img4, sizep, img1p->depth, img1p->nChannels );
	allocateOnDemand( &img5, sizep, img1p->depth, img1p->nChannels );

	img1 = padcn(img1p,5);
	img2 = padcn(img2p,5);
	img3 = padcn(img3p,5);
	img4 = padcn(img4p,5);
	img5 = padcn(img5p,5);

//	ArrayofImgs = calloc( 5, sizeof( IplImage** ) );

/*
	ArryofImgs[0] = cvClone(img1);
	ArryofImgs[1] = cvClone(img2);
	ArryofImgs[2] = cvClone(img3);
	ArryofImgs[3] = cvClone(img4);
	ArryofImgs[4] = cvClone(img5);
	panorama = ConstrucMosaic4NImgs( &ArryofImgs, 5 );
*/

	//panorama = ConstrucMosaic4NImgs( ArryofImgs );
	/// First part:  Interpolation : resizing the image to 2 times the original size


	sizeD1.height = D*(img1->height);
	sizeD1.width = D*(img1->width);

	sizeD2.height = D*(img2->height);
	sizeD2.width = D*(img2->width);

	sizeD3.height = D*(img2->height);
	sizeD3.width = D*(img2->width);

	sizeD4.height = D*(img4->height);
	sizeD4.width = D*(img4->width);

	sizeD5.height = D*(img5->height);
	sizeD5.width = D*(img5->width);


	size1_inv.height = (img1->height);
	size1_inv.width =  (img1->width);

	size2_inv.height = (img2->height);
	size2_inv.width = (img2->width);

	size3_inv.height = (img2->height);
	size3_inv.width = (img2->width);

	size4_inv.height = (img4->height);
	size4_inv.width = (img4->width);

	size5_inv.height = (img5->height);
	size5_inv.width = (img5->width);
	

	//allocateOnDemand( &Dimg1, sizeD1, img1->depth, img1->nChannels );
	Dimg1 = cvCreateImage(sizeD1, img1->depth, img1->nChannels);
	cvZero(Dimg1);
	//allocateOnDemand( &Dimg2, sizeD2, img1->depth, img1->nChannels );
	Dimg2 = cvCreateImage(sizeD2, img1->depth, img1->nChannels);
	cvZero(Dimg2);
	dst2_wp = cvCreateImage(sizeD2, img1->depth, img1->nChannels);
	cvZero(dst2_wp);

	//allocateOnDemand( &Dimg3, sizeD3, img1->depth, img1->nChannels );
	Dimg3 = cvCreateImage(sizeD3, img1->depth, img1->nChannels);
	cvZero(Dimg3);
	//allocateOnDemand( &Dimg4, sizeD4, img1->depth, img1->nChannels );
	Dimg4 = cvCreateImage(sizeD4, img1->depth, img1->nChannels);
	cvZero(Dimg4);
	//allocateOnDemand( &Dimg5, sizeD5, img1->depth, img1->nChannels );
	Dimg5 = cvCreateImage(sizeD5, img1->depth, img1->nChannels);
	cvZero(Dimg5);


	/*
	img1_inv = cvCreateImage(size1_inv, img1->depth, img1->nChannels);
	cvZero(img1_inv);

	img2_inv = cvCreateImage(size2_inv, img1->depth, img1->nChannels);
	cvZero(img2_inv);

	img3_inv = cvCreateImage(size3_inv, img1->depth, img1->nChannels);
	cvZero(img3_inv);

	img4_inv = cvCreateImage(size4_inv, img1->depth, img1->nChannels);
	cvZero(img4_inv);

	img5_inv = cvCreateImage(size5_inv, img1->depth, img1->nChannels);
	cvZero(img5_inv);
	*/

	dimg1 = cvCreateImage(size1_inv, img1->depth, img1->nChannels);
	cvZero(dimg1);

	dimg2 = cvCreateImage(size1_inv, img1->depth, img1->nChannels);
	cvZero(dimg2);

	dimg3 = cvCreateImage(size1_inv, img1->depth, img1->nChannels);
	cvZero(dimg3);

	dimg4 = cvCreateImage(size1_inv, img1->depth, img1->nChannels);
	cvZero(dimg4);

	dimg5 = cvCreateImage(size1_inv, img1->depth, img1->nChannels);
	cvZero(dimg5);

	printf("Upsampling \n");

	UpSampleImageD(img1, Dimg1);
/*
	cvNamedWindow( "img1", 1 );
    cvShowImage( "img1", img1 );
    
	cvNamedWindow( "Dimg1", 1 );
    cvShowImage( "Dimg1", Dimg1 );
    cvWaitKey(0);
*/
	UpSampleImageD(img2, Dimg2);
	UpSampleImageD(img3, Dimg3);
	UpSampleImageD(img4, Dimg4);
	UpSampleImageD(img5, Dimg5);

	
	IplImageDICOM(DICOM_File, img1);
	//MostrarDICOM(DICOM_File);
	//getchar();
	//VTKI//mage1


/*
	cvResize(img1, Dimg1,CV_INTER_CUBIC );
	cvResize(img2, Dimg2,CV_INTER_CUBIC );
	cvResize(img3, Dimg3,CV_INTER_CUBIC );
	cvResize(img4, Dimg4,CV_INTER_CUBIC );
	cvResize(img5, Dimg5,CV_INTER_CUBIC );

	

	allocateOnDemand( &Uimg1, sizeD1,  img1->depth, img1->nChannels ); 
	allocateOnDemand( &Uimg2, sizeD1,  img1->depth, img1->nChannels );
 	allocateOnDemand( &Uimg3, sizeD1,  img1->depth, img1->nChannels );
	allocateOnDemand( &Uimg4, sizeD1,  img1->depth, img1->nChannels );
	allocateOnDemand( &Uimg5, sizeD1,  img1->depth, img1->nChannels );

	UpSampleImageD(img1, Uimg1, D);
	UpSampleImageD(img2, Uimg2, D);
	UpSampleImageD(img3, Uimg3, D);
	UpSampleImageD(img4, Uimg4, D);
	UpSampleImageD(img5, Uimg5, D);

	cvNamedWindow( "Uimg1", 1 );
    cvShowImage( "Uimg1", Uimg1 );
    cvWaitKey(0);


	cvNamedWindow( "Uimg2", 1 );
    cvShowImage( "Uimg2", Uimg2 );
    cvWaitKey(0);
   
	cvNamedWindow( "Uimg3", 1 );
    cvShowImage( "Uimg3", Uimg3 );
    cvWaitKey(0);

	cvNamedWindow( "Uimg4", 1 );
    cvShowImage( "Uimg4", Uimg4 );
    cvWaitKey(0);

	cvNamedWindow( "Uimg5", 1 );
    cvShowImage( "Uimg5", Uimg5 );
    cvWaitKey(0);
	*/
//	H = CalcTransfMatrix(Dimg1,Dimg2);
	printf("\H12n");
//	PrintElementMatrix(H);
	


	// H12:
	n1 = sift_features( Dimg1, &feat1 );
	n2 = sift_features( Dimg2, &feat2 );	

//	printf("n1 = %d, n2 = %d \n", n1, n2);



	H12 = CalcTransfMatrixF( feat1, feat2,n1, n2, Dimg1->height);
	PrintElementMatrix(H12);

    
	//starts here
	free(feat1);
	n1 = n2;

	feat1 = (feature *)calloc(n1,sizeof(struct feature));
	memcpy(feat1,feat2,n1*sizeof(struct feature));
	free(feat2);	  
	
	// H23:
	n2 = sift_features( Dimg3, &feat2 );	// After this I have feat1 and  feat2
	// ends here
	printf("\n");

	//H23 = CalcTransfMatrix(img2, img3);
	H23 = CalcTransfMatrixF( feat1, feat2,n1, n2, Dimg2->height);
	printf("\H23n");
	PrintElementMatrix(H23);
	printf("\n");

	free(feat1);
	n1 = n2;
	feat1 = (feature *) calloc(n1, sizeof(struct feature));
	memcpy(feat1,feat2,n1*sizeof(struct feature));
	free(feat2);

	// H34
	n2 = sift_features( Dimg4, &feat2 );	

	//printf("\n");

	//H23 = CalcTransfMatrix(img2, img3);
	H34 = CalcTransfMatrixF( feat1, feat2,n1, n2, Dimg3->height);
	printf("\nH34");
	PrintElementMatrix(H34);
	printf("\n");
    
	free(feat1);
	n1 = n2;
	feat1 = (feature *)calloc(n1, sizeof(struct feature));
	memcpy(feat1,feat2,n1*sizeof(struct feature));
	free(feat2);

// H45
	n2 = sift_features( Dimg5, &feat2 );	
	//printf("\n");

	H45 = CalcTransfMatrixF( feat1, feat2,n1, n2, Dimg3->height);
	printf("\nH45");
	PrintElementMatrix(H45);
	printf("\n");
	//getchar();
    
	free(feat1);
	n1 = n2;
	feat1 = (feature *)calloc(n1, sizeof(struct feature));
	memcpy(feat1,feat2,n1*sizeof(struct feature));
	free(feat2);
	free(feat1);

	printf("Multiplication of Matrices \n");
	MultHomMat2(H12,H23,H13);
	//printf("\H13n");
	//PrintElementMatrix(H13);
	//H34 = CalcTransfMatrix(img3, img4);
	//printf("\H34n");
	//PrintElementMatrix(H34);
	//printf("\n");
	MultHomMat2(H13,H34,H14);
	//printf("\H14n");
	//PrintElementMatrix(H14);
	//printf("\n");
	//H45 = CalcTransfMatrix(img4, img5);
	//printf("\H45n");
	//PrintElementMatrix(H45);
	//printf("\n");
	//printf("\H15n");
	MultHomMat2(H14,H45,H15);
	//PrintElementMatrix(H15);
	//printf("\n");

         
	panorama5 = cvCreateImage( panorama_size , IPL_DEPTH_8U , img1p->nChannels ); // the original size was 1360 by 640
	  cvZero( panorama5 );

	  WarpImgvideo =  cvCreateImage( panorama_size , IPL_DEPTH_8U , img1p->nChannels ); 
	  cvZero( WarpImgvideo );
	  
	   dst1 = cvCreateImage( panorama_size , IPL_DEPTH_8U , img1p->nChannels ); 
	   cvZero(dst1);
	   dst1->origin = Dimg1->origin;

	   dst2 = cvCreateImage( panorama_size , IPL_DEPTH_8U , img1p->nChannels ); 
	   cvZero(dst2);
	   dst2->origin = Dimg2->origin;

	  // dst2_inv = cvCreateImage( cvSize(1360, 640) , IPL_DEPTH_8U , 3 ); 
	  // cvZero(dst2_inv);
	   //dst2_inv->origin = Dimg2->origin;

	  

	   dst3 = cvCreateImage( panorama_size , IPL_DEPTH_8U , img1p->nChannels ); 
	   cvZero(dst3);
	   dst3->origin = Dimg3->origin;

	   dst4 = cvCreateImage( panorama_size , IPL_DEPTH_8U , img1p->nChannels ); 
	   cvZero(dst4);
	   dst4->origin = Dimg4->origin;

	   dst5 = cvCreateImage( panorama_size , IPL_DEPTH_8U , img1p->nChannels ); 
	   cvZero(dst5);
	   dst5->origin = Dimg5->origin;
	    
 
// Panorama 5

Hf1 = cvCreateMat(3,3,CV_32FC1);
cvSetIdentity(Hf1,cvRealScalar(1));

BBoxf1 = BBoxP0(Dimg1,Hf1);

ROI_frame1 = FindCornersROI(BBoxf1);
ROI_frame2 = FindCornersImg(Dimg2, H12);
ROI_frame3 = FindCornersImg(Dimg3, H13);
ROI_frame4 = FindCornersImg(Dimg4, H14);
ROI_frame5 = FindCornersImg(Dimg5, H15);

//ROI_frame_X = FindCornersROI(BBoxf1);

 ROI_frame1.P1.x = ROI_frame1.P1.x + x_offset;
 ROI_frame1.P1.y = ROI_frame1.P1.y + y_offset;
 ROI_frame1.P2.x = ROI_frame1.P2.x + x_offset;
 ROI_frame1.P2.y = ROI_frame1.P2.y + y_offset;
 ROI_frame1.P3.x = ROI_frame1.P3.x + x_offset;
 ROI_frame1.P3.y = ROI_frame1.P3.y + y_offset;
 ROI_frame1.P4.x = ROI_frame1.P4.x + x_offset;
 ROI_frame1.P4.y = ROI_frame1.P4.y + y_offset;


 WarpImageUnderQuad(Dimg1,dst1,ROI_frame1);
 //cvNamedWindow( "Perspective_Warp1", 1 );
 //cvShowImage( "Perspective_Warp1", dst1 );
 //cvWaitKey(0);
// cvSaveImage("Perspective_Warp1.jpg",dst1);


 ROI_frame2.P1.x = ROI_frame2.P1.x + x_offset;
 ROI_frame2.P1.y = ROI_frame2.P1.y + y_offset;
 ROI_frame2.P2.x = ROI_frame2.P2.x + x_offset;
 ROI_frame2.P2.y = ROI_frame2.P2.y + y_offset;
 ROI_frame2.P3.x = ROI_frame2.P3.x + x_offset;
 ROI_frame2.P3.y = ROI_frame2.P3.y + y_offset;
 ROI_frame2.P4.x = ROI_frame2.P4.x + x_offset;
 ROI_frame2.P4.y = ROI_frame2.P4.y + y_offset;

  WarpImageUnderQuad(Dimg2,dst2,ROI_frame2);

  //cvNamedWindow( "Perspective_Warp_dst2", 1 );
  //cvShowImage( "Perspective_Warp_dst2", dst2 );
  //cvWaitKey(0);

 ROI_frame3.P1.x = ROI_frame3.P1.x + x_offset;
 ROI_frame3.P1.y = ROI_frame3.P1.y + y_offset;
 ROI_frame3.P2.x = ROI_frame3.P2.x + x_offset;
 ROI_frame3.P2.y = ROI_frame3.P2.y + y_offset;
 ROI_frame3.P3.x = ROI_frame3.P3.x + x_offset;
 ROI_frame3.P3.y = ROI_frame3.P3.y + y_offset;
 ROI_frame3.P4.x = ROI_frame3.P4.x + x_offset;
 ROI_frame3.P4.y = ROI_frame3.P4.y + y_offset;

 WarpImageUnderQuad(Dimg3,dst3,ROI_frame3);
 
 //cvNamedWindow( "Perspective_Warp3", 1 );
 //cvShowImage( "Perspective_Warp3", dst3 );
 //cvWaitKey(0);

 /*********************************************************************/


 ROI_frame4.P1.x = ROI_frame4.P1.x + x_offset;
 ROI_frame4.P1.y = ROI_frame4.P1.y + y_offset;
 ROI_frame4.P2.x = ROI_frame4.P2.x + x_offset;
 ROI_frame4.P2.y = ROI_frame4.P2.y + y_offset;
 ROI_frame4.P3.x = ROI_frame4.P3.x + x_offset;
 ROI_frame4.P3.y = ROI_frame4.P3.y + y_offset;
 ROI_frame4.P4.x = ROI_frame4.P4.x + x_offset;
 ROI_frame4.P4.y = ROI_frame4.P4.y + y_offset;

 WarpImageUnderQuad(Dimg4,dst4,ROI_frame4);

 /*********************************************************************/
 ROI_frame5.P1.x = ROI_frame5.P1.x + x_offset;
 ROI_frame5.P1.y = ROI_frame5.P1.y + y_offset;
 ROI_frame5.P2.x = ROI_frame5.P2.x + x_offset;
 ROI_frame5.P2.y = ROI_frame5.P2.y + y_offset;
 ROI_frame5.P3.x = ROI_frame5.P3.x + x_offset;
 ROI_frame5.P3.y = ROI_frame5.P3.y + y_offset;
 ROI_frame5.P4.x = ROI_frame5.P4.x + x_offset;
 ROI_frame5.P4.y = ROI_frame5.P4.y + y_offset;


 WarpImageUnderQuad(Dimg5,dst5,ROI_frame5);

 /*
 allocateOnDemand( &X1_dst, sizeD1, img1->depth, img1->nChannels );
 allocateOnDemand( &X2_dst, sizeD1, img1->depth, img1->nChannels );
 allocateOnDemand( &X3_dst, sizeD1, img1->depth, img1->nChannels );
 allocateOnDemand( &X4_dst, sizeD1, img1->depth, img1->nChannels );
 allocateOnDemand( &X5_dst, sizeD1, img1->depth, img1->nChannels );

 WarpImageUnderQuad(Dimg1,X1_dst,ROI_frame_X);
 WarpImageUnderQuad(Dimg2,X2_dst,ROI_frame_X);
 WarpImageUnderQuad(Dimg3,X3_dst,ROI_frame_X);
 WarpImageUnderQuad(Dimg4,X4_dst,ROI_frame_X);
 WarpImageUnderQuad(Dimg5,X5_dst,ROI_frame_X);

 */
 
/*
 cvSaveImage("X1.jpg", X1_dst);
 cvSaveImage("X2.jpg", X2_dst);
 cvSaveImage("X3.jpg", X3_dst);
 cvSaveImage("X4.jpg", X4_dst);
 cvSaveImage("X5.jpg", X5_dst);

 cvShowManyImages("Images of X",5, X1_dst, X2_dst, X3_dst, X4_dst, X5_dst);

*/

////////////////////////////
// Finding the region S2  //
////////////////////////////

   S2 = FindSRegion(ROI_frame2);


	  for(i = 0; i < panorama5->height; i ++){ //rows
		  for(j = 0; j < panorama5->width; j++){ //cols
			  P.x = j;
			  P.y = i;
			  if(Belongs2S(P,S2))cvSet2D(panorama5,i,j,cvGet2D(dst2,i,j));
			  else cvSet2D(panorama5,i,j,cvGet2D(dst1,i,j));			  
		  }//end for
	  }// end for

	//  cvNamedWindow( "panorama5", 1 );
	//  cvShowImage( "panorama5", panorama5 );
	 // cvWaitKey(0);
	  //cvSaveImage("panorama5.jpg",panorama5);


////////////////////////////
// Finding the region S3 //
////////////////////////////

   S3 = FindSRegion(ROI_frame3);


	  for(i = 0; i < panorama5->height; i ++){ //rows
		  for(j = 0; j < panorama5->width; j++){ //cols
			  P.x = j;
			  P.y = i;
			  if(Belongs2S(P,S3))cvSet2D(panorama5,i,j,cvGet2D(dst3,i,j));
			  else cvSet2D(panorama5,i,j,cvGet2D(panorama5,i,j));			  
		  }//end for
	  }// end for

	//  cvNamedWindow( "panorama5", 1 );
	//  cvShowImage( "panorama5", panorama5 );
	//  cvWaitKey(0);
	  //cvSaveImage("panorama5.jpg",panorama5);



	  ////////////////////////////
// Finding the region S4 //
////////////////////////////

   S4 = FindSRegion(ROI_frame4);


	  for(i = 0; i < panorama5->height; i ++){ //rows
		  for(j = 0; j < panorama5->width; j++){ //cols
			  P.x = j;
			  P.y = i;
			  if(Belongs2S(P,S4))cvSet2D(panorama5,i,j,cvGet2D(dst4,i,j));
			  else cvSet2D(panorama5,i,j,cvGet2D(panorama5,i,j));			  
		  }//end for
	  }// end for

	 // cvNamedWindow( "panorama5", 1 );
	 // cvShowImage( "panorama5", panorama5 );
	 // cvWaitKey(0);
	  //cvSaveImage("panorama5.jpg",panorama5);


	    ////////////////////////////
		// Finding the region S4 //
		////////////////////////////

   S5 = FindSRegion(ROI_frame5);


	  for(i = 0; i < panorama5->height; i ++){ //rows
		  for(j = 0; j < panorama5->width; j++){ //cols
			  P.x = j;
			  P.y = i;
			  if(Belongs2S(P,S5))cvSet2D(panorama5,i,j,cvGet2D(dst5,i,j));
			  else cvSet2D(panorama5,i,j,cvGet2D(panorama5,i,j));			  
		  }//end for
	  }// end for

	//cvNamedWindow( "panorama5", 1 );
	//cvShowImage( "panorama5", panorama5 );
	//cvWaitKey(0);
	//cvSaveImage("panorama5.jpg",panorama5);

	 
      allocateOnDemand( &prev_panorama, cvSize(panorama5->width,panorama5->height),  img1->depth, img1->nChannels );  
	  allocateOnDemand( &panoramaSR, cvSize(panorama5->width,panorama5->height),  img1->depth, img1->nChannels );  
	  prev_panorama = cvCloneImage(panorama5);


	 //lambda = 0;

	 /*
     lambda = ComputeLambda(prev_panorama, 3.56, 2);
	 printf("\n lambda = %f", lambda);
	 getchar();
	 */


	   // End time
	time( &endltime );
	_ftime( &endltimeb );

    // Display end time
	endltimebuffer = ctime( &endltime );

	// Display elapsed time
	seconds = ( (FP)endltime + 1e-03*(FP)endltimeb.millitm )
	          - ( (FP)startltime + 1e-03*(FP)startltimeb.millitm );
	minutes = seconds/60.0;
	hours = minutes/60.0;
	seconds -= floor(minutes)*60.0;
	minutes -= floor(hours)*60.0;

		fprintf(stderr, "Elapsed Time:  %04.0f:%02.0f:%06.3f  (HHHH:MM:SS)", floor(hours), floor(minutes), seconds );
		fprintf(stderr, "\n\n\n");



		printf("Computing the Super-resolution ... \n");
	    
   /**************************************************************************************/ 
   //																						
   //   Using the equation (4) of the paper: A Transform-Domain Approach to Super-Resolution //
   //   Mosaicing of Compressed Images                                                       //
   //
   /**************************************************************************************/
		
	// pFile = fopen("lambda_it.txt", "w");
	 //*lambda_it = 0.0;

		if (option_SR == 0) {  // Steepest Descent Method
			lambda_it = (double *)calloc( MAX_NUM_ITERATIONS, sizeof( double ) );

			  while( ++it <= MAX_NUM_ITERATIONS){
		      //printf("\n Working with iteration .....  %d   ....", it);
		        SRMosaicking(img1, img2, img3, img4, img5, prev_panorama, ROI_frame1, ROI_frame2, ROI_frame3, ROI_frame4, ROI_frame5,
				panoramaSR, lambda_it);//, D);
		      // SRMosaickingYi(img1, img2, img3, img4, img5, prev_panorama, ROI_frame1, ROI_frame2, ROI_frame3, ROI_frame4, ROI_frame5,
			  // panoramaSR, D);
		     //  fprintf(pFile, "%f\n", *lambda_it);
        		 cvZero(prev_panorama);
		         prev_panorama = cvCloneImage(panoramaSR);		
			  }
			   // fclose(pFile);
				free(lambda_it);
		}
		else if( option_SR == 1 ) { // Using Conjugated Gradient
		   lambda_it = (double *)calloc( MAX_NUM_ITERATIONS, sizeof( double ) );
           errorSRp = (double *)calloc( MAX_NUM_ITERATIONS, sizeof( double ) );
		   pmp = (double *)calloc( MAX_NUM_ITERATIONS, sizeof( double ) );
		   alpha_nump = (double *)calloc( MAX_NUM_ITERATIONS, sizeof( double ) );
	
		   allocateOnDemand( &GradientMosaic, cvSize(panorama5->width,panorama5->height),  img1->depth, img1->nChannels );  
		   allocateOnDemand( &prev_GradientMosaic, cvSize(panorama5->width,panorama5->height),  img1->depth, img1->nChannels );  
		   Gradient_Mosaic(img1, img2, img3, img4, img5, prev_panorama, ROI_frame1, ROI_frame2, ROI_frame3, ROI_frame4, ROI_frame5, prev_GradientMosaic, lambda_it);
	
		   allocateOnDemand( &pseudoHessianMosaic, cvSize(panorama5->width,panorama5->height),  img1->depth, img1->nChannels );  
		   allocateOnDemand( &prev_pseudoHessianMosaic, cvSize(panorama5->width,panorama5->height),  img1->depth, img1->nChannels );  
		   cvZero(prev_pseudoHessianMosaic);

		   	allocateOnDemand( &SRCG_w, cvSize(panorama5->width,panorama5->height),  img1->depth, img1->nChannels );  
			cvZero(SRCG_w);
			allocateOnDemand( &SRCG_r, cvSize(panorama5->width,panorama5->height),  img1->depth, img1->nChannels );  
			cvZero(SRCG_r);

			SRCG_find_w(img1, img2, img3, img4, img5, prev_panorama, ROI_frame1, ROI_frame2, ROI_frame3, ROI_frame4, ROI_frame5, SRCG_w);
			alpha_nump[0] = SRCG_find_r(img1, img2, img3, img4, img5, prev_panorama, ROI_frame1, ROI_frame2, ROI_frame3, ROI_frame4, ROI_frame5, SRCG_r);

			printf("\n alpha_nump = %f", alpha_nump[0]);

			allocateOnDemand( &prev_Pn, cvSize(panorama5->width,panorama5->height),  img1->depth, img1->nChannels );  
			allocateOnDemand( &Pn, cvSize(panorama5->width,panorama5->height),  img1->depth, img1->nChannels );  

			prev_Pn = cvCloneImage(SRCG_r);
				while( ++it <= MAX_NUM_ITERATIONS - 1){
					printf("\n Working with iteration .....  %d   ....\n", it);

					SRMosaicking_Conjugate_Gradient(img1, img2, img3, img4, img5, prev_panorama, ROI_frame1, ROI_frame2, ROI_frame3, ROI_frame4, ROI_frame5,
	  panoramaSR, prev_Pn, Pn, lambda_it, errorSRp, alpha_nump, it);
		  
		  //fprintf(pFile, "%f\n", *lambda_it);

					cvZero(prev_panorama);
					prev_panorama = cvCloneImage(panoramaSR);		
				    cvZero(prev_Pn);
					prev_Pn = cvCloneImage(Pn);	
						  }
			// fclose(pFile);
					free( lambda_it );
					free( errorSRp );
			 //free( status );
					 free ( alpha_nump );

					errorSR_LR = cvNorm(panoramaSR, panorama5, CV_RELATIVE_L2, NULL);
					printf("\n errorSR_LR = %f", errorSR_LR);
		}
		else if ( option_SR == 2 ) { // Using Levenberg Marquardt    ) 
			lambda_it = (double *)calloc( MAX_NUM_ITERATIONS, sizeof( double ) );
			errorSRp = (double *)calloc( MAX_NUM_ITERATIONS, sizeof( double ) );
	
			allocateOnDemand( &GradientMosaic, cvSize(panorama5->width,panorama5->height),  img1->depth, img1->nChannels );  
			allocateOnDemand( &prev_GradientMosaic, cvSize(panorama5->width,panorama5->height),  img1->depth, img1->nChannels );  
			Gradient_Mosaic(img1, img2, img3, img4, img5, prev_panorama, ROI_frame1, ROI_frame2, ROI_frame3, ROI_frame4, ROI_frame5, prev_GradientMosaic, lambda_it);
	

			allocateOnDemand( &pseudoHessianMosaic, cvSize(panorama5->width,panorama5->height),  img1->depth, img1->nChannels );  
			allocateOnDemand( &prev_pseudoHessianMosaic, cvSize(panorama5->width,panorama5->height),  img1->depth, img1->nChannels );  
			cvZero(prev_pseudoHessianMosaic);
		
			//	cvMul(GradientMosaic, pseudoHessianMosaic, GradientMosaic,1);

			//cvNamedWindow( "GradientMosaic_it0", 1 );
			//cvShowImage( "GradientMosaic_it0", prev_GradientMosaic );
			//cvWaitKey(0);
	       while( ++it <= MAX_NUM_ITERATIONS){
			 printf("\n Working with iteration .....  %d   ....\n", it);
	       		  
	        SRMosaicking_Levenberg_Marquardt(img1, img2, img3, img4, img5, prev_panorama, ROI_frame1, ROI_frame2, ROI_frame3, ROI_frame4, ROI_frame5,
		    panoramaSR, lambda_it, errorSRp, it);
		
		    cvZero(prev_panorama);
		    prev_panorama = cvCloneImage(panoramaSR);		

		    cvZero(prev_GradientMosaic);
		    prev_GradientMosaic = cvCloneImage(GradientMosaic);		

		    cvZero(prev_pseudoHessianMosaic);
		    prev_pseudoHessianMosaic = cvCloneImage(pseudoHessianMosaic);		


	  }
    // fclose(pFile);
	 free( lambda_it );
	 free( errorSRp );
	 //free( status );

	 errorSR_LR = cvNorm(panoramaSR, panorama5, CV_RELATIVE_L2, NULL);
	 printf("\n errorSR_LR = %f", errorSR_LR);



		
		}
	    
		

	 // End time
	time( &endltime );
	_ftime( &endltimeb );

    // Display end time
	endltimebuffer = ctime( &endltime );

	// Display elapsed time
	seconds = ( (FP)endltime + 1e-03*(FP)endltimeb.millitm )
	          - ( (FP)startltime + 1e-03*(FP)startltimeb.millitm );
	minutes = seconds/60.0;
	hours = minutes/60.0;
	seconds -= floor(minutes)*60.0;
	minutes -= floor(hours)*60.0;

		fprintf(stderr, "\n Elapsed Time:  %04.0f:%02.0f:%06.3f  (HHHH:MM:SS)", floor(hours), floor(minutes), seconds );
		fprintf(stderr, "\n\n\n");
	

	cvNamedWindow( "SR_Final", 1 );
	cvShowImage( "SR_Final", panoramaSR );
	//cvWaitKey(0);

	
  //  DICOM_File = "Test1_DICOM";

	printf("Mostrando la Imagen en Formato DICOM \n");




	printf("Guardando la Imagen en Super-Resolucion en Formato DICOM \n");	
	allocateOnDemand( &panoramaSR_32f, cvSize(panorama5->width,panorama5->height),  IPL_DEPTH_32F, 1);  
	cvConvertScale( panoramaSR, panoramaSR_32f, 1.0, 0 );
	
	//IplImageDICOM(DICOM_File, panoramaSR_32f);
	//MostrarDICOM(DICOM_File);

	


   printf("Guardando la Imagen en Super-Resolucion en Formato JPG \n");

	cvSaveImage("SR_Image.jpg",panoramaSR);

	cvNamedWindow( "LR_Image", 1 );
	cvShowImage( "LR_Image", panorama5 );
	cvWaitKey(0);

	cvSaveImage("LR_Image.jpg",panorama5);
  



	cvReleaseMat( &H12 );			
	cvReleaseMat( &BBoxf1 );
	//free(feat1);
	//free(feat2);
	free(feat1_t);
	cvReleaseMat( &Hf1);
	cvReleaseMat( &Htemp);	
	//cvReleaseMat( &H12);
	cvReleaseMat( &H21);
	cvReleaseMat( &H13);
	cvReleaseMat( &H31);
	cvReleaseMat( &H23);	
	cvReleaseMat( &H34);
	cvReleaseMat( &H14);
	cvReleaseMat( &H45);
	cvReleaseMat( &H15);
	cvReleaseMat( &kernelLLt);

	cvReleaseImage( &img1 );
	cvReleaseImage( &img2 );
	cvReleaseImage( &img3 );
	cvReleaseImage( &img4 );
	cvReleaseImage( &img5 );

	cvReleaseImage( &img1pp );
	cvReleaseImage( &img2pp );
	cvReleaseImage( &img3pp );
	cvReleaseImage( &img4pp );
	cvReleaseImage( &img5pp );

	cvReleaseImage( &img1ppp );
	cvReleaseImage( &img2ppp );
	cvReleaseImage( &img3ppp );
	cvReleaseImage( &img4ppp );
	cvReleaseImage( &img5ppp );

	//cvReleaseImage( &img1_inv );
	//cvReleaseImage( &img2_inv );
	//cvReleaseImage( &img3_inv );
	//cvReleaseImage( &img4_inv );
	//cvReleaseImage( &img5_inv );

	cvReleaseImage( &dimg1 );
	cvReleaseImage( &dimg2 );
	cvReleaseImage( &dimg3 );
	cvReleaseImage( &dimg4 );
	cvReleaseImage( &dimg5 );

	cvReleaseImage( &Dimg1 );
	cvReleaseImage( &Dimg2 );
	cvReleaseImage( &Dimg3 );
	cvReleaseImage( &Dimg4 );
	//cvReleaseImage( &dst2_wp );
	cvReleaseImage( &Dimg5 );


	cvReleaseImage( &Ddimg1 );
	cvReleaseImage( &Ddimg2 );
	cvReleaseImage( &Ddimg3 );
	cvReleaseImage( &Ddimg4 );
	cvReleaseImage( &Ddimg5 );

	cvReleaseImage( &panorama5 );	
	cvReleaseImage( &WarpImgvideo );	
	cvReleaseImage( &dst1 );
	cvReleaseImage( &dst2 );
	cvReleaseImage( &dst3 );
	cvReleaseImage( &dst4 );
	cvReleaseImage( &dst5 );
//	cvReleaseImage( &dst2_inv );
//	cvReleaseImage( &dst3_inv );
//	cvReleaseImage( &dst4_inv );
//	cvReleaseImage( &dst5_inv );


	return 0;
}
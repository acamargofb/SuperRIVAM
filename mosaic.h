  
#ifndef MOSAIC_H
#define MOSAIC_H

#include <cxcore.h>
#include <highgui.h>
#include <cv.h>
#include "xform.h"


 #define crossProduct(a,b,c) \
	(a)[0] = (b)[1] * (c)[2] - (c)[1] * (b)[2]; \
	(a)[1] = (b)[2] * (c)[0] - (c)[2] * (b)[0]; \
	(a)[2] = (b)[0] * (c)[1] - (c)[0] * (b)[1];

#ifndef MAX
#define MAX(a,b)  ((a) < (b) ? (b) : (a))
#endif

#ifndef MIN
#define MIN(a,b)  ((a) > (b) ? (b) : (a))
#endif


struct image_data{
  IplImage* img;                  // the image
  struct feature* features;       // image features
  int n;                          // number of image features
  int* distinct_matches;          // distinct matches, index being img id (higher#=better match)
  int* verification;              // listing which of the five matches are verified matches
  int* dist;                      // number of distinct matches in top-5
  CvPoint2D64f top_left;          // top-left point after transformation
  CvPoint2D64f top_right;         // top-right point after transformation
  CvPoint2D64f bottom_left;       // bottom-left point after transformation
  CvPoint2D64f bottom_right;       // bottom-right point after transformation

  // RANSAC Information for Match#1
  CvMat* H1;
  struct feature*** inliers1;
  int* n_in1;

  // RANSAC Information for Match#2
  CvMat* H2;
  struct feature*** inliers2;
  int* n_in2;

  // RANSAC Information for Match#3
  CvMat* H3;
  struct feature*** inliers3;
  int* n_in3;

  // RANSAC Information for Match#4
  CvMat* H4;
  struct feature*** inliers4;
  int* n_in4;

  // RANSAC Information for Match#5
  CvMat* H5;
  struct feature*** inliers5;
  int* n_in5;
};

extern struct component_data{
  int size;               // length of largest connected component;
  int* comp;              // best order of largest connected component
  int* distance;          // distance from image to center (center is first image)
  int* largest_component; // largest connect component
  int* num_links;         // number of links each image has
  int* homog_direction;   // determines whether the homography is own or trg-inverse
  int* usable;            // determines whether the homography is usable or not
  int** adj;              // adjacency matrix
  CvMat** homog;          // array of homographies relating to center image
};


extern struct image_border{
  CvSize ROISize;       // size of the border of the image
  int x_min;	       // minimum value in the x direction (columns)
  int y_min;           // minimum value in the y direction (rows)
  int x_max;	    	//  maximum value in th x direction 
  int y_max;			//  maximum value in th y direction 
  CvPoint2D64f P1;      // 2D coordinates of the first corner
  CvPoint2D64f P2;		// 2D coordinates of the second corner
  CvPoint2D64f P3;		// 2D coordinates of the third corner
  CvPoint2D64f P4;		// 2D coordinates of the fourth corner
};


extern struct RegionS{
  CvPoint3D32f L1;       // Line equation for the points P1 and P2
  CvPoint3D32f L2;		 // Line equation for the points P2 and P3
  CvPoint3D32f L3;		 // Line equation for the points P3 and P4
  CvPoint3D32f L4;		 // Line equation for the points P4 and P1
};

extern struct RegionS1{
  CvPoint3D32f L1;       // Line equation for the points P12 and PI1
  CvPoint3D32f L2;       // Line equation for the points PI1 and P3S
  CvPoint3D32f L3;		 // Line equation for the points P3S and P4S
  CvPoint3D32f L4;		 // Line equation for the points P4S and P5S
  CvPoint3D32f L5;		 // Line equation for the points P5S and P6S
  CvPoint3D32f L6;		 // Line equation for the points P6S and PI3
  CvPoint3D32f L7;		 // Line equation for the points PI3 and P42
  CvPoint3D32f L8;		 // Line equation for the points P42 and P12
  CvPoint3D32f L9;		 // Line equation for the points P42 and P5S
  CvPoint3D32f L10;		 // Line equation for the points P4S and P3S


};
extern struct borderS1{ // case 1 to find the blending mask
	/*
  CvPoint2D64f P12;     // 2D coordinates of the first corner
  CvPoint2D64f PI1;		// 2D coordinates of the second corner
  CvPoint2D64f P3S;		// 2D coordinates of the third corner
  CvPoint2D64f P4S;		// 2D coordinates of the fourth corner
  CvPoint2D64f P5S;		// 2D coordinates of the fifth corner
  CvPoint2D64f P6S;		// 2D coordinates of the sixth corner
  CvPoint2D64f PI3;		// 2D coordinates of the seventh corner
  CvPoint2D64f P42;		// 2D coordinates of the eightth corner
  */
  CvPoint P12;     // 2D coordinates of the first corner
  CvPoint PI1;		// 2D coordinates of the second corner
  CvPoint P3S;		// 2D coordinates of the third corner
  CvPoint P4S;		// 2D coordinates of the fourth corner
  CvPoint P5S;		// 2D coordinates of the fifth corner
  CvPoint P6S;		// 2D coordinates of the sixth corner
  CvPoint PI3;		// 2D coordinates of the seventh corner
  CvPoint P42;		// 2D coordinates of the eightth corn
};

struct dataimgs{
  CvMat* H11;  
  CvMat* H12;  
  CvMat* H13;  
  CvMat* H14;  
  CvMat* H15;  
  IplImage* img1;  
  IplImage* img2;  
  IplImage* img3;  
  IplImage* img4;  
  IplImage* img5;  
};


//The first function here is find_bounds. This takes all of the images in the connected component, locates the corners of their transformed versions, and finds the highest x&y and lowest x&y, and returns those values.

extern double* find_bounds( struct image_data** images, struct component_data* component );

//And I have the rendoring function. Its unfinished and more of a brute-force method, but for testing purposes its working pretty well. The important part is to find the total x-length (distance between x-max and x-min) and y-length. That becomes the dimensions of the panorama image (plus a small buffer). Then, whenever I fill in a pixel in the panorama image, I add the offset to the pixel location (the offset is the absolute value of x-neg & y-neg if <0 ).

extern IplImage* render_panorama( int nImg, struct image_data** images, struct component_data* component );

//extern CVAPI(CvScalar) linear_interpolation(int xp, int yp, IplImage* img, float x, float y);
extern CvScalar linear_interpolation(int xp, int yp, IplImage* img, float x, float y);

//extern IplImage* mosaicing(CvMat *H, IplImage* im1, IplImage* im2, double x_offset, double y_offset, double x_length, double y_length);

extern IplImage* interp2(IplImage* im1, CvMat *H, int x_offset, int y_offset, int x_length, int y_length);

//extern IplImage* interp2ir(IplImage* im1, CvMat *H, int x_offset, int y_offset, int x_length, int y_length);
extern IplImage* interp2ir(IplImage* im1, CvMat *H, int x_offset, int y_offset);

IplImage* interp2ic(IplImage* im1, CvMat *H, int x_offset, int y_offset);

extern void mosaicing(IplImage* im1, IplImage* im2,CvMat *H, int x_offset, int y_offset, int x_length, int y_length);
//extern void draw_features( IplImage* img, struct feature* feat, int n );

extern IplImage* laplace_pyramid(IplImage* img, int level);

extern IplImage* reduce(IplImage* img, int level);
//extern void reduce(IplImage* img);

extern void expand(IplImage* img);

extern void laplacepyr(IplImage* img);

extern IplImage** build_gaussb_pyr( IplImage* base );

extern IplImage** build_laplace_pyr( IplImage* base );

void releaseb_pyr( IplImage*** pyr, int levels );

extern IplImage* expandl( IplImage* img);

extern IplImage* expandl2( IplImage* img, int r, int c );

extern void test_laplace_pyr( IplImage* base );

extern IplImage** build_laplace2_pyr( IplImage* base );

extern IplImage** build_gaussb2_pyr( IplImage* base );

extern void expandlp( IplImage* img );

extern IplImage* equal2( IplImage* img);

extern IplImage** build_gaussb3_pyr( IplImage* base );

extern IplImage** build_laplace3_pyr( IplImage* base );

extern IplImage* findingS( IplImage* img1, IplImage* img2, int x_offset, int y_offset, int add_x, int add_y);

extern IplImage** build_laplace_lpyr( IplImage* base, int levels );

extern IplImage** build_gaussb_lpyr( IplImage* base, int levels );

extern IplImage* compositive( IplImage** laplace_pyr, IplImage** gauss_pyr, int levels );

extern IplImage* mosaic(IplImage* img1, IplImage* img2);

extern IplImage* RGBtoIntensity(IplImage* img);

extern CvMat* CalcTransfMatrix( IplImage* img1, IplImage* img2);

extern CvMat* BBoxP0(IplImage* img, CvMat* H);

extern void WarpImageH(IplImage* im1,IplImage* mosaicing, CvMat *H, int x_offset, int y_offset);

extern void PrintElementMatrix(CvMat *H);

extern struct image_border FindCornersROI(CvMat *BBox);

extern void WarpImagetoBox(IplImage* im1,IplImage* mosaicing, CvMat *H, CvSize Box, int x_offset, int y_offset);

//extern void VideoMosaicking(IplImage* frame1, IplImage* frame2, IplImage* mosaicking, int x_offset, int y_offset, CvMat* H, CvMat* Hf1,CvMat* BBoxf1, CvMat* BBoxf2);

extern CvPoint findOffsets(IplImage* img2, CvMat* H);

extern CvPoint2D64f persp_xform_pt2( CvPoint2D64f pt, CvMat* H);

extern IplImage* mosaickH(IplImage* img1, IplImage* img2, CvMat* H);

extern void displaceImg(IplImage* img1,IplImage* dispImg, int row, int col);

extern void ShiftImage(IplImage* src, IplImage* dst, int x_offset, int y_offset);

extern void WarpImageUnderQuad(IplImage* src, IplImage* dst, struct image_border border);

extern struct RegionS FindSRegion(struct image_border border);

extern int Belongs2S(CvPoint2D32f point, struct RegionS S);// maybe for SR could be float type

extern struct image_border FindCornersImg(IplImage* img2, CvMat* H);

extern void MultHomMat(CvMat* H1, CvMat* H2, CvMat* Hresult);

extern void MultHomMat2(CvMat* H1, CvMat* H2, CvMat* Hresult);

extern CvMat* CalcTransfMatrixF( struct feature* feat1, struct feature* feat2,int n1, int n2, int img1_height);

//extern CvMat* CalcTransfMatrixF_inliers( struct feature* feat1, struct feature* feat2,int n1, int n2, int img1_height, CvMat* CovarianceH);

extern CvMat* CalcTransfMatrixF_inliers( struct feature* feat1, struct feature* feat2,int n1, int n2, int img1_height,  CvMat* Hcv, CvMat* Hcv_Xi, CvMat* Hcv_Xin);

extern IplImage* FrameDifference(IplImage *img1,IplImage *img2);

extern void WarpImageUnderQuadInv(IplImage* src, IplImage* dst, struct image_border border);

extern void SRMosaicking(IplImage* img1, IplImage* img2, IplImage* img3, IplImage* img4, IplImage* img5, IplImage* prev_panorama, 
					struct image_border ROI_frame1, struct image_border ROI_frame2, struct image_border ROI_frame3, struct image_border ROI_frame4, struct image_border ROI_frame5,
					IplImage* panoramaSR, double* lambda_it);//, int D);

extern void SRMosaicking2(IplImage* img1, IplImage* img2, IplImage* img3, IplImage* img4, IplImage* img5, IplImage* prev_panorama, 
					struct image_border ROI_frame1, struct image_border ROI_frame2, struct image_border ROI_frame3, struct image_border ROI_frame4, struct image_border ROI_frame5,
					IplImage* panoramaSR);

static void allocateOnDemand( IplImage **img, CvSize size, int depth, int channels );

extern IplImage* MulImagewithDouble(IplImage* img, double factor);

extern double PSNR(IplImage* X, IplImage* Xhat);

extern void DownSampleImage(IplImage* Img, IplImage* DownSampleImage, int D);

extern void UpSampleImage(IplImage* Img, IplImage* UpSampleImage, int D);

extern void UpSampleImageD(IplImage* Img, IplImage* UpSampleImageD);

extern void cvShowManyImages(char* title, int nArgs, ...);

// extract a ROI from src into dst, based on dx, dy and tetha than can be got from the homography

extern IplImage* CropImageROI(IplImage* src, double dx, double dy, double tetha, int w, int h);

extern void WarpImageUnderROIQuad(IplImage* src, IplImage* dst, struct image_border border_src);

extern void PrintHomography2File( CvMat* H, char* name);

extern double DerivateHubbertFunction(double x, float alpha);

extern float signx(double x);

extern void HubbertRegularization(IplImage* input, IplImage* output, float alpha);

extern void HubbertFunctionCliques(IplImage* input, IplImage* output, float alpha);

extern double HubbertFunction(double x, float alpha);

extern void Regularizationv4(IplImage* X, IplImage* Xregularized, int m, int l, float alpha);

extern void SignImage(IplImage* Img, IplImage* SignImg);

extern void ShiftXdirecction(IplImage* Img, IplImage* ShiftImage, int l);

extern void ShiftYdirecction(IplImage* Img, IplImage* ShiftImage, int m);

extern void padR(IplImage* img, IplImage* imgpadR);

extern IplImage* convert_gray32_to_color( IplImage* gray32 );

extern void FastRobustRegularization(IplImage* Xinit, IplImage* Xregularized, int p, float alpha);

extern void Regularizationv5(IplImage* X, IplImage* Xregularized, int m, int l, double alpha);

extern IplImage* padcn( IplImage* img, int n);

extern float ComputeLambda(IplImage* X, float alpha, int p);

extern double BilateralRegularization(IplImage* Xinit, int p, float alpha);

extern struct RegionS1 FindSRegionS1(struct borderS1 borderS1);

extern struct borderS1 FindBorderS1(struct image_border ROI_frame1, struct image_border ROI_frame2);

//extern IplImage* padcncvg( IplImage* img, int n);

//extern void UpSampleImageDcvg(IplImage* Img, IplImage* UpSampleImageD);

//extern int Belongs2S1(CvPoint2D32f point, struct RegionS1 S1);

extern int Belongs2S1(CvPoint2D32f point, struct RegionS1 S1, struct borderS1 bS1);

extern IplImage* FindRegionR1(struct image_border ROI_frame1, struct image_border ROI_frame2, IplImage* panorama);

extern IplImage** build_gaussb_pyr_blending( IplImage* base, int levels );

extern IplImage** build_laplace_lpyr_blending( IplImage* base, int levels );

extern IplImage* BlendImageswithMask(IplImage* im1, IplImage* im2, IplImage* im3, int levels);

extern IplImage* ReconstructPyramid(IplImage** laplace_pyr, int levels);

extern IplImage* expand_blending(IplImage* input);

extern void speedy_convolution( const CvMat* A, // Size: M1xN1
						 const CvMat* B, //Size: M2xN2
						 CvMat* C // size ( A->rows+B->rows-1 )x( A->cols+B->cols-1 )
						 );

extern IplImage* convert2gray32( IplImage* img );

extern IplImage* convert_gray32_to_color_blending( IplImage* gray32 );

extern IplImage* convert_gray32_to_color_DICOM( IplImage* gray32 );

extern IplImage* downsample_blending( IplImage* img );

extern IplImage* CreateCircularMask(IplImage* img);

extern IplImage* downsample_blending_p( IplImage* img );

extern IplImage* expand_blending_p(IplImage* input);

extern IplImage** build_gaussb_pyr_blending_p( IplImage* base, int levels );

extern IplImage** build_laplace_lpyr_blending_p( IplImage* base, int levels );

extern IplImage* BlendImageswithMask_p(IplImage* im1, IplImage* im2, IplImage* im3, int levels);

extern IplImage* ReconstructPyramid_p(IplImage** laplace_pyr, int levels);

void refine_mask_p(IplImage* panorama5, IplImage* dst_mask_im1, IplImage* dst_mask_im2);

extern struct image_border FindCornersMosaic(struct image_border ROI_frame1, struct image_border ROI_frame2);

extern IplImage* FindMosaicRegion(IplImage* prev_mosaic,struct RegionS new_imag_region);

//extern IplImage* FindInitMosaicRegion(IplImage* dst1,struct RegionS new_imag_region);

extern int Belongs2S1v2(CvPoint2D32f point, struct image_border borderS1);

extern IplImage* FindInitMosaicRegion(IplImage* dst1,struct image_border new_imag_border);

extern int Belongs2Mosaic(CvPoint2D32f point, IplImage* mosaic );

extern void FindIntersectionEdgeMosaic(IplImage* mosaic, struct RegionS RegionNewImage);

extern IplImage* FindIntersectionImageMosaic(IplImage* mosaic, struct RegionS RegionNewImage);

extern void voidFindMosaicRegion(IplImage* mosaic,struct RegionS new_imag_region);

//extern void FindEdgeMosaic(IplImage* mosaic);
extern void FindEdgeMosaic_void(IplImage* mosaic);

extern IplImage* FindEdgeMosaic(IplImage* mosaic);

extern double ComputeMedianKernel(double* kernel, int n);

extern void RefineEdgeMosaic(IplImage* mosaic, IplImage* edge_mosaic);

//extern float median_vector(double* vector);

extern void RefineEdgeBinaryMosaic(IplImage* mosaic, IplImage* edge_mosaic, int delta);

//extern IplImage* MosaicWithBlending(IplImage* prev_mosaic, IplImage* mosaic, IplImage* circ_mask_u8, struct image_border border1, struct image_border border2, struct RegionS Snew, int status);

//extern void MosaicWithBlending(IplImage* prev_mosaic, IplImage* prev_binaryf, IplImage* mosaic, IplImage* binary_panoramaf, IplImage* circ_mask_u8, IplImage* C, struct image_border border1, struct image_border border2, struct RegionS Snew, int status);

extern IplImage* MosaicWithBlending(IplImage* prev_mosaic, IplImage* prev_binaryf, IplImage* mosaic, IplImage* binary_panoramaf, IplImage* circ_mask_u8, struct image_border border1, struct image_border border2, struct RegionS Snew, int status);

extern void void_MosaicWithBlending(IplImage* prev_mosaic, IplImage* prev_binaryf, IplImage* mosaic, IplImage* binary_panoramaf, IplImage* C, IplImage* circ_mask_u8, struct image_border border1, struct image_border border2, struct RegionS Snew, int status);

extern IplImage* Convert2ColumnImage(IplImage* img);

extern void Convert2ColumnMatrix(CvMat* matrix, CvMat* matrix_column);

extern IplImage* Convert2MatrixImage(IplImage* img,int m, int n);

extern void ConvertColumn2MatrixCvMat(CvMat* vector, CvMat* Matrix, int m, int n);

extern void mrqcof(IplImage* inpimg, IplImage* inpinvimg, IplImage* J, IplImage* Jc, IplImage* ep, IplImage* epc, IplImage* gc, IplImage* A);

extern void LevenbergMarquardtNL(IplImage* img1, IplImage* img2, IplImage* img3, IplImage* img4, IplImage* img5, struct image_border ROI_frame1, struct image_border ROI_frame2, struct image_border ROI_frame3, struct image_border ROI_frame4, struct image_border ROI_frame5, IplImage* p0, IplImage* p); 

extern void LevenbergMarquardtNL_Matrix(IplImage* img1, IplImage* img2, IplImage* img3, IplImage* img4, IplImage* img5, struct image_border ROI_frame1, struct image_border ROI_frame2, struct image_border ROI_frame3, struct image_border ROI_frame4, struct image_border ROI_frame5, IplImage* p0, IplImage* p);

extern double MaxValueDiagonal(IplImage* A);

extern double MaxValueDiagonalMatrix(CvMat* A);

extern void ContructAuI(IplImage* A, double mu);

extern void ContructAuI_Matrix(CvMat* A, double mu);

extern void mrqcof_modified(IplImage* img1, IplImage* img2, IplImage* img3, IplImage* img4, IplImage* img5, IplImage* prev_panorama, IplImage* panorama5d, IplImage* panorama5_Delta,
					struct image_border ROI_frame1, struct image_border ROI_frame2, struct image_border ROI_frame3, struct image_border ROI_frame4, struct image_border ROI_frame5,
					double* lambda_itp);//, int D)


extern float multvectort_vector(IplImage* vectort, IplImage* vector);

extern void multvector_vectort(IplImage* vector, IplImage* vectort, IplImage* result);

extern void convertIplImage2CvMat(IplImage* img, CvMat* matrix);

extern void convertCvMat2IplImage(CvMat* matrix, IplImage* img);

extern void MulMatrixwithDouble(CvMat* A, float factor, CvMat* Result);

extern void multvector_vectort_cvMat(CvMat* vector, CvMat* vectort, CvMat* result);

extern void SRMosaicking_Gauss_Newton(IplImage* img1, IplImage* img2, IplImage* img3, IplImage* img4, IplImage* img5, IplImage* prev_panorama, 
					struct image_border ROI_frame1, struct image_border ROI_frame2, struct image_border ROI_frame3, struct image_border ROI_frame4, struct image_border ROI_frame5,
					IplImage* panoramaSR, double* lambda_itp, double* errorSRp, int it);//, int *status);

extern void SRMosaicking_Levenberg_Marquardt(IplImage* img1, IplImage* img2, IplImage* img3, IplImage* img4, IplImage* img5, IplImage* prev_panorama, 
					struct image_border ROI_frame1, struct image_border ROI_frame2, struct image_border ROI_frame3, struct image_border ROI_frame4, struct image_border ROI_frame5,
					IplImage* panoramaSR, double* lambda_itp,double* errorSRp, int it);//, int *status)//, int D)

extern void SRMosaicking_Quasi_Newton_DFP(IplImage* img1, IplImage* img2, IplImage* img3, IplImage* img4, IplImage* img5, IplImage* prev_panorama, 
					struct image_border ROI_frame1, struct image_border ROI_frame2, struct image_border ROI_frame3, struct image_border ROI_frame4, struct image_border ROI_frame5,
					IplImage* panoramaSR, IplImage* prev_GradientMosaic, IplImage* GradientMosaic, IplImage* prev_pseudoHessianMosaic, IplImage* pseudoHessianMosaic, double* lambda_itp,double* errorSRp, int it);

extern void SRMosaicking_Quasi_Newton_DFP_pixel32(IplImage* img1, IplImage* img2, IplImage* img3, IplImage* img4, IplImage* img5, IplImage* prev_panorama, 
					struct image_border ROI_frame1, struct image_border ROI_frame2, struct image_border ROI_frame3, struct image_border ROI_frame4, struct image_border ROI_frame5,
					IplImage* panoramaSR, IplImage* prev_GradientMosaic, IplImage* GradientMosaic, IplImage* prev_pseudoHessianMosaic, IplImage* pseudoHessianMosaic, double* lambda_itp,double* errorSRp, int it);

extern void SRMosaicking_Conjugate_Gradient(IplImage* img1, IplImage* img2, IplImage* img3, IplImage* img4, IplImage* img5, IplImage* prev_panorama, 
					struct image_border ROI_frame1, struct image_border ROI_frame2, struct image_border ROI_frame3, struct image_border ROI_frame4, struct image_border ROI_frame5,
					 IplImage* panoramaSR, IplImage* prev_Pn, IplImage* Pn,  double* lambda_itp, double* errorSRp, double* alpha_nump, int it);


extern void Gradient_Mosaic(IplImage* img1, IplImage* img2, IplImage* img3, IplImage* img4, IplImage* img5, IplImage* prev_panorama, 
					struct image_border ROI_frame1, struct image_border ROI_frame2, struct image_border ROI_frame3, struct image_border ROI_frame4, struct image_border ROI_frame5,
					IplImage* GradientMosaic, double* lambda_itp);

extern void pad2size_panorama(IplImage* input_imgf, IplImage* output_imgf);
/*
extern struct feature* get_match_inliers( struct feature* feat, int mtype );

extern int get_matched_features_inliers( struct feature* features, int n, int mtype, struct feature*** matched );

extern CvMat* ransac_xform_inliers( struct feature* features, int n, int mtype,
					ransac_xform_fn xform_fn, int m, double p_badxform,
					ransac_err_fn err_fn, double err_tol,
					struct feature*** inliers, int* n_in );

extern int find_consensus_inliers( struct feature** features, int n, int mtype,
				   CvMat* M, ransac_err_fn err_fn, double err_tol,
				   struct feature*** consensus );
*/

extern void ComputeCovariance(CvMat* H1_col, CvMat* H2_col, CvMat* H_col,int n_elements, CvMat* CovarianceH);

extern float SRCG_find_r(IplImage* img1, IplImage* img2, IplImage* img3, IplImage* img4, IplImage* img5, IplImage* prev_panorama, 
			struct image_border ROI_frame1, struct image_border ROI_frame2, struct image_border ROI_frame3, struct image_border ROI_frame4, struct image_border ROI_frame5,
			IplImage* SRCG_r);

extern void SRCG_find_w(IplImage* img1, IplImage* img2, IplImage* img3, IplImage* img4, IplImage* img5, IplImage* prev_panorama, 
					struct image_border ROI_frame1, struct image_border ROI_frame2, struct image_border ROI_frame3, struct image_border ROI_frame4, struct image_border ROI_frame5,
					IplImage* SRCG_w);

// extern IplImage* ConstrucMosaic4NImgs(IplImage** Imgs, int NImgs);



#endif 
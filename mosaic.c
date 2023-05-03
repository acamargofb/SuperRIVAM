#include "mosaic.h"
#include <math.h>
#include <stdarg.h>
#include <gsl/gsl_sort.h>
#include <gsl/gsl_statistics.h>
#include <gsl/gsl_sf.h>
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include "utils.h"
#include "sift.h"
#include "kdtree.h"
#include "imgfeatures.h"
#include "xform.h"
//#include "SR2.h"

//local functions:

static __inline void release_mem_inliers( CvPoint2D64f*, CvPoint2D64f*,struct feature** );

//The first function here is find_bounds. This takes all of the images in the connected component, locates the corners of their transformed versions, and finds the highest x&y and lowest x&y, and returns those values.

double* find_bounds( struct image_data** images, struct component_data* component ){
  int i;
  double x_neg = 0.0;
  double x_pos = 0.0;
  double y_neg = 0.0;
  double y_pos = 0.0;
  double neg_x;
  double neg_y;
  double pos_x;
  double pos_y;
  double* bounds; //x-,x+,y-,y+

  bounds = calloc( 4, sizeof( double ) );

  // find the corners of the transformed images
  for( i=0; i<component->size; i++ ){
   // images[component->comp[i]]->img->width-1 ), component->homog[i] );
    images[component->comp[i]]->top_left = persp_xform_pt( cvPoint2D64f( 0, 0 ), component->homog[i] );
    images[component->comp[i]]->top_right = persp_xform_pt( cvPoint2D64f( images[component->comp[i]]->img->width-1, 0 ), component->homog[i] );
    images[component->comp[i]]->bottom_left = persp_xform_pt( cvPoint2D64f( 0, images[component->comp[i]]->img->height-1), component->homog[i] );
    images[component->comp[i]]->bottom_right = persp_xform_pt( cvPoint2D64f( images[component->comp[i]]->img->width-1, images[component->comp[i]]->img->height-1 ), component->homog[i] );
  }
 
  // find the most -/+ x&y
  for( i=0; i<component->size; i++ ){
    neg_x = images[component->comp[i]]->top_left.x;
    neg_y = images[component->comp[i]]->top_left.y;
    pos_x = images[component->comp[i]]->top_left.x;
    pos_y = images[component->comp[i]]->top_left.y;

    if( images[component->comp[i]]->top_right.x < neg_x )
      neg_x = images[component->comp[i]]->top_right.x;
    if( images[component->comp[i]]->top_right.x > pos_x )
      pos_x = images[component->comp[i]]->top_right.x;
    if( images[component->comp[i]]->top_right.y < neg_y )
      neg_y = images[component->comp[i]]->top_right.y;
    if( images[component->comp[i]]->top_right.y > pos_y )
      pos_y = images[component->comp[i]]->top_right.y;

    if( images[component->comp[i]]->bottom_left.x < neg_x )
      neg_x = images[component->comp[i]]->bottom_left.x;
    if( images[component->comp[i]]->bottom_left.x > pos_x )
      pos_x = images[component->comp[i]]->bottom_left.x;
    if( images[component->comp[i]]->bottom_left.y < neg_y )
      neg_y = images[component->comp[i]]->bottom_left.y;
    if( images[component->comp[i]]->bottom_left.y > pos_y )
      pos_y = images[component->comp[i]]->bottom_left.y;

    if( images[component->comp[i]]->bottom_right.x < neg_x )
      neg_x = images[component->comp[i]]->bottom_right.x;
    if( images[component->comp[i]]->bottom_right.x > pos_x )
      pos_x = images[component->comp[i]]->bottom_right.x;
    if( images[component->comp[i]]->bottom_right.y < neg_y )
      neg_y = images[component->comp[i]]->bottom_right.y;
    if( images[component->comp[i]]->bottom_right.y > pos_y )
      pos_y = images[component->comp[i]]->bottom_right.y;
 
    if( pos_x/neg_x > 1.0001 || pos_x/neg_x < 0.9999 ){
      if( neg_x < x_neg ) x_neg = neg_x;
      if( pos_x > x_pos ) x_pos = pos_x;
      if( neg_y < y_neg ) y_neg = neg_y;
      if( pos_y > y_pos ) y_pos = pos_y;
      component->usable[i] = 1;
    }
  }
  bounds[0] = x_neg;
  bounds[1] = x_pos;
  bounds[2] = y_neg;
  bounds[3] = y_pos;

  return bounds;
}

//And I have the rendoring function. Its unfinished and more of a brute-force method, but for testing purposes its working pretty well. The important part is to find the total x-length (distance between x-max and x-min) and y-length. That becomes the dimensions of the panorama image (plus a small buffer). Then, whenever I fill in a pixel in the panorama image, I add the offset to the pixel location (the offset is the absolute value of x-neg & y-neg if <0 ).

IplImage* render_panorama( int nImg, struct image_data** images, struct component_data* component ){
  IplImage* panorama;

  int i;
  int j;
  int k;
  double* bounds;

  time_t begin;
  time_t end;

  int x_length = 0;
  int y_length = 0;
  int x_offset = 0;
  int y_offset = 0; 

  CvPoint2D64f current;
  CvPoint2D64f target;

  CvScalar s;
  CvScalar t;
  CvScalar ave;


  // this call only if NOT using the bundle adjuster
 // component = modify_homog( component, images, nImg );
  //component = bundle_adjustment( nImg, images, component );

  //begin = time( NULL );
//  fprintf( stderr, "[      Rendering Panorama      ]\t" );

  /**/
  bounds = find_bounds( images, component );

  if( bounds[0] < 0 && bounds[1] < 0 )
    x_length = floor(abs(bounds[0]) - abs(bounds[1]));
  else if( bounds[0] < 0 && bounds[1] >= 0 )
    x_length = floor(abs(bounds[0]) + bounds[1]);
  else if( bounds[0] >= 0 && bounds[1] >= 0 )
    x_length = floor(bounds[1] - bounds[0]);
 // else fatal_error( "x-bounds are off\n" );
   else printf( "x-bounds are off\n" );

  if( bounds[2] < 0 && bounds[3] < 0 )
    y_length = floor(abs(bounds[2]) - abs(bounds[3]));
  else if( bounds[2] < 0 && bounds[3] >= 0 )
    y_length = floor(abs(bounds[2]) + bounds[3]);
  else if( bounds[2] >= 0 && bounds[3] >= 0 )
    y_length = floor(bounds[3] - bounds[2]);
  //else fatal_error( "y-bounds are off\n" );
  else printf( "y-bounds are off\n" );

  x_offset = floor(abs(bounds[0]))+5;
  y_offset = floor(abs(bounds[2]))+5;

  panorama = cvCreateImage( cvSize( x_length+10, y_length+10 ) , IPL_DEPTH_8U , 3 );
  cvZero( panorama );

  // add center image to panorama
  //fprintf( stderr, "\nAdding Image#%i to Panorama\n", component->comp[0] );
  for( i=0; i<(images[component->comp[0]]->img->width); i++ ){
    for( j=0; j<(images[component->comp[0]]->img->height); j++ ){
      s=cvGet2D( images[component->comp[0]]->img, j, i );
      cvSet2D( panorama, j+y_offset, i+x_offset, s );
    }
    //display_big_img( panorama, "Panorama" );
    cvWaitKey(2);
  }

  for( k=1; k<component->size; k++ ){
   
    if( component->usable[k] == 1 ){
    //  fprintf( stderr, "Adding Image#%i to Panorama\n", component->comp[k] );

      for( i=0; i<(images[component->comp[k]]->img->width); i++ ){
    for( j=0; j<(images[component->comp[k]]->img->height); j++ ){
      current = cvPoint2D64f( i, j );
      target = persp_xform_pt( current, component->homog[k] );    
      s=cvGet2D( images[component->comp[k]]->img, j, i );
      t=cvGet2D( panorama, target.y+y_offset, target.x+x_offset );

      //fprintf( stderr, "R=%f, G=%f, B=%f\n", t.val[0], t.val[1], t.val[2] );
      if( t.val[0] < 1 && t.val[1] < 1 && t.val[2] < 1 ){
        //fprintf( stderr, "Black Pixel\n" );
        cvSet2D( panorama, target.y+y_offset, target.x+x_offset, s );
      }
      else{
        //fprintf( stderr, "Color Pixel\t" );
        if( s.val[0] < 1 && s.val[1] < 1 && s.val[2] < 1 ){
          //fprintf( stderr, "Black Pixel\n" );
          cvSet2D( panorama, target.y+y_offset, target.x+x_offset, t );
        }
        else{
          //fprintf( stderr, "Color Pixel\n" );
          ave.val[0] = (t.val[0] + s.val[0])/2;
          ave.val[1] = (t.val[1] + s.val[1])/2;
          ave.val[2] = ( t.val[2] + s.val[2])/2;
          cvSet2D( panorama, target.y+y_offset, target.x+x_offset, ave );
        }
      }
    }
    // displays the panorama as its being built
    //display_big_img( panorama, "Panorama" );
    cvWaitKey(2);
      }
    }
  }
  /**/

  end = time( NULL );
  //display_time( begin, end );

  // dummy setting
  //panorama = cvCreateImage( cvSize(1,1) , IPL_DEPTH_8U , 1 );
  cvSaveImage( " panorama.png", panorama,0 );
  return panorama;
}
// Function that performs the bilinear interpolation
//CV_IMPL CvScalar
CvScalar linear_interpolation(int xp, int yp, IplImage* img, float x, float y)
{
  int i,j,m,n,nsx,nsy,x_1,x_2,y_1,y_2;
  float difx, dify, diftx, difty;
  CvScalar z1,z2,z3,z4,zl,zr,z;

 // x : column position where we want to interpolate
 // y : row position where we want to interpolate
 // m : number of rows =   5
 // n : number of columns = 3

  n = img->width; 
  m = img->height; 

  // Working first with the columns
    difx = abs(x - xp);
	nsx = 0;
	for(i = 0; i <= n - 1; i ++){
	 diftx = abs(x - i);
	  if(diftx < difx){
	   nsx = i;
	   difx = diftx;
	  }//if
	 } //for

	// Now work with the rows
    dify = abs(y - yp);
	nsy = 0;
	for(j = 0; j <= m - 1; j++){  
       difty = abs(y - j);
       if(difty < dify){
         nsy = j;
         dify = difty;
        }//if
       }//for
       
  //printf("\n After 2nd for");   
  //printf("\ndifx = %f", difx);
  //printf("\ndify = %f", dify); 
  //printf("\n nsx = %d\t n = %d", nsx,n);   
  //printf("\n nsy = %d\t m = %d", nsy,m);   

   x_1 = nsx;
   x_2 = nsx+1;
   y_1 = nsy;
   y_2 = nsy+1; // Try with nsy++ if it works
   
  //printf("\n x_1 and x_2");  
   
   /*z1.val[0] = (float)cvGet2D(img,nsx,nsy);
   z2.val[0] = (float)cvGet2D(img,nsx+1,nsy);
   z3.val[0] = (float)cvGet2D(img,nsx,nsy+1);
   z4.val[0] = (float)cvGet2D(img,nsx+1,nsy+1); */

   z1 = cvGet2D(img,nsy,nsx); // cvGet2D(img, row_indx, col_indx)
   z2 = cvGet2D(img,nsy,nsx+1);
   z3 = cvGet2D(img,nsy+1,nsx);
   z4 = cvGet2D(img,nsy+1,nsx+1);
   
   zl.val[0] = (z2.val[0]-z1.val[0])*(x-x_1)/(x_2-x_1) + z1.val[0]; // Left position
   zr.val[0] = (z4.val[0]-z3.val[0])*(x-x_1)/(x_2-x_1) + z3.val[0]; // Right position
   z.val[0]  = (zr.val[0]-zl.val[0])*(y-y_1)/(y_2-y_1) + zl.val[0]; // Interpolation
   
   zl.val[1] = (z2.val[1]-z1.val[1])*(x-x_1)/(x_2-x_1) + z1.val[1]; // Left position
   zr.val[1] = (z4.val[1]-z3.val[1])*(x-x_1)/(x_2-x_1) + z3.val[1]; // Right position
   z.val[1]  = (zr.val[1]-zl.val[1])*(y-y_1)/(y_2-y_1) + zl.val[1]; // Interpolation value

   zl.val[2] = (z2.val[2]-z1.val[2])*(x-x_1)/(x_2-x_1) + z1.val[2]; // Left position
   zr.val[2] = (z4.val[2]-z3.val[2])*(x-x_1)/(x_2-x_1) + z3.val[2]; // Right position
   z.val[2]  = (zr.val[2]-zl.val[2])*(y-y_1)/(y_2-y_1) + zl.val[2]; // Interpolation value

       
   //printf("The value of the interpolation is : %f", z);
   //getchar();  
   //return z.val[0];
   return z;
} 


IplImage* interp2(IplImage* im1, CvMat *H, int x_offset, int y_offset, int x_length, int y_length)
{
 IplImage* mosaicing;
 int h, w, x_row, x_col;
 int  y_row, y_col;
 int d = 3, i, j;
 int aux_x, aux_y;
 double aux_min, aux_max;
  
 CvMat *y1 = cvCreateMat(3,4,CV_32FC1);
 CvMat *temp = cvCreateMat(3,4,CV_32FC1);
 CvMat *H1 = cvCreateMat(3,3,CV_32FC1);
 CvMat *H1_inv = cvCreateMat(3,3,CV_32FC1);
 CvMat *box = cvCreateMat(4,1,CV_32FC1);

 CvPoint2D64f current;
 CvPoint2D64f target;

 CvScalar z;  // To use with the linear interpolation
 CvScalar s;
 CvScalar t;
 CvScalar ave;

 CvMat* x;
 CvMat* y;
 CvMat* x2;
 CvMat* y2;
 CvMat* ap;
 CvMat* a;
 //CvMat* adj;

 h = im1->height;
 w = im1->width;

 cvmSet(temp,0,0,1.00);
 cvmSet(temp,0,1,1.00);
 cvmSet(temp,0,2,w);
 cvmSet(temp,0,3,w);
 cvmSet(temp,1,0,1.00);
 cvmSet(temp,1,1,h);
 cvmSet(temp,1,2,h);
 cvmSet(temp,1,3,1.00);
 cvmSet(temp,2,0,1.00);
 cvmSet(temp,2,1,1.00);
 cvmSet(temp,2,2,1.00);
 cvmSet(temp,2,3,1.00);
 

for(i = 0; i < 3; i++){ //row
	for(j = 0; j < 3; j++){ //col
      cvmSet(H1,i,j,cvmGet(H,i,j));
	}}

cvMatMul(H1,temp,y1);
//cvMul(H1,temp,y1,1);

// Part of matlab:
//y(1,:) = y(1,:)./y(3,:);
//y(2,:) = y(2,:)./y(3,:);
for(i = 0; i < 2; i++){ //rows
	for(j=0; j < 4; j ++){ // columns
     cvmSet(y1,i,j,cvmGet(y1,i,j)/cvmGet(y1,2,j));
	 
	}}

printf("\n Values of the temp matrix");
for(i = 0; i < 3; i++){
	for(j = 0; j < 4; j++){
     printf("\n y1(%d,%d) = %f",i,j,cvmGet(y1,i,j));
	}}
 
 aux_min = cvmGet(y1,0,0);
 aux_max = cvmGet(y1,0,0);
 for (i = 0; i < 4; i++){
	 if (aux_min > cvmGet(y1,0,i))
		 aux_min = cvmGet(y1,0,i);
	 if (aux_max < cvmGet(y1,0,i))
	     aux_max = cvmGet(y1,0,i);
 }
 
 cvmSet(box,0,0,ceil(aux_min));
 cvmSet(box,1,0,ceil(aux_max));

 aux_min = cvmGet(y1,1,0);
 aux_max = cvmGet(y1,1,0);

 for (i = 0; i < 4; i++){
	 if (aux_min > cvmGet(y1,1,i))
		 aux_min = cvmGet(y1,1,i);
	 if (aux_max < cvmGet(y1,1,i))
	     aux_max = cvmGet(y1,1,i);
 }
 cvmSet(box,2,0,ceil(aux_min));
 cvmSet(box,3,0,ceil(aux_max));

 for(i = 0; i < 4; i++){
	 printf("\n box(%d,0) = %f",i,cvmGet(box,i,0));
	}

  x_row = cvmGet(box,3,0) - cvmGet(box,2,0);
  x_col = cvmGet(box,1,0) - cvmGet(box,0,0);
  
  ////////////////////////////////////////////////////////
  // Inserting this to see if the mosaicing could change
  //x_row = y_length;
  //x_col = x_length;
  //////////////////////////////////////////////////////

  y_row = x_row;
  y_col = x_col;
  
  printf("\n x_row = %d, \n x_col = %d",x_row, x_col);

  x = cvCreateMat(x_row,x_col,CV_32FC1);
  y = cvCreateMat(y_row,y_col,CV_32FC1);

  for (i = 0; i < x_row; i++){
	  for (j = 0; j < x_col; j++){
          aux_x = cvmGet(box,0,0) + j;
		  cvmSet(x,i,j,aux_x);}}

  // If there is a problem in the future review this because I did get the same answer as Matlab
// The difference make sense, but still it's important to take account this.

  for (i = 0; i < y_col; i++){
	  for (j = 0; j < y_row; j++){
          aux_y = cvmGet(box,2,0) + j;  
		  cvmSet(y,j,i,aux_y);}}

printf("\n Values of the meshgrid matrix x and y");
for(i = 0; i < 4; i++){
	for(j = 0; j < 4; j++){
     printf("\n x(%d,%d) = %f",i,j,cvmGet(x,i,j));
	 printf("\n y(%d,%d) = %f",i,j,cvmGet(y,i,j));
	}} 

  cvInvert(H1,H1_inv,CV_LU);         // Inverting the matrix H1 using LU factorization

  for(i = 0; i < 3; i++){
	for(j = 0; j < 3; j++){
     printf("\n H1_inv(%d,%d) = %f",i,j,cvmGet(H1_inv,i,j));
	}}

    // Implementation of a = inv(warp)*[x(:) y(:) ones(prod(size(x)),1)]';
  x2 = cvCreateMat(1,x_row*x_col,CV_32FC1);
  y2 = cvCreateMat(1,x_row*x_col,CV_32FC1);
  ap = cvCreateMat(3,x_row*x_col,CV_32FC1);
  a = cvCreateMat(3,x_row*x_col,CV_32FC1);

 for (i = 0; i < x_row; i++){
  for (j = 0; j < x_col; j++){
	  cvmSet(x2,0,i*x_col+j,cvmGet(x,i,j));}}
 
 for (i = 0; i < y_row; i++){
  for (j = 0; j < y_col; j++){
	  cvmSet(y2,0,i*y_col+j,cvmGet(y,i,j));}}

  for(i = 0; i < 3; i++){
	    printf("\n x2(0,%d) = %f",i,cvmGet(x2,0,i));
		printf("\n y2(0,%d) = %f",i,cvmGet(y2,0,i));
	}
  for(i = 0; i< x_col*x_row; i++){
    cvmSet(ap,0,i,cvmGet(x2,0,i));}
  for(i = 0; i< x_col*x_row; i++){
    cvmSet(ap,1,i,cvmGet(y2,0,i));}
  for(i = 0; i< x_col*x_row; i++){
    cvmSet(ap,2,i,1.0);}

   cvMatMul(H1_inv,ap,a);
   
	for(i = 0; i < 3; i++){
		for(j=0; j < 3; j++){
			printf("\n a(%d,%d) = %f",i,j,cvmGet(a,i,j));}}
	// Implementing:
	// x(:) = a(1,:)./a(3,:);
    // y(:) = a(2,:)./a(3,:);

  for(i = 0; i< x_col*x_row; i++){
    cvmSet(x2,0,i,cvmGet(a,0,i)/cvmGet(a,2,i));}
  for(i = 0; i< x_col*x_row; i++){
    cvmSet(y2,0,i,cvmGet(a,1,i)/cvmGet(a,2,i));}
  

  printf("\nThe values of the final x and y are");
  for (i = 0; i < x_row; i++){
   for (j = 0; j < x_col; j++){
	  cvmSet(x,i,j,cvmGet(x2,0,j+x_col*i));
	  cvmSet(y,i,j,cvmGet(y2,0,j+x_col*i));}}

	for(i = 0; i < 3; i++){
		for(j=0; j < 3; j++){
			printf("\n x(%d,%d) = %f",i,j,cvmGet(x,i,j));
		    printf("\n y(%d,%d) = %f",i,j,cvmGet(y,i,j));}}
	printf("\n x(4,121)= %f",cvmGet(x,4,121));
	printf("\n y(4,121)= %f",cvmGet(y,4,121));


//
/// Another try with the imag1
//	mosaicing = cvCreateImage( cvSize(800, 600)  , IPL_DEPTH_8U , 3 );
	mosaicing = cvCreateImage( cvSize(x_col, x_row)  , IPL_DEPTH_8U , 3 );
	cvZero( mosaicing );

	printf("\n Memory asssigned to the mosaicing image variable " );

	for( i=0; i<im1->width; i++ ){ //img1->width
      for( j=0; j<im1->height; j++ ){//img1->height
		current = cvPoint2D64f( i, j ); // i->x and j->y
		target = persp_xform_pt( current, H );   //H 
		s=cvGet2D( im1, j, i ); //img1  // Get the pixel value in (j,i) of img1
		if(target.y+y_offset<mosaicing->height && target.y+y_offset>= 0 && target.x+x_offset < mosaicing->width && target.x+x_offset >= 0)
		{
		 cvSet2D( mosaicing, target.y+y_offset, target.x+x_offset, s );
		}   
	}} // End of the for loops

  /*		for( i=0; i<im1->width; i++ ){ //img1->width
          for( j=0; j<im1->height; j++ ){//img1->height
			current = cvPoint2D64f( i, j ); // i->x and j->y
			target = ( current, H );   //H 
			s=cvGet2D( im1, j, i ); //img1  // Get the pixel value in (j,i) of img1
			cvSet2D( mosaicing, target.y+y_offset, target.x+x_offset, s ); }}  */

  printf("\n The first image was warped " );

for (i = 0; i <  x_row; i++){
  for (j = 0; j < x_col; j++){
	  if(cvmGet(x,i,j) > 0 && cvmGet(x,i,j) < im1->width - 1 && cvmGet(y,i,j) > 0 && cvmGet(y,i,j) < im1->height - 1)
	  {
	    z= linear_interpolation(0, 0, im1, cvmGet(x,i,j), cvmGet(y,i,j));
	    cvSet2D( mosaicing, i, j, z ); 
	  }
	  else
	  {
	    z.val[0] = 0;
		z.val[1] = 0;
		z.val[2] = 0;
		cvSet2D( mosaicing, i, j, z ); 
	  }
  }
}

printf("\n The linear interpolation was done " );

//cvNamedWindow( "Mosaicing", 1 );
//cvShowImage( "Mosaicing", mosaicing );
//cvWaitKey(0);


  cvReleaseMat( &y1 );
  cvReleaseMat( &temp );
  cvReleaseMat( &H1 );
  cvReleaseMat( &H1_inv );
  cvReleaseMat( &box );
  cvReleaseMat( &x );
  cvReleaseMat( &y );
  cvReleaseMat( &x2 );
  cvReleaseMat( &y2 );
  cvReleaseMat( &ap );
  cvReleaseMat( &a );
  //cvReleaseImage( &mosaicing );
 
  return mosaicing;
}


void mosaicing(IplImage* im1, IplImage* im2,CvMat *H, int x_offset, int y_offset, int x_length, int y_length)
{
 IplImage* mosaicing;
 IplImage* panorama2;
 int h, w, x_row, x_col;
 int  y_row, y_col;
 int d = 3, i, j;
 int aux_x, aux_y;
 double aux_min, aux_max;
  
 CvMat *y1 = cvCreateMat(3,4,CV_32FC1);
 CvMat *temp = cvCreateMat(3,4,CV_32FC1);
 CvMat *H1 = cvCreateMat(3,3,CV_32FC1);
 CvMat *H1_inv = cvCreateMat(3,3,CV_32FC1);
 CvMat *box = cvCreateMat(4,1,CV_32FC1);

 CvPoint2D64f current;
 CvPoint2D64f target;

 CvScalar z;  // To use with the linear interpolation
 CvScalar s;
 CvScalar t;
 CvScalar ave;

 CvMat* x;
 CvMat* y;
 CvMat* x2;
 CvMat* y2;
 CvMat* ap;
 CvMat* a;
 CvMat* adj;

 h = im1->height;
 w = im1->width;

 cvmSet(temp,0,0,1.00);
 cvmSet(temp,0,1,1.00);
 cvmSet(temp,0,2,w);
 cvmSet(temp,0,3,w);
 cvmSet(temp,1,0,1.00);
 cvmSet(temp,1,1,h);
 cvmSet(temp,1,2,h);
 cvmSet(temp,1,3,1.00);
 cvmSet(temp,2,0,1.00);
 cvmSet(temp,2,1,1.00);
 cvmSet(temp,2,2,1.00);
 cvmSet(temp,2,3,1.00);
 

for(i = 0; i < 3; i++){ //row
	for(j = 0; j < 3; j++){ //col
      cvmSet(H1,i,j,cvmGet(H,i,j));
	}}

cvMatMul(H1,temp,y1);
//cvMul(H1,temp,y1,1);

// Part of matlab:
//y(1,:) = y(1,:)./y(3,:);
//y(2,:) = y(2,:)./y(3,:);
for(i = 0; i < 2; i++){ //rows
	for(j=0; j < 4; j ++){ // columns
     cvmSet(y1,i,j,cvmGet(y1,i,j)/cvmGet(y1,2,j));
	 
	}}

printf("\n Values of the temp matrix");
for(i = 0; i < 3; i++){
	for(j = 0; j < 4; j++){
     printf("\n y1(%d,%d) = %f",i,j,cvmGet(y1,i,j));
	}}
 
 aux_min = cvmGet(y1,0,0);
 aux_max = cvmGet(y1,0,0);
 for (i = 0; i < 4; i++){
	 if (aux_min > cvmGet(y1,0,i))
		 aux_min = cvmGet(y1,0,i);
	 if (aux_max < cvmGet(y1,0,i))
	     aux_max = cvmGet(y1,0,i);
 }
 
 cvmSet(box,0,0,ceil(aux_min));
 cvmSet(box,1,0,ceil(aux_max));

 aux_min = cvmGet(y1,1,0);
 aux_max = cvmGet(y1,1,0);

 for (i = 0; i < 4; i++){
	 if (aux_min > cvmGet(y1,1,i))
		 aux_min = cvmGet(y1,1,i);
	 if (aux_max < cvmGet(y1,1,i))
	     aux_max = cvmGet(y1,1,i);
 }
 cvmSet(box,2,0,ceil(aux_min));
 cvmSet(box,3,0,ceil(aux_max));

 for(i = 0; i < 4; i++){
	 printf("\n box(%d,0) = %f",i,cvmGet(box,i,0));
	}

  x_row = cvmGet(box,3,0) - cvmGet(box,2,0);
  x_col = cvmGet(box,1,0) - cvmGet(box,0,0);
  
  ////////////////////////////////////////////////////////
  // Inserting this to see if the mosaicing could change
  //x_row = y_length;
  //x_col = x_length;
  //////////////////////////////////////////////////////

  y_row = x_row;
  y_col = x_col;
  
  printf("\n x_row = %d, \n x_col = %d",x_row, x_col);

  x = cvCreateMat(x_row,x_col,CV_32FC1);
  y = cvCreateMat(y_row,y_col,CV_32FC1);

  for (i = 0; i < x_row; i++){
	  for (j = 0; j < x_col; j++){
          aux_x = cvmGet(box,0,0) + j;
		  cvmSet(x,i,j,aux_x);}}

  // If there is a problem in the future review this because I did get the same answer as Matlab
// The difference make sense, but still it's important to take account this.

  for (i = 0; i < y_col; i++){
	  for (j = 0; j < y_row; j++){
          aux_y = cvmGet(box,2,0) + j;  
		  cvmSet(y,j,i,aux_y);}}

printf("\n Values of the meshgrid matrix x and y");
for(i = 0; i < 4; i++){
	for(j = 0; j < 4; j++){
     printf("\n x(%d,%d) = %f",i,j,cvmGet(x,i,j));
	 printf("\n y(%d,%d) = %f",i,j,cvmGet(y,i,j));
	}} 

  cvInvert(H1,H1_inv,CV_LU);         // Inverting the matrix H1 using LU factorization

  for(i = 0; i < 3; i++){
	for(j = 0; j < 3; j++){
     printf("\n H1_inv(%d,%d) = %f",i,j,cvmGet(H1_inv,i,j));
	}}

    // Implementation of a = inv(warp)*[x(:) y(:) ones(prod(size(x)),1)]';
  x2 = cvCreateMat(1,x_row*x_col,CV_32FC1);
  y2 = cvCreateMat(1,x_row*x_col,CV_32FC1);
  ap = cvCreateMat(3,x_row*x_col,CV_32FC1);
  a = cvCreateMat(3,x_row*x_col,CV_32FC1);

 for (i = 0; i < x_row; i++){
  for (j = 0; j < x_col; j++){
	  cvmSet(x2,0,i*x_col+j,cvmGet(x,i,j));}}
 
 for (i = 0; i < y_row; i++){
  for (j = 0; j < y_col; j++){
	  cvmSet(y2,0,i*y_col+j,cvmGet(y,i,j));}}

  for(i = 0; i < 3; i++){
	    printf("\n x2(0,%d) = %f",i,cvmGet(x2,0,i));
		printf("\n y2(0,%d) = %f",i,cvmGet(y2,0,i));
	}
  for(i = 0; i< x_col*x_row; i++){
    cvmSet(ap,0,i,cvmGet(x2,0,i));}
  for(i = 0; i< x_col*x_row; i++){
    cvmSet(ap,1,i,cvmGet(y2,0,i));}
  for(i = 0; i< x_col*x_row; i++){
    cvmSet(ap,2,i,1.0);}

   cvMatMul(H1_inv,ap,a);
   
	for(i = 0; i < 3; i++){
		for(j=0; j < 3; j++){
			printf("\n a(%d,%d) = %f",i,j,cvmGet(a,i,j));}}
	// Implementing:
	// x(:) = a(1,:)./a(3,:);
    // y(:) = a(2,:)./a(3,:);

  for(i = 0; i< x_col*x_row; i++){
    cvmSet(x2,0,i,cvmGet(a,0,i)/cvmGet(a,2,i));}
  for(i = 0; i< x_col*x_row; i++){
    cvmSet(y2,0,i,cvmGet(a,1,i)/cvmGet(a,2,i));}
  

  printf("\nThe values of the final x and y are");
  for (i = 0; i < x_row; i++){
   for (j = 0; j < x_col; j++){
	  cvmSet(x,i,j,cvmGet(x2,0,j+x_col*i));
	  cvmSet(y,i,j,cvmGet(y2,0,j+x_col*i));}}

	for(i = 0; i < 3; i++){
		for(j=0; j < 3; j++){
			printf("\n x(%d,%d) = %f",i,j,cvmGet(x,i,j));
		    printf("\n y(%d,%d) = %f",i,j,cvmGet(y,i,j));}}
	printf("\n x(4,121)= %f",cvmGet(x,4,121));
	printf("\n y(4,121)= %f",cvmGet(y,4,121));


//
/// Another try with the imag1
//	mosaicing = cvCreateImage( cvSize(800, 600)  , IPL_DEPTH_8U , 3 );
	mosaicing = cvCreateImage( cvSize(x_col, x_row)  , IPL_DEPTH_8U , 3 );
	cvZero( mosaicing );

	printf("\n Memory asssigned to the mosaicing image variable " );

	for( i=0; i<im1->width; i++ ){ //img1->width
      for( j=0; j<im1->height; j++ ){//img1->height
		current = cvPoint2D64f( i, j ); // i->x and j->y
		target = persp_xform_pt( current, H );   //H 
		s=cvGet2D( im1, j, i ); //img1  // Get the pixel value in (j,i) of img1
		if(target.y+y_offset<mosaicing->height && target.y+y_offset>= 0 && target.x+x_offset < mosaicing->width && target.x+x_offset >= 0)
		{
		 cvSet2D( mosaicing, target.y+y_offset, target.x+x_offset, s );
		}   
	}} // End of the for loops

  /*		for( i=0; i<im1->width; i++ ){ //img1->width
          for( j=0; j<im1->height; j++ ){//img1->height
			current = cvPoint2D64f( i, j ); // i->x and j->y
			target = ( current, H );   //H 
			s=cvGet2D( im1, j, i ); //img1  // Get the pixel value in (j,i) of img1
			cvSet2D( mosaicing, target.y+y_offset, target.x+x_offset, s ); }}  */

  printf("\n The first image was warped " );

for (i = 0; i <  x_row; i++){
  for (j = 0; j < x_col; j++){
	  if(cvmGet(x,i,j) > 0 && cvmGet(x,i,j) < im1->width - 1 && cvmGet(y,i,j) > 0 && cvmGet(y,i,j) < im1->height - 1)
	  {
	    z= linear_interpolation(0, 0, im1, cvmGet(x,i,j), cvmGet(y,i,j));
	    cvSet2D( mosaicing, i, j, z ); 
	  }
	  else
	  {
	    z.val[0] = 0;
		z.val[1] = 0;
		z.val[2] = 0;
		cvSet2D( mosaicing, i, j, z ); 
	  }
  }
}

printf("\n The linear interpolation was done " );

cvNamedWindow( "Mosaicing", 1 );
cvShowImage( "Mosaicing", mosaicing );
cvWaitKey(0);
/// Now this part will work with the im2 (the reference one) to do the offset and then the warping of im1 
/// into im2
// Matlab :
// x_offset = 193;
// y_offset = 521;
// adj = [ 1 0 x_offset; 0 1 y_offset; 0 0 1];

 /*adj = cvCreateMat(3,3,CV_64FC1);
 cvmSet(adj,0,0,1.00);
 cvmSet(adj,0,1,0.00);
 cvmSet(adj,0,2,x_offset);
 cvmSet(adj,1,0,0.00);
 cvmSet(adj,1,1,1.00);
 cvmSet(adj,1,2,y_offset);
 cvmSet(adj,2,0,0.00);
 cvmSet(adj,2,1,0.00);
 cvmSet(adj,2,2,1.00); */

panorama2 = cvCreateImage( cvSize( x_length+10+im2->width, y_length+10+im2->height ) , IPL_DEPTH_8U , 3 );
cvZero( panorama2 );

 for( i=0; i<im2->width; i++ ){//img2
          for( j=0; j<im2->height; j++ ){ //img2
           s=cvGet2D(im2, j, i ); //img2
           cvSet2D( panorama2, j+y_offset, i+x_offset, s );
		   }
          } 

/*for( i=0; i<im2->width; i++ ){ //img2
   for( j=0; j<im2->height; j++ ){//img2
		current = cvPoint2D64f( i, j );
		target = ( current, adj );   //  target = adj*current; 

		// Matlab:
		//  panorama(target(1),target(2),k) = im2(i,j,k);
	
		s=cvGet2D( im2, j, i ); //img1  // Get the pixel value in (j,i) of im2
		cvSet2D( panorama2, target.y, target.x, s );//Set the pixel value in target with the value s 
     }
   }*/

// Now working with the 2 images
// This part joint the 2 worked images

  for( i=0; i<mosaicing->width; i++ ){ //img1
    for( j=0; j<mosaicing->height; j++ ){//img1
		target = cvPoint2D64f( i, j );
		s=cvGet2D( mosaicing, j, i );
		t=cvGet2D( panorama2, target.y, target.x ); 
        if( t.val[0] < 1 && t.val[1] < 1 && t.val[2] < 1 ){
		   cvSet2D( panorama2, target.y, target.x, s );//Set the pixel value in target with the value s 
		  }
		else{
		  if( s.val[0] < 1 && s.val[1] < 1 && s.val[2] < 1 ){
			cvSet2D( panorama2, target.y, target.x, t );
		     }//If
		  else{
			ave.val[0] = (t.val[0] + s.val[0])/2;
			ave.val[1] = (t.val[1] + s.val[1])/2;
			ave.val[2] = ( t.val[2] + s.val[2])/2; 
			cvSet2D( panorama2, target.y, target.x, ave );
		   } // Second Else
		 } // First Else
		} // Second For
	  } // First For  

 /*for(i=0;i<mosaicing->height;i++){// height
	 for(j=0;j<mosaicing->width;j++){// width
       z = cvGet2D(mosaicing,i,j);
	   s = cvGet2D(panorama,i,j);
	   if(z.val[0] != 0 && z.val[1] != 0 && z.val[2] != 0){
		   if(s.val[0]!= 0 && s.val[1] != 0 && s.val[2] != 0){
			   //Matlab :   panorama(i,j,k) = newim(i,j,k);
			   s.val[0] = z.val[0];
			   s.val[1] = z.val[1];
			   s.val[2] = z.val[2];
			   cvSet2D( panorama, i, j, s ); 
		   }
		   else {
            // Matlab:  panorama(i,j,k) = (newim(i,j,k) + panorama(i,j,k))/2;
			   s.val[0] = (z.val[0]+s.val[0])/2;
			   s.val[1] = (z.val[1]+s.val[1])/2;
			   s.val[2] = (z.val[2]+s.val[2])/2;
			   cvSet2D( panorama, i, j, s ); 
		        }
	   }   
	   else {
		   // Matlab :if  i > x_offset && j > y_offset
		   if(  i > x_offset && j > y_offset ){
			   // Matlab: panorama(i,j,k) = panorama(i,j,k); 
			   s.val[0] = s.val[0];
			   s.val[1] = s.val[1];
			   s.val[2] = s.val[2];
			   cvSet2D( panorama, i, j, s ); 
		   }
	      }
           //cvSet2D( mosaicing, i, j, z ); 
	  }// Second For
	 } //First For  */



  cvNamedWindow( "Panorama", 1 );
  cvShowImage( "Panorama", panorama2 );
  cvWaitKey(0);

  cvReleaseMat( &y1 );
  cvReleaseMat( &temp );
  cvReleaseMat( &H1 );
  cvReleaseMat( &H1_inv );
  cvReleaseMat( &box );
  cvReleaseMat( &x );
  cvReleaseMat( &y );
  cvReleaseMat( &x2 );
  cvReleaseMat( &y2 );
  cvReleaseMat( &ap );
  cvReleaseMat( &a );
 // cvReleaseMat( &adj );
  cvReleaseImage( &mosaicing );
  cvReleaseImage( &panorama2 );


 //return mosaicing;
}
//IplImage* interp2ir(IplImage* im1, CvMat *H, int x_offset, int y_offset, int x_length, int y_length)
IplImage* interp2ir(IplImage* im1, CvMat *H, int x_offset, int y_offset)
{
 IplImage* mosaicing;
 int h, w, x_row, x_col;
 int  y_row, y_col;
 int d = 3, i, j;
 int aux_x, aux_y;
 double aux_min, aux_max;
  
 CvMat *y1 = cvCreateMat(3,4,CV_32FC1);
 CvMat *temp = cvCreateMat(3,4,CV_32FC1);
 CvMat *H1 = cvCreateMat(3,3,CV_32FC1);
 CvMat *H1_inv = cvCreateMat(3,3,CV_32FC1);
 CvMat *box = cvCreateMat(4,1,CV_32FC1);

 CvPoint2D64f current;
 CvPoint2D64f target;

 CvScalar z;  // To use with the linear interpolation
 CvScalar s;
 CvScalar t;
 CvScalar ave;

 CvMat* x;
 CvMat* y;
 CvMat* x2;
 CvMat* y2;
 CvMat* ap;
 CvMat* a;
 //CvMat* adj;

 h = im1->height;
 w = im1->width;

 cvmSet(temp,0,0,1.00);
 cvmSet(temp,0,1,1.00);
 cvmSet(temp,0,2,w);
 cvmSet(temp,0,3,w);
 cvmSet(temp,1,0,1.00);
 cvmSet(temp,1,1,h);
 cvmSet(temp,1,2,h);
 cvmSet(temp,1,3,1.00);
 cvmSet(temp,2,0,1.00);
 cvmSet(temp,2,1,1.00);
 cvmSet(temp,2,2,1.00);
 cvmSet(temp,2,3,1.00);
 

for(i = 0; i < 3; i++){ //row
	for(j = 0; j < 3; j++){ //col
      cvmSet(H1,i,j,cvmGet(H,i,j));
	}}

cvMatMul(H1,temp,y1);
//cvMul(H1,temp,y1,1);

// Part of matlab:
//y(1,:) = y(1,:)./y(3,:);
//y(2,:) = y(2,:)./y(3,:);
for(i = 0; i < 2; i++){ //rows
	for(j=0; j < 4; j ++){ // columns
     cvmSet(y1,i,j,cvmGet(y1,i,j)/cvmGet(y1,2,j));
	 
	}}

/*
printf("\n Values of the temp matrix");
for(i = 0; i < 3; i++){
	for(j = 0; j < 4; j++){
     printf("\n y1(%d,%d) = %f",i,j,cvmGet(y1,i,j));
	}}
*/
 
 aux_min = cvmGet(y1,0,0);
 aux_max = cvmGet(y1,0,0);
 for (i = 0; i < 4; i++){
	 if (aux_min > cvmGet(y1,0,i))
		 aux_min = cvmGet(y1,0,i);
	 if (aux_max < cvmGet(y1,0,i))
	     aux_max = cvmGet(y1,0,i);
 }
 
 cvmSet(box,0,0,ceil(aux_min));
 cvmSet(box,1,0,ceil(aux_max));

 aux_min = cvmGet(y1,1,0);
 aux_max = cvmGet(y1,1,0);

 for (i = 0; i < 4; i++){
	 if (aux_min > cvmGet(y1,1,i))
		 aux_min = cvmGet(y1,1,i);
	 if (aux_max < cvmGet(y1,1,i))
	     aux_max = cvmGet(y1,1,i);
 }
 cvmSet(box,2,0,ceil(aux_min));
 cvmSet(box,3,0,ceil(aux_max));
/*
 for(i = 0; i < 4; i++){
	 printf("\n box(%d,0) = %f",i,cvmGet(box,i,0));
	}
*/
  x_row = cvmGet(box,3,0) - cvmGet(box,2,0);
  x_col = cvmGet(box,1,0) - cvmGet(box,0,0);
  
  y_row = x_row;
  y_col = x_col;
  
 // printf("\n x_row = %d, \n x_col = %d",x_row, x_col);

  x = cvCreateMat(x_row,x_col,CV_32FC1);
  y = cvCreateMat(y_row,y_col,CV_32FC1);

  for (i = 0; i < x_row; i++){
	  for (j = 0; j < x_col; j++){
          aux_x = cvmGet(box,0,0) + j;
		  cvmSet(x,i,j,aux_x);}}

  // If there is a problem in the future review this because I did get the same answer as Matlab
// The difference make sense, but still it's important to take account this.

  for (i = 0; i < y_col; i++){
	  for (j = 0; j < y_row; j++){
          aux_y = cvmGet(box,2,0) + j;  
		  cvmSet(y,j,i,aux_y);}}

  /*
printf("\n Values of the meshgrid matrix x and y");
for(i = 0; i < 4; i++){
	for(j = 0; j < 4; j++){
     printf("\n x(%d,%d) = %f",i,j,cvmGet(x,i,j));
	 printf("\n y(%d,%d) = %f",i,j,cvmGet(y,i,j));
	}} 
*/
  cvInvert(H1,H1_inv,CV_LU);         // Inverting the matrix H1 using LU factorization
/*
  for(i = 0; i < 3; i++){
	for(j = 0; j < 3; j++){
     printf("\n H1_inv(%d,%d) = %f",i,j,cvmGet(H1_inv,i,j));
	}}
*/

    // Implementation of a = inv(warp)*[x(:) y(:) ones(prod(size(x)),1)]';
  x2 = cvCreateMat(1,x_row*x_col,CV_32FC1);
  y2 = cvCreateMat(1,x_row*x_col,CV_32FC1);
  ap = cvCreateMat(3,x_row*x_col,CV_32FC1);
  a = cvCreateMat(3,x_row*x_col,CV_32FC1);

 for (i = 0; i < x_row; i++){
  for (j = 0; j < x_col; j++){
	  cvmSet(x2,0,i*x_col+j,cvmGet(x,i,j));}}
 
 for (i = 0; i < y_row; i++){
  for (j = 0; j < y_col; j++){
	  cvmSet(y2,0,i*y_col+j,cvmGet(y,i,j));}}
/*
  for(i = 0; i < 3; i++){
	    printf("\n x2(0,%d) = %f",i,cvmGet(x2,0,i));
		printf("\n y2(0,%d) = %f",i,cvmGet(y2,0,i));
	}
*/
  for(i = 0; i< x_col*x_row; i++){
    cvmSet(ap,0,i,cvmGet(x2,0,i));}
  for(i = 0; i< x_col*x_row; i++){
    cvmSet(ap,1,i,cvmGet(y2,0,i));}
  for(i = 0; i< x_col*x_row; i++){
    cvmSet(ap,2,i,1.0);}

   cvMatMul(H1_inv,ap,a);
   /*
	for(i = 0; i < 3; i++){
		for(j=0; j < 3; j++){
			printf("\n a(%d,%d) = %f",i,j,cvmGet(a,i,j));}}
*/
	// Implementing:
	// x(:) = a(1,:)./a(3,:);
    // y(:) = a(2,:)./a(3,:);

  for(i = 0; i< x_col*x_row; i++){
    cvmSet(x2,0,i,cvmGet(a,0,i)/cvmGet(a,2,i));}
  for(i = 0; i< x_col*x_row; i++){
    cvmSet(y2,0,i,cvmGet(a,1,i)/cvmGet(a,2,i));}
  

 // printf("\nThe values of the final x and y are");
  for (i = 0; i < x_row; i++){
   for (j = 0; j < x_col; j++){
	  cvmSet(x,i,j,cvmGet(x2,0,j+x_col*i));
	  cvmSet(y,i,j,cvmGet(y2,0,j+x_col*i));}}
/*
	for(i = 0; i < 3; i++){
		for(j=0; j < 3; j++){
			printf("\n x(%d,%d) = %f",i,j,cvmGet(x,i,j));
		    printf("\n y(%d,%d) = %f",i,j,cvmGet(y,i,j));}}
	printf("\n x(4,121)= %f",cvmGet(x,4,121));
	printf("\n y(4,121)= %f",cvmGet(y,4,121));
*/

	mosaicing = cvCreateImage( cvSize(x_col, x_row)  , IPL_DEPTH_8U , 1 );
	cvZero( mosaicing );
	//cvSetImageROI(im1,cvRect(0,0,x_col,x_row));

	//printf("\n Memory asssigned to the mosaicing image variable " );

	for( i=0; i<im1->width; i++ ){ //img1->width
      for( j=0; j<im1->height; j++ ){//img1->height
		current = cvPoint2D64f( i, j ); // i->x and j->y
		target = persp_xform_pt( current, H );   //H 
		s=cvGet2D( im1, j, i ); //img1  // Get the pixel value in (j,i) of img1
		if(target.y+y_offset<mosaicing->height && target.y+y_offset>= 0 && target.x+x_offset < mosaicing->width && target.x+x_offset >= 0)
		{
		 cvSet2D( mosaicing, target.y+y_offset, target.x+x_offset, s );
		}   
	}} // End of the for loops

  /*		for( i=0; i<im1->width; i++ ){ //img1->width
          for( j=0; j<im1->height; j++ ){//img1->height
			current = cvPoint2D64f( i, j ); // i->x and j->y
			target = ( current, H );   //H 
			s=cvGet2D( im1, j, i ); //img1  // Get the pixel value in (j,i) of img1
			cvSet2D( mosaicing, target.y+y_offset, target.x+x_offset, s ); }}  */

 // printf("\n The first image was warped " );

for (i = 0; i <  x_row; i++){
  for (j = 0; j < x_col; j++){
	  if(cvmGet(x,i,j) > 0 && cvmGet(x,i,j) < im1->width - 1 && cvmGet(y,i,j) > 0 && cvmGet(y,i,j) < im1->height - 1)
	  {
	    z= linear_interpolation(0, 0, im1, cvmGet(x,i,j), cvmGet(y,i,j));
	    cvSet2D( mosaicing, i, j, z ); 
	  }
	  else
	  {
	    z.val[0] = 0;
//		z.val[1] = 0;
//		z.val[2] = 0;
		cvSet2D( mosaicing, i, j, z ); 
	  }
  }
}

//printf("\n The linear interpolation was done " );

//cvNamedWindow( "Mosaicing", 1 );
//cvShowImage( "Mosaicing", mosaicing );
//cvWaitKey(0);


  cvReleaseMat( &y1 );
  cvReleaseMat( &temp );
  cvReleaseMat( &H1 );
  cvReleaseMat( &H1_inv );
  cvReleaseMat( &box );
  cvReleaseMat( &x );
  cvReleaseMat( &y );
  cvReleaseMat( &x2 );
  cvReleaseMat( &y2 );
  cvReleaseMat( &ap );
  cvReleaseMat( &a );
  //cvReleaseImage( &mosaicing );
 
  return mosaicing;
}


IplImage* interp2ic(IplImage* im1, CvMat *H, int x_offset, int y_offset)
{
 IplImage* mosaicing;
 int h, w, x_row, x_col;
 int  y_row, y_col;
 int d = 3, i, j;
 int aux_x, aux_y;
 double aux_min, aux_max;
  
 CvMat *y1 = cvCreateMat(3,4,CV_32FC1);
 CvMat *temp = cvCreateMat(3,4,CV_32FC1);
 CvMat *H1 = cvCreateMat(3,3,CV_32FC1);
 CvMat *H1_inv = cvCreateMat(3,3,CV_32FC1);
 CvMat *box = cvCreateMat(4,1,CV_32FC1);

 CvPoint2D64f current;
 CvPoint2D64f target;

 CvScalar z;  // To use with the linear interpolation
 CvScalar s;
 CvScalar t;
 CvScalar ave;

 CvMat* x;
 CvMat* y;
 CvMat* x2;
 CvMat* y2;
 CvMat* ap;
 CvMat* a;
 //CvMat* adj;

 h = im1->height;
 w = im1->width;

 cvmSet(temp,0,0,1.00);
 cvmSet(temp,0,1,1.00);
 cvmSet(temp,0,2,w);
 cvmSet(temp,0,3,w);
 cvmSet(temp,1,0,1.00);
 cvmSet(temp,1,1,h);
 cvmSet(temp,1,2,h);
 cvmSet(temp,1,3,1.00);
 cvmSet(temp,2,0,1.00);
 cvmSet(temp,2,1,1.00);
 cvmSet(temp,2,2,1.00);
 cvmSet(temp,2,3,1.00);
 

for(i = 0; i < 3; i++){ //row
	for(j = 0; j < 3; j++){ //col
      cvmSet(H1,i,j,cvmGet(H,i,j));
	}}

cvMatMul(H1,temp,y1);
//cvMul(H1,temp,y1,1);

// Part of matlab:
//y(1,:) = y(1,:)./y(3,:);
//y(2,:) = y(2,:)./y(3,:);
for(i = 0; i < 2; i++){ //rows
	for(j=0; j < 4; j ++){ // columns
     cvmSet(y1,i,j,cvmGet(y1,i,j)/cvmGet(y1,2,j));
	 
	}}

/*
printf("\n Values of the temp matrix");
for(i = 0; i < 3; i++){
	for(j = 0; j < 4; j++){
     printf("\n y1(%d,%d) = %f",i,j,cvmGet(y1,i,j));
	}}
*/
 
 aux_min = cvmGet(y1,0,0);
 aux_max = cvmGet(y1,0,0);
 for (i = 0; i < 4; i++){
	 if (aux_min > cvmGet(y1,0,i))
		 aux_min = cvmGet(y1,0,i);
	 if (aux_max < cvmGet(y1,0,i))
	     aux_max = cvmGet(y1,0,i);
 }
 
 cvmSet(box,0,0,ceil(aux_min));
 cvmSet(box,1,0,ceil(aux_max));

 aux_min = cvmGet(y1,1,0);
 aux_max = cvmGet(y1,1,0);

 for (i = 0; i < 4; i++){
	 if (aux_min > cvmGet(y1,1,i))
		 aux_min = cvmGet(y1,1,i);
	 if (aux_max < cvmGet(y1,1,i))
	     aux_max = cvmGet(y1,1,i);
 }
 cvmSet(box,2,0,ceil(aux_min));
 cvmSet(box,3,0,ceil(aux_max));
/*
 for(i = 0; i < 4; i++){
	 printf("\n box(%d,0) = %f",i,cvmGet(box,i,0));
	}
*/
  x_row = cvmGet(box,3,0) - cvmGet(box,2,0);
  x_col = cvmGet(box,1,0) - cvmGet(box,0,0);
  
  y_row = x_row;
  y_col = x_col;
  
 // printf("\n x_row = %d, \n x_col = %d",x_row, x_col);

  x = cvCreateMat(x_row,x_col,CV_32FC1);
  y = cvCreateMat(y_row,y_col,CV_32FC1);

  for (i = 0; i < x_row; i++){
	  for (j = 0; j < x_col; j++){
          aux_x = cvmGet(box,0,0) + j;
		  cvmSet(x,i,j,aux_x);}}

  // If there is a problem in the future review this because I did get the same answer as Matlab
// The difference make sense, but still it's important to take account this.

  for (i = 0; i < y_col; i++){
	  for (j = 0; j < y_row; j++){
          aux_y = cvmGet(box,2,0) + j;  
		  cvmSet(y,j,i,aux_y);}}

  /*
printf("\n Values of the meshgrid matrix x and y");
for(i = 0; i < 4; i++){
	for(j = 0; j < 4; j++){
     printf("\n x(%d,%d) = %f",i,j,cvmGet(x,i,j));
	 printf("\n y(%d,%d) = %f",i,j,cvmGet(y,i,j));
	}} 
*/
  cvInvert(H1,H1_inv,CV_LU);         // Inverting the matrix H1 using LU factorization
/*
  for(i = 0; i < 3; i++){
	for(j = 0; j < 3; j++){
     printf("\n H1_inv(%d,%d) = %f",i,j,cvmGet(H1_inv,i,j));
	}}
*/

    // Implementation of a = inv(warp)*[x(:) y(:) ones(prod(size(x)),1)]';
  x2 = cvCreateMat(1,x_row*x_col,CV_32FC1);
  y2 = cvCreateMat(1,x_row*x_col,CV_32FC1);
  ap = cvCreateMat(3,x_row*x_col,CV_32FC1);
  a = cvCreateMat(3,x_row*x_col,CV_32FC1);

 for (i = 0; i < x_row; i++){
  for (j = 0; j < x_col; j++){
	  cvmSet(x2,0,i*x_col+j,cvmGet(x,i,j));}}
 
 for (i = 0; i < y_row; i++){
  for (j = 0; j < y_col; j++){
	  cvmSet(y2,0,i*y_col+j,cvmGet(y,i,j));}}
/*
  for(i = 0; i < 3; i++){
	    printf("\n x2(0,%d) = %f",i,cvmGet(x2,0,i));
		printf("\n y2(0,%d) = %f",i,cvmGet(y2,0,i));
	}
*/
  for(i = 0; i< x_col*x_row; i++){
    cvmSet(ap,0,i,cvmGet(x2,0,i));}
  for(i = 0; i< x_col*x_row; i++){
    cvmSet(ap,1,i,cvmGet(y2,0,i));}
  for(i = 0; i< x_col*x_row; i++){
    cvmSet(ap,2,i,1.0);}

   cvMatMul(H1_inv,ap,a);
   /*
	for(i = 0; i < 3; i++){
		for(j=0; j < 3; j++){
			printf("\n a(%d,%d) = %f",i,j,cvmGet(a,i,j));}}
*/
	// Implementing:
	// x(:) = a(1,:)./a(3,:);
    // y(:) = a(2,:)./a(3,:);

  for(i = 0; i< x_col*x_row; i++){
    cvmSet(x2,0,i,cvmGet(a,0,i)/cvmGet(a,2,i));}
  for(i = 0; i< x_col*x_row; i++){
    cvmSet(y2,0,i,cvmGet(a,1,i)/cvmGet(a,2,i));}
  

 // printf("\nThe values of the final x and y are");
  for (i = 0; i < x_row; i++){
   for (j = 0; j < x_col; j++){
	  cvmSet(x,i,j,cvmGet(x2,0,j+x_col*i));
	  cvmSet(y,i,j,cvmGet(y2,0,j+x_col*i));}}
/*
	for(i = 0; i < 3; i++){
		for(j=0; j < 3; j++){
			printf("\n x(%d,%d) = %f",i,j,cvmGet(x,i,j));
		    printf("\n y(%d,%d) = %f",i,j,cvmGet(y,i,j));}}
	printf("\n x(4,121)= %f",cvmGet(x,4,121));
	printf("\n y(4,121)= %f",cvmGet(y,4,121));
*/

	mosaicing = cvCreateImage( cvSize(x_col, x_row)  , IPL_DEPTH_8U , 3 );
	cvZero( mosaicing );
	//cvSetImageROI(im1,cvRect(0,0,x_col,x_row));

	//printf("\n Memory asssigned to the mosaicing image variable " );

	for( i=0; i<im1->width; i++ ){ //img1->width
      for( j=0; j<im1->height; j++ ){//img1->height
		current = cvPoint2D64f( i, j ); // i->x and j->y
        target = persp_xform_pt2( current, H );   //H 
		s=cvGet2D( im1, j, i ); //img1  // Get the pixel value in (j,i) of img1
		if(target.y+y_offset<mosaicing->height && target.y+y_offset>= 0 && target.x+x_offset < mosaicing->width && target.x+x_offset >= 0)
		{
		 cvSet2D( mosaicing, target.y+y_offset, target.x+x_offset, s );
		}   
	}} // End of the for loops

  /*		for( i=0; i<im1->width; i++ ){ //img1->width
          for( j=0; j<im1->height; j++ ){//img1->height
			current = cvPoint2D64f( i, j ); // i->x and j->y
			target = ( current, H );   //H 
			s=cvGet2D( im1, j, i ); //img1  // Get the pixel value in (j,i) of img1
			cvSet2D( mosaicing, target.y+y_offset, target.x+x_offset, s ); }}  */

 // printf("\n The first image was warped " );

for (i = 0; i <  x_row; i++){
  for (j = 0; j < x_col; j++){
	  if(cvmGet(x,i,j) > 0 && cvmGet(x,i,j) < im1->width - 1 && cvmGet(y,i,j) > 0 && cvmGet(y,i,j) < im1->height - 1)
	  {
	    z= linear_interpolation(0, 0, im1, cvmGet(x,i,j), cvmGet(y,i,j));
	    cvSet2D( mosaicing, i, j, z ); 
	  }
	  else
	  {
	    z.val[0] = 0;
	    z.val[1] = 0;
		z.val[2] = 0;
		cvSet2D( mosaicing, i, j, z ); 
	  }
  }
}

//printf("\n The linear interpolation was done " );

//cvNamedWindow( "Mosaicing", 1 );
//cvShowImage( "Mosaicing", mosaicing );
//cvWaitKey(0);


  cvReleaseMat( &y1 );
  cvReleaseMat( &temp );
  cvReleaseMat( &H1 );
  cvReleaseMat( &H1_inv );
  cvReleaseMat( &box );
  cvReleaseMat( &x );
  cvReleaseMat( &y );
  cvReleaseMat( &x2 );
  cvReleaseMat( &y2 );
  cvReleaseMat( &ap );
  cvReleaseMat( &a );
  //cvReleaseImage( &mosaicing );
 
  return mosaicing;
}

extern IplImage* laplace_pyramid_down(IplImage* img, int level)
{
 IplImage* result;
 result = cvCreateImage( cvSize(img->width/2, img->height/2)  , IPL_DEPTH_8U , 1 );
 cvPyrDown(img, result, CV_GAUSSIAN_5x5);
 return result;
}

extern IplImage* reduce(IplImage* img, int level)
//extern void reduce(IplImage* img)
{
 CvMat* W;
 IplImage* reduced;
 int i,j,k,l,m,n;
 double Wk,Wl;
 CvScalar s;
 CvScalar t;
 double s1, s2;
 W = cvCreateMat(5,1,CV_32FC1);
 reduced = cvCreateImage( cvSize(img->width/2, img->height/2)  , IPL_DEPTH_8U , 1 );

 cvmSet(W,0,0,-0.05);
 cvmSet(W,1,0,0.25);
 cvmSet(W,2,0,0.4);
 cvmSet(W,3,0,0.25);
 cvmSet(W,4,0,0.05);
 

 for (i = 2; i < img->height/2 - 1; i ++){ // rows
   for (j = 2; j < img->width/2 - 1; j ++){ // columns
      k = 0;
	  s2 = 0;
		for( m = -2; m < 3; m++){
			l = 0;
				for( n = -2; n < 3; n++){
					t=cvGet2D( img, 2*i + m, 2*j + n ); //i rows, j columns
					Wk  = cvmGet(W,k,0);
					Wl  = cvmGet(W,l,0);
                    //printf("\n Values of Wk and Wl are: %f %f  for k =  %d and l = %d", Wk, Wl,k, l);
					//printf("\n Values of img t.val = %f, (t.val)*Wl = %f ", t.val[0], Wk*Wl*t.val[0]);
					s1= (double)t.val[0];
					//s2 = Wk*Wl*s1 + s2;
					s2 = Wk*Wl*t.val[0] + s2;
					s.val[0] = s2;
					//s.val[0] = Wk*Wl*t.val[0];
					//printf("\n Values of the reduced are and s2: %f, %f", s.val[0], s2);
					cvSet2D(reduced,i,j,s); // img, row, col, value
					l++;
				} // 4th For
				k++;
			} // 3rd For
	 } // Second For
 }// First For

  //	 cvNamedWindow( "reduce", 1 );
  //   cvShowImage( "reduce", reduced);
	//cvWaitKey(0);

 return reduced;
//	cvReleaseImage( &reduced);
//	cvReleaseMat( &W );

}


extern void expand(IplImage* img)
{
 CvMat* W;
 IplImage* expanded;
 int i,j,k,l,m,n;
 double Wk,Wl;
 CvScalar s;
 CvScalar t;
 double s1, s2;
 W = cvCreateMat(5,1,CV_32FC1);
 expanded = cvCreateImage( cvSize(2*img->width, 2*img->height)  , IPL_DEPTH_8U , 1 );
 cvZero(expanded);

 cvmSet(W,0,0,-0.05);
 cvmSet(W,1,0,0.25);
 cvmSet(W,2,0,0.4);
 cvmSet(W,3,0,0.25);
 cvmSet(W,4,0,0.05);
 
 //printf("\n size of expanded is height %d, width %d", expanded->height, expanded->width);
// getchar();

 for (i = 2; i < expanded->height - 2; i ++){ // rows
   for (j = 2; j < expanded->width - 2; j ++){ // columns
      k = 0;
	  s2 = 0;
		for( m = -2; m < 3; m++){
			l = 0;
				for( n = -2; n < 3; n++){
					if(fmod(i-m,2.0) == 0)
					{
					 t=cvGet2D( img,  (i - m)/2, (j - n)/2); //i rows, j columns
					 Wk  = cvmGet(W,k,0);
					 Wl  = cvmGet(W,l,0);
                     s2 = 4*Wk*Wl*t.val[0] + s2;
					}
					else
					{
						s2 = 0;
					}
					s.val[0] = s2;
					//s.val[0] = Wk*Wl*t.val[0];
					//printf("\n Values of the expanded are and s2: %f, %f, for i %d, j %d", s.val[0], s2, i, j);
					cvSet2D(expanded,i,j,s); // img, row, col, value
					l++;
				} // 4th For
				k++;
			} // 3rd For
	 } // Second For
 }// First For

  	 cvNamedWindow( "expanded", 1 );
     cvShowImage( "expanded", expanded);
	 cvWaitKey(0);
	 cvReleaseImage( &expanded);
	 cvReleaseMat( &W );
}


extern IplImage** build_gaussb_pyr( IplImage* base )
{
	IplImage** gaussb_pyr;
	int i, o;
	int levels;

    levels = log( MIN( base->width, base->height ) ) / log(2) + 1;

	gaussb_pyr = calloc( levels, sizeof( IplImage** ) );
	for( i = 0; i < levels; i++ )
		gaussb_pyr[i] = calloc( levels, sizeof( IplImage* ) );

	
	for( o = 0; o < levels; o++ )
		{
			if( o == 0 )
				gaussb_pyr[o] = cvCloneImage(base);

			/* base of new octvave is halved image from end of previous octave */

			 else 
			 {  
				// gaussb_pyr[o] = cvCreateImage( cvGetSize(gaussb_pyr[o-1]),IPL_DEPTH_8U, 3 );
				 //cvPyrDown( gaussb_pyr[o],gaussb_pyr[o-1], 7 );	
				gaussb_pyr[o] = downsample( gaussb_pyr[o-1] );				
				cvSmooth(gaussb_pyr[o],gaussb_pyr[o], CV_GAUSSIAN, 5, 5, 0, 0 );
			 }
	   }
	
	return gaussb_pyr;
}

extern IplImage** build_laplace_pyr( IplImage* base )
{
	IplImage** laplace_pyr;
	IplImage** gaussb_pyr;
	IplImage* temp;
	int i, o;
	int levels;

    levels = log( MIN( base->width, base->height ) ) / log(2) + 1;
    printf("\n levels gauss = % d" , levels);
	levels = levels / 2 + 1; // Because the images after that are losing data in the downsampling and upsampling
    printf("\n levels laplace = % d" , levels);
	gaussb_pyr = build_gaussb_pyr( base );

	laplace_pyr = calloc( levels, sizeof( IplImage** ) );

	for( i = 0; i < levels; i++ )
		laplace_pyr[i] = calloc( levels, sizeof( IplImage* ) );

	
	for( o = 0; o < levels; o++ )
		{
			if( o == levels - 1 )
			{
			    printf("\n levels laplace = % d" , o);
                printf("\n last level ");
				//laplace_pyr[o] = cvCreateImage( cvGetSize(gaussb_pyr[o]), IPL_DEPTH_8U, 3);
				temp = cvCreateImage( cvGetSize(gaussb_pyr[o]), IPL_DEPTH_8U, 3);
				cvZero(temp);
				cvSub(gaussb_pyr[o],temp,laplace_pyr[o],NULL );
				//laplace_pyr[o] = cvCloneImage(gaussb_pyr[o]);
			}   
 
			 else 
			 { 
                laplace_pyr[o] = cvCreateImage( cvGetSize(gaussb_pyr[o]), IPL_DEPTH_8U, 3);
//				 temp = cvCloneImage(gaussb_pyr[o]);
//				cvZero(temp);
				temp = cvCreateImage( cvGetSize(gaussb_pyr[o]), IPL_DEPTH_8U, 3);
				temp = expandl(gaussb_pyr[ o + 1 ]);
				printf("\n gaussb_pyr[o] is %d x %d x %d", gaussb_pyr[o]->height , gaussb_pyr[o] ->width, gaussb_pyr[o] ->nChannels);
				printf("\n expandl(gaussb_pyr[o+1]) is %d x %d x %d", temp->height , temp ->width, temp ->nChannels);
				printf("\n laplace_pyr[o] is %d x %d x %d", laplace_pyr[o]->height , laplace_pyr[o] ->width, laplace_pyr[o] ->nChannels);
				//cvSub(gaussb_pyr[o],expandl(gaussb_pyr[ o + 1 ]),laplace_pyr[o],NULL );
				cvSub(gaussb_pyr[o],temp,laplace_pyr[o],NULL );
			 }
	   }
	
	releaseb_pyr( &gaussb_pyr, levels );
	cvReleaseImage( temp );

	return laplace_pyr;
}


void releaseb_pyr( IplImage*** pyr, int levels )
{
	int i;
	for( i = 0; i < levels; i++ )
	{
		cvReleaseImage( &(*pyr)[i] );
		free( (*pyr)[i] );
	}
	free( *pyr );
	*pyr = NULL;
}

extern IplImage* expandl( IplImage* img)
{
	//CvSize sz;
//	IplImage *bigger;
//	sz =cvSize( img->width & -2, img->height & -2 );
//	bigger = cvCreateImage( cvSize(2*sz.width, 2*sz.height),IPL_DEPTH_8U ,3);
  IplImage* bigger = cvCreateImage( cvSize(img->width * 2, img->height * 2),img->depth, img->nChannels);

	cvResize( img, bigger, CV_INTER_NN );

     	cvSmooth(bigger, bigger, CV_GAUSSIAN, 5, 5, 0, 0 );


	//cvPyrUp(img, bigger,7);	

	return bigger;
}

extern void test_laplace_pyr( IplImage* base )
{
	IplImage** laplace_pyr;
	IplImage** gaussb_pyr;
	IplImage* temp;
	int i, o;
	int levels;
    IplImage* timg;
    
//	timg = cvCreateImage( cvSize(base->width, base->height)  , base->depth, base->nChannels );
//	temp = cvCreateImage( cvSize(base->width, base->height)  , base->depth, base->nChannels );
	 
	levels = log( MIN( base->width, base->height ) ) / log(2) + 1;
	levels = levels / 2; // Because the images after that are losing data in the downsampling and upsampling
	
	printf("\n Creating the laplacian pyramid " );
    
	gaussb_pyr = build_gaussb_pyr( base );

	//laplace_pyr = calloc( levels, sizeof( IplImage** ) );

	//for( i = 0; i < levels; i++ )
	//	laplace_pyr[i] = calloc( levels, sizeof( IplImage* ) );

   // cvSub(gaussb_pyr[0],temp,laplace_pyr[0], NULL);

     timg =  cvCreateImage( cvGetSize(gaussb_pyr[0]),IPL_DEPTH_32F, 3 );
	 temp =  cvCreateImage( cvGetSize(gaussb_pyr[0]),IPL_DEPTH_32F, 3 );

	 timg = expandl(gaussb_pyr[1]);
	 cvNamedWindow( "expandl", 1 );
     cvShowImage( "expandl", timg);
	 cvWaitKey(0);
	 printf("\n The parameters of gaussb_pyr[0] are rows x colx depth %d x %d x %d", gaussb_pyr[0]->height, gaussb_pyr[0]->width,gaussb_pyr[0]->nChannels);
     cvSub(gaussb_pyr[0],timg,temp,NULL);
	 
	
 //	cvNamedWindow( "Laplace_0", 1 );
  //  cvShowImage( "Laplace_0", laplace_pyr[0]);
//	cvWaitKey(0);
		
	releaseb_pyr( &gaussb_pyr, levels );
	cvReleaseImage( temp );
	cvReleaseImage( timg );
	
}

extern IplImage** build_laplace2_pyr( IplImage* base )
{
	IplImage** laplace_pyr;
	IplImage** gaussb_pyr;
	IplImage* temp;
	int i, o;
	int levels;
	int r, c, n; // to compute the dimmensions of each level
	CvScalar s;

    levels = log( MIN( base->width, base->height ) ) / log(2) + 1;
    printf("\n levels laplace = % d" , levels);
	
	n = levels - 1;

	gaussb_pyr = build_gaussb2_pyr( base );

	laplace_pyr = calloc( levels, sizeof( IplImage** ) );
	for( i = 0; i < levels; i++ ){
	//	laplace_pyr[i] = calloc( 1, sizeof( IplImage* ) );
	laplace_pyr[i] = cvCreateImage( cvGetSize(gaussb_pyr[i]), IPL_DEPTH_8U, 3);
	}

	for( o = 0; o < levels-1; o++ )
		{
			r = pow(2,n-o) +  1;
            c = pow(2,n-o) +  1;

  
 
				//laplace_pyr[o] = cvCreateImage( cvGetSize(gaussb_pyr[o]), IPL_DEPTH_8U, 3);
				//temp = cvCreateImage( cvGetSize(gaussb_pyr[o]), IPL_DEPTH_8U, 3);
						
				temp = expandl2(gaussb_pyr[ o + 1 ],r,c);
				printf("\n expandl(gaussb_pyr[o+1]) is %d x %d x %d", temp->height , temp ->width, temp ->nChannels);
				printf("\n gaussb_pyr[o] is %d x %d x %d", gaussb_pyr[o]->height , gaussb_pyr[o] ->width, gaussb_pyr[o] ->nChannels);
				
				printf("\n laplace_pyr[o] is %d x %d x %d", laplace_pyr[o]->height , laplace_pyr[o] ->width, laplace_pyr[o] ->nChannels);
				//cvSub(gaussb_pyr[o],expandl(gaussb_pyr[ o + 1 ]),laplace_pyr[o],NULL );
				//cvPyrUp( gaussb_pyr[o + 1],temp, 7 );	
				cvSub(gaussb_pyr[o],temp,laplace_pyr[o],NULL );
				cvReleaseImage( &temp );


	   }


			    printf("\n levels laplace = % d" , o);
                printf("\n last level ");
				printf("\n r = %d and c =  %d", r, c);
				printf("\n gauss_pyr-height x width x channels = %d x %d x %d\n", gaussb_pyr[levels-1]->height,gaussb_pyr[levels-1]->width,gaussb_pyr[levels-1]->nChannels );
                laplace_pyr[levels-1] = cvCreateImage( cvGetSize(gaussb_pyr[levels-1]), IPL_DEPTH_8U, 3);
				s=cvGet2D( gaussb_pyr[levels-1], 0, 0 );
				printf("\n Value of that pixel = %f  %f  %f\n", s.val[0], s.val[1], s.val[2] );
				//laplace_pyr[o] = equal2(gaussb_pyr[o]);
				
				cvSet2D(laplace_pyr[levels-1], 0, 0, s );  
				//laplace_pyr[o] = cvCreateImage( cvGetSize(gaussb_pyr[o]), IPL_DEPTH_8U, 3);
				//temp = cvCreateImage( cvGetSize(gaussb_pyr[o]), IPL_DEPTH_8U, 3);
				//cvZero(temp);
				//cvSub(gaussb_pyr[o],temp,laplace_pyr[o],NULL );
				//laplace_pyr[o] = cvCloneImage(gaussb_pyr[o]);
				 

	
	releaseb_pyr( &gaussb_pyr, levels );
	printf("yo\n");
 
	return laplace_pyr;
}

extern IplImage** build_gaussb2_pyr( IplImage* base )
{
	IplImage** gaussb_pyr;
	int i, o;
	int n;
	int levels;
	int r, c; // row and col for the images in each level

	levels = log( MIN( base->width, base->height ) ) / log(2) + 1;
	n = levels - 1;

	gaussb_pyr = calloc( levels, sizeof( IplImage** ) );
	for( i = 0; i < levels; i++ )
		gaussb_pyr[i] = calloc( levels, sizeof( IplImage* ) );

	for( o = 0; o < levels; o++ )
		{
			r = pow(2,n-o) +  1;
            c = pow(2,n-o) +  1;

			if( o == 0 )
				gaussb_pyr[o] = cvCloneImage(base);

			/* base of new octvave is halved image from end of previous octave */

			 else 
			 {  
		     	if(gaussb_pyr[o - 1] ->height == 3 && gaussb_pyr[o - 1]->width == 3)
				{
				  r = 1;
				  c = 1;
				}		
				gaussb_pyr[o] = cvCreateImage(cvSize(c,r),IPL_DEPTH_8U, 3 );
				cvPyrDown( gaussb_pyr[o-1],gaussb_pyr[o], 7 );	
				//gaussb_pyr[o] = reduce( gaussb_pyr[o-1] );				
				//cvSmooth(gaussb_pyr[o],gaussb_pyr[o], CV_GAUSSIAN, 5, 5, 0, 0 );
			 }			
	   }
	
	return gaussb_pyr;
}

extern void expandlp( IplImage* img )
{
	//CvSize sz;
	//IplImage *bigger;
//	sz =cvSize( img->width & -2, img->height & -2 );
//	bigger = cvCreateImage( cvSize(2*sz.width, 2*sz.height),IPL_DEPTH_8U ,3);
    IplImage* bigger = cvCreateImage( cvSize(img->width * 2, img->height * 2),img->depth, img->nChannels);
	cvResize( img, bigger, CV_INTER_NN );
	cvSmooth(bigger, bigger, CV_GAUSSIAN, 5, 5, 0, 0 );

	//cvPyrUp(img, bigger,7);	

}


extern IplImage* expandl2( IplImage* img, int r, int c)
{
	//CvSize sz;
	//IplImage *bigger;
//	sz =cvSize( img->width & -2, img->height & -2 );
//	bigger = cvCreateImage( cvSize(2*sz.width, 2*sz.height),IPL_DEPTH_8U ,3);
    //IplImage* bigger = cvCreateImage( cvSize(img->width * 2, img->height * 2),img->depth, img->nChannels);
	IplImage* bigger = cvCreateImage( cvSize(c, r),img->depth, img->nChannels);
	cvResize( img, bigger, CV_INTER_NN );
	if (r == 3 && c == 3)
	{
		cvSmooth(bigger, bigger, CV_GAUSSIAN, 3, 3, 0, 0 );
	}
	else
	{
     	cvSmooth(bigger, bigger, CV_GAUSSIAN, 5, 5, 0, 0 );
	}

	//cvPyrUp(img, bigger,7);	

	return bigger;
}

extern IplImage* equal2( IplImage* img)
{
  CvScalar s;
  int i, j;
  IplImage* result;
  result = cvCreateImage(cvSize(img->width,img->height),img->depth, img->nChannels);
  if(img->height > 1 && img->width > 1)
  {
   for( i=0; i<img->width; i++ ){
       for( j=0; j<img->height; j++ ){ 
           s=cvGet2D(img, j, i ); 
           cvSet2D(result, j, i, s );
		   }
          }
  }
  else
  {
	  s=cvGet2D( img, 0, 0 );
	  cvSet2D(result, 0, 0, s );
  }
  return result;
}

extern IplImage** build_gaussb3_pyr( IplImage* base )
{
	IplImage** gaussb_pyr;
	int i, o;
	int levels;

    levels = log( MIN( base->width, base->height ) ) / log(2) + 1;

	gaussb_pyr = calloc( levels, sizeof( IplImage** ) );
	for( i = 0; i < levels; i++ )
		gaussb_pyr[i] = calloc( levels, sizeof( IplImage* ) );

	
	for( o = 0; o < levels; o++ )
		{
			if( o == 0 )
				gaussb_pyr[o] = cvCloneImage(base);

			 else 
			 {  
			
				gaussb_pyr[o] = downsample( gaussb_pyr[o-1] );				
				cvSmooth(gaussb_pyr[o],gaussb_pyr[o], CV_GAUSSIAN, 5, 5, 0, 0 );
			 }
	   }
	
	return gaussb_pyr;
}

extern IplImage** build_laplace3_pyr( IplImage* base )
{
	IplImage** laplace_pyr;
	IplImage** gaussb_pyr;
	IplImage* temp;
	int i, o;
	int levels, levels2;
	int r, c, n; // to compute the dimmensions of each level
	CvScalar s;

    levels = log( MIN( base->width, base->height ) ) / log(2) + 1;
    
	
	levels2 = levels - 1; 
	printf("\n levels laplace = % d" , levels2);
	n = levels - 1;

	gaussb_pyr = build_gaussb3_pyr( base );

	laplace_pyr = calloc( levels2, sizeof( IplImage** ) );
	for( i = 0; i < levels2; i++ ){
		laplace_pyr[i] = cvCreateImage( cvGetSize(gaussb_pyr[i]), IPL_DEPTH_8U, 3);
	}

	for( o = 0; o < levels2; o++ )
		{
				r = gaussb_pyr[o] ->height;
				c = gaussb_pyr[o] ->width;
			    temp = expandl2(gaussb_pyr[ o + 1 ],r,c);
				printf("\n expandl(gaussb_pyr[o+1]) is %d x %d x %d", temp->height , temp ->width, temp ->nChannels);
				printf("\n gaussb_pyr[o] is %d x %d x %d", gaussb_pyr[o]->height , gaussb_pyr[o] ->width, gaussb_pyr[o] ->nChannels);
				printf("\n laplace_pyr[o] is %d x %d x %d", laplace_pyr[o]->height , laplace_pyr[o] ->width, laplace_pyr[o] ->nChannels);
				cvSub(gaussb_pyr[o],temp,laplace_pyr[o],NULL );
				cvReleaseImage( &temp );
	   }


	releaseb_pyr( &gaussb_pyr, levels2 );
	
 
	return laplace_pyr;
}


extern IplImage** build_gaussb_lpyr( IplImage* base, int levels )
{
	IplImage** gaussb_pyr;
	int i, o;

	gaussb_pyr = calloc( levels, sizeof( IplImage** ) );
	for( i = 0; i < levels; i++ )
		gaussb_pyr[i] = calloc( levels, sizeof( IplImage* ) );

	
	for( o = 0; o < levels; o++ )
		{
			if( o == 0 )
				gaussb_pyr[o] = cvCloneImage(base);

			 else 
			 {  
			
				gaussb_pyr[o] = downsample( gaussb_pyr[o-1] );				
				cvSmooth(gaussb_pyr[o],gaussb_pyr[o], CV_GAUSSIAN, 5, 5, 0, 0 );
			 }
	   }
	
	return gaussb_pyr;
}

extern IplImage** build_laplace_lpyr( IplImage* base, int levels )
{
	IplImage** laplace_pyr;
	IplImage** gaussb_pyr;
	IplImage* temp;
	int i, o;
	int levels2;
	int r, c, n; // to compute the dimmensions of each level
	CvScalar s;

    levels2 = levels;
		
	gaussb_pyr = build_gaussb3_pyr( base );

	laplace_pyr = calloc( levels2, sizeof( IplImage** ) );
	for( i = 0; i < levels2; i++ ){
		laplace_pyr[i] = cvCreateImage( cvGetSize(gaussb_pyr[i]), IPL_DEPTH_8U, 1);
	}

	for( o = 0; o < levels2; o++ )
		{
				r = gaussb_pyr[o] ->height;
				c = gaussb_pyr[o] ->width;
			    temp = expandl2(gaussb_pyr[ o + 1 ],r,c);
				printf("\n expandl(gaussb_pyr[o+1]) is %d x %d x %d", temp->height , temp ->width, temp ->nChannels);
				printf("\n gaussb_pyr[o] is %d x %d x %d", gaussb_pyr[o]->height , gaussb_pyr[o] ->width, gaussb_pyr[o] ->nChannels);
				printf("\n laplace_pyr[o] is %d x %d x %d", laplace_pyr[o]->height , laplace_pyr[o] ->width, laplace_pyr[o] ->nChannels);
				cvSub(gaussb_pyr[o],temp,laplace_pyr[o],NULL );
				cvReleaseImage( &temp );
	   }


	releaseb_pyr( &gaussb_pyr, levels2 );
	
 
	return laplace_pyr;
}

extern IplImage* compositive( IplImage** laplace_pyr, IplImage** gauss_pyr, int levels )
{
  	
	IplImage* temp;
	IplImage* compositive;
	//IplImage** tempcompositive;
	int i, o;
	int levels2;

	int r, c, n; // to compute the dimmensions of each level
	CvScalar s;
    levels2 = levels;

	/*

	tempcompositive = calloc( levels2, sizeof( IplImage** ) );
	for( i = 0; i < levels2; i++ ){
		tempcompositive[i] = cvCreateImage( cvGetSize(laplace_pyr[i]), IPL_DEPTH_8U, 1);
	}

	*/

	printf("\n I am here !!! ");
    for( o = levels2-1; o > -1; o-- )
	{
		if( o - 1 >= 0 )
		{
	    	r = gauss_pyr[o - 1]->height; // Getting the row and columns 
	        c = gauss_pyr[o - 1]-> width; // of the next level. The maximum level is the level 
             										// with the smaller image
         	temp = expandl2(gauss_pyr[o],r,c);
			//cvAdd(laplace_pyr[o-1],temp,tempcompositive[o-1],NULL );
			
			cvAdd(laplace_pyr[o-1],temp,gauss_pyr[o-1],NULL );
			cvReleaseImage( &temp );
		}
		/*
		if (o == 0)
		{
			r = laplace_pyr[o]->height; // Getting the row and columns 
	        c = laplace_pyr[o]-> width; // of the next level. The maximum level is the level 
             								// with the smaller image
         
			temp = expandl2(laplace_pyr[o+1],r,c);
			cvAdd(laplace_pyr[o],temp,laplace_pyr[o],NULL );
			cvReleaseImage( &temp );
		}
		*/
		
	}

	/*
	for( o = 0; o < levels2; o++ )
		{
				r = gaussb_pyr[o] ->height;
				c = gaussb_pyr[o] ->width;
			    temp = expandl2(gaussb_pyr[ o + 1 ],r,c);
				printf("\n expandl(gaussb_pyr[o+1]) is %d x %d x %d", temp->height , temp ->width, temp ->nChannels);
				printf("\n gaussb_pyr[o] is %d x %d x %d", gaussb_pyr[o]->height , gaussb_pyr[o] ->width, gaussb_pyr[o] ->nChannels);
				printf("\n laplace_pyr[o] is %d x %d x %d", laplace_pyr[o]->height , laplace_pyr[o] ->width, laplace_pyr[o] ->nChannels);
				cvSub(gaussb_pyr[o],temp,laplace_pyr[o],NULL );
				cvReleaseImage( &temp );
	   }
    */

    compositive = cvCloneImage(laplace_pyr[0]);


	//releaseb_pyr( &tempcompositive, levels2 );

	return compositive;

}


extern IplImage* findingS( IplImage* img1, IplImage* img2, int x_offset, int y_offset, int add_x, int add_y)
{
	IplImage* S, *back;
	int i, j;
	CvScalar s;
	CvScalar t;
	CvScalar z;
	CvScalar temp;

    
	// First step:
	back = cvCreateImage(cvSize(add_x + img1->width + 10, add_y + img1->height + 10), IPL_DEPTH_8U, 1 );
	S = cvCreateImage(cvSize(add_x + img1->width + 10, add_y + img1->height + 10), IPL_DEPTH_8U, 1 );

	cvZero(back);
	
	// Second step: moving the back image

	for(i = 0; i< img1->width; i++){
		for(j = 0; j < img1->height; j++){
			s = cvGet2D(img1,j,i);
			cvSet2D(back,j + y_offset, i + x_offset, s);
		}
	}

	// Step 3: Find the region S
	for(i = 0; i< img2->width; i++){
		for(j = 0; j < img2->height; j++){
			s = cvGet2D(img2,j,i);
 			t = cvGet2D(back,j,i);
			if(s.val[0] > 1 && t.val[0] > 1)
			{
			    	
			//temp.val[0] = 255; // White is 255
              cvSet2D(S,j,i,s);				
			}
           
			
			/*
			if(s.val[0] > 1 && t.val[0] > 1 && i+45 < img2->width && j+45 < img2->height)
			{
			//	printf("\n value of temp is %f", temp.val[0]);
		//		z =  cvGet2D(back, j, i);
		//		temp.val[0] = 255; // White is 255
				
				//cvSet2D(S,j,i,temp);				
			
				
			    	z =  cvGet2D(back, j+45, i+45);
			    	if(z.val[0] < 1){
				   	temp.val[0] = 255;
					cvSet2D(S,j,i,temp);
				 }
				 else{
					temp.val[0] = 0;
					cvSet2D(S,j,i,temp);					
					}
			     
			}

    */


		}
	}
  cvReleaseImage(& back);
  return S;
}

extern IplImage* RGBtoIntensity(IplImage* img)
{
    int nrows, ncols, i, j;
	CvScalar zi;
	CvScalar zf;
	IplImage* IntensImage;
	IntensImage = cvCreateImage(cvSize(img->width,img->height),IPL_DEPTH_8U , 1);

	 nrows = img->height; //rows
	 ncols = img->width; //cols
	for( i = 0; i < nrows; i++){
		for( j = 0; j < ncols; j++){
			zi = cvGet2D(img,i,j);
			zf.val[0] = (zi.val[0])*0.299 + (zi.val[1])*0.587 +(zi.val[2])* 0.114;
			cvSet2D(IntensImage,i,j,zf);
		}
	}

	return IntensImage;
}
extern CvMat* CalcTransfMatrix( IplImage* img1, IplImage* img2)
{
	struct feature* feat1, * feat2, *feat;
	struct feature** nbrs;
	struct kd_node* kd_root;
	
	struct feature* feature_lowe1; // I use this to find out the number of inliers
    struct feature* feature_lowe2; // I use this to find out the number of inliers

/* the maximum number of keypoint NN candidates to check during BBF search */
#define KDTREE_BBF_MAX_NN_CHKS 200

/* threshold on squared ratio of distances between NN and 2nd NN */
#define NN_SQ_DIST_RATIO_THR 0.49

    CvPoint pt1, pt2;
	double d0, d1;
	int n1, n2, k, i, m = 0, dx, dy;
	int n=0;
	int levels, levels2;
	unsigned char * ptr;

   CvPoint2D32f srcQuad[4], dstQuad[4];
   CvMat* warp_matrix = cvCreateMat(3,3,CV_32FC1);
   IplImage *dst = NULL;
   CvMat* H = cvCreateMat(3,3,CV_32FC1);
 	    
   fprintf( stderr, "\nFinding features in %s...\n", img1);
	n1 = sift_features( img1, &feat1 );
	fprintf( stderr, "Finding features in %s...\n", img2 );
	n2 = sift_features( img2, &feat2 );

    kd_root = kdtree_build( feat1, n1 ); // feat2 and n2
	// I want to change the order of the images I have to use feat1, n1

	for( i = 0; i < n2; i++ ) //n1 // to chage order n2
	{
		feat = feat2 + i; //feat1 // to change order feat2
		k = kdtree_bbf_knn( kd_root, feat, 2, &nbrs, KDTREE_BBF_MAX_NN_CHKS );
		if( k == 2 )
		{
	//Returns the squared Euclidian distance between the descriptors of  feat and nbrs[0]
			d0 = descr_dist_sq( feat, nbrs[0] ); 
			d1 = descr_dist_sq( feat, nbrs[1] );
			if( d0 < d1 * NN_SQ_DIST_RATIO_THR ) // Selection of the "distintic match"
			{
				pt1 = cvPoint( cvRound( feat->x ), cvRound( feat->y ) );
				pt2 = cvPoint( cvRound( nbrs[0]->x ), cvRound( nbrs[0]->y ) );
				pt2.y += img1->height;
				//cvLine( stacked, pt1, pt2, CV_RGB(255,0,255), 1, 8, 0 );
				m++;
				feat2[i].fwd_match = nbrs[0]; //feat1[i].fwd_match // to change order feat2[i].fwd_match
				//printf("\n The value of this match is : %f",nbrs[0]);
				
			}
		}
		free( nbrs );
	}
 	   
	 H = ransac_xform( feat2, n2, FEATURE_FWD_MATCH, lsq_homog, 4, 0.01,homog_xfer_err, 3.0, NULL, NULL );

	 //missing many things, re
	 return H;
	 free(feat1);
	 free(feat2);

}

//extern CvPoint2D64f BBoxP0(IplImage* img, CvMat* H)
extern CvMat* BBoxP0(IplImage* img, CvMat* H)
{
   int numRows, numCols, i;
   CvMat *bbox = cvCreateMat(2,4,CV_32FC1);
   CvMat *H312 = cvCreateMat(1,2,CV_32FC1);
   CvMat *H112 = cvCreateMat(1,2,CV_32FC1);
   CvMat *H212 = cvCreateMat(1,2,CV_32FC1);
   CvMat *BBox = cvCreateMat(2,4,CV_32FC1);
   CvMat *temp1 = cvCreateMat(1,4,CV_32FC1);
   CvMat *temp2 = cvCreateMat(1,4,CV_32FC1);
   CvMat *temp3 = cvCreateMat(1,4,CV_32FC1);
   CvMat *temp4 = cvCreateMat(1,4,CV_32FC1);
   CvMat *temp5 = cvCreateMat(1,4,CV_32FC1);
   CvMat *base = cvCreateMat(1,4,CV_32FC1);
   CvMat *BBox1 = cvCreateMat(1,4,CV_32FC1);
   CvMat *BBox2 = cvCreateMat(1,4,CV_32FC1);

   CvScalar value1, value2, value3;
   //float base;


   // MATLAB: 
   /*
     numRows = size(I, 1);
     numCols = size(I, 2);
  */
   numRows = img->height; //rows
   numCols = img->width; //cols

   // MATLAB: 
   /*
		bbox = single([1, 1; 1, numCols; numRows, numCols; numRows, 1]' - 1);
  */

    
   cvmSet(bbox,0,0,0.00);
   cvmSet(bbox,0,1,0.00);
   cvmSet(bbox,0,2,(double)numRows - 1.0);
   cvmSet(bbox,0,3,(double)numRows - 1.0);
   cvmSet(bbox,1,0, 0.00);
   cvmSet(bbox,1,1,(double)numCols - 1.0);
   cvmSet(bbox,1,2,(double)numCols - 1.0);
   cvmSet(bbox,1,3,0.00);

   // MATLAB : BBox = single(zeros(2, 4));
   cvZero(BBox);

   //MATLAB: base = TForm(3,1:2) * bbox + TForm(3,3);
   
    cvmSet(H312,0,0,cvmGet(H,2,0));
	cvmSet(H312,0,1,cvmGet(H,2,1));
    cvMatMul(H312,bbox,temp1);
	value1.val[0] = cvmGet(H,2,2);
	cvAddS(temp1,value1,base,0);// Check if works

  //MATLAB: BBox(1,:) = (TForm(1,1:2) * bbox + TForm(1,3)) ./ base;
    
    cvmSet(H112,0,0,cvmGet(H,0,0));
	cvmSet(H112,0,1,cvmGet(H,0,1));
    cvMatMul(H112,bbox,temp2);
	value2.val[0] = cvmGet(H,0,2);
	cvAddS(temp2,value2,temp3,0);// Check if works
	cvDiv(temp3,base,BBox1,1);

	//BBox(2,:) = (TForm(2,1:2) * bbox + TForm(2,3)) ./ base;
    cvmSet(H212,0,0,cvmGet(H,1,0));
	cvmSet(H212,0,1,cvmGet(H,1,1));
    cvMatMul(H212,bbox,temp4);
	value3.val[0] = cvmGet(H,1,2);
	cvAddS(temp4,value3,temp5,0);// Check if works
	cvDiv(temp5,base,BBox2,1);
    
	for(i = 0; i < 4; i++)
	   cvmSet(BBox,0,i,cvmGet(BBox1,0,i));
	for(i = 0; i < 4; i++)
	   cvmSet(BBox,1,i,cvmGet(BBox2,0,i));


    cvReleaseMat( &temp1 );
	cvReleaseMat( &temp2 );
	cvReleaseMat( &temp3 );
	cvReleaseMat( &temp4 );
	cvReleaseMat( &temp5 );
	cvReleaseMat( &base );
	cvReleaseMat( &BBox1 );
	cvReleaseMat( &BBox2 );
	cvReleaseMat( &H312 );
	cvReleaseMat( &H112 );
	cvReleaseMat( &H212 );
	
	return BBox;

}

extern void WarpImageH(IplImage* im1,IplImage* mosaicing, CvMat *H, int x_offset, int y_offset)
{

//IplImage* mosaicing;
 int h, w, x_row, x_col;
 int  y_row, y_col;
 int d = 3, i, j;
 int aux_x, aux_y;
 double aux_min, aux_max;

 CvMat *y1 = cvCreateMat(3,4,CV_32FC1);
  CvMat *temp = cvCreateMat(3,4,CV_32FC1);
  CvMat *H1 = cvCreateMat(3,3,CV_32FC1);
  CvMat *H1_inv = cvCreateMat(3,3,CV_32FC1);
  CvMat *box = cvCreateMat(4,1,CV_32FC1);

 CvPoint2D64f current;
 CvPoint2D64f target;

 CvScalar z;  // To use with the linear interpolation
 CvScalar s;
 CvScalar t;
 CvScalar ave;



// Check if H is Identity

 //if((cvmGet(H,0,0) != 1.0) & (cvmGet(H,1,1) != 1.0) & (cvmGet(H,2,2) != 1.0))
 if((cvmGet(H,0,0) != 1.0) && (cvmGet(H,1,1) != 1.0))
 {
  
  CvMat* x;
 CvMat* y;
 CvMat* x2;
 CvMat* y2;
 CvMat* ap;
 CvMat* a;


 //CvMat* adj;

 h = im1->height;
 w = im1->width;

 cvmSet(temp,0,0,1.00);
 cvmSet(temp,0,1,1.00);
 cvmSet(temp,0,2,w);
 cvmSet(temp,0,3,w);
 cvmSet(temp,1,0,1.00);
 cvmSet(temp,1,1,h);
 cvmSet(temp,1,2,h);
 cvmSet(temp,1,3,1.00);
 cvmSet(temp,2,0,1.00);
 cvmSet(temp,2,1,1.00);
 cvmSet(temp,2,2,1.00);
 cvmSet(temp,2,3,1.00);
 

for(i = 0; i < 3; i++){ //row
	for(j = 0; j < 3; j++){ //col
      cvmSet(H1,i,j,cvmGet(H,i,j));
	}}

cvMatMul(H1,temp,y1);
//cvMul(H1,temp,y1,1);

// Part of matlab:
//y(1,:) = y(1,:)./y(3,:);
//y(2,:) = y(2,:)./y(3,:);
for(i = 0; i < 2; i++){ //rows
	for(j=0; j < 4; j ++){ // columns
     cvmSet(y1,i,j,cvmGet(y1,i,j)/cvmGet(y1,2,j));
	 
	}}

/*
printf("\n Values of the temp matrix");
for(i = 0; i < 3; i++){
	for(j = 0; j < 4; j++){
     printf("\n y1(%d,%d) = %f",i,j,cvmGet(y1,i,j));
	}}
*/
printf("\n H is not identity matrix");
 
 aux_min = cvmGet(y1,0,0);
 aux_max = cvmGet(y1,0,0);
 for (i = 0; i < 4; i++){
	 if (aux_min > cvmGet(y1,0,i))
		 aux_min = cvmGet(y1,0,i);
	 if (aux_max < cvmGet(y1,0,i))
	     aux_max = cvmGet(y1,0,i);
 }
 
 cvmSet(box,0,0,ceil(aux_min));
 cvmSet(box,1,0,ceil(aux_max));

 aux_min = cvmGet(y1,1,0);
 aux_max = cvmGet(y1,1,0);

 for (i = 0; i < 4; i++){
	 if (aux_min > cvmGet(y1,1,i))
		 aux_min = cvmGet(y1,1,i);
	 if (aux_max < cvmGet(y1,1,i))
	     aux_max = cvmGet(y1,1,i);
 }
 cvmSet(box,2,0,ceil(aux_min));
 cvmSet(box,3,0,ceil(aux_max));
/*
 for(i = 0; i < 4; i++){
	 printf("\n box(%d,0) = %f",i,cvmGet(box,i,0));
	}
*/
  x_row = cvmGet(box,3,0) - cvmGet(box,2,0);
  x_col = cvmGet(box,1,0) - cvmGet(box,0,0);
  
  y_row = x_row;
  y_col = x_col;
  
 // printf("\n x_row = %d, \n x_col = %d",x_row, x_col);

  x = cvCreateMat(x_row,x_col,CV_32FC1);
  y = cvCreateMat(y_row,y_col,CV_32FC1);

  for (i = 0; i < x_row; i++){
	  for (j = 0; j < x_col; j++){
          aux_x = cvmGet(box,0,0) + j;
		  cvmSet(x,i,j,aux_x);}}

  // If there is a problem in the future review this because I did get the same answer as Matlab
// The difference make sense, but still it's important to take account this.

  for (i = 0; i < y_col; i++){
	  for (j = 0; j < y_row; j++){
          aux_y = cvmGet(box,2,0) + j;  
		  cvmSet(y,j,i,aux_y);}}

  /*
printf("\n Values of the meshgrid matrix x and y");
for(i = 0; i < 4; i++){
	for(j = 0; j < 4; j++){
     printf("\n x(%d,%d) = %f",i,j,cvmGet(x,i,j));
	 printf("\n y(%d,%d) = %f",i,j,cvmGet(y,i,j));
	}} 
*/
  cvInvert(H1,H1_inv,CV_LU);         // Inverting the matrix H1 using LU factorization
/*
  for(i = 0; i < 3; i++){
	for(j = 0; j < 3; j++){
     printf("\n H1_inv(%d,%d) = %f",i,j,cvmGet(H1_inv,i,j));
	}}
*/

    // Implementation of a = inv(warp)*[x(:) y(:) ones(prod(size(x)),1)]';
  x2 = cvCreateMat(1,x_row*x_col,CV_32FC1);
  y2 = cvCreateMat(1,x_row*x_col,CV_32FC1);
  ap = cvCreateMat(3,x_row*x_col,CV_32FC1);
  a = cvCreateMat(3,x_row*x_col,CV_32FC1);

 for (i = 0; i < x_row; i++){
  for (j = 0; j < x_col; j++){
	  cvmSet(x2,0,i*x_col+j,cvmGet(x,i,j));}}
 
 for (i = 0; i < y_row; i++){
  for (j = 0; j < y_col; j++){
	  cvmSet(y2,0,i*y_col+j,cvmGet(y,i,j));}}
/*
  for(i = 0; i < 3; i++){
	    printf("\n x2(0,%d) = %f",i,cvmGet(x2,0,i));
		printf("\n y2(0,%d) = %f",i,cvmGet(y2,0,i));
	}
*/
  for(i = 0; i< x_col*x_row; i++){
    cvmSet(ap,0,i,cvmGet(x2,0,i));}
  for(i = 0; i< x_col*x_row; i++){
    cvmSet(ap,1,i,cvmGet(y2,0,i));}
  for(i = 0; i< x_col*x_row; i++){
    cvmSet(ap,2,i,1.0);}

   cvMatMul(H1_inv,ap,a);
   /*
	for(i = 0; i < 3; i++){
		for(j=0; j < 3; j++){
			printf("\n a(%d,%d) = %f",i,j,cvmGet(a,i,j));}}
*/
	// Implementing:
	// x(:) = a(1,:)./a(3,:);
    // y(:) = a(2,:)./a(3,:);

  for(i = 0; i< x_col*x_row; i++){
    cvmSet(x2,0,i,cvmGet(a,0,i)/cvmGet(a,2,i));}
  for(i = 0; i< x_col*x_row; i++){
    cvmSet(y2,0,i,cvmGet(a,1,i)/cvmGet(a,2,i));}
  

 // printf("\nThe values of the final x and y are");
  for (i = 0; i < x_row; i++){
   for (j = 0; j < x_col; j++){
	  cvmSet(x,i,j,cvmGet(x2,0,j+x_col*i));
	  cvmSet(y,i,j,cvmGet(y2,0,j+x_col*i));}}
/*
	for(i = 0; i < 3; i++){
		for(j=0; j < 3; j++){
			printf("\n x(%d,%d) = %f",i,j,cvmGet(x,i,j));
		    printf("\n y(%d,%d) = %f",i,j,cvmGet(y,i,j));}}
	printf("\n x(4,121)= %f",cvmGet(x,4,121));
	printf("\n y(4,121)= %f",cvmGet(y,4,121));
*/

	//mosaicing = cvCreateImage( cvSize(x_col, x_row)  , IPL_DEPTH_8U , 1 );
	//cvZero( mosaicing );
	//cvSetImageROI(im1,cvRect(0,0,x_col,x_row));

	//printf("\n Memory asssigned to the mosaicing image variable " );

	for( i=0; i<im1->width; i++ ){ //img1->width
      for( j=0; j<im1->height; j++ ){//img1->height
		current = cvPoint2D64f( i, j ); // i->x and j->y
		target = persp_xform_pt2( current, H );   //H 
		s=cvGet2D( im1, j, i ); //img1  // Get the pixel value in (j,i) of img1
		if(target.y+y_offset<mosaicing->height && target.y+y_offset>= 0 && target.x+x_offset < mosaicing->width && target.x+x_offset >= 0)
		{
		 cvSet2D( mosaicing, target.y+y_offset, target.x+x_offset, s );
		}   
	}} // End of the for loops

  /*		for( i=0; i<im1->width; i++ ){ //img1->width
          for( j=0; j<im1->height; j++ ){//img1->height
			current = cvPoint2D64f( i, j ); // i->x and j->y
			target = ( current, H );   //H 
			s=cvGet2D( im1, j, i ); //img1  // Get the pixel value in (j,i) of img1
			cvSet2D( mosaicing, target.y+y_offset, target.x+x_offset, s ); }}  */

 // printf("\n The first image was warped " );

for (i = 0; i <  x_row; i++){
  for (j = 0; j < x_col; j++){
	  if(cvmGet(x,i,j) > 0 && cvmGet(x,i,j) < (im1->width - 1) && cvmGet(y,i,j) > 0 && cvmGet(y,i,j) < (im1->height - 1))
	  {
	    z= linear_interpolation(0, 0, im1, cvmGet(x,i,j), cvmGet(y,i,j));
	    cvSet2D( mosaicing, i, j, z ); 
	  }
	  else
	  {
		z.val[0] = 0;
//		z.val[1] = 0;
//		z.val[2] = 0;
		cvSet2D( mosaicing, i, j, z ); 
	  }
  }
}

  cvReleaseMat( &x2 );
  cvReleaseMat( &y2 );
  cvReleaseMat( &ap );
  cvReleaseMat( &a );
  cvReleaseMat( &x );
  cvReleaseMat( &y );
}
else  // The Homography is Identity
{

	printf("\n H is an identity matrix");
	for(i = 0; i < im1->height;i++){ //rows
		for(j = 0; j <im1->width; j++){ //cols
			cvSet2D(mosaicing,i + y_offset,j + x_offset,cvGet2D(im1,i,j));
		}
	}
}


  cvReleaseMat( &y1 );
  cvReleaseMat( &temp );
  cvReleaseMat( &H1 );
  cvReleaseMat( &H1_inv );
  cvReleaseMat( &box );
 
 
  //cvReleaseImage( &mosaicing );
 
  return;
  
}

extern void PrintElementMatrix(CvMat *H)
{
  int nRows, nCols;
  int i, j;
  nRows = H->rows;
  nCols = H->cols;

  for( i = 0; i < nRows; i++){
	  for( j = 0; j < nCols; j++){
		printf("\n Element (%d,%d) = %f", i,j,cvmGet(H,i,j));
	  }
  }

  return;
}

extern struct image_border FindCornersROI(CvMat *BBox)
{
  
  struct image_border border;
  float px_min, px_max,py_min, py_max;
  float p1_x, p2_x, p3_x, p4_x;
  float p1_y, p2_y, p3_y, p4_y;
  float Box_width;
  float Box_height;
   

  p1_x = cvmGet(BBox,1,0);
  p1_y = cvmGet(BBox,0,0);
  p2_x = cvmGet(BBox,1,1);
  p2_y = cvmGet(BBox,0,1);
  p3_x = cvmGet(BBox,1,2);
  p3_y = cvmGet(BBox,0,2);
  p4_x = cvmGet(BBox,1,3);
  p4_y = cvmGet(BBox,0,3);

  px_min = MIN(MIN(MIN(p1_x,p2_x),p3_x),p4_x);
  px_max = MAX(MAX(MAX(p1_x,p2_x),p3_x),p4_x);

  py_min = MIN(MIN(MIN(p1_y,p2_y),p3_y),p4_y);
  py_max = MAX(MAX(MAX(p1_y,p2_y),p3_y),p4_y);


  Box_width = px_max - px_min + 1;
  Box_height = py_max - py_min + 1;

  border.P1.x = p1_x;
  border.P1.y = p1_y;
  border.P2.x = p2_x;
  border.P2.y = p2_y;
  border.P3.x = p3_x;
  border.P3.y = p3_y;
  border.P4.x = p4_x;
  border.P4.y = p4_y;

  border.x_max = px_max;
  border.y_max = py_max;

  border.x_min = px_min;
  border.y_min = py_min;

  border.ROISize.width = (int)Box_width;
  border.ROISize.height = (int)Box_height;


  //printf("\n Box_widthf = %d, Box_heightf = %d",(int)Box_width, (int)Box_height);

  return border;

}


extern void WarpImagetoBox(IplImage* im1,IplImage* mosaicing, CvMat *H, CvSize Box, int x_offset, int y_offset)
{

//IplImage* mosaicing;
 int h, w, x_row, x_col;
 int  y_row, y_col;
 int d = 3, i, j;
 int aux_x, aux_y;
 double aux_min, aux_max;

 CvMat *y1 = cvCreateMat(3,4,CV_32FC1);
  CvMat *temp = cvCreateMat(3,4,CV_32FC1);
  CvMat *H1 = cvCreateMat(3,3,CV_32FC1);
  CvMat *H1_inv = cvCreateMat(3,3,CV_32FC1);
  CvMat *box = cvCreateMat(4,1,CV_32FC1);

 CvPoint2D64f current;
 CvPoint2D64f target;

 CvScalar z;  // To use with the linear interpolation
 CvScalar s;
 CvScalar t;
 CvScalar ave;



// Check if H is Identity

 if((cvmGet(H,0,0) != 1.0) & (cvmGet(H,1,1) != 1.0) & (cvmGet(H,2,2) == 1.0))
 {
  CvMat* x;
 CvMat* y;
 CvMat* x2;
 CvMat* y2;
 CvMat* ap;
 CvMat* a;


 //CvMat* adj;

 h = im1->height;
 w = im1->width;

 cvmSet(temp,0,0,1.00);
 cvmSet(temp,0,1,1.00);
 cvmSet(temp,0,2,w);
 cvmSet(temp,0,3,w);
 cvmSet(temp,1,0,1.00);
 cvmSet(temp,1,1,h);
 cvmSet(temp,1,2,h);
 cvmSet(temp,1,3,1.00);
 cvmSet(temp,2,0,1.00);
 cvmSet(temp,2,1,1.00);
 cvmSet(temp,2,2,1.00);
 cvmSet(temp,2,3,1.00);
 

for(i = 0; i < 3; i++){ //row
	for(j = 0; j < 3; j++){ //col
      cvmSet(H1,i,j,cvmGet(H,i,j));
	}}

cvMatMul(H1,temp,y1);
//cvMul(H1,temp,y1,1);

// Part of matlab:
//y(1,:) = y(1,:)./y(3,:);
//y(2,:) = y(2,:)./y(3,:);
for(i = 0; i < 2; i++){ //rows
	for(j=0; j < 4; j ++){ // columns
     cvmSet(y1,i,j,cvmGet(y1,i,j)/cvmGet(y1,2,j));
	 
	}}

/*
printf("\n Values of the temp matrix");
for(i = 0; i < 3; i++){
	for(j = 0; j < 4; j++){
     printf("\n y1(%d,%d) = %f",i,j,cvmGet(y1,i,j));
	}}
*/
 
 aux_min = cvmGet(y1,0,0);
 aux_max = cvmGet(y1,0,0);
 for (i = 0; i < 4; i++){
	 if (aux_min > cvmGet(y1,0,i))
		 aux_min = cvmGet(y1,0,i);
	 if (aux_max < cvmGet(y1,0,i))
	     aux_max = cvmGet(y1,0,i);
 }
 
 cvmSet(box,0,0,ceil(aux_min));
 cvmSet(box,1,0,ceil(aux_max));

 aux_min = cvmGet(y1,1,0);
 aux_max = cvmGet(y1,1,0);

 for (i = 0; i < 4; i++){
	 if (aux_min > cvmGet(y1,1,i))
		 aux_min = cvmGet(y1,1,i);
	 if (aux_max < cvmGet(y1,1,i))
	     aux_max = cvmGet(y1,1,i);
 }
 cvmSet(box,2,0,ceil(aux_min));
 cvmSet(box,3,0,ceil(aux_max));
/*
 for(i = 0; i < 4; i++){
	 printf("\n box(%d,0) = %f",i,cvmGet(box,i,0));
	}
*/
 // x_row = cvmGet(box,3,0) - cvmGet(box,2,0);
 // x_col = cvmGet(box,1,0) - cvmGet(box,0,0);
  
  x_row = Box.height;
  x_col = Box.width;

  y_row = x_row;
  y_col = x_col;

  
 // printf("\n x_row = %d, \n x_col = %d",x_row, x_col);

  x = cvCreateMat(x_row,x_col,CV_32FC1);
  y = cvCreateMat(y_row,y_col,CV_32FC1);

  for (i = 0; i < x_row; i++){
	  for (j = 0; j < x_col; j++){
          aux_x = cvmGet(box,0,0) + j;
		  cvmSet(x,i,j,aux_x);}}

  // If there is a problem in the future review this because I did get the same answer as Matlab
// The difference make sense, but still it's important to take account this.

  for (i = 0; i < y_col; i++){
	  for (j = 0; j < y_row; j++){
          aux_y = cvmGet(box,2,0) + j;  
		  cvmSet(y,j,i,aux_y);}}

  /*
printf("\n Values of the meshgrid matrix x and y");
for(i = 0; i < 4; i++){
	for(j = 0; j < 4; j++){
     printf("\n x(%d,%d) = %f",i,j,cvmGet(x,i,j));
	 printf("\n y(%d,%d) = %f",i,j,cvmGet(y,i,j));
	}} 
*/
  cvInvert(H1,H1_inv,CV_LU);         // Inverting the matrix H1 using LU factorization
/*
  for(i = 0; i < 3; i++){
	for(j = 0; j < 3; j++){
     printf("\n H1_inv(%d,%d) = %f",i,j,cvmGet(H1_inv,i,j));
	}}
*/

    // Implementation of a = inv(warp)*[x(:) y(:) ones(prod(size(x)),1)]';
  x2 = cvCreateMat(1,x_row*x_col,CV_32FC1);
  y2 = cvCreateMat(1,x_row*x_col,CV_32FC1);
  ap = cvCreateMat(3,x_row*x_col,CV_32FC1);
  a = cvCreateMat(3,x_row*x_col,CV_32FC1);

 for (i = 0; i < x_row; i++){
  for (j = 0; j < x_col; j++){
	  cvmSet(x2,0,i*x_col+j,cvmGet(x,i,j));}}
 
 for (i = 0; i < y_row; i++){
  for (j = 0; j < y_col; j++){
	  cvmSet(y2,0,i*y_col+j,cvmGet(y,i,j));}}
/*
  for(i = 0; i < 3; i++){
	    printf("\n x2(0,%d) = %f",i,cvmGet(x2,0,i));
		printf("\n y2(0,%d) = %f",i,cvmGet(y2,0,i));
	}
*/
  for(i = 0; i< x_col*x_row; i++){
    cvmSet(ap,0,i,cvmGet(x2,0,i));}
  for(i = 0; i< x_col*x_row; i++){
    cvmSet(ap,1,i,cvmGet(y2,0,i));}
  for(i = 0; i< x_col*x_row; i++){
    cvmSet(ap,2,i,1.0);}

   cvMatMul(H1_inv,ap,a);
   /*
	for(i = 0; i < 3; i++){
		for(j=0; j < 3; j++){
			printf("\n a(%d,%d) = %f",i,j,cvmGet(a,i,j));}}
*/
	// Implementing:
	// x(:) = a(1,:)./a(3,:);
    // y(:) = a(2,:)./a(3,:);

  for(i = 0; i< x_col*x_row; i++){
    cvmSet(x2,0,i,cvmGet(a,0,i)/cvmGet(a,2,i));}
  for(i = 0; i< x_col*x_row; i++){
    cvmSet(y2,0,i,cvmGet(a,1,i)/cvmGet(a,2,i));}
  

 // printf("\nThe values of the final x and y are");
  for (i = 0; i < x_row; i++){
   for (j = 0; j < x_col; j++){
	  cvmSet(x,i,j,cvmGet(x2,0,j+x_col*i));
	  cvmSet(y,i,j,cvmGet(y2,0,j+x_col*i));}}
/*
	for(i = 0; i < 3; i++){
		for(j=0; j < 3; j++){
			printf("\n x(%d,%d) = %f",i,j,cvmGet(x,i,j));
		    printf("\n y(%d,%d) = %f",i,j,cvmGet(y,i,j));}}
	printf("\n x(4,121)= %f",cvmGet(x,4,121));
	printf("\n y(4,121)= %f",cvmGet(y,4,121));
*/

	//mosaicing = cvCreateImage( cvSize(x_col, x_row)  , IPL_DEPTH_8U , 1 );
	//cvZero( mosaicing );
	//cvSetImageROI(im1,cvRect(0,0,x_col,x_row));

	//printf("\n Memory asssigned to the mosaicing image variable " );

	for( i=0; i<im1->width; i++ ){ //img1->width
      for( j=0; j<im1->height; j++ ){//img1->height
		current = cvPoint2D64f( i, j ); // i->x and j->y
		target = persp_xform_pt( current, H );   //H 
		s=cvGet2D( im1, j, i ); //img1  // Get the pixel value in (j,i) of img1
		if(target.y+y_offset<mosaicing->height && target.y+y_offset>= 0 && target.x+x_offset < mosaicing->width && target.x+x_offset >= 0)
		{
		 cvSet2D( mosaicing, target.y+y_offset, target.x+x_offset, s );
		}   
	}} // End of the for loops

  /*		for( i=0; i<im1->width; i++ ){ //img1->width
          for( j=0; j<im1->height; j++ ){//img1->height
			current = cvPoint2D64f( i, j ); // i->x and j->y
			target = ( current, H );   //H 
			s=cvGet2D( im1, j, i ); //img1  // Get the pixel value in (j,i) of img1
			cvSet2D( mosaicing, target.y+y_offset, target.x+x_offset, s ); }}  */

 // printf("\n The first image was warped " );

for (i = 0; i <  x_row; i++){
  for (j = 0; j < x_col; j++){
	  if(cvmGet(x,i,j) > 0 && cvmGet(x,i,j) < (im1->width - 1) && cvmGet(y,i,j) > 0 && cvmGet(y,i,j) < (im1->height - 1))
	  {
	    z= linear_interpolation(0, 0, im1, cvmGet(x,i,j), cvmGet(y,i,j));
	    cvSet2D( mosaicing, i, j, z ); 
	  }
	  else
	  {
	    z.val[0] = 0;
//		z.val[1] = 0;
//		z.val[2] = 0;
		cvSet2D( mosaicing, i, j, z ); 
	  }
  }
}

  cvReleaseMat( &x2 );
  cvReleaseMat( &y2 );
  cvReleaseMat( &ap );
  cvReleaseMat( &a );
  cvReleaseMat( &x );
  cvReleaseMat( &y );
}
else  // The Homography is Identity
{
	for(i = 0; i < im1->height;i++){ //rows
		for(j = 0; j <im1->width; j++){ //cols
			cvSet2D(mosaicing,i + x_offset,j + y_offset,cvGet2D(im1,i,j));
		}
	}
}


  cvReleaseMat( &y1 );
  cvReleaseMat( &temp );
  cvReleaseMat( &H1 );
  cvReleaseMat( &H1_inv );
  cvReleaseMat( &box );
 
 
  //cvReleaseImage( &mosaicing );
 
  return;
  
}

/*
extern void VideoMosaicking(IplImage* frame1, IplImage* frame2, IplImage* mosaicking, int x_offset, int y_offset, CvMat* H, CvMat* Hf1,CvMat* BBoxf1, CvMat* BBoxf2)
{

    struct image_border ROI_frame1, ROI_frame2;
	IplImage* ROI_Img_frame1;
	IplImage* ROI_Img_frame2;
    int p1_x_f1, p1_y_f1, p1_x_f2, p1_y_f2;
    int dx_offset, dy_offset;

	CvRect ROI_Rect_frame1;
	CvRect ROI_Rect_frame2;

    ROI_frame2 = FindCornersROI(BBoxf2);
	ROI_frame1 = FindCornersROI(BBoxf1);

	p1_x_f1 = (int)cvmGet(BBoxf1,1,0);
	p1_y_f1 = (int)cvmGet(BBoxf1,0,0);

	p1_x_f2 = (int)cvmGet(BBoxf2,1,0);
	p1_y_f2 = (int)cvmGet(BBoxf2,0,0);


	//Warping frame 2
	ROI_Rect_frame2 = cvRect(p1_x_f2 + x_offset,p1_y_f2 + y_offset,ROI_frame2.width+x_offset+10, ROI_frame2.height+y_offset+10);
	ROI_Img_frame2 = cvCreateImageHeader(cvSize(ROI_Rect_frame2.width,ROI_Rect_frame2.height), mosaicking->depth, mosaicking->nChannels);

	ROI_Img_frame2->origin = mosaicking->origin;
	ROI_Img_frame2->widthStep = mosaicking->widthStep;
	ROI_Img_frame2->imageData = mosaicking->imageData + ROI_Rect_frame2.y*mosaicking->widthStep + ROI_Rect_frame2.x*mosaicking->nChannels;

	dx_offset = p1_x_f1 - p1_x_f2;
	dy_offset = p1_y_f1 - p1_y_f2;  
	WarpImagetoBox(frame2,ROI_Img_frame2, H, ROI_frame2, dx_offset, dy_offset);

   //Warping frame 1
	ROI_Rect_frame1 = cvRect(p1_x_f2 + x_offset,p1_y_f2 + y_offset,ROI_frame1.width+x_offset+10, ROI_frame1.height+y_offset+10);
	ROI_Img_frame1 = cvCreateImageHeader(cvSize(ROI_Rect_frame1.width,ROI_Rect_frame1.height), mosaicking->depth, mosaicking->nChannels);

	ROI_Img_frame1->origin = mosaicking->origin;
	ROI_Img_frame1->widthStep = mosaicking->widthStep;
	ROI_Img_frame1->imageData = mosaicking->imageData + ROI_Rect_frame1.y*mosaicking->widthStep + ROI_Rect_frame1.x*mosaicking->nChannels;

	WarpImagetoBox(frame1,ROI_Img_frame1, Hf1, ROI_frame1, x_offset, y_offset);

	printf("\n end function");

	cvReleaseImageHeader(&ROI_Img_frame2);
	cvReleaseImageHeader(&ROI_Img_frame1);

	return;

}
*/

/*

extern void findingMask( IplImage* img2, IplImage* S, int x_offset, int y_offset)
{
	IplImage* back;
	int i, j;
	CvScalar s;
	CvScalar t;
	CvScalar z;
	CvScalar temp;

    
	// First step:
	back = cvCreateImage(cvGetSize(S), IPL_DEPTH_8U, 1 );
	//S = cvCreateImage(cvSize(img1->width + 10, img1->height + 10), IPL_DEPTH_8U, 1 );

	cvZero(back);
	
	// Second step: moving the back image

	for(i = 0; i< img1->width; i++){
		for(j = 0; j < img1->height; j++){
			s = cvGet2D(img1,j,i);
			cvSet2D(back,j + y_offset, i + x_offset, s);
		}
	}

	// Step 3: Find the region S
	for(i = 0; i< img2->width; i++){
		for(j = 0; j < img2->height; j++){
			s = cvGet2D(img2,j,i);
 			t = cvGet2D(back,j,i);
			if(s.val[0] > 1 && t.val[0] > 1)
			{
			    	
			//temp.val[0] = 255; // White is 255
              cvSet2D(S,j,i,s);				
			}
           
			
		


		}
	}
  cvReleaseImage(& back);
  return S;
}
*/

extern IplImage* mosaic(IplImage* img1, IplImage* img2)
{ // Begin of function
        IplImage* panorama;
	struct feature* feat1, * feat2, *feat;
	struct feature** nbrs;
	struct kd_node* kd_root;
	
	struct feature* feature_lowe1; // I use this to find out the number of inliers
        struct feature* feature_lowe2; // I use this to find out the number of inliers

	CvScalar z;
	double * pBounds;
	double add_x;
	double add_y;   // variables to compute the new size of the mosaicing

        CvPoint pt1, pt2;
	double d0, d1;
	int n1, n2, k, i, m = 0, dx, dy;
	int n=0;
	int levels, levels2;
	unsigned char * ptr;

     
 	    
   fprintf( stderr, "\nFinding features in %s...\n", img1);
	n1 = sift_features( img1, &feat1 );
	fprintf( stderr, "Finding features in %s...\n", img2 );
	n2 = sift_features( img2, &feat2 );

    kd_root = kdtree_build( feat1, n1 ); // feat2 and n2
	// I want to change the order of the images I have to use feat1, n1

	for( i = 0; i < n2; i++ ) //n1 // to chage order n2
	{
		feat = feat2 + i; //feat1 // to change order feat2
		k = kdtree_bbf_knn( kd_root, feat, 2, &nbrs, KDTREE_BBF_MAX_NN_CHKS );
		if( k == 2 )
		{
	//Returns the squared Euclidian distance between the descriptors of  feat and nbrs[0]
			d0 = descr_dist_sq( feat, nbrs[0] ); 
			d1 = descr_dist_sq( feat, nbrs[1] );
			if( d0 < d1 * NN_SQ_DIST_RATIO_THR ) // Selection of the "distintic match"
			{
				pt1 = cvPoint( cvRound( feat->x ), cvRound( feat->y ) );
				pt2 = cvPoint( cvRound( nbrs[0]->x ), cvRound( nbrs[0]->y ) );
				pt2.y += img1->height;
				//cvLine( stacked, pt1, pt2, CV_RGB(255,0,255), 1, 8, 0 );
				m++;
				feat2[i].fwd_match = nbrs[0]; //feat1[i].fwd_match // to change order feat2[i].fwd_match
				//printf("\n The value of this match is : %f",nbrs[0]);
				
			}
		}
		free( nbrs );
	}

	
	{   
	 CvMat* H;
	 
  		   
	 H = ransac_xform( feat2, n2, FEATURE_FWD_MATCH, lsq_homog, 4, 0.01,
       			homog_xfer_err, 3.0, NULL, NULL ); //feat1, n1, FEATURE_FWD_MATCH 
	 // I want to change the order of the images I have to use feat2, n2
      
		if( H )
		{
		   
		   IplImage* newimg2; // newimg1// change order: newimg2
		  // IplImage* newimg1_interp; // because I am changing
		    IplImage* newimg2_interp; // the order of the images
		    CvPoint2D64f ptimg1;
		    CvPoint2D64f ptnewimg1;
		 // Begin of the declaration for the test of mosaicing with 2 images
		    CvPoint2D64f top_left;          // top-left point after transformation
		    CvPoint2D64f top_right;         // top-right point after transformation
		    CvPoint2D64f bottom_left;       // bottom-left point after transformation
		    CvPoint2D64f bottom_right;       // bottom-right point after transformation
		    CvPoint2D64f top_left2;         // to compute the dy for the interp imag

	            double x_neg = 0.0;
		    double x_pos = 0.0;
		    double y_neg = 0.0;
             	    double y_pos = 0.0;
		    double neg_x;
             	    double neg_y;
             	    double pos_x;
             	    double pos_y;
		    CvSize sz;
			 
		    CvMat* warp;  // with the offset in the Homography matrix
                    int i;
	            int j;
           	    int k;
		    int area;
		    int x_length = 0;
		    int y_length = 0;
		    int x_offset = 0;
		    int y_offset = 0; 
     
		    CvPoint2D64f current;
                    CvPoint2D64f target;
		    CvPoint2D64f temp;
		    CvPoint2D64f box_left; // This is the left coordenate of the final box of the img1 (no reference frame)
		   CvPoint2D64f box_right; // and this is the right one
		      
		   CvScalar s;
		   CvScalar t;
           	   CvScalar u;
		   CvScalar ave;
               
      pBounds = (double*) calloc( 4, sizeof( double ) );

      top_left = persp_xform_pt( cvPoint2D64f( 0, 0 ), H );//H2 -> H
      top_right = persp_xform_pt( cvPoint2D64f( img2->width-1, 0 ), H ); //H2 -> H //img1 ->img2
      bottom_left = persp_xform_pt( cvPoint2D64f( 0,img2->height-1), H ); //H2 -> H  //img1 ->img2
      bottom_right = persp_xform_pt( cvPoint2D64f( img2->width-1, img2->height-1 ), H ); //same
			
		   neg_x = top_left.x;
		   neg_y = top_left.y;
		   pos_x = top_left.x;
		   pos_y = top_left.y;

	if( top_right.x < neg_x )
      	neg_x = top_right.x;
        if( top_right.x > pos_x )
        pos_x = top_right.x;
        if( top_right.y < neg_y )
    	neg_y = top_right.y;
    	if( top_right.y > pos_y )
      	pos_y = top_right.y;
    	if( bottom_left.x < neg_x )
      	neg_x = bottom_left.x;
    	if( bottom_left.x > pos_x )
      	pos_x = bottom_left.x;
    	if( bottom_left.y < neg_y )
      	neg_y = bottom_left.y;
    	if( bottom_left.y > pos_y )
      	pos_y = bottom_left.y;

    	if( bottom_right.x < neg_x )
      	neg_x = bottom_right.x;
    	if( bottom_right.x > pos_x )
      	pos_x = bottom_right.x;
    	if( bottom_right.y < neg_y )
      	neg_y = bottom_right.y;
    	if( bottom_right.y > pos_y )
      	pos_y = bottom_right.y;
 
    	if( pos_x/neg_x > 1.0001 || pos_x/neg_x < 0.9999 ){
      	if( neg_x < x_neg ) x_neg = neg_x;
      	if( pos_x > x_pos ) x_pos = pos_x;
      	if( neg_y < y_neg ) y_neg = neg_y;
      	if( pos_y > y_pos ) y_pos = pos_y;
	}
	 pBounds[0] = x_neg;
	 pBounds[1] = x_pos;
     pBounds[2] = y_neg;
     pBounds[3] = y_pos;



// End of the part of the code that computes the bounds,
// Now our next step is to perform the rendering to form the panorama
// Let's see how it works :D

     if( x_neg < 0 && x_pos < 0 )
          x_length = floor(abs(x_neg) - abs(x_pos));
		 else if( x_neg < 0 && x_pos >= 0 )
		  x_length = floor(abs(x_neg) + x_pos);
		 else if( x_neg >= 0 && x_pos >= 0 )
		  x_length = floor(x_pos - x_neg);
	   		//else fatal_error( "x-bounds are off\n" );
		 else printf("x-bounds are off\n" );
		 if( y_neg < 0 && y_pos < 0 )
			y_length = floor(abs(y_neg) - abs(y_pos));
	     else if( y_neg < 0 && y_pos >= 0 )
			y_length = floor(abs(y_pos-y_neg));
		 else if( y_neg >= 0 && y_pos >= 0 )
			y_length = floor(y_pos - y_neg);
		// else fatal_error( "y-bounds are off\n" );
		  else printf( "y-bounds are off\n" );

		x_offset = floor(abs(pBounds[0]))+5;
		y_offset = floor(abs(pBounds[2]))+5;



	  // The calculus of x_length and y_length of this program aren't working appropiately so, I will modify them
      
      // Compute the max value of the top_right and bottom_right in the x direcction
	  if(top_right.x >= bottom_right.x) 
		  x_pos = top_right.x;
	  else
		  x_pos = bottom_right.x;
      // Now finding the minimum value in the left side in the x direcction

	  if(top_left.x <= bottom_left.x) 
		  x_neg = top_left.x;
	  else
		  x_neg = bottom_left.x;

	  // Compute the min value of the top_right and bottom_right in the y direcction
	  if(top_right.y <= top_left.y) 
		  y_neg = top_right.y;
	  else
		  y_neg = top_left.y;
      // Now finding the max value in the left side in the y direcction

	  if(bottom_right.y >= bottom_left.y) 
		  y_pos = bottom_right.y;
	  else
		  y_pos = bottom_left.y;
        
	  x_length = ceil(x_pos - x_neg);
	  y_length = ceil(y_pos - y_neg);
	  x_offset = floor(abs(x_neg));
	  y_offset = floor(abs(y_neg));

 	  add_x = add_y = 0; 
 	  if(top_left.x < 0) add_x -= ceil(top_left.x);
	  if(top_left.y < 0) add_y -= ceil(top_left.y);
	  if(bottom_left.x < 0) add_x -= ceil(bottom_left.x);
	  if(bottom_left.y > img1->height) add_y += ceil(bottom_left.y  - img1->height);
	  if(top_right.x < 0) add_x -= ceil(top_right.x);
	  if(top_right.y < 0) add_y -= ceil(top_right.y);
	  if(top_right.x > img1->width) add_x += ceil(top_right.x - img1->width);
	  if(bottom_right.x > img1->width) add_x += ceil(bottom_right.x - img1->width);
	  if(bottom_right.y > img1->height) add_y += ceil(bottom_right.y - img1->height);
 
  	  panorama = cvCreateImage( cvSize( add_x + x_offset + img1->width, add_y + y_offset + img1->height ) , IPL_DEPTH_8U , 1 ); //img2 -> img1
	  cvZero( panorama );
	  
	  newimg2 = cvCreateImage( cvSize(x_length,y_length) , IPL_DEPTH_8U , 1 );  //newimg1 -> newimg2
	
	 // printf("\n mosaic _ I am here 1");


       for( i=0; i<img2->width; i++ ){ //img1->width //img2->width
          for( j=0; j<img2->height; j++ ){//img1->height //img2->width
	        current = cvPoint2D64f( i, j ); // i->x and j->y
		    target = persp_xform_pt( current, H );   //H 
		    s=cvGet2D( img2, j, i ); //img1 -> img2 // Get the pixel value in (j,i) of img1
			// newimg1->newimg2
		    if(target.y+y_offset<newimg2->height && target.y+y_offset>= 0 && target.x+x_offset < newimg2->width && target.x+x_offset >= 0)
		     {
		      cvSet2D( newimg2, target.y+y_offset, target.x+x_offset, s ); 	
		      }   
	   }} // End of the for loops
	 // printf("\n mosaic _ I am here 2");
        newimg2_interp = interp2ir(img2, H, x_offset, y_offset, x_length, y_length); 
	 //printf("\n mosaic _ I am here 3");
	 //printf("\n panorama size: %dx%d\t img1 size: %dx%d ", panorama->height, panorama->width, img1->height, img1->width);
	 //printf("\n x_offset: %d\t y_offset: %d ", x_offset, y_offset);
		 
      for( i=0; i<img1->width; i++ ){//img2 // If I want to change I just have to use img1 instead
         for( j=0; j<img1->height; j++ ){ //img2
           s=cvGet2D(img1, j, i ); //img2
           cvSet2D( panorama, j+y_offset, i+x_offset, s );
	  }
        }

  //printf("\n mosaic _ I am here 4");

 ////This part will try to see why the warping of the img1 is not perfect instead it does when I work with both images
 //// at the same time.

 for( i=0; i<newimg2_interp ->width; i++ ){ //img1 //newimg1_interp -> newimg2_interp
    for( j=0; j<newimg2_interp ->height; j++ ){//img1 //newimg1_interp -> newimg2_interp
		target = cvPoint2D64f( i, j );
		s=cvGet2D( newimg2_interp , j, i ); //img1 //newimg1_interp -> newimg2_interp
		t=cvGet2D( panorama, target.y, target.x ); 		
	//	u = cvGet2D(newimg2, target.y, target.x ); 
        if( t.val[0] < 1){
		   cvSet2D( panorama, target.y, target.x, s );//Set the pixel value in target with the value s 
		  }
		else{
		  if( s.val[0] < 1){
			cvSet2D( panorama, target.y, target.x, t );
		     }//If
		  else{
			//ave.val[0] = (t.val[0] + u.val[0])/2;
			//cvSet2D( panorama, target.y, target.x, ave );
			  cvSet2D( panorama, target.y, target.x, t );
		   } // Second Else
		 } // First Else
		} // Second For
	  } // First For  

  
  //cvNamedWindow( "Panorama", 1 );
  //cvShowImage( "Panorama", panorama );
  //cvWaitKey(0);
 
 cvReleaseMat( &H );
 cvReleaseImage( &newimg2 ); // newimg1 // change order newimg1
 cvReleaseImage( &newimg2_interp ); //newimg1_interp //change order newim2_interp
 
 
 }
}
 	
        //cvReleaseImage( &img1 );
	//cvReleaseImage( &img2 );
	kdtree_release( kd_root );
	free( feat1 );
        free( feat2 );
        free( pBounds );	
	return panorama;

} // End of the function



extern CvPoint findOffsets(IplImage* img2, CvMat* H)
{

    double x_neg = 0.0;
	double x_pos = 0.0;
	double y_neg = 0.0;
    double y_pos = 0.0;
	double neg_x;
    double neg_y;
    double pos_x;
    double pos_y;
	CvSize sz;
	CvPoint offset;
	double * pBounds;

	CvPoint2D64f top_left;          // top-left point after transformation
	CvPoint2D64f top_right;         // top-right point after transformation
	CvPoint2D64f bottom_left;       // bottom-left point after transformation
	CvPoint2D64f bottom_right;       // bottom-right point after transformation
	CvPoint2D64f top_left2;         // to compute the dy for the interp imag
			 
	//CvMat* warp;  // with the offset in the Homography matrix
    int i;
	int j;
    int k;
	int area;
	int x_length = 0;
	int y_length = 0;
	int x_offset = 0;
	int y_offset = 0; 
     
	
	CvPoint2D64f box_left; // This is the left coordenate of the final box of the img1 (no reference frame)
	CvPoint2D64f box_right; // and this is the right one
		      
	               
    pBounds = (double*) calloc( 4, sizeof( double ) );

     top_left = persp_xform_pt( cvPoint2D64f( 0, 0 ), H );//H2 -> H
     top_right = persp_xform_pt( cvPoint2D64f( img2->width-1, 0 ), H ); //H2 -> H //img1 ->img2
     bottom_left = persp_xform_pt( cvPoint2D64f( 0,img2->height-1), H ); //H2 -> H  //img1 ->img2
     bottom_right = persp_xform_pt( cvPoint2D64f( img2->width-1, img2->height-1 ), H ); //same
			
	 neg_x = top_left.x;
	 neg_y = top_left.y;
	 pos_x = top_left.x;
	 pos_y = top_left.y;

	if( top_right.x < neg_x )
      	neg_x = top_right.x;
        if( top_right.x > pos_x )
        pos_x = top_right.x;
        if( top_right.y < neg_y )
    	neg_y = top_right.y;
    	if( top_right.y > pos_y )
      	pos_y = top_right.y;
    	if( bottom_left.x < neg_x )
      	neg_x = bottom_left.x;
    	if( bottom_left.x > pos_x )
      	pos_x = bottom_left.x;
    	if( bottom_left.y < neg_y )
      	neg_y = bottom_left.y;
    	if( bottom_left.y > pos_y )
      	pos_y = bottom_left.y;

    	if( bottom_right.x < neg_x )
      	neg_x = bottom_right.x;
    	if( bottom_right.x > pos_x )
      	pos_x = bottom_right.x;
    	if( bottom_right.y < neg_y )
      	neg_y = bottom_right.y;
    	if( bottom_right.y > pos_y )
      	pos_y = bottom_right.y;
 
    	if( pos_x/neg_x > 1.0001 || pos_x/neg_x < 0.9999 ){
      	if( neg_x < x_neg ) x_neg = neg_x;
      	if( pos_x > x_pos ) x_pos = pos_x;
      	if( neg_y < y_neg ) y_neg = neg_y;
      	if( pos_y > y_pos ) y_pos = pos_y;
	}
	 pBounds[0] = x_neg;
	 pBounds[1] = x_pos;
     pBounds[2] = y_neg;
     pBounds[3] = y_pos;



// End of the part of the code that computes the bounds,
// Now our next step is to perform the rendering to form the panorama
// Let's see how it works :D

     if( x_neg < 0 && x_pos < 0 )
          x_length = floor(abs(x_neg) - abs(x_pos));
		 else if( x_neg < 0 && x_pos >= 0 )
		  x_length = floor(abs(x_neg) + x_pos);
		 else if( x_neg >= 0 && x_pos >= 0 )
		  x_length = floor(x_pos - x_neg);
	   	//	else fatal_error( "x-bounds are off\n" );
		 	else printf( "x-bounds are off\n" );
		 if( y_neg < 0 && y_pos < 0 )
			y_length = floor(abs(y_neg) - abs(y_pos));
	     else if( y_neg < 0 && y_pos >= 0 )
			y_length = floor(abs(y_pos-y_neg));
		 else if( y_neg >= 0 && y_pos >= 0 )
			y_length = floor(y_pos - y_neg);
		 // else fatal_error( "y-bounds are off\n" );
		 else printf( "y-bounds are off\n" );

		x_offset = floor(abs(pBounds[0]))+5;
		y_offset = floor(abs(pBounds[2]))+5;



	  // The calculus of x_length and y_length of this program aren't working appropiately so, I will modify them
      
      // Compute the max value of the top_right and bottom_right in the x direcction
	  if(top_right.x >= bottom_right.x) 
		  x_pos = top_right.x;
	  else
		  x_pos = bottom_right.x;
      // Now finding the minimum value in the left side in the x direcction

	  if(top_left.x <= bottom_left.x) 
		  x_neg = top_left.x;
	  else
		  x_neg = bottom_left.x;

	  // Compute the min value of the top_right and bottom_right in the y direcction
	  if(top_right.y <= top_left.y) 
		  y_neg = top_right.y;
	  else
		  y_neg = top_left.y;
      // Now finding the max value in the left side in the y direcction

	  if(bottom_right.y >= bottom_left.y) 
		  y_pos = bottom_right.y;
	  else
		  y_pos = bottom_left.y;
        
	  x_length = ceil(x_pos - x_neg);
	  y_length = ceil(y_pos - y_neg);
	  x_offset = floor(abs(x_neg));
	  y_offset = floor(abs(y_neg));

	  offset.x = x_offset;
	  offset.y = y_offset;

	  return offset;

}

extern CvPoint2D64f persp_xform_pt2( CvPoint2D64f pt, CvMat* H)
{
   /*
   CvMat* src = NULL;
   CvMat* dst = NULL;
   src = cvCreateMat(3,1,CV_64FC1);
   dst = cvCreateMat(3,1,CV_64FC1);
   */
   CvPoint2D64f rslt;  
   double a,b,D;
   
 
  a = (pt.x)*cvmGet(H,0,0) + (pt.y)*cvmGet(H,0,1) + cvmGet(H,0,2);
  b = (pt.x)*cvmGet(H,1,0) + (pt.y)*cvmGet(H,1,1) + cvmGet(H,1,2);
  D = (pt.x)*cvmGet(H,2,0) + (pt.y)*cvmGet(H,2,1) + 1;

  a = a/D;
  b = b/D;
  
  /*
  cvmSet(dst,0,0,a);
  cvmSet(dst,0,1,b);
  cvmSet(dst,0,2,1);
  */
    
  rslt.x = a;
  rslt.y = b;
         
  return rslt;

}


extern IplImage* mosaickH(IplImage* img1, IplImage* img2, CvMat* H)
{
 //		  IplImage* mosaickH = NULL;
  
		  IplImage* panorama = NULL;
          CvScalar z;
	      double *pBounds = NULL;
	      double add_x;
	      double add_y;   
		  CvMat* Hp;
		   IplImage* newimg2 = NULL; // newimg1// change order: newimg2
		   IplImage* newimg2_interp = NULL; // the order of the images
			
			// To implement the algorithm for the overlapping area
			IplImage* overlaping1 = NULL;
			IplImage* overlaping2 = NULL;
			IplImage* AreaCommon = NULL;
			
  
//			double error;
					
			// end of the declarations for the overapping area
			
		    CvPoint2D64f ptimg1;
		    CvPoint2D64f ptnewimg1;
		 // Begin of the declaration for the test of mosaicing with 2 images
		    CvPoint2D64f top_left;          // top-left point after transformation
		    CvPoint2D64f top_right;         // top-right point after transformation
		    CvPoint2D64f bottom_left;       // bottom-left point after transformation
		    CvPoint2D64f bottom_right;       // bottom-right point after transformation
		    CvPoint2D64f top_left2;         // to compute the dy for the interp imag

	        double x_neg = 0.0;
		    double x_pos = 0.0;
		    double y_neg = 0.0;
            double y_pos = 0.0;
		    double neg_x;
            double neg_y;
            double pos_x;
            double pos_y;
		    CvSize sz;
			 
//		    CvMat* warp = NULL;  // with the offset in the Homography matrix
            int i;
	        int j;
           	int k;
		    int area;
		    int x_length = 0;
		    int y_length = 0;
		    int x_offset = 0;
		    int y_offset = 0; 
     
		    CvPoint2D64f current;
            CvPoint2D64f target;
		    CvPoint2D64f temp;
		    CvPoint2D64f box_left; // This is the left coordenate of the final box of the img1 (no reference frame)
		    CvPoint2D64f box_right; // and this is the right one
		      
		   CvScalar s;
		   CvScalar t;
           CvScalar u;
		   CvScalar ave;
                
           pBounds = (double*) calloc( 4, sizeof( double ) );
		   

       
		
		
      //for(i = 0; i < 3; i++){
	 // for(j = 0; j < 3; j++){
	    // printf("H(%d,%d) = %f\n", i,j,(float)cvmGet(H,i,j));}}
        
         
	  
	  top_left = persp_xform_pt2( cvPoint2D64f( 0, 0 ), H );//H2 -> H
      top_right = persp_xform_pt2( cvPoint2D64f( img2->width-1, 0 ), H ); //H2 -> H //img1 ->img2
      bottom_left = persp_xform_pt2( cvPoint2D64f( 0,img2->height-1), H ); //H2 -> H  //img1 ->img2
      bottom_right = persp_xform_pt2( cvPoint2D64f( img2->width-1, img2->height-1 ), H ); //same

	 // printf("\n top_left(x,y) = %f, %f \t top_right(x,y) = %f,%f \n", top_left.x, top_left.y, top_right.x, top_right.y);
	 // printf("\n bottom_left(x,y) = %f, %f \t bottom_right(x,y) = %f,%f \n", bottom_left.x, bottom_left.y, bottom_right.x, bottom_right.y);
	 // getchar();

  
			
		   neg_x = top_left.x;
		   neg_y = top_left.y;
		   pos_x = top_left.x;
		   pos_y = top_left.y;

	if( top_right.x < neg_x )
      	neg_x = top_right.x;
        if( top_right.x > pos_x )
        pos_x = top_right.x;
        if( top_right.y < neg_y )
    	neg_y = top_right.y;
    	if( top_right.y > pos_y )
      	pos_y = top_right.y;
    	if( bottom_left.x < neg_x )
      	neg_x = bottom_left.x;
    	if( bottom_left.x > pos_x )
      	pos_x = bottom_left.x;
    	if( bottom_left.y < neg_y )
      	neg_y = bottom_left.y;
    	if( bottom_left.y > pos_y )
      	pos_y = bottom_left.y;

    	if( bottom_right.x < neg_x )
      	neg_x = bottom_right.x;
    	if( bottom_right.x > pos_x )
      	pos_x = bottom_right.x;
    	if( bottom_right.y < neg_y )
      	neg_y = bottom_right.y;
    	if( bottom_right.y > pos_y )
      	pos_y = bottom_right.y;
 
    	if( pos_x/neg_x > 1.0001 || pos_x/neg_x < 0.9999 ){
      	if( neg_x < x_neg ) x_neg = neg_x;
      	if( pos_x > x_pos ) x_pos = pos_x;
      	if( neg_y < y_neg ) y_neg = neg_y;
      	if( pos_y > y_pos ) y_pos = pos_y;
	}
	 pBounds[0] = x_neg;
	 pBounds[1] = x_pos;
     pBounds[2] = y_neg;
     pBounds[3] = y_pos;



// End of the part of the code that computes the bounds,
// Now our next step is to perform the rendering to form the panorama
// Let's see how it works :D

     if( x_neg < 0 && x_pos < 0 )
          x_length = floor(abs(x_neg) - abs(x_pos));
		 else if( x_neg < 0 && x_pos >= 0 )
		  x_length = floor(abs(x_neg) + x_pos);
		 else if( x_neg >= 0 && x_pos >= 0 )
		  x_length = floor(x_pos - x_neg);
	   		else printf( "x-bounds are off\n" );
		 if( y_neg < 0 && y_pos < 0 )
			y_length = floor(abs(y_neg) - abs(y_pos));
	     else if( y_neg < 0 && y_pos >= 0 )
			y_length = floor(abs(y_pos-y_neg));
		 else if( y_neg >= 0 && y_pos >= 0 )
			y_length = floor(y_pos - y_neg);
		 else printf( "y-bounds are off\n" );

		x_offset = floor(abs(pBounds[0]))+5;
		y_offset = floor(abs(pBounds[2]))+5;

		
		printf("\n x_offset = %d", x_offset);
		printf("\n y_offset = %d", y_offset);


	  // The calculus of x_length and y_length of this program aren't working appropiately so, I will modify them
      
      // Compute the max value of the top_right and bottom_right in the x direcction
	  if(top_right.x >= bottom_right.x) 
		  x_pos = top_right.x;
	  else
		  x_pos = bottom_right.x;
      // Now finding the minimum value in the left side in the x direcction

	  if(top_left.x <= bottom_left.x) 
		  x_neg = top_left.x;
	  else
		  x_neg = bottom_left.x;

	  // Compute the min value of the top_right and bottom_right in the y direcction
	  if(top_right.y <= top_left.y) 
		  y_neg = top_right.y;
	  else
		  y_neg = top_left.y;
      // Now finding the max value in the left side in the y direcction

	  if(bottom_right.y >= bottom_left.y) 
		  y_pos = bottom_right.y;
	  else
		  y_pos = bottom_left.y;
        
	  x_length = ceil(x_pos - x_neg);
	  y_length = ceil(y_pos - y_neg);
	  x_offset = floor(abs(x_neg));
	  y_offset = floor(abs(y_neg));

	 

 	  add_x = add_y = 0; 
 	  if(top_left.x < 0) add_x -= ceil(top_left.x);
	  if(top_left.y < 0) add_y -= ceil(top_left.y);
	  if(bottom_left.x < 0) add_x -= ceil(bottom_left.x);
	  if(bottom_left.y > img1->height) add_y += ceil(bottom_left.y  - img1->height);
	  if(top_right.x < 0) add_x -= ceil(top_right.x);
	  if(top_right.y < 0) add_y -= ceil(top_right.y);
	  if(top_right.x > img1->width) add_x += ceil(top_right.x - img1->width);
	  if(bottom_right.x > img1->width) add_x += ceil(bottom_right.x - img1->width);
	  if(bottom_right.y > img1->height) add_y += ceil(bottom_right.y - img1->height);
 
  
           
  	  panorama = cvCreateImage( cvSize( add_x + 10 + img1->width, add_y +10 + img1->height ) , IPL_DEPTH_8U , 1 ); //img2 -> img1
	  cvZero( panorama );
	  
	     
	  // This part was added to perform the Szeliski Algorithm based on: " Video Mosaics for Virtual Environments"
	  overlaping1 = cvCreateImage( cvSize( add_x + 10 + img1->width, add_y +10 + img1->height ) , IPL_DEPTH_8U , 1 ); 
	  cvZero( overlaping1 );
	  
	  overlaping2 = cvCreateImage( cvSize( add_x + 10 + img1->width, add_y +10 + img1->height ) , IPL_DEPTH_8U , 1 ); 
	  cvZero( overlaping2 );
	  
	  AreaCommon = cvCreateImage( cvSize( add_x + 10 + img1->width, add_y +10 + img1->height ) , IPL_DEPTH_8U , 1 ); 
	  cvZero( AreaCommon );
	  
	  // end of the algorithm of Szeliski
	  
	  newimg2 = cvCreateImage( cvSize(x_length,y_length) , IPL_DEPTH_8U , 1 );  //newimg1 -> newimg2
	  cvZero( newimg2 );
	
	   
       for( i=0; i<img2->width; i++ ){ //img1->width //img2->width
          for( j=0; j<img2->height; j++ ){//img1->height //img2->width
	        current = cvPoint2D64f( i, j ); // i->x and j->y
		target = persp_xform_pt2( current, H );   //H 
		s=cvGet2D( img2, j, i ); //img1 -> img2 // Get the pixel value in (j,i) of img1
			// newimg1->newimg2
		if(target.y+y_offset<newimg2->height && target.y+y_offset>= 0 && target.x+x_offset < newimg2->width && target.x+x_offset >= 0)
		{
		 cvSet2D( newimg2, target.y+y_offset, target.x+x_offset, s ); 	
		}   
	}} // End of the for loops
	
	
        newimg2_interp = interp2ir(img2, H, x_offset, y_offset, x_length, y_length); 
	
      for( i=0; i<img1->width; i++ ){//img2 // If I want to change I just have to use img1 instead
         for( j=0; j<img1->height; j++ ){ //img2
           s=cvGet2D(img1, j, i ); //img2
           cvSet2D( panorama, j+y_offset, i+x_offset, s );
	  }
        }
 ////This part will try to see why the warping of the img1 is not perfect instead it does when I work with both images
 //// at the same time.

	
 for( i=0; i<newimg2_interp ->width; i++ ){ //img1 //newimg1_interp -> newimg2_interp
    for( j=0; j<newimg2_interp ->height; j++ ){//img1 //newimg1_interp -> newimg2_interp
		target = cvPoint2D64f( i, j );
		s=cvGet2D( newimg2_interp , j, i ); //img1 //newimg1_interp -> newimg2_interp
		t=cvGet2D( panorama, target.y, target.x ); 	
        u.val[0] = 255;
		u.val[1] = 255;
		u.val[2] = 255;
		
	//	u = cvGet2D(newimg2, target.y, target.x ); 
        if( t.val[0] < 1){ // Is the img1' negative or zero
		   cvSet2D( panorama, target.y, target.x, s );//Set the pixel value in target with the value s 
		  }
		else{
		  if( s.val[0] < 1){
			cvSet2D( panorama, target.y, target.x, t );
		     }//If
		  else{  // Common Area of overlaping
			//ave.val[0] = (t.val[0] + u.val[0])/2;
			//cvSet2D( panorama, target.y, target.x, ave );
			  cvSet2D( overlaping1, target.y, target.x, t );
			  cvSet2D( overlaping2, target.y, target.x, s );
			  cvSet2D( AreaCommon, target.y, target.x, u );
			  //printf("\n I am inside the overlaping area");
		  	  
			              
			  
		   } // Second Else
		 } // First Else
		} // Second For
	  } // First For  
	  



 
 cvReleaseImage( &newimg2 ); // newimg1 // change order newimg1
 cvReleaseImage( &newimg2_interp ); //newimg1_interp //change order newim2_interp
 cvReleaseImage( &overlaping1 ); //newimg1_interp //change order newim2_interp
 cvReleaseImage( &overlaping2 ); //newimg1_interp //change order newim2_interp
 cvReleaseImage( &AreaCommon ); //newimg1_interp //change order newim2_interp


  return panorama;
}

extern void displaceImg(IplImage* img1,IplImage* dispImg, int row, int col)
{
  int i, j;
     
  for(i = 0; i < img1->height; i++){ //rows
    for(j = 0; j < img1->width; j++){ //cols
		         if( ((i + row) < dispImg->height) && ((j + col) < dispImg->width)){
					 cvSet2D(dispImg,i + row,j + col, cvGet2D(img1,i,j));
				   }
	 }
   }
return;
}


extern void ShiftImage(IplImage* src, IplImage* dst, int x_offset, int y_offset)
{
//    IplImage* ShiftedImage;
    int i, j;
    CvScalar s;
   // ShiftedImage = cvCreateImage( cvSize( 2*src->width, 2*src->height ) , IPL_DEPTH_8U , src->nChannels ); 
    //cvZero( ShiftedImage );
	printf("\n %dx%d", src->height, src->width);
	getchar();
    for(i = 0; src->height; i++){//rows
       for(j = 0; src->width; j++){//cols
           if( i + y_offset < dst->height, j + x_offset< dst->width){
           s=cvGet2D( src , i, j ); 
	   cvSet2D( dst, i + y_offset, j + x_offset, s ); 
          printf("\n dst(%d,%d) =  %f", i,j,s.val[0]);
           }
          } 
        }    
    
 return;
 
}

extern void WarpImageUnderQuad(IplImage* src, IplImage* dst, struct image_border border)
{ 
  CvPoint2D32f srcQuad[4], dstQuad[4];
  CvMat* warp_matrix = cvCreateMat(3,3,CV_32FC1);

   srcQuad[0].x = 0;           //src Top left
   srcQuad[0].y = 0;
   srcQuad[1].x = src->width - 1;  //src Top right
   srcQuad[1].y = 0;
   srcQuad[2].x = 0;           //src Bottom left
   srcQuad[2].y = src->height - 1;
   srcQuad[3].x = src->width - 1;  //src Bot right
   srcQuad[3].y = src->height - 1;
      //- - - - - - - - - - - - - -//
   dstQuad[0].x = border.P1.x;  //dst Top left
   dstQuad[0].y = border.P1.y;
   dstQuad[1].x = border.P2.x;  //dst Top right
   dstQuad[1].y = border.P2.y;
   dstQuad[2].x = border.P4.x;  //dst Bottom left
   dstQuad[2].y = border.P4.y;      
   dstQuad[3].x = border.P3.x;  //dst Bot right
   dstQuad[3].y = border.P3.y;

   cvGetPerspectiveTransform(srcQuad,dstQuad,warp_matrix);

   cvWarpPerspective(src,dst,warp_matrix,CV_INTER_LINEAR+CV_WARP_FILL_OUTLIERS,
            cvScalarAll(0), cvPoint(0,0));
  // cvWarpPerspective(src,dst,warp_matrix,CV_INTER_LINEAR+CV_WARP_FILL_OUTLIERS,
           //   cvScalarAll(0));

   cvReleaseMat(&warp_matrix);

 return;
}


extern void WarpImageUnderROIQuad(IplImage* src, IplImage* dst, struct image_border border_src)
{ 
  CvPoint2D32f srcQuad[4], dstQuad[4];
  CvMat* warp_matrix = cvCreateMat(3,3,CV_32FC1);

  /*
  struct image_border border_dst;
  int widthROI;
  int heightROI;
  

  widthROI = MAX(border_src.P2.x,border_src.P3.x) - MIN(border_src.P1.x,border_src.P4.x);
  heightROI = MAX(border_src.P4.y,border_src.P3.y) - MIN(border_src.P1.y,border_src.P2.y);
  */

   srcQuad[0].x = border_src.P1.x;           //src Top left
   srcQuad[0].y = border_src.P1.y; ;
   srcQuad[1].x = border_src.P2.x;  //src Top right
   srcQuad[1].y = border_src.P2.y;
   srcQuad[2].x = border_src.P4.x;           //src Bottom left
   srcQuad[2].y = border_src.P4.y;
   srcQuad[3].x = border_src.P3.x;  //src Bot right
   srcQuad[3].y = border_src.P3.y;
      //- - - - - - - - - - - - - -//

   dstQuad[0].x = 0;           //dst Top left
   dstQuad[0].y = 0;
   dstQuad[1].x = dst->width - 1;  //dst Top right
   dstQuad[1].y = 0;
   dstQuad[2].x = 0;           //dst Bottom left
   dstQuad[2].y = dst->height - 1;
   dstQuad[3].x = dst->width - 1;  //dst Bot right
   dstQuad[3].y = dst->height - 1;

   cvGetPerspectiveTransform(srcQuad,dstQuad,warp_matrix);
   cvWarpPerspective(src,dst,warp_matrix,CV_INTER_LINEAR+CV_WARP_FILL_OUTLIERS,
              cvScalarAll(0), cvPoint(0,0));

   cvReleaseMat(&warp_matrix);

 return;
}

extern void WarpImageUnderQuadInv(IplImage* src, IplImage* dst, struct image_border border)
{ 
  CvPoint2D32f srcQuad[4], dstQuad[4];
  CvMat* warp_matrix = cvCreateMat(3,3,CV_32FC1);

   
   srcQuad[0].x = border.P1.x;  //src Top left
   srcQuad[0].y = border.P1.y;
   srcQuad[1].x = border.P2.x;  //src Top right
   srcQuad[1].y = border.P2.y;
   srcQuad[2].x = border.P4.x;  //src Bottom left
   srcQuad[2].y = border.P4.y;      
   srcQuad[3].x = border.P3.x;  //src Bot right
   srcQuad[3].y = border.P3.y;
  
  
   dstQuad[0].x = 0;           //dst Top left
   dstQuad[0].y = 0;
   dstQuad[1].x = dst->width - 1;  //dst Top right
   dstQuad[1].y = 0;
   dstQuad[2].x = 0;           //dst Bottom left
   dstQuad[2].y = dst->height - 1;
   dstQuad[3].x = dst->width - 1;  //dst Bot right
   dstQuad[3].y = dst->height - 1;
      //- - - - - - - - - - - - - -//
  

   cvGetPerspectiveTransform(srcQuad,dstQuad,warp_matrix);
   cvWarpPerspective(src,dst,warp_matrix,CV_INTER_LINEAR+CV_WARP_FILL_OUTLIERS,
              cvScalarAll(0), cvPoint(0,0));

   cvReleaseMat(&warp_matrix);

 return;
}
// This function finds the region S within the image_border
// the region S is represented by 4 Lines (L1, L2, L3 and L4)
// L1 : P1 -> P2
// L2 : P2 -> P3
// L3 : P3 -> P4
// L4 : P4 -> P1

extern struct RegionS FindSRegion(struct image_border border)
{
 struct RegionS S;
 double vP1[3], vP2[3], vP3[3], vP4[3];
 double vR1[3],vR2[3],vR3[3],vR4[3];

 CvPoint3D32f P1_3d;
 CvPoint3D32f P2_3d;
 CvPoint3D32f P3_3d;
 CvPoint3D32f P4_3d;

 P1_3d.x = border.P1.x;
 P1_3d.y = border.P1.y;
 P1_3d.z = 1.0;

 vP1[0] = border.P1.x;
 vP1[1] = border.P1.y;
 vP1[2] = 1.0;

 P2_3d.x = border.P2.x;
 P2_3d.y = border.P2.y;
 P2_3d.z = 1.0;

 vP2[0] = border.P2.x;
 vP2[1] = border.P2.y;
 vP2[2] = 1.0;

 P3_3d.x = border.P3.x;
 P3_3d.y = border.P3.y;
 P3_3d.z = 1.0;

 vP3[0] = border.P3.x;
 vP3[1] = border.P3.y;
 vP3[2] = 1.0;

 P4_3d.x = border.P4.x;
 P4_3d.y = border.P4.y;
 P4_3d.z = 1.0;

 vP4[0] = border.P4.x;
 vP4[1] = border.P4.y;
 vP4[2] = 1.0;

 crossProduct(vR1,vP1,vP2);
 crossProduct(vR2,vP2,vP3);
 crossProduct(vR3,vP3,vP4);
 crossProduct(vR4,vP4,vP1);

 S.L1.x = (vR1[0])/(vR1[1]); //a   a*x + b*y + c = 0;
 S.L1.y = (vR1[1])/(vR1[1]);  // b
 S.L1.z = (vR1[2])/(vR1[1]);  //c

 S.L2.x = (vR2[0])/(vR2[1]);
 S.L2.y = (vR2[1])/(vR2[1]);
 S.L2.z = (vR2[2])/(vR2[1]);

 S.L3.x = (vR3[0])/(vR3[1]);
 S.L3.y = (vR3[1])/(vR3[1]);
 S.L3.z = (vR3[2])/(vR3[1]);

 S.L4.x = (vR4[0])/(vR4[1]);
 S.L4.y = (vR4[1])/(vR4[1]);
 S.L4.z = (vR4[2])/(vR4[1]);


  return S;
}

// This function tells if the point belongs to the region S 

extern int Belongs2S(CvPoint2D32f point, struct RegionS S)
{
  int result;
  float xq2; // is the x coordinates of the interception of the y value in the Line L2
  float xq4; // is the x coordinates of the interception of the y value in the Line L4
  float yq1; // is the y coordinates of the interception of the x value in the Line L1
  float yq3; // is the x coordinates of the interception of the x value in the Line L3
  float x, y; // the x and y coordinates of the point

  xq2 = - (S.L2.z)/(S.L2.x) - ((S.L2.y)/(S.L2.x))*(point.y);
  xq4 = - (S.L4.z)/(S.L4.x) - ((S.L4.y)/(S.L4.x))*(point.y);
  yq1 = - (S.L1.z)/(S.L1.y) - ((S.L1.x)/(S.L1.y))*(point.x);
  yq3 = - (S.L3.z)/(S.L3.y) - ((S.L3.x)/(S.L3.y))*(point.x);

  x = point.x;
  y = point.y;
  
  if( (y >= yq1) && (x <= xq2) && (y <= yq3) && (x >= xq4)){
  result = 1; // the point belongs to S
  } // end if
  else{
	  result = 0; // the point doesn't belong to S;
  }

  return result; // 0: the point doesn't belong to S, and 1 otherwise
}



/*
extern IplImage* mosaickH_border(IplImage* img1, IplImage* img2, CvMat* H)
{
 //		  IplImage* mosaickH = NULL;
  
		  IplImage* panorama = NULL;
          CvScalar z;
	      double *pBounds = NULL;
	      double add_x;
	      double add_y;   
		  CvMat* Hp;
		   IplImage* newimg2 = NULL; // newimg1// change order: newimg2
		   IplImage* newimg2_interp = NULL; // the order of the images
			
			// To implement the algorithm for the overlapping area
			IplImage* overlaping1 = NULL;
			IplImage* overlaping2 = NULL;
			IplImage* AreaCommon = NULL;
			
  
//			double error;
					
			// end of the declarations for the overapping area
			
		    CvPoint2D64f ptimg1;
		    CvPoint2D64f ptnewimg1;
		 // Begin of the declaration for the test of mosaicing with 2 images
		    CvPoint2D64f top_left;          // top-left point after transformation
		    CvPoint2D64f top_right;         // top-right point after transformation
		    CvPoint2D64f bottom_left;       // bottom-left point after transformation
		    CvPoint2D64f bottom_right;       // bottom-right point after transformation
		    CvPoint2D64f top_left2;         // to compute the dy for the interp imag

	        double x_neg = 0.0;
		    double x_pos = 0.0;
		    double y_neg = 0.0;
            double y_pos = 0.0;
		    double neg_x;
            double neg_y;
            double pos_x;
            double pos_y;
		    CvSize sz;
			 
//		    CvMat* warp = NULL;  // with the offset in the Homography matrix
            int i;
	        int j;
           	int k;
		    int area;
		    int x_length = 0;
		    int y_length = 0;
		    int x_offset = 0;
		    int y_offset = 0; 
     
		    CvPoint2D64f current;
            CvPoint2D64f target;
		    CvPoint2D64f temp;
		    CvPoint2D64f box_left; // This is the left coordenate of the final box of the img1 (no reference frame)
		    CvPoint2D64f box_right; // and this is the right one
		      
		   CvScalar s;
		   CvScalar t;
           CvScalar u;
		   CvScalar ave;
                
           pBounds = (double*) calloc( 4, sizeof( double ) );
		   

       
		
		
      //for(i = 0; i < 3; i++){
	 // for(j = 0; j < 3; j++){
	    // printf("H(%d,%d) = %f\n", i,j,(float)cvmGet(H,i,j));}}
        
         
	  
	  top_left = 2( cvPoint2D64f( 0, 0 ), H );//H2 -> H
      top_right = 2( cvPoint2D64f( img2->width-1, 0 ), H ); //H2 -> H //img1 ->img2
      bottom_left = 2( cvPoint2D64f( 0,img2->height-1), H ); //H2 -> H  //img1 ->img2
      bottom_right = 2( cvPoint2D64f( img2->width-1, img2->height-1 ), H ); //same
  
			
		   neg_x = top_left.x;
		   neg_y = top_left.y;
		   pos_x = top_left.x;
		   pos_y = top_left.y;

	if( top_right.x < neg_x )
      	neg_x = top_right.x;
        if( top_right.x > pos_x )
        pos_x = top_right.x;
        if( top_right.y < neg_y )
    	neg_y = top_right.y;
    	if( top_right.y > pos_y )
      	pos_y = top_right.y;
    	if( bottom_left.x < neg_x )
      	neg_x = bottom_left.x;
    	if( bottom_left.x > pos_x )
      	pos_x = bottom_left.x;
    	if( bottom_left.y < neg_y )
      	neg_y = bottom_left.y;
    	if( bottom_left.y > pos_y )
      	pos_y = bottom_left.y;

    	if( bottom_right.x < neg_x )
      	neg_x = bottom_right.x;
    	if( bottom_right.x > pos_x )
      	pos_x = bottom_right.x;
    	if( bottom_right.y < neg_y )
      	neg_y = bottom_right.y;
    	if( bottom_right.y > pos_y )
      	pos_y = bottom_right.y;
 
    	if( pos_x/neg_x > 1.0001 || pos_x/neg_x < 0.9999 ){
      	if( neg_x < x_neg ) x_neg = neg_x;
      	if( pos_x > x_pos ) x_pos = pos_x;
      	if( neg_y < y_neg ) y_neg = neg_y;
      	if( pos_y > y_pos ) y_pos = pos_y;
	}
	 pBounds[0] = x_neg;
	 pBounds[1] = x_pos;
     pBounds[2] = y_neg;
     pBounds[3] = y_pos;



// End of the part of the code that computes the bounds,
// Now our next step is to perform the rendering to form the panorama
// Let's see how it works :D

     if( x_neg < 0 && x_pos < 0 )
          x_length = floor(abs(x_neg) - abs(x_pos));
		 else if( x_neg < 0 && x_pos >= 0 )
		  x_length = floor(abs(x_neg) + x_pos);
		 else if( x_neg >= 0 && x_pos >= 0 )
		  x_length = floor(x_pos - x_neg);
	   		else printf( "x-bounds are off\n" );
		 if( y_neg < 0 && y_pos < 0 )
			y_length = floor(abs(y_neg) - abs(y_pos));
	     else if( y_neg < 0 && y_pos >= 0 )
			y_length = floor(abs(y_pos-y_neg));
		 else if( y_neg >= 0 && y_pos >= 0 )
			y_length = floor(y_pos - y_neg);
		 else printf( "y-bounds are off\n" );

		x_offset = floor(abs(pBounds[0]))+5;
		y_offset = floor(abs(pBounds[2]))+5;

		
		printf("\n x_offset = %d", x_offset);
		printf("\n y_offset = %d", y_offset);


	  // The calculus of x_length and y_length of this program aren't working appropiately so, I will modify them
      
      // Compute the max value of the top_right and bottom_right in the x direcction
	  if(top_right.x >= bottom_right.x) 
		  x_pos = top_right.x;
	  else
		  x_pos = bottom_right.x;
      // Now finding the minimum value in the left side in the x direcction

	  if(top_left.x <= bottom_left.x) 
		  x_neg = top_left.x;
	  else
		  x_neg = bottom_left.x;

	  // Compute the min value of the top_right and bottom_right in the y direcction
	  if(top_right.y <= top_left.y) 
		  y_neg = top_right.y;
	  else
		  y_neg = top_left.y;
      // Now finding the max value in the left side in the y direcction

	  if(bottom_right.y >= bottom_left.y) 
		  y_pos = bottom_right.y;
	  else
		  y_pos = bottom_left.y;
        
	  x_length = ceil(x_pos - x_neg);
	  y_length = ceil(y_pos - y_neg);
	  x_offset = floor(abs(x_neg));
	  y_offset = floor(abs(y_neg));

 	  add_x = add_y = 0; 
 	  if(top_left.x < 0) add_x -= ceil(top_left.x);
	  if(top_left.y < 0) add_y -= ceil(top_left.y);
	  if(bottom_left.x < 0) add_x -= ceil(bottom_left.x);
	  if(bottom_left.y > img1->height) add_y += ceil(bottom_left.y  - img1->height);
	  if(top_right.x < 0) add_x -= ceil(top_right.x);
	  if(top_right.y < 0) add_y -= ceil(top_right.y);
	  if(top_right.x > img1->width) add_x += ceil(top_right.x - img1->width);
	  if(bottom_right.x > img1->width) add_x += ceil(bottom_right.x - img1->width);
	  if(bottom_right.y > img1->height) add_y += ceil(bottom_right.y - img1->height);
 
  
           
  	  panorama = cvCreateImage( cvSize( add_x + 10 + img1->width, add_y +10 + img1->height ) , IPL_DEPTH_8U , 1 ); //img2 -> img1
	  cvZero( panorama );
	  
	     
	  // This part was added to perform the Szeliski Algorithm based on: " Video Mosaics for Virtual Environments"
	  overlaping1 = cvCreateImage( cvSize( add_x + 10 + img1->width, add_y +10 + img1->height ) , IPL_DEPTH_8U , 1 ); 
	  cvZero( overlaping1 );
	  
	  overlaping2 = cvCreateImage( cvSize( add_x + 10 + img1->width, add_y +10 + img1->height ) , IPL_DEPTH_8U , 1 ); 
	  cvZero( overlaping2 );
	  
	  AreaCommon = cvCreateImage( cvSize( add_x + 10 + img1->width, add_y +10 + img1->height ) , IPL_DEPTH_8U , 1 ); 
	  cvZero( AreaCommon );
	  
	  // end of the algorithm of Szeliski
	  
	  newimg2 = cvCreateImage( cvSize(x_length,y_length) , IPL_DEPTH_8U , 1 );  //newimg1 -> newimg2
	  cvZero( newimg2 );
	
	   
       for( i=0; i<img2->width; i++ ){ //img1->width //img2->width
          for( j=0; j<img2->height; j++ ){//img1->height //img2->width
	        current = cvPoint2D64f( i, j ); // i->x and j->y
		target = 2( current, H );   //H 
		s=cvGet2D( img2, j, i ); //img1 -> img2 // Get the pixel value in (j,i) of img1
			// newimg1->newimg2
		if(target.y+y_offset<newimg2->height && target.y+y_offset>= 0 && target.x+x_offset < newimg2->width && target.x+x_offset >= 0)
		{
		 cvSet2D( newimg2, target.y+y_offset, target.x+x_offset, s ); 	
		}   
	}} // End of the for loops
	
	
        newimg2_interp = interp2ir(img2, H, x_offset, y_offset, x_length, y_length); 
	
      for( i=0; i<img1->width; i++ ){//img2 // If I want to change I just have to use img1 instead
         for( j=0; j<img1->height; j++ ){ //img2
           s=cvGet2D(img1, j, i ); //img2
           cvSet2D( panorama, j+y_offset, i+x_offset, s );
	  }
        }
 ////This part will try to see why the warping of the img1 is not perfect instead it does when I work with both images
 //// at the same time.

 for( i=0; i<newimg2_interp ->width; i++ ){ //img1 //newimg1_interp -> newimg2_interp
    for( j=0; j<newimg2_interp ->height; j++ ){//img1 //newimg1_interp -> newimg2_interp
		target = cvPoint2D64f( i, j );
		s=cvGet2D( newimg2_interp , j, i ); //img1 //newimg1_interp -> newimg2_interp
		t=cvGet2D( panorama, target.y, target.x ); 	
        u.val[0] = 255;
		u.val[1] = 255;
		u.val[2] = 255;
		
	//	u = cvGet2D(newimg2, target.y, target.x ); 
        if( t.val[0] < 1){ // Is the img1' negative or zero
		   cvSet2D( panorama, target.y, target.x, s );//Set the pixel value in target with the value s 
		  }
		else{
		  if( s.val[0] < 1){
			cvSet2D( panorama, target.y, target.x, t );
		     }//If
		  else{  // Common Area of overlaping
			//ave.val[0] = (t.val[0] + u.val[0])/2;
			//cvSet2D( panorama, target.y, target.x, ave );
			  cvSet2D( overlaping1, target.y, target.x, t );
			  cvSet2D( overlaping2, target.y, target.x, s );
			  cvSet2D( AreaCommon, target.y, target.x, u );
			  //printf("\n I am inside the overlaping area");
		  	  
			              
			  
		   } // Second Else
		 } // First Else
		} // Second For
	  } // First For  
	  

 
 cvReleaseImage( &newimg2 ); // newimg1 // change order newimg1
 cvReleaseImage( &newimg2_interp ); //newimg1_interp //change order newim2_interp
 cvReleaseImage( &overlaping1 ); //newimg1_interp //change order newim2_interp
 cvReleaseImage( &overlaping2 ); //newimg1_interp //change order newim2_interp
 cvReleaseImage( &AreaCommon ); //newimg1_interp //change order newim2_interp


  return panorama;
}

*/
extern struct image_border FindCornersImg(IplImage* img2, CvMat* H)
{

	 struct image_border border;
	 CvPoint2D64f top_left;          // top-left point after transformation
	 CvPoint2D64f top_right;         // top-right point after transformation
	 CvPoint2D64f bottom_left;       // bottom-left point after transformation
	 CvPoint2D64f bottom_right;       // bottom-right point after transformation

	 float px_min, px_max,py_min, py_max;
	 float p1_x, p2_x, p3_x, p4_x;
	 float p1_y, p2_y, p3_y, p4_y;
	 float Box_width;
	 float Box_height;

	 top_left = persp_xform_pt2( cvPoint2D64f( 0, 0 ), H );//H2 -> H
     top_right = persp_xform_pt2( cvPoint2D64f( img2->width-1, 0 ), H ); //H2 -> H //img1 ->img2
     bottom_left = persp_xform_pt2( cvPoint2D64f( 0,img2->height-1), H ); //H2 -> H  //img1 ->img2
     bottom_right = persp_xform_pt2( cvPoint2D64f( img2->width-1, img2->height-1 ), H ); //same
   

	 p1_x = top_left.x;
	 p1_y = top_left.y;
	 p2_x = top_right.x;
	 p2_y = top_right.y;
	 p3_x = bottom_right.x;
	 p3_y = bottom_right.y;
	 p4_x = bottom_left.x;
	 p4_y = bottom_left.y;

  px_min = MIN(MIN(MIN(p1_x,p2_x),p3_x),p4_x);
  px_max = MAX(MAX(MAX(p1_x,p2_x),p3_x),p4_x);

  py_min = MIN(MIN(MIN(p1_y,p2_y),p3_y),p4_y);
  py_max = MAX(MAX(MAX(p1_y,p2_y),p3_y),p4_y);


  Box_width = px_max - px_min + 1;
  Box_height = py_max - py_min + 1;

  border.P1.x = p1_x;
  border.P1.y = p1_y;
  border.P2.x = p2_x;
  border.P2.y = p2_y;
  border.P3.x = p3_x;
  border.P3.y = p3_y;
  border.P4.x = p4_x;
  border.P4.y = p4_y;

  border.x_max = px_max;
  border.y_max = py_max;

  border.x_min = px_min;
  border.y_min = py_min;

  border.ROISize.width = (int)Box_width;
  border.ROISize.height = (int)Box_height;

 return border;

}

extern void MultHomMat(CvMat* H1, CvMat* H2, CvMat* Hresult)
{
  int nRows, nCols;
  int i, j;

  CvMat *Htemp = cvCreateMat(3,3,CV_32FC1);

  cvMatMul(H1,H2,Htemp);


  nRows = Htemp->rows;
  nCols = Htemp->cols;

  for( i = 0; i < nRows; i++){
	  for( j = 0; j < nCols; j++){
		 // cvmSet(Hresult,i,j,(cvmGet(Htemp,i,j))/(cvmGet(Htemp,2,2)));
		   cvmSet(Hresult,i,j,cvmGet(Htemp,i,j));
	 }
  }
	cvReleaseMat( &Htemp );
	return;
}

extern void MultHomMat2(CvMat* H1, CvMat* H2, CvMat* Hresult)
{
  int nRows, nCols;
  int i, j;
	
  CvMat *H1_temp = cvCreateMat(3,3,CV_32FC1);
  CvMat *H2_temp = cvCreateMat(3,3,CV_32FC1);
 


  nRows = H1_temp->rows;
  nCols = H1_temp->cols;

  for( i = 0; i < nRows; i++){
	  for( j = 0; j < nCols; j++){
		cvmSet(H1_temp,i,j,cvmGet(H1,i,j));
		cvmSet(H2_temp,i,j,cvmGet(H2,i,j));
	 }
  }
   cvMatMul(H1_temp,H2_temp,Hresult);
   cvReleaseMat( &H1_temp );
   cvReleaseMat( &H2_temp );

	return;
}



extern CvMat* CalcTransfMatrixF( struct feature* feat1, struct feature* feat2,int n1, int n2, int img1_height)
{
	struct feature *feat;
	struct feature** nbrs;
	struct kd_node* kd_root;
	
	struct feature* feature_lowe1; // I use this to find out the number of inliers
    struct feature* feature_lowe2; // I use this to find out the number of inliers


    CvPoint pt1, pt2;
	double d0, d1;
	int  k, i, m = 0, dx, dy;
	int n=0;
	int levels, levels2;
	
   
   CvMat* H = cvCreateMat(3,3,CV_32FC1);
 	    

    kd_root = kdtree_build( feat1, n1 ); // feat2 and n2
	// I want to change the order of the images I have to use feat1, n1

	for( i = 0; i < n2; i++ ) //n1 // to chage order n2
	{
		feat = feat2 + i; //feat1 // to change order feat2
		k = kdtree_bbf_knn( kd_root, feat, 2, &nbrs, KDTREE_BBF_MAX_NN_CHKS );
		if( k == 2 )
		{
	//Returns the squared Euclidian distance between the descriptors of  feat and nbrs[0]
			d0 = descr_dist_sq( feat, nbrs[0] ); 
			d1 = descr_dist_sq( feat, nbrs[1] );
			if( d0 < d1 * NN_SQ_DIST_RATIO_THR ) // Selection of the "distintic match"
			{
				pt1 = cvPoint( cvRound( feat->x ), cvRound( feat->y ) );
				pt2 = cvPoint( cvRound( nbrs[0]->x ), cvRound( nbrs[0]->y ) );
				pt2.y += img1_height;
				//cvLine( stacked, pt1, pt2, CV_RGB(255,0,255), 1, 8, 0 );
				m++;
				feat2[i].fwd_match = nbrs[0]; //feat1[i].fwd_match // to change order feat2[i].fwd_match
				//printf("\n The value of this match is : %f",nbrs[0]);
				
			}
		}
		free( nbrs );
	}

  	   
	 H = ransac_xform( feat2, n2, FEATURE_FWD_MATCH, lsq_homog, 4, 0.01,homog_xfer_err, 3.0, NULL, NULL );
	
	 return H;
}

extern CvMat* CalcTransfMatrixF_inliers( struct feature* feat1, struct feature* feat2,int n1, int n2, int img1_height,CvMat* Hcv, CvMat* Hcv_Xi, CvMat* Hcv_Xin)//, CvMat* CovarianceH)
{
	struct feature *feat;
	struct feature** nbrs;
	struct kd_node* kd_root;

	struct feature **inliers;
	
	struct feature* feature_lowe1; // I use this to find out the number of inliers
    struct feature* feature_lowe2; // I use this to find out the number of inliers


    CvPoint pt1, pt2;
	double d0, d1;
	int  k, i, m = 0, dx, dy;
	int n=0;
	int levels, levels2;
	int nbr_inliers;
	struct feature* match;
	CvPoint2D64f pt, mpt;
	double sigma, temp;
	CvMat* Mpt;
	CvMat* Mmpt;
	CvMat* Mpt_Xi;
	CvMat* Mmpt_Xi;
	CvMat* Mpt_Xin;
	CvMat* Mmpt_Xin;
	//double *apt, *ampt;
	float* Mpt_data;
	float* Mmpt_data;
	float* Mpt_data_Xi;
	float* Mmpt_data_Xi;
	float* Mpt_data_Xin;
	float* Mmpt_data_Xin;
	//struct feature **matched;
	//int nm;
	CvPoint2D64f* _pts, * _mpts;
   
   CvMat* H = cvCreateMat(3,3,CV_32FC1);
   //CvMat* Hcv = cvCreateMat(3,3,CV_32FC1);
   //CvMat* Hcv_Xi = cvCreateMat(3,3,CV_32FC1);
   //CvMat* Hcv_Xin = cvCreateMat(3,3,CV_32FC1);
   CvMat* Hcv_Xi_col, *Hcv_Xin_col, *H_col; 

   int n_elements;
 	    

    kd_root = kdtree_build( feat1, n1 ); // feat2 and n2
	// I want to change the order of the images I have to use feat1, n1

	for( i = 0; i < n2; i++ ) //n1 // to chage order n2
	{
		feat = feat2 + i; //feat1 // to change order feat2
		k = kdtree_bbf_knn( kd_root, feat, 2, &nbrs, KDTREE_BBF_MAX_NN_CHKS );
		if( k == 2 )
		{
	//Returns the squared Euclidian distance between the descriptors of  feat and nbrs[0]
			d0 = descr_dist_sq( feat, nbrs[0] ); 
			d1 = descr_dist_sq( feat, nbrs[1] );
			if( d0 < d1 * NN_SQ_DIST_RATIO_THR ) // Selection of the "distintic match"
			{
				pt1 = cvPoint( cvRound( feat->x ), cvRound( feat->y ) );
				pt2 = cvPoint( cvRound( nbrs[0]->x ), cvRound( nbrs[0]->y ) );
				pt2.y += img1_height;
				//cvLine( stacked, pt1, pt2, CV_RGB(255,0,255), 1, 8, 0 );
				m++;
				feat2[i].fwd_match = nbrs[0]; //feat1[i].fwd_match // to change order feat2[i].fwd_match
				//printf("\n The value of this match is : %f",nbrs[0]);
				
			}
		}
		free( nbrs );
	}
  	   
	 H = ransac_xform( feat2, n2, FEATURE_FWD_MATCH, lsq_homog, 4, 0.01,homog_xfer_err, 3.0, &inliers, &nbr_inliers );
     //
	// H = ransac_xform( feat2, n2, FEATURE_BCK_MATCH, lsq_homog, 4, 0.01,homog_xfer_err, 3.0, &inliers, &nbr_inliers );
	 printf("number of inliers : ... %d .... \n", nbr_inliers);
	 // Computing Sigma
     
	 printf("computing sigma.... \n");

	 Mpt = cvCreateMat(nbr_inliers, 2, CV_32FC1);
	 Mmpt = cvCreateMat(nbr_inliers, 2, CV_32FC1);

	 Mpt_data = Mpt->data.fl;
	 Mmpt_data = Mmpt->data.fl;

	 _pts = calloc( n, sizeof( CvPoint2D64f ) );
	_mpts = calloc( n, sizeof( CvPoint2D64f ) );

	extract_corresp_pts(inliers, nbr_inliers,FEATURE_FWD_MATCH, &_pts , &_mpts);

     sigma = 0.0;
	
	 for( i = 0; i < nbr_inliers; i++){
		pt =  _pts[i];
		mpt = _mpts[i];
		temp = homog_xfer_err(pt, mpt, H);
		sigma += pow(temp,2.0);

		// Xo
		Mpt_data[i*2] = pt.x;
		Mpt_data[i*2 + 1] = pt.y;
		Mmpt_data[i*2] = mpt.x;
		Mmpt_data[i*2+1]= mpt.y;	 
		
	 }
	 sigma = sqrt(sigma/nbr_inliers);
	 printf("sigma : ... %f  \n", sigma);

    printf("finding the homographies.... \n");
	// This homography is for Xo

	printf(" \n Computing H for Xo case..\n");
   // cvFindHomography(Mpt, Mmpt, Hcv);
	 cvFindHomography(Mpt, Mmpt, Hcv, 0, 3, 0);
	//PrintElementMatrix(Hcv);
    // This is the homography for Xi and Xi+n

	

	 Mpt_Xi = cvCreateMat(nbr_inliers, 2, CV_32FC1);
	 Mmpt_Xi = cvCreateMat(nbr_inliers, 2, CV_32FC1);

	 Mpt_Xin = cvCreateMat(nbr_inliers, 2, CV_32FC1);
	 Mmpt_Xin = cvCreateMat(nbr_inliers, 2, CV_32FC1);

	 Mpt_data_Xi = Mpt_Xi->data.fl;
	 Mmpt_data_Xi = Mmpt_Xi->data.fl;

	 Mpt_data_Xin = Mpt_Xin->data.fl;
	 Mmpt_data_Xin = Mmpt_Xin->data.fl;
	
	for( i = 0; i < nbr_inliers; i++){
		pt =  _pts[i];
		mpt = _mpts[i];
		
		//Xi
		Mpt_data_Xi[i*2] = pt.x + sigma;
		Mpt_data_Xi[i*2 + 1] = pt.y + sigma;
		Mmpt_data_Xi[i*2] = mpt.x + sigma;
		Mmpt_data_Xi[i*2+1]= mpt.y + sigma;	

		//Xi+n
		Mpt_data_Xin[i*2] = pt.x - sigma;
		Mpt_data_Xin[i*2 + 1] = pt.y - sigma;
		Mmpt_data_Xin[i*2] = mpt.x - sigma;
		Mmpt_data_Xin[i*2+1]= mpt.y - sigma;			
	 }
	 
	// This is the homography for Xi
	printf(" \n Computing H for Xi case..\n");
//	cvFindHomography(Mpt_Xi, Mmpt_Xi, Hcv_Xi);
	cvFindHomography(Mpt_Xi, Mmpt_Xi, Hcv_Xi, 0, 3, 0);
	//PrintElementMatrix(Hcv_Xi);


	// This is the homography for Xi+n
	printf(" \n Computing H for Xi+n case..\n");
	//cvFindHomography(Mpt_Xin, Mmpt_Xin, Hcv_Xin);
	cvFindHomography(Mpt_Xin, Mmpt_Xin, Hcv_Xin,0,3,0);
	//PrintElementMatrix(Hcv_Xin);

	H_col = cvCreateMat( 9,1, CV_32FC1 );
	Hcv_Xi_col = cvCreateMat( 9,1, CV_32FC1 );
	Hcv_Xin_col = cvCreateMat( 9,1, CV_32FC1 );

	Convert2ColumnMatrix(H, H_col);
	Convert2ColumnMatrix(Hcv_Xi, Hcv_Xi_col);
	Convert2ColumnMatrix(Hcv_Xin, Hcv_Xin_col);

	n_elements = 2; // Number of variables to compute the Covariance

   // Computing the covariance of the matrix, this matrix is 8x8 elements
  //  ComputeCovariance(Hcv_Xi_col, Hcv_Xin_col, H_col,n_elements, CovarianceH);

//	PrintElementMatrix(CovarianceH);

   

    free( inliers );
	free(_pts);
	free(_mpts);

	//cvReleaseMat(&Hcv);	
	//cvReleaseMat(&Hcv_Xi);	
	//cvReleaseMat(&Hcv_Xin);	
	cvReleaseMat(&Mpt);
	cvReleaseMat(&H_col);	
	cvReleaseMat(&Hcv_Xi_col);	
	cvReleaseMat(&Hcv_Xin_col);	
	cvReleaseMat(&Mpt);
	cvReleaseMat(&Mmpt);
	cvReleaseMat(&Mpt_Xi);
	cvReleaseMat(&Mmpt_Xi);
	cvReleaseMat(&Mpt_Xin);
	cvReleaseMat(&Mmpt_Xin);

	 return H;
}

extern void ComputeCovariance(CvMat* H1_col, CvMat* H2_col, CvMat* H_col,int n_elements, CvMat* CovarianceH){
	int i, j;
	CvMat* temp = cvCreateMat(8,1, CV_32FC1); // To work with the first homography
	CvMat* temp_2 = cvCreateMat(8,1, CV_32FC1); // To work with the second homography
	CvMat* temp2 = cvCreateMat(8,8, CV_32FC1); // Partial result for the first homography
	CvMat* temp2_2 = cvCreateMat(8,8, CV_32FC1); // Partial result for the second homography
	float *H1_col_data = H1_col->data.fl;
	float *H2_col_data = H2_col->data.fl;
	float *H_col_data = H_col->data.fl;
	float *temp_data = temp->data.fl;
	float *temp_2_data = temp_2->data.fl;


	int n = temp2->cols;

	float h1_value;
	float h2_value;
	float diff;
	float diff2;
	float h_value;
    
	for ( i  = 0; i < (H1_col->rows - 1); i++){
		h1_value =  H1_col_data[i];// because is a column vector
		h_value =  H_col_data[i];// because is a column vector
		diff = h1_value - h_value;
		temp_data[i] = 0.5*diff; // Because there are only two different matrices

		h2_value =  H2_col_data[i];// because is a column vector
		diff2 = h2_value - h_value;
		temp_2_data[i] = 0.5*diff2; // Because there are only two different matrices
	}
	
   cvMulTransposed(temp, temp2,0, 0,1.0);
   cvMulTransposed(temp_2, temp2_2,0,0,1.0);


   
   cvAdd(temp2, temp2_2, CovarianceH, 0);

  cvReleaseMat( &temp );
  cvReleaseMat( &temp2 );
  cvReleaseMat( &temp2_2 );
  cvReleaseMat( &temp_2 );
  

  return;
}


extern IplImage* FrameDifference(IplImage *img1,IplImage *img2)

{
	int x1, x2;
	IplImage* ImageDifference;
	CvScalar z1, z2,zR;

	ImageDifference = cvCreateImage( cvSize(img1->width,img1->height ) , img1->depth , img1->nChannels ); //img2 -> img1
	cvZero( ImageDifference );


	// Compute frame difference
	
	for (x1=0; x1<img1->height; x1++) { //rows
		for (x2=0; x2<img1->width; x2++) {//cols
			 z1 =  cvGet2D(img1,x1,x2);
			 z2 =  cvGet2D(img2,x1,x2);
			 zR.val[0] = z1.val[0] - z2.val[0];
			 zR.val[1] = z1.val[1] - z2.val[1];
			 zR.val[2] = z1.val[2] - z2.val[2];
             cvSet2D(ImageDifference,x1,x2,zR);
          	}
	}


	return ImageDifference;
}


extern void SRMosaicking(IplImage* img1, IplImage* img2, IplImage* img3, IplImage* img4, IplImage* img5, IplImage* prev_panorama, 
					struct image_border ROI_frame1, struct image_border ROI_frame2, struct image_border ROI_frame3, struct image_border ROI_frame4, struct image_border ROI_frame5,
					IplImage* panoramaSR, double* lambda_itp)//, int D)
					{

//IplImage* panoramaSR = NULL;
IplImage* img1_inv = NULL, *img2_inv = NULL, *img3_inv=NULL, *img4_inv=NULL, *img5_inv=NULL;
IplImage* img1_inv_temp1 = NULL, *img2_inv_temp1 = NULL, *img3_inv_temp1=NULL, *img4_inv_temp1=NULL, *img5_inv_temp1=NULL;
IplImage* img1_inv_temp2 = NULL, *img2_inv_temp2 = NULL, *img3_inv_temp2=NULL, *img4_inv_temp2=NULL, *img5_inv_temp2=NULL;

IplImage* dimg1=NULL, *dimg2=NULL, *dimg3=NULL, *dimg4=NULL, *dimg5=NULL;
IplImage* Ddimg1=NULL, *Ddimg2=NULL, *Ddimg3=NULL, *Ddimg4=NULL, *Ddimg5=NULL;
IplImage* ddst1 = NULL, *ddst2 = NULL, *ddst3 = NULL, *ddst4 = NULL, *ddst5=NULL;
IplImage* panorama5d = NULL;
IplImage* panorama5d2 = NULL; //after multiplication with alpha
IplImage* panorama5LLt = NULL;
IplImage* panorama5LLt2 = NULL; //after multiplication
IplImage* panorama5_Delta = NULL;
IplImage* Xregularized = NULL; // To use with the algorithm Fast and Robust Regularization
IplImage* Limg = NULL;

CvMat* kernelLLt = cvCreateMat(3,3,CV_32FC1);
CvMat* kernelB = cvCreateMat(3,3,CV_32FC1);
CvMat* kernelL = cvCreateMat(3,3,CV_32FC1);

CvMat* lambdaM = NULL;

double lambda_1, lambda_2, lambda_3, lambda_4, lambda_5;
double lambda_num, lambda_den;
double lambdad_1, lambdad_2, lambdad_3, lambdad_4, lambdad_5;
double lambda_it;
double alpha_it = 1.1127;// was 7, 1, 1.1127
double errorSR;

float alpha = 0.0;

CvPoint2D32f P; // P for the mosaic

int i, j, D;

struct RegionS  S1, S2, S3, S4, S5;

CvSize sizeD1, sizeD2, sizeD3, sizeD4, sizeD5;
CvSize size; //size of the frames

//FILE *pFile;

cvZero(panoramaSR);


	/// First part:  Interpolation : resizing the image to 2 times the original size

  D = 2;

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

	size.height = img1->height;
	size.width = img1->width;

/**************************************************************************************/ 
   //   Starting a new iteration (it = 3)
   //   
	//  First Step: Getting the imgs from the mosaic image
   //
   /**************************************************************************************/
/*
	allocateOnDemand( &img1_inv, cvSize(img2->width,img2->height),  img1->depth, img1->nChannels );
    WarpImageUnderQuadInv(prev_panorama,img1_inv,ROI_frame1);

    allocateOnDemand( &img2_inv, cvSize(img2->width,img2->height),  img1->depth, img1->nChannels );
    WarpImageUnderQuadInv(prev_panorama,img2_inv,ROI_frame2);

	allocateOnDemand( &img3_inv, cvSize(img2->width,img2->height),  img1->depth, img1->nChannels );
	WarpImageUnderQuadInv(prev_panorama,img3_inv,ROI_frame3);

	allocateOnDemand( &img4_inv, cvSize(img2->width,img2->height),  img1->depth, img1->nChannels );
	WarpImageUnderQuadInv(prev_panorama,img4_inv,ROI_frame4);

	allocateOnDemand( &img5_inv, cvSize(img2->width,img2->height),  img1->depth, img1->nChannels );
	WarpImageUnderQuadInv(prev_panorama,img5_inv,ROI_frame5);
*/

	allocateOnDemand( &img1_inv_temp1, sizeD1,  img1->depth, img1->nChannels );
    WarpImageUnderQuadInv(prev_panorama,img1_inv_temp1,ROI_frame1);

    allocateOnDemand( &img2_inv_temp1, sizeD1,  img1->depth, img1->nChannels );
    WarpImageUnderQuadInv(prev_panorama,img2_inv_temp1,ROI_frame2);

	allocateOnDemand( &img3_inv_temp1, sizeD1,  img1->depth, img1->nChannels );
	WarpImageUnderQuadInv(prev_panorama,img3_inv_temp1,ROI_frame3);

	allocateOnDemand( &img4_inv_temp1, sizeD1,  img1->depth, img1->nChannels );
	WarpImageUnderQuadInv(prev_panorama,img4_inv_temp1,ROI_frame4);

	allocateOnDemand( &img5_inv_temp1, sizeD1,  img1->depth, img1->nChannels );
	WarpImageUnderQuadInv(prev_panorama,img5_inv_temp1,ROI_frame5);

	// Using a Gaussian Filter to perform an Antia-alias filtering
	
	allocateOnDemand( &img1_inv_temp2, sizeD1,  img1->depth, img1->nChannels );
    allocateOnDemand( &img2_inv_temp2, sizeD1,  img1->depth, img1->nChannels );
    allocateOnDemand( &img3_inv_temp2, sizeD1,  img1->depth, img1->nChannels );
	allocateOnDemand( &img4_inv_temp2, sizeD1,  img1->depth, img1->nChannels );
	allocateOnDemand( &img5_inv_temp2, sizeD1,  img1->depth, img1->nChannels );
/*
	cvSmooth(img1_inv_temp1,img1_inv_temp2, CV_GAUSSIAN, 3, 0, 0, 0);
	cvSmooth(img2_inv_temp1,img2_inv_temp2, CV_GAUSSIAN, 3, 0, 0, 0);
	cvSmooth(img3_inv_temp1,img3_inv_temp2, CV_GAUSSIAN, 3, 0, 0, 0);
	cvSmooth(img4_inv_temp1,img4_inv_temp2, CV_GAUSSIAN, 3, 0, 0, 0);
	cvSmooth(img5_inv_temp1,img5_inv_temp2, CV_GAUSSIAN, 3, 0, 0, 0);
*/
	// Using Filter 2D:
    //This kernel is the same as the used in the paper: " A Transform-Domain Approach to Super-resolution Mosaicing of Compressed Images"

	cvmSet(kernelB,0,0,0.1016);
	cvmSet(kernelB,0,1,0.1172);
	cvmSet(kernelB,0,2,0.1016);
	cvmSet(kernelB,1,0,0.1172);
	cvmSet(kernelB,1,1,0.1250);
	cvmSet(kernelB,1,2,0.1172);
	cvmSet(kernelB,2,0,0.1016);
	cvmSet(kernelB,2,1,0.1172);
	cvmSet(kernelB,2,2,0.1016);

	cvFilter2D(img1_inv_temp1,img1_inv_temp2,kernelB,cvPoint(-1,1));
	cvFilter2D(img2_inv_temp1,img2_inv_temp2,kernelB,cvPoint(-1,1));
	cvFilter2D(img3_inv_temp1,img3_inv_temp2,kernelB,cvPoint(-1,1));
	cvFilter2D(img4_inv_temp1,img4_inv_temp2,kernelB,cvPoint(-1,1));
	cvFilter2D(img5_inv_temp1,img5_inv_temp2,kernelB,cvPoint(-1,1));
	
  // Downsampling using cubic interpolation

	allocateOnDemand( &img1_inv, size,  img1->depth, img1->nChannels );
    allocateOnDemand( &img2_inv, size,  img1->depth, img1->nChannels );
    allocateOnDemand( &img3_inv, size,  img1->depth, img1->nChannels );
	allocateOnDemand( &img4_inv, size,  img1->depth, img1->nChannels );
	allocateOnDemand( &img5_inv, size,  img1->depth, img1->nChannels );
	
	cvResize(img1_inv_temp2, img1_inv, CV_INTER_CUBIC);
	cvResize(img2_inv_temp2, img2_inv, CV_INTER_CUBIC);
	cvResize(img3_inv_temp2, img3_inv, CV_INTER_CUBIC);
	cvResize(img4_inv_temp2, img4_inv, CV_INTER_CUBIC);
	cvResize(img5_inv_temp2, img5_inv, CV_INTER_CUBIC);

	/*
// Downsampling using my own code

	DownSampleImage(img1_inv_temp2, img1_inv, 2);
	DownSampleImage(img2_inv_temp2, img2_inv, 2);
	DownSampleImage(img3_inv_temp2, img3_inv, 2);
	DownSampleImage(img4_inv_temp2, img4_inv, 2);
	DownSampleImage(img5_inv_temp2, img5_inv, 2);
*/
/*
	cvSaveImage("DBWRx1.jpg", img1_inv);
	cvSaveImage("DBWRx2.jpg", img2_inv);
	cvSaveImage("DBWRx3.jpg", img3_inv);
	cvSaveImage("DBWRx4.jpg", img4_inv);
	cvSaveImage("DBWRx5.jpg", img5_inv);


	cvNamedWindow( "img1_inv", 1 );
	cvShowImage( "img1_inv", img1_inv);
	cvWaitKey(0);

	cvNamedWindow( "img2_inv", 1 );
	cvShowImage( "img2_inv", img2_inv);
	cvWaitKey(0);

	cvNamedWindow( "img3_inv", 1 );
	cvShowImage( "img3_inv", img3_inv );
	cvWaitKey(0);

	cvNamedWindow( "img4_inv", 1 );
	cvShowImage( "img4_inv", img4_inv );
	cvWaitKey(0);

	cvNamedWindow( "img5_inv", 1 );
	cvShowImage( "img5_inv", img5_inv );
	cvWaitKey(0);
*/	

   /**************************************************************************************/ 
   //    
   //   
   //  Second Step: Compute the frame difference between the original low frames and the LR reconstructed ones
   //
   /**************************************************************************************/

  /************************************************************************/
  //  Doing the frame substracion
  /****************************************************************************/
 //  printf("\n Doing the frame substracion ...\n" );


  allocateOnDemand( &dimg1, cvSize(img2->width,img2->height),  img1->depth, img1->nChannels );
  allocateOnDemand( &dimg2, cvSize(img2->width,img2->height),  img1->depth, img1->nChannels );
  allocateOnDemand( &dimg3, cvSize(img2->width,img2->height),  img1->depth, img1->nChannels );
  allocateOnDemand( &dimg4, cvSize(img2->width,img2->height),  img1->depth, img1->nChannels );
  allocateOnDemand( &dimg5, cvSize(img2->width,img2->height),  img1->depth, img1->nChannels );
/*
   printf("\n size_img1 = (%d , %d) \t size_img1_inv = (%d, %d)", img1->height, img1->width, img1_inv->height, img1_inv->width);
   printf("\n size_dimg1 = (%d , %d) ", dimg1->height, dimg1->width);

   getchar();
*/

  cvSub(img1,img1_inv,dimg1,NULL);
  cvSub(img2,img2_inv,dimg2,NULL);
  cvSub(img3,img3_inv,dimg3,NULL);
  cvSub(img4,img4_inv,dimg4,NULL);
  cvSub(img5,img5_inv,dimg5,NULL);

 
  //*********************************************************************************************//
  //  Doing the interpolation of the images differences											//
  //*********************************************************************************************//
    //printf("\n Doing the frame interpolation of the difference of frames ...\n" );
    allocateOnDemand( &Ddimg1, sizeD1,  img1->depth, img1->nChannels );  
	allocateOnDemand( &Ddimg2, sizeD2,  img1->depth, img1->nChannels );  
	allocateOnDemand( &Ddimg3, sizeD3,  img1->depth, img1->nChannels );  
	allocateOnDemand( &Ddimg4, sizeD4,  img1->depth, img1->nChannels );  
	allocateOnDemand( &Ddimg5, sizeD5,  img1->depth, img1->nChannels );  

	/*
  	cvResize(dimg1, Ddimg1,CV_INTER_CUBIC);
	cvResize(dimg2, Ddimg2,CV_INTER_CUBIC);
	cvResize(dimg3, Ddimg3,CV_INTER_CUBIC);
	cvResize(dimg4, Ddimg4,CV_INTER_CUBIC);
	cvResize(dimg5, Ddimg5,CV_INTER_CUBIC);
	*/

	UpSampleImageD(dimg1, Ddimg1);
	UpSampleImageD(dimg2, Ddimg2);
	UpSampleImageD(dimg3, Ddimg3);
	UpSampleImageD(dimg4, Ddimg4);
	UpSampleImageD(dimg5, Ddimg5);

/***********************************************************************************/
//			 Doing the Mosaicking of the Error Images								//
//																					//
/***********************************************************************************/
	//printf("\n Doing the Mosaicking of the Error Images ...\n" );

	allocateOnDemand( &ddst1,  cvSize(panoramaSR->width,panoramaSR->height),  img1->depth, img1->nChannels );  
	allocateOnDemand( &ddst2,  cvSize(panoramaSR->width,panoramaSR->height),  img1->depth, img1->nChannels );  
	allocateOnDemand( &ddst3,  cvSize(panoramaSR->width,panoramaSR->height),  img1->depth, img1->nChannels );  
	allocateOnDemand( &ddst4,  cvSize(panoramaSR->width,panoramaSR->height),  img1->depth, img1->nChannels );  
	allocateOnDemand( &ddst5,  cvSize(panoramaSR->width,panoramaSR->height),  img1->depth, img1->nChannels );  
	
	/*
	allocateOnDemand( &ddst1,  cvSize(1360, 640),  img1->depth, img1->nChannels );  
	allocateOnDemand( &ddst2,  cvSize(1360, 640),  img1->depth, img1->nChannels );  
	allocateOnDemand( &ddst3,  cvSize(1360, 640),  img1->depth, img1->nChannels );  
	allocateOnDemand( &ddst4,  cvSize(1360, 640),  img1->depth, img1->nChannels );  
	allocateOnDemand( &ddst5,  cvSize(1360, 640),  img1->depth, img1->nChannels );  
*/

    WarpImageUnderQuad(Ddimg1,ddst1,ROI_frame1);
	//cvNamedWindow( "Perspective_Warpdd1", 1 );
	//cvShowImage( "Perspective_Warpdd1", ddst1);
	//cvWaitKey(10);


  WarpImageUnderQuad(Ddimg2,ddst2,ROI_frame2);

 // cvNamedWindow( "Perspective_Warpdd2", 1 );
 // cvShowImage( "Perspective_Warpdd2", ddst2 );
  //cvWaitKey(10);


 WarpImageUnderQuad(Ddimg3,ddst3,ROI_frame3);
 //cvNamedWindow( "Perspective_Warpdd3", 1 );
 //cvShowImage( "Perspective_Warpdd3", ddst3 );
 //cvWaitKey(10);


 WarpImageUnderQuad(Ddimg4,ddst4,ROI_frame4);
 //cvNamedWindow( "Perspective_Warpdd4", 1 );
 //cvShowImage( "Perspective_Warpdd4", ddst4 );
 //cvWaitKey(10);

 
 WarpImageUnderQuad(Ddimg5,ddst5,ROI_frame5);
 //cvNamedWindow( "Perspective_Warpdd5", 1 );
 ///cvShowImage( "Perspective_Warpdd5", ddst5 );
 //cvWaitKey(10);

 allocateOnDemand( &panorama5d,  cvSize(prev_panorama->width, prev_panorama->height),  img1->depth, img1->nChannels );  

 S2 = FindSRegion(ROI_frame2);

	  for(i = 0; i < panorama5d->height; i ++){ //rows
		  for(j = 0; j < panorama5d->width; j++){ //cols
			  P.x = j;
			  P.y = i;
			  if(Belongs2S(P,S2))cvSet2D(panorama5d,i,j,cvGet2D(ddst2,i,j));
			  else cvSet2D(panorama5d,i,j,cvGet2D(ddst1,i,j));			  
		  }//end for
	  }// end for

	

////////////////////////////
// Finding the region S3 //
////////////////////////////

   S3 = FindSRegion(ROI_frame3);

	  for(i = 0; i < panorama5d->height; i ++){ //rows
		  for(j = 0; j < panorama5d->width; j++){ //cols
			  P.x = j;
			  P.y = i;
			  if(Belongs2S(P,S3))cvSet2D(panorama5d,i,j,cvGet2D(ddst3,i,j));
			  else cvSet2D(panorama5d,i,j,cvGet2D(panorama5d,i,j));			  
		  }//end for
	  }// end for



 ////////////////////////////
// Finding the region S4 //
////////////////////////////

   S4 = FindSRegion(ROI_frame4);

	  for(i = 0; i < panorama5d->height; i ++){ //rows
		  for(j = 0; j < panorama5d->width; j++){ //cols
			  P.x = j;
			  P.y = i;
			  if(Belongs2S(P,S4))cvSet2D(panorama5d,i,j,cvGet2D(ddst4,i,j));
			  else cvSet2D(panorama5d,i,j,cvGet2D(panorama5d,i,j));			  
		  }//end for
	  }// end for


	    ////////////////////////////
		// Finding the region S4 //
		////////////////////////////

   S5 = FindSRegion(ROI_frame5);


	  for(i = 0; i < panorama5d->height; i ++){ //rows
		  for(j = 0; j < panorama5d->width; j++){ //cols
			  P.x = j;
			  P.y = i;
			  if(Belongs2S(P,S5))cvSet2D(panorama5d,i,j,cvGet2D(ddst5,i,j));
			  else cvSet2D(panorama5d,i,j,cvGet2D(panorama5d,i,j));			  
		  }//end for
	  }// end for

	// cvNamedWindow( "panorama5d", 1 );
	//cvShowImage( "panorama5d", panorama5d );
	 //cvWaitKey(0);



   /**************************************************************************************/
   //
   // Computing the normalizing parameter lambda^(n) 
   //
   /**************************************************************************************/
    
	  lambda_1 = cvNorm(img1, img1_inv, CV_L2, NULL);
	 // printf("\n lambda_1 = %f", lambda_1);
	  lambda_2 = cvNorm(img2, img2_inv, CV_L2, NULL);
	 // printf("\n lambda_2 = %f", lambda_2);
	  lambda_3 = cvNorm(img3, img3_inv, CV_L2, NULL);
	 // printf("\n lambda_3 = %f", lambda_3);
	  lambda_4 = cvNorm(img4, img4_inv, CV_L2, NULL);
	 // printf("\n lambda_4 = %f", lambda_4);
	  lambda_5 = cvNorm(img5, img5_inv, CV_L2, NULL);
	 // printf("\n lambda_5 = %f", lambda_5);

   /**************************************************************************************/
   //
   // Computing the Laplace of the x^(n) image x(n) comes to be the panorama5 variables (mosaic of the previous state)
   //
   /**************************************************************************************/
   allocateOnDemand( &Limg, cvSize(prev_panorama->width,prev_panorama->height),  img1->depth, img1->nChannels );  
   //cvLaplace(prev_panorama, Limg,3);   
/*
// Uncomment this to use Laplace Prior Function

   // Using in stead cvFilter2D

   	cvmSet(kernelL,0,0,0.0000);
	cvmSet(kernelL,0,1,0.2500);
	cvmSet(kernelL,0,2,0.0000);
	cvmSet(kernelL,1,0,0.2500);
	cvmSet(kernelL,1,1,-1.000);
	cvmSet(kernelL,1,2,0.2500);
	cvmSet(kernelL,2,0,0.0000);
	cvmSet(kernelL,2,1,0.2500);
	cvmSet(kernelL,2,2,0.0000);

   cvFilter2D(prev_panorama, Limg, kernelL, cvPoint(-1,-1));
*/
 // Uncomment this to use Huber Prior Function
  // Using Huber Prior Function to find lambda

     alpha = 1.75;//was 1.75

     HubbertFunctionCliques(prev_panorama, Limg, alpha);

  // cvNamedWindow( "Limg", 1 );
  // cvShowImage( "Limg", Limg );
  // cvWaitKey(0);


   /**************************************************************************************/
   //
   // Computing the K norm(x^(n),2) , K = 1, 2 ...5
   //
   /**************************************************************************************/

  lambda_num = pow((lambda_1 + lambda_2 + lambda_3 + lambda_4 + lambda_5),2);
  lambdad_1 = cvNorm(Limg, NULL, CV_L2, NULL);
 

    /**************************************************************************************/
   //
   // Computing the lambda(n), K = 5, because I am just working with 5 frames
   //
   /**************************************************************************************/
 
  lambda_den = pow((5*lambdad_1),2);

    
  lambda_it = lambda_num/lambda_den;

  
  // printf("\n lambda_num = %f", lambda_num);
  // printf("\n lambda_den = %f", lambda_den);
  // printf("\n lambda_it = %f", lambda_it);

   *lambda_itp =  lambda_it;

   printf("\n lambda_it = %f \n", *lambda_itp);

  // pFile = fopen(name, "w");

   //fprintf(pFile, "%f\n", lambda_it);

  // fclose(pFile);
   
  // lambda_it = 1.1127;

  // printf("\n Using alpha from GCV = %f", alpha_it);

    /**************************************************************************************/
   //
   // Computing the lambda(n)*L'*L*x(n)
   //
   /**************************************************************************************/

   /*
    cvmSet(kernelLLt,0,0,1.000*lambda_it);
	cvmSet(kernelLLt,0,1,-4.000*lambda_it);
	cvmSet(kernelLLt,0,2,1.000*lambda_it);
	cvmSet(kernelLLt,1,0,-4.000*lambda_it);
	cvmSet(kernelLLt,1,1,18.000*lambda_it);
	cvmSet(kernelLLt,1,2,-4.000*lambda_it);
	cvmSet(kernelLLt,2,0,1.000*lambda_it);
	cvmSet(kernelLLt,2,1,-4.000*lambda_it);
	cvmSet(kernelLLt,2,2,1.000*lambda_it);
*/
	cvmSet(kernelLLt,0,0,1.000);
	cvmSet(kernelLLt,0,1,-4.000);
	cvmSet(kernelLLt,0,2,1.000);
	cvmSet(kernelLLt,1,0,-4.000);
	cvmSet(kernelLLt,1,1,18.000);
	cvmSet(kernelLLt,1,2,-4.000);
	cvmSet(kernelLLt,2,0,1.000);
	cvmSet(kernelLLt,2,1,-4.000);
	cvmSet(kernelLLt,2,2,1.000);

	allocateOnDemand( &panorama5LLt, cvSize(prev_panorama->width,prev_panorama->height),  img1->depth, img1->nChannels );  
	allocateOnDemand( &panorama5LLt2, cvSize(prev_panorama->width,prev_panorama->height),  img1->depth, img1->nChannels );  
	allocateOnDemand( &panorama5d2, cvSize(prev_panorama->width,prev_panorama->height),  img1->depth, img1->nChannels ); 

 //	printf("\n Limg->height = %d \t Limg->width = %d", Limg->height, Limg->width);
 //	printf("\n prev_panorama->height = %d \t prev_panorama->width = %d", prev_panorama->height, prev_panorama->width);

  //  Uncomment to use Huber Regularization

    // Using Huber Regularization here as well
	HubbertRegularization(Limg, panorama5LLt, alpha);
/*
  // Uncomment to use Laplace Regularization
    // Using Laplace prior
	cvFilter2D(Limg,panorama5LLt,kernelL,cvPoint(-1,1));
	cvFilter2D(panorama5LLt,panorama5LLt2, kernelL,cvPoint(-1,1)); // Because os L^t*L*x
*/	

/*
    //Using the regularization proposed in the paper: "Fast and Robust Super-Resolution"

	allocateOnDemand( &Xregularized, cvSize(prev_panorama->width,prev_panorama->height), IPL_DEPTH_32F, 1 );

	FastRobustRegularization(Limg, Xregularized, 2, alpha); // p = 2

	panorama5LLt = convert_gray32_to_color(Xregularized);

*/

	/*
	//
	// Converting to Matrix lambda doesn't work, because of the size of the mosaicking
	//

	lambdaM = cvCreateMat(panorama5LLt->height,panorama5LLt->width,CV_32FC1);
	cvSetIdentity(lambdaM,cvRealScalar(lambda_it));

	cvMatMul(panorama5LLt,lambdaM, panorama5LLt2);
	*/

	panorama5LLt2 = MulImagewithDouble(panorama5LLt, lambda_it);// Use this for Huber Regularization
	
	//panorama5LLt2 = MulImagewithDouble(panorama5LLt2, lambda_it); // Use this for Laplace Regularization
	
	panorama5d2 = MulImagewithDouble(panorama5d, alpha_it);

   /**************************************************************************************/ 
   //
   //	Now updating the iteration of x(n+1). First I have to do: panorama5d - panorama5LLt
   //   Then I have to do : panorma5_2 = panorama5_1 + (panorama5d - panorama5LLt)
   //
   /**************************************************************************************/
	
	//allocateOnDemand( &panoramaSR, cvSize(prev_panorama->width,prev_panorama->height),  img1->depth, img1->nChannels );  
	allocateOnDemand( &panorama5_Delta, cvSize(prev_panorama->width,prev_panorama->height),  img1->depth, img1->nChannels );  

	cvSub(panorama5d2,panorama5LLt2,panorama5_Delta,NULL);
	cvAdd(panorama5_Delta,prev_panorama,panoramaSR,NULL);


	/***********************************************************************************
	Computing the error of the SR
	*****************************************************************************************/

	errorSR = cvNorm(panoramaSR, prev_panorama, CV_RELATIVE_L2, NULL);
	//errorSR = cvNorm(panoramaSR, prev_panorama, CV_L2, NULL);

    printf(" \n errorSR = %f\n", errorSR);

	//cvNamedWindow( "panoramaSR", 1 );
	//cvShowImage( "panoramaSR", panoramaSR );
	//cvWaitKey(10);

	cvReleaseMat( &kernelLLt );
	cvReleaseMat( &kernelL );
	cvReleaseMat( &kernelB );
	cvReleaseMat( &lambdaM );

	return;

}

//
// This function use M as identit.
// M is a contraint operator
//
extern void SRMosaicking2(IplImage* img1, IplImage* img2, IplImage* img3, IplImage* img4, IplImage* img5, IplImage* prev_panorama, 
					struct image_border ROI_frame1, struct image_border ROI_frame2, struct image_border ROI_frame3, struct image_border ROI_frame4, struct image_border ROI_frame5,
					IplImage* panoramaSR)
					{

//IplImage* panoramaSR = NULL;
IplImage* img1_inv = NULL, *img2_inv = NULL, *img3_inv=NULL, *img4_inv=NULL, *img5_inv=NULL;
IplImage* dimg1=NULL, *dimg2=NULL, *dimg3=NULL, *dimg4=NULL, *dimg5=NULL;
IplImage* Ddimg1=NULL, *Ddimg2=NULL, *Ddimg3=NULL, *Ddimg4=NULL, *Ddimg5=NULL;
IplImage* ddst1 = NULL, *ddst2 = NULL, *ddst3 = NULL, *ddst4 = NULL, *ddst5=NULL;
IplImage* panorama5d = NULL;
IplImage* panorama5d2 = NULL; //after multiplication with alpha
//IplImage* panorama5LLt = NULL;
IplImage* panorama5LLt2 = NULL; //after multiplication
IplImage* panorama5_Delta = NULL;
IplImage* Limg = NULL;

CvMat* kernelLLt = cvCreateMat(3,3,CV_32FC1);

double lambda_1, lambda_2, lambda_3, lambda_4, lambda_5;
double lambda_num, lambda_den;
double lambdad_1, lambdad_2, lambdad_3, lambdad_4, lambdad_5;
double lambda_it;
double alpha_it = 7.00;

CvPoint2D32f P; // P for the mosaic

int i, j;

struct RegionS  S1, S2, S3, S4, S5;

CvSize sizeD1, sizeD2, sizeD3, sizeD4, sizeD5;


cvZero(panoramaSR);


	/// First part:  Interpolation : resizing the image to 2 times the original size

	sizeD1.height = 2*(img1->height);
	sizeD1.width = 2*(img1->width);

	sizeD2.height = 2*(img2->height);
	sizeD2.width = 2*(img2->width);

	sizeD3.height = 2*(img2->height);
	sizeD3.width = 2*(img2->width);

	sizeD4.height = 2*(img4->height);
	sizeD4.width = 2*(img4->width);

	sizeD5.height = 2*(img5->height);
	sizeD5.width = 2*(img5->width);

/**************************************************************************************/ 
   //   Starting a new iteration (it = 3)
   //   
	//  First Step: Getting the imgs from the mosaic image
   //
   /**************************************************************************************/

	allocateOnDemand( &img1_inv, cvSize(img2->width,img2->height),  img1->depth, img1->nChannels );
    WarpImageUnderQuadInv(prev_panorama,img1_inv,ROI_frame1);

    allocateOnDemand( &img2_inv, cvSize(img2->width,img2->height),  img1->depth, img1->nChannels );
    WarpImageUnderQuadInv(prev_panorama,img2_inv,ROI_frame2);

	allocateOnDemand( &img3_inv, cvSize(img2->width,img2->height),  img1->depth, img1->nChannels );
	WarpImageUnderQuadInv(prev_panorama,img3_inv,ROI_frame3);

	allocateOnDemand( &img4_inv, cvSize(img2->width,img2->height),  img1->depth, img1->nChannels );
	WarpImageUnderQuadInv(prev_panorama,img4_inv,ROI_frame4);

	allocateOnDemand( &img5_inv, cvSize(img2->width,img2->height),  img1->depth, img1->nChannels );
	WarpImageUnderQuadInv(prev_panorama,img5_inv,ROI_frame5);
	/*

	cvNamedWindow( "img1_inv", 1 );
	cvShowImage( "img1_inv", img1_inv);
	cvWaitKey(10);

	cvNamedWindow( "img2_inv", 1 );
	cvShowImage( "img2_inv", img2_inv);
	cvWaitKey(10);

	cvNamedWindow( "img3_inv", 1 );
	cvShowImage( "img3_inv", img3_inv );
	cvWaitKey(10);

	cvNamedWindow( "img4_inv", 1 );
	cvShowImage( "img4_inv", img4_inv );
	cvWaitKey(10);

	cvNamedWindow( "img5_inv", 1 );
	cvShowImage( "img5_inv", img5_inv );
	cvWaitKey(10);
	*/

   /**************************************************************************************/ 
   //    
   //   
   //  Second Step: Compute the frame difference between the original low frames and the LR reconstructed ones
   //
   /**************************************************************************************/

  /************************************************************************/
  //  Doing the frame substracion
  /****************************************************************************/

  allocateOnDemand( &dimg1, cvSize(img2->width,img2->height),  img1->depth, img1->nChannels );
  allocateOnDemand( &dimg2, cvSize(img2->width,img2->height),  img1->depth, img1->nChannels );
  allocateOnDemand( &dimg3, cvSize(img2->width,img2->height),  img1->depth, img1->nChannels );
  allocateOnDemand( &dimg4, cvSize(img2->width,img2->height),  img1->depth, img1->nChannels );
  allocateOnDemand( &dimg5, cvSize(img2->width,img2->height),  img1->depth, img1->nChannels );


  cvSub(img1,img1_inv,dimg1,NULL);
  cvSub(img2,img2_inv,dimg2,NULL);
  cvSub(img3,img3_inv,dimg3,NULL);
  cvSub(img4,img4_inv,dimg4,NULL);
  cvSub(img5,img5_inv,dimg5,NULL);

 
  //*********************************************************************************************//
  //  Doing the interpolation of the images differences											//
  //*********************************************************************************************//

    allocateOnDemand( &Ddimg1, sizeD1,  img1->depth, img1->nChannels );  
	allocateOnDemand( &Ddimg2, sizeD2,  img1->depth, img1->nChannels );  
	allocateOnDemand( &Ddimg3, sizeD3,  img1->depth, img1->nChannels );  
	allocateOnDemand( &Ddimg4, sizeD4,  img1->depth, img1->nChannels );  
	allocateOnDemand( &Ddimg5, sizeD5,  img1->depth, img1->nChannels );  

	
  	cvResize(dimg1, Ddimg1,CV_INTER_CUBIC);
	cvResize(dimg2, Ddimg2,CV_INTER_CUBIC);
	cvResize(dimg3, Ddimg3,CV_INTER_CUBIC);
	cvResize(dimg4, Ddimg4,CV_INTER_CUBIC);
	cvResize(dimg5, Ddimg5,CV_INTER_CUBIC);

/***********************************************************************************/
//			 Doing the Mosaicking of the Error Images								//
//																					//
/***********************************************************************************/
	allocateOnDemand( &ddst1,  cvSize(1360, 640),  img1->depth, img1->nChannels );  
	allocateOnDemand( &ddst2,  cvSize(1360, 640),  img1->depth, img1->nChannels );  
	allocateOnDemand( &ddst3,  cvSize(1360, 640),  img1->depth, img1->nChannels );  
	allocateOnDemand( &ddst4,  cvSize(1360, 640),  img1->depth, img1->nChannels );  
	allocateOnDemand( &ddst5,  cvSize(1360, 640),  img1->depth, img1->nChannels );  


    WarpImageUnderQuad(Ddimg1,ddst1,ROI_frame1);
	//cvNamedWindow( "Perspective_Warpdd1", 1 );
	//cvShowImage( "Perspective_Warpdd1", ddst1);
	//cvWaitKey(10);


  WarpImageUnderQuad(Ddimg2,ddst2,ROI_frame2);

 // cvNamedWindow( "Perspective_Warpdd2", 1 );
 // cvShowImage( "Perspective_Warpdd2", ddst2 );
  //cvWaitKey(10);


 WarpImageUnderQuad(Ddimg3,ddst3,ROI_frame3);
 //cvNamedWindow( "Perspective_Warpdd3", 1 );
 //cvShowImage( "Perspective_Warpdd3", ddst3 );
 //cvWaitKey(10);


 WarpImageUnderQuad(Ddimg4,ddst4,ROI_frame4);
 //cvNamedWindow( "Perspective_Warpdd4", 1 );
 //cvShowImage( "Perspective_Warpdd4", ddst4 );
 //cvWaitKey(10);

 
 WarpImageUnderQuad(Ddimg5,ddst5,ROI_frame5);
 //cvNamedWindow( "Perspective_Warpdd5", 1 );
 ///cvShowImage( "Perspective_Warpdd5", ddst5 );
 //cvWaitKey(10);

 allocateOnDemand( &panorama5d,  cvSize(prev_panorama->width, prev_panorama->height),  img1->depth, img1->nChannels );  

 S2 = FindSRegion(ROI_frame2);

	  for(i = 0; i < panorama5d->height; i ++){ //rows
		  for(j = 0; j < panorama5d->width; j++){ //cols
			  P.x = j;
			  P.y = i;
			  if(Belongs2S(P,S2))cvSet2D(panorama5d,i,j,cvGet2D(ddst2,i,j));
			  else cvSet2D(panorama5d,i,j,cvGet2D(ddst1,i,j));			  
		  }//end for
	  }// end for

	

////////////////////////////
// Finding the region S3 //
////////////////////////////

   S3 = FindSRegion(ROI_frame3);

	  for(i = 0; i < panorama5d->height; i ++){ //rows
		  for(j = 0; j < panorama5d->width; j++){ //cols
			  P.x = j;
			  P.y = i;
			  if(Belongs2S(P,S3))cvSet2D(panorama5d,i,j,cvGet2D(ddst3,i,j));
			  else cvSet2D(panorama5d,i,j,cvGet2D(panorama5d,i,j));			  
		  }//end for
	  }// end for



 ////////////////////////////
// Finding the region S4 //
////////////////////////////

   S4 = FindSRegion(ROI_frame4);

	  for(i = 0; i < panorama5d->height; i ++){ //rows
		  for(j = 0; j < panorama5d->width; j++){ //cols
			  P.x = j;
			  P.y = i;
			  if(Belongs2S(P,S4))cvSet2D(panorama5d,i,j,cvGet2D(ddst4,i,j));
			  else cvSet2D(panorama5d,i,j,cvGet2D(panorama5d,i,j));			  
		  }//end for
	  }// end for


	    ////////////////////////////
		// Finding the region S4 //
		////////////////////////////

   S5 = FindSRegion(ROI_frame5);


	  for(i = 0; i < panorama5d->height; i ++){ //rows
		  for(j = 0; j < panorama5d->width; j++){ //cols
			  P.x = j;
			  P.y = i;
			  if(Belongs2S(P,S5))cvSet2D(panorama5d,i,j,cvGet2D(ddst5,i,j));
			  else cvSet2D(panorama5d,i,j,cvGet2D(panorama5d,i,j));			  
		  }//end for
	  }// end for

	// cvNamedWindow( "panorama5d", 1 );
	 //cvShowImage( "panorama5d", panorama5d );
	 //cvWaitKey(10);



   /**************************************************************************************/
   //
   // Computing the normalizing parameter lambda^(n) 
   //
   /**************************************************************************************/
    
	  lambda_1 = cvNorm(img1, img1_inv, CV_L2, NULL);
	  printf("\n lambda_1 = %f", lambda_1);
	  lambda_2 = cvNorm(img2, img2_inv, CV_L2, NULL);
	  printf("\n lambda_2 = %f", lambda_2);
	  lambda_3 = cvNorm(img3, img3_inv, CV_L2, NULL);
	  printf("\n lambda_3 = %f", lambda_3);
	  lambda_4 = cvNorm(img4, img4_inv, CV_L2, NULL);
	  printf("\n lambda_4 = %f", lambda_4);
	  lambda_5 = cvNorm(img5, img5_inv, CV_L2, NULL);
	  printf("\n lambda_5 = %f", lambda_5);

   /**************************************************************************************/
   //
   // Computing the Laplace of the x^(n) image x(n) comes to be the panorama5 variables (mosaic of the previous state)
   //
   /**************************************************************************************/
   allocateOnDemand( &Limg, cvSize(prev_panorama->width,prev_panorama->height),  img1->depth, img1->nChannels );   
  // cvLaplace(prev_panorama, Limg,3);   
   /**************************************************************************************/
   //
   // Computing the K norm(x^(n),2) , K = 1, 2 ...5
   //
   /**************************************************************************************/

  lambda_num = pow((lambda_1 + lambda_2 + lambda_3 + lambda_4 + lambda_5),2);
  lambdad_1 = cvNorm(Limg, NULL, CV_L2, NULL);

  

    /**************************************************************************************/
   //
   // Computing the lambda(n), K = 5, because I am just working with 5 frames
   //
   /**************************************************************************************/
 
  lambda_den = pow((5*lambdad_1),2);

  lambda_it =  lambda_num/lambda_den;

   printf("\n lambda_num = %f", lambda_num);
   printf("\n lambda_den = %f", lambda_den);
   printf("\n lambda_it = %f", lambda_it);

    /**************************************************************************************/
   //
   // Computing the lambda(n)*L'*L*x(n)
   //
   /**************************************************************************************/

   /*
    cvmSet(kernelLLt,0,0,1.000*lambda_it);
	cvmSet(kernelLLt,0,1,-4.000*lambda_it);
	cvmSet(kernelLLt,0,2,1.000*lambda_it);
	cvmSet(kernelLLt,1,0,-4.000*lambda_it);
	cvmSet(kernelLLt,1,1,18.000*lambda_it);
	cvmSet(kernelLLt,1,2,-4.000*lambda_it);
	cvmSet(kernelLLt,2,0,1.000*lambda_it);
	cvmSet(kernelLLt,2,1,-4.000*lambda_it);
	cvmSet(kernelLLt,2,2,1.000*lambda_it);
*/
	cvmSet(kernelLLt,0,0,1.000);
	cvmSet(kernelLLt,0,1,-4.000);
	cvmSet(kernelLLt,0,2,1.000);
	cvmSet(kernelLLt,1,0,-4.000);
	cvmSet(kernelLLt,1,1,18.000);
	cvmSet(kernelLLt,1,2,-4.000);
	cvmSet(kernelLLt,2,0,1.000);
	cvmSet(kernelLLt,2,1,-4.000);
	cvmSet(kernelLLt,2,2,1.000);


	 

	//allocateOnDemand( &panorama5LLt, cvSize(prev_panorama->width,prev_panorama->height),  img1->depth, img1->nChannels );  
	allocateOnDemand( &panorama5LLt2, cvSize(prev_panorama->width,prev_panorama->height),  img1->depth, img1->nChannels );  
	allocateOnDemand( &panorama5d2, cvSize(prev_panorama->width,prev_panorama->height),  img1->depth, img1->nChannels ); 


 

//	cvFilter2D(prev_panorama,panorama5LLt,kernelLLt,cvPoint(-1,1));

	//panorama5LLt2 = MulImagewithDouble(panorama5LLt, lambda_it);
	panorama5LLt2 = MulImagewithDouble(prev_panorama, lambda_it);

	panorama5d2 = MulImagewithDouble(panorama5d, alpha_it);

   /**************************************************************************************/ 
   //
   // Now updating the iteration of x(n+1). First I have to do: panorama5d - panorama5LLt
	//   Then I have to do : panorma5_2 = panorama5_1 + (panorama5d - panorama5LLt)
   //
   /**************************************************************************************/
	
	//allocateOnDemand( &panoramaSR, cvSize(prev_panorama->width,prev_panorama->height),  img1->depth, img1->nChannels );  
	allocateOnDemand( &panorama5_Delta, cvSize(prev_panorama->width,prev_panorama->height),  img1->depth, img1->nChannels );  

	cvSub(panorama5d2,panorama5LLt2,panorama5_Delta,NULL);
	cvAdd(panorama5_Delta,prev_panorama,panoramaSR,NULL);

	//cvNamedWindow( "panoramaSR", 1 );
	//cvShowImage( "panoramaSR", panoramaSR );
	//cvWaitKey(10);

	cvReleaseMat( &kernelLLt);

	return;

}

extern void SRMosaickingYi(IplImage* img1, IplImage* img2, IplImage* img3, IplImage* img4, IplImage* img5, IplImage* prev_panorama, 
					struct image_border ROI_frame1, struct image_border ROI_frame2, struct image_border ROI_frame3, struct image_border ROI_frame4, struct image_border ROI_frame5,
					IplImage* panoramaSR, int D)
					{

//IplImage* panoramaSR = NULL;
IplImage* img1_inv = NULL, *img2_inv = NULL, *img3_inv=NULL, *img4_inv=NULL, *img5_inv=NULL;
IplImage* img1_inv_temp1 = NULL, *img2_inv_temp1 = NULL, *img3_inv_temp1=NULL, *img4_inv_temp1=NULL, *img5_inv_temp1=NULL;
IplImage* img1_inv_temp2 = NULL, *img2_inv_temp2 = NULL, *img3_inv_temp2=NULL, *img4_inv_temp2=NULL, *img5_inv_temp2=NULL;
IplImage* Signdimg1 = NULL, *Signdimg2 = NULL, *Signdimg3 = NULL, *Signdimg4 = NULL, *Signdimg5 = NULL;

IplImage* dimg1=NULL, *dimg2=NULL, *dimg3=NULL, *dimg4=NULL, *dimg5=NULL;
IplImage* Ddimg1=NULL, *Ddimg2=NULL, *Ddimg3=NULL, *Ddimg4=NULL, *Ddimg5=NULL;
IplImage* ddst1 = NULL, *ddst2 = NULL, *ddst3 = NULL, *ddst4 = NULL, *ddst5=NULL;
IplImage* panorama5d = NULL;
IplImage* panorama5d2 = NULL; //after multiplication with alpha
IplImage* panorama5LLt = NULL;
IplImage* panorama5LLt2 = NULL; //after multiplication
IplImage* panorama5_Delta = NULL;
IplImage* Xregularized = NULL; // To use with the algorithm Fast and Robust Regularization
IplImage* Limg = NULL;

CvMat* kernelLLt = cvCreateMat(3,3,CV_32FC1);
CvMat* kernelB = cvCreateMat(3,3,CV_32FC1);
CvMat* kernelL = cvCreateMat(3,3,CV_32FC1);

CvMat* lambdaM = NULL;

double lambda_1, lambda_2, lambda_3, lambda_4, lambda_5;
double lambda_num, lambda_den;
double lambdad_1, lambdad_2, lambdad_3, lambdad_4, lambdad_5;
double lambda_it;
double alpha_it = 1.1127;// was 7, 1, 1.1127
double errorSR;

float alpha;

CvPoint2D32f P; // P for the mosaic

int i, j;//, D;

struct RegionS  S1, S2, S3, S4, S5;

CvSize sizeD1, sizeD2, sizeD3, sizeD4, sizeD5;
CvSize size; //size of the frames
int p;


cvZero(panoramaSR);


	/// First part:  Interpolation : resizing the image to 2 times the original size

 // D = 2;

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

	size.height = img1->height;
	size.width = img1->width;

/**************************************************************************************/ 
   //   Starting a new iteration (it = 3)
   //   
	//  First Step: Getting the imgs from the mosaic image
   //
   /**************************************************************************************/

	allocateOnDemand( &img1_inv_temp1, sizeD1,  img1->depth, img1->nChannels );
    WarpImageUnderQuadInv(prev_panorama,img1_inv_temp1,ROI_frame1);

    allocateOnDemand( &img2_inv_temp1, sizeD1,  img1->depth, img1->nChannels );
    WarpImageUnderQuadInv(prev_panorama,img2_inv_temp1,ROI_frame2);

	allocateOnDemand( &img3_inv_temp1, sizeD1,  img1->depth, img1->nChannels );
	WarpImageUnderQuadInv(prev_panorama,img3_inv_temp1,ROI_frame3);

	allocateOnDemand( &img4_inv_temp1, sizeD1,  img1->depth, img1->nChannels );
	WarpImageUnderQuadInv(prev_panorama,img4_inv_temp1,ROI_frame4);

	allocateOnDemand( &img5_inv_temp1, sizeD1,  img1->depth, img1->nChannels );
	WarpImageUnderQuadInv(prev_panorama,img5_inv_temp1,ROI_frame5);

	// Using a Gaussian Filter to perform an Antia-alias filtering
	
	allocateOnDemand( &img1_inv_temp2, sizeD1,  img1->depth, img1->nChannels );
    allocateOnDemand( &img2_inv_temp2, sizeD1,  img1->depth, img1->nChannels );
    allocateOnDemand( &img3_inv_temp2, sizeD1,  img1->depth, img1->nChannels );
	allocateOnDemand( &img4_inv_temp2, sizeD1,  img1->depth, img1->nChannels );
	allocateOnDemand( &img5_inv_temp2, sizeD1,  img1->depth, img1->nChannels );
/*
	cvSmooth(img1_inv_temp1,img1_inv_temp2, CV_GAUSSIAN, 3, 0, 0, 0);
	cvSmooth(img2_inv_temp1,img2_inv_temp2, CV_GAUSSIAN, 3, 0, 0, 0);
	cvSmooth(img3_inv_temp1,img3_inv_temp2, CV_GAUSSIAN, 3, 0, 0, 0);
	cvSmooth(img4_inv_temp1,img4_inv_temp2, CV_GAUSSIAN, 3, 0, 0, 0);
	cvSmooth(img5_inv_temp1,img5_inv_temp2, CV_GAUSSIAN, 3, 0, 0, 0);
*/
	// Using Filter 2D:
    //This kernel is the same as the used in the paper: " A Transform-Domain Approach to Super-resolution Mosaicing of Compressed Images"

	cvmSet(kernelB,0,0,0.1016);
	cvmSet(kernelB,0,1,0.1172);
	cvmSet(kernelB,0,2,0.1016);
	cvmSet(kernelB,1,0,0.1172);
	cvmSet(kernelB,1,1,0.1250);
	cvmSet(kernelB,1,2,0.1172);
	cvmSet(kernelB,2,0,0.1016);
	cvmSet(kernelB,2,1,0.1172);
	cvmSet(kernelB,2,2,0.1016);

	cvFilter2D(img1_inv_temp1,img1_inv_temp2,kernelB,cvPoint(-1,1));
	cvFilter2D(img2_inv_temp1,img2_inv_temp2,kernelB,cvPoint(-1,1));
	cvFilter2D(img3_inv_temp1,img3_inv_temp2,kernelB,cvPoint(-1,1));
	cvFilter2D(img4_inv_temp1,img4_inv_temp2,kernelB,cvPoint(-1,1));
	cvFilter2D(img5_inv_temp1,img5_inv_temp2,kernelB,cvPoint(-1,1));
	
  // Downsampling using cubic interpolation

	allocateOnDemand( &img1_inv, size,  img1->depth, img1->nChannels );
    allocateOnDemand( &img2_inv, size,  img1->depth, img1->nChannels );
    allocateOnDemand( &img3_inv, size,  img1->depth, img1->nChannels );
	allocateOnDemand( &img4_inv, size,  img1->depth, img1->nChannels );
	allocateOnDemand( &img5_inv, size,  img1->depth, img1->nChannels );
	
	cvResize(img1_inv_temp2, img1_inv, CV_INTER_CUBIC);
	cvResize(img2_inv_temp2, img2_inv, CV_INTER_CUBIC);
	cvResize(img3_inv_temp2, img3_inv, CV_INTER_CUBIC);
	cvResize(img4_inv_temp2, img4_inv, CV_INTER_CUBIC);
	cvResize(img5_inv_temp2, img5_inv, CV_INTER_CUBIC);

	/*
// Downsampling using my own code

	DownSampleImage(img1_inv_temp2, img1_inv, 2);
	DownSampleImage(img2_inv_temp2, img2_inv, 2);
	DownSampleImage(img3_inv_temp2, img3_inv, 2);
	DownSampleImage(img4_inv_temp2, img4_inv, 2);
	DownSampleImage(img5_inv_temp2, img5_inv, 2);
*/
/*
	cvSaveImage("DBWRx1.jpg", img1_inv);
	cvSaveImage("DBWRx2.jpg", img2_inv);
	cvSaveImage("DBWRx3.jpg", img3_inv);
	cvSaveImage("DBWRx4.jpg", img4_inv);
	cvSaveImage("DBWRx5.jpg", img5_inv);


	cvNamedWindow( "img1_inv", 1 );
	cvShowImage( "img1_inv", img1_inv);
	cvWaitKey(0);

	cvNamedWindow( "img2_inv", 1 );
	cvShowImage( "img2_inv", img2_inv);
	cvWaitKey(0);

	cvNamedWindow( "img3_inv", 1 );
	cvShowImage( "img3_inv", img3_inv );
	cvWaitKey(0);

	cvNamedWindow( "img4_inv", 1 );
	cvShowImage( "img4_inv", img4_inv );
	cvWaitKey(0);

	cvNamedWindow( "img5_inv", 1 );
	cvShowImage( "img5_inv", img5_inv );
	cvWaitKey(0);
*/	

   /**************************************************************************************/ 
   //    
   //   
   //  Second Step: Compute the frame difference between the original low frames and the LR reconstructed ones
   //
   /**************************************************************************************/

  /************************************************************************/
  //  Doing the frame substracion
  /****************************************************************************/
   printf("\n Doing the frame substracion ...\n" );


  allocateOnDemand( &dimg1, cvSize(img2->width,img2->height),  img1->depth, img1->nChannels );
  allocateOnDemand( &dimg2, cvSize(img2->width,img2->height),  img1->depth, img1->nChannels );
  allocateOnDemand( &dimg3, cvSize(img2->width,img2->height),  img1->depth, img1->nChannels );
  allocateOnDemand( &dimg4, cvSize(img2->width,img2->height),  img1->depth, img1->nChannels );
  allocateOnDemand( &dimg5, cvSize(img2->width,img2->height),  img1->depth, img1->nChannels );
/*
   printf("\n size_img1 = (%d , %d) \t size_img1_inv = (%d, %d)", img1->height, img1->width, img1_inv->height, img1_inv->width);
   printf("\n size_dimg1 = (%d , %d) ", dimg1->height, dimg1->width);

   getchar();
*/

  cvSub(img1,img1_inv,dimg1,NULL);
  cvSub(img2,img2_inv,dimg2,NULL);
  cvSub(img3,img3_inv,dimg3,NULL);
  cvSub(img4,img4_inv,dimg4,NULL);
  cvSub(img5,img5_inv,dimg5,NULL);

  /************************************************************************/
  //  Taking the sign of the error images
  /****************************************************************************/
  allocateOnDemand( &Signdimg1, cvSize(img2->width,img2->height),  img1->depth, img1->nChannels );
  allocateOnDemand( &Signdimg2, cvSize(img2->width,img2->height),  img1->depth, img1->nChannels );
  allocateOnDemand( &Signdimg3, cvSize(img2->width,img2->height),  img1->depth, img1->nChannels );
  allocateOnDemand( &Signdimg4, cvSize(img2->width,img2->height),  img1->depth, img1->nChannels );
  allocateOnDemand( &Signdimg5, cvSize(img2->width,img2->height),  img1->depth, img1->nChannels );

  SignImage(dimg1, Signdimg1);
  SignImage(dimg2, Signdimg2);
  SignImage(dimg3, Signdimg3);
  SignImage(dimg4, Signdimg4);
  SignImage(dimg5, Signdimg5);

  
 
  //*********************************************************************************************//
  //  Doing the interpolation of the images differences											//
  //*********************************************************************************************//
    printf("\n Doing the frame interpolation of the difference of frames ...\n" );
    allocateOnDemand( &Ddimg1, sizeD1,  img1->depth, img1->nChannels );  
	allocateOnDemand( &Ddimg2, sizeD2,  img1->depth, img1->nChannels );  
	allocateOnDemand( &Ddimg3, sizeD3,  img1->depth, img1->nChannels );  
	allocateOnDemand( &Ddimg4, sizeD4,  img1->depth, img1->nChannels );  
	allocateOnDemand( &Ddimg5, sizeD5,  img1->depth, img1->nChannels );  

	/*
  	cvResize(dimg1, Ddimg1,CV_INTER_CUBIC);
	cvResize(dimg2, Ddimg2,CV_INTER_CUBIC);
	cvResize(dimg3, Ddimg3,CV_INTER_CUBIC);
	cvResize(dimg4, Ddimg4,CV_INTER_CUBIC);
	cvResize(dimg5, Ddimg5,CV_INTER_CUBIC);
	*/

	UpSampleImageD(Signdimg1, Ddimg1);
	UpSampleImageD(Signdimg2, Ddimg2);
	UpSampleImageD(Signdimg3, Ddimg3);
	UpSampleImageD(Signdimg4, Ddimg4);
	UpSampleImageD(Signdimg5, Ddimg5);

/***********************************************************************************/
//			 Doing the Mosaicking of the Error Images								//
//																					//
/***********************************************************************************/
	printf("\n Doing the Mosaicking of the Error Images ...\n" );

	allocateOnDemand( &ddst1,  cvSize(panoramaSR->width,panoramaSR->height),  img1->depth, img1->nChannels );  
	allocateOnDemand( &ddst2,  cvSize(panoramaSR->width,panoramaSR->height),  img1->depth, img1->nChannels );  
	allocateOnDemand( &ddst3,  cvSize(panoramaSR->width,panoramaSR->height),  img1->depth, img1->nChannels );  
	allocateOnDemand( &ddst4,  cvSize(panoramaSR->width,panoramaSR->height),  img1->depth, img1->nChannels );  
	allocateOnDemand( &ddst5,  cvSize(panoramaSR->width,panoramaSR->height),  img1->depth, img1->nChannels );  
	
	/*
	allocateOnDemand( &ddst1,  cvSize(1360, 640),  img1->depth, img1->nChannels );  
	allocateOnDemand( &ddst2,  cvSize(1360, 640),  img1->depth, img1->nChannels );  
	allocateOnDemand( &ddst3,  cvSize(1360, 640),  img1->depth, img1->nChannels );  
	allocateOnDemand( &ddst4,  cvSize(1360, 640),  img1->depth, img1->nChannels );  
	allocateOnDemand( &ddst5,  cvSize(1360, 640),  img1->depth, img1->nChannels );  
*/

    WarpImageUnderQuad(Ddimg1,ddst1,ROI_frame1);
	//cvNamedWindow( "Perspective_Warpdd1", 1 );
	//cvShowImage( "Perspective_Warpdd1", ddst1);
	//cvWaitKey(10);


  WarpImageUnderQuad(Ddimg2,ddst2,ROI_frame2);

 // cvNamedWindow( "Perspective_Warpdd2", 1 );
 // cvShowImage( "Perspective_Warpdd2", ddst2 );
  //cvWaitKey(10);


 WarpImageUnderQuad(Ddimg3,ddst3,ROI_frame3);
 //cvNamedWindow( "Perspective_Warpdd3", 1 );
 //cvShowImage( "Perspective_Warpdd3", ddst3 );
 //cvWaitKey(10);


 WarpImageUnderQuad(Ddimg4,ddst4,ROI_frame4);
 //cvNamedWindow( "Perspective_Warpdd4", 1 );
 //cvShowImage( "Perspective_Warpdd4", ddst4 );
 //cvWaitKey(10);

 
 WarpImageUnderQuad(Ddimg5,ddst5,ROI_frame5);
 //cvNamedWindow( "Perspective_Warpdd5", 1 );
 ///cvShowImage( "Perspective_Warpdd5", ddst5 );
 //cvWaitKey(10);

 allocateOnDemand( &panorama5d,  cvSize(prev_panorama->width, prev_panorama->height),  img1->depth, img1->nChannels );  

 S2 = FindSRegion(ROI_frame2);

	  for(i = 0; i < panorama5d->height; i ++){ //rows
		  for(j = 0; j < panorama5d->width; j++){ //cols
			  P.x = j;
			  P.y = i;
			  if(Belongs2S(P,S2))cvSet2D(panorama5d,i,j,cvGet2D(ddst2,i,j));
			  else cvSet2D(panorama5d,i,j,cvGet2D(ddst1,i,j));			  
		  }//end for
	  }// end for

	

////////////////////////////
// Finding the region S3 //
////////////////////////////

   S3 = FindSRegion(ROI_frame3);

	  for(i = 0; i < panorama5d->height; i ++){ //rows
		  for(j = 0; j < panorama5d->width; j++){ //cols
			  P.x = j;
			  P.y = i;
			  if(Belongs2S(P,S3))cvSet2D(panorama5d,i,j,cvGet2D(ddst3,i,j));
			  else cvSet2D(panorama5d,i,j,cvGet2D(panorama5d,i,j));			  
		  }//end for
	  }// end for



 ////////////////////////////
// Finding the region S4 //
////////////////////////////

   S4 = FindSRegion(ROI_frame4);

	  for(i = 0; i < panorama5d->height; i ++){ //rows
		  for(j = 0; j < panorama5d->width; j++){ //cols
			  P.x = j;
			  P.y = i;
			  if(Belongs2S(P,S4))cvSet2D(panorama5d,i,j,cvGet2D(ddst4,i,j));
			  else cvSet2D(panorama5d,i,j,cvGet2D(panorama5d,i,j));			  
		  }//end for
	  }// end for


	    ////////////////////////////
		// Finding the region S4 //
		////////////////////////////

   S5 = FindSRegion(ROI_frame5);


	  for(i = 0; i < panorama5d->height; i ++){ //rows
		  for(j = 0; j < panorama5d->width; j++){ //cols
			  P.x = j;
			  P.y = i;
			  if(Belongs2S(P,S5))cvSet2D(panorama5d,i,j,cvGet2D(ddst5,i,j));
			  else cvSet2D(panorama5d,i,j,cvGet2D(panorama5d,i,j));			  
		  }//end for
	  }// end for

	// cvNamedWindow( "panorama5d", 1 );
	 //cvShowImage( "panorama5d", panorama5d );
	 //cvWaitKey(10);



   /**************************************************************************************/
   //
   // Computing the normalizing parameter lambda^(n) 
   //
   /**************************************************************************************/
    
	  lambda_1 = cvNorm(img1, img1_inv, CV_L2, NULL);
	  printf("\n lambda_1 = %f", lambda_1);
	  lambda_2 = cvNorm(img2, img2_inv, CV_L2, NULL);
	  printf("\n lambda_2 = %f", lambda_2);
	  lambda_3 = cvNorm(img3, img3_inv, CV_L2, NULL);
	  printf("\n lambda_3 = %f", lambda_3);
	  lambda_4 = cvNorm(img4, img4_inv, CV_L2, NULL);
	  printf("\n lambda_4 = %f", lambda_4);
	  lambda_5 = cvNorm(img5, img5_inv, CV_L2, NULL);
	  printf("\n lambda_5 = %f", lambda_5);

   /**************************************************************************************/
   //
   // Computing the Laplace of the x^(n) image x(n) comes to be the panorama5 variables (mosaic of the previous state)
   //
   /**************************************************************************************/
   allocateOnDemand( &Limg, cvSize(prev_panorama->width,prev_panorama->height),  img1->depth, 3 );  
   //cvLaplace(prev_panorama, Limg,3);   

 /*
   // Using in stead cvFilter2D

   	cvmSet(kernelL,0,0,0.0000);
	cvmSet(kernelL,0,1,0.2500);
	cvmSet(kernelL,0,2,0.0000);
	cvmSet(kernelL,1,0,0.2500);
	cvmSet(kernelL,1,1,-1.000);
	cvmSet(kernelL,1,2,0.2500);
	cvmSet(kernelL,2,0,0.0000);
	cvmSet(kernelL,2,1,0.2500);
	cvmSet(kernelL,2,2,0.0000);

   cvFilter2D(prev_panorama, Limg, kernelL, cvPoint(-1,-1));
*/

  // Using Huber Prior Function to find lambda

   alpha = 1.75;//was 1.75
   p = 2;

   //HubbertFunctionCliques(prev_panorama, Limg, alpha);

  lambda_1 = BilateralRegularization(prev_panorama, p, alpha);

  FastRobustRegularization(prev_panorama, Limg, p, alpha);

  // cvNamedWindow( "Limg", 1 );
  // cvShowImage( "Limg", Limg );
  // cvWaitKey(0);

  



   /**************************************************************************************/
   //
   // Computing the K norm(x^(n),2) , K = 1, 2 ...5
   //
   /**************************************************************************************/

  lambda_num = pow((lambda_1 + lambda_2 + lambda_3 + lambda_4 + lambda_5),2);
  //lambdad_1 = cvNorm(Limg, NULL, CV_L2, NULL); // Because it was already computed in the function BilateralRegularization

  

    /**************************************************************************************/
   //
   // Computing the lambda(n), K = 5, because I am just working with 5 frames
   //
   /**************************************************************************************/
 
  lambda_den = pow((5*lambda_1),2);//lambda_1

  lambda_it =  lambda_num/lambda_den;

   printf("\n lambda_num = %f", lambda_num);
   printf("\n lambda_den = %f", lambda_den);
   printf("\n lambda_it = %f", lambda_it);
   
  lambda_it = 1.1127;

   printf("\n Using alpha from GCV = %f", alpha_it);



    /**************************************************************************************/
   //
   // Computing the lambda(n)*L'*L*x(n)
   //
   /**************************************************************************************/

   /*
    cvmSet(kernelLLt,0,0,1.000*lambda_it);
	cvmSet(kernelLLt,0,1,-4.000*lambda_it);
	cvmSet(kernelLLt,0,2,1.000*lambda_it);
	cvmSet(kernelLLt,1,0,-4.000*lambda_it);
	cvmSet(kernelLLt,1,1,18.000*lambda_it);
	cvmSet(kernelLLt,1,2,-4.000*lambda_it);
	cvmSet(kernelLLt,2,0,1.000*lambda_it);
	cvmSet(kernelLLt,2,1,-4.000*lambda_it);
	cvmSet(kernelLLt,2,2,1.000*lambda_it);
*/
	cvmSet(kernelLLt,0,0,1.000);
	cvmSet(kernelLLt,0,1,-4.000);
	cvmSet(kernelLLt,0,2,1.000);
	cvmSet(kernelLLt,1,0,-4.000);
	cvmSet(kernelLLt,1,1,18.000);
	cvmSet(kernelLLt,1,2,-4.000);
	cvmSet(kernelLLt,2,0,1.000);
	cvmSet(kernelLLt,2,1,-4.000);
	cvmSet(kernelLLt,2,2,1.000);

	allocateOnDemand( &panorama5LLt, cvSize(prev_panorama->width,prev_panorama->height),  img1->depth, img1->nChannels );  
	allocateOnDemand( &panorama5LLt2, cvSize(prev_panorama->width,prev_panorama->height),  img1->depth, img1->nChannels );  
	allocateOnDemand( &panorama5d2, cvSize(prev_panorama->width,prev_panorama->height),  img1->depth, img1->nChannels ); 

	printf("\n Limg->height = %d \t Limg->width = %d", Limg->height, Limg->width);
	printf("\n prev_panorama->height = %d \t prev_panorama->width = %d", prev_panorama->height, prev_panorama->width);


    // Using Huber Regularization here as well

//HubbertRegularization(Limg, panorama5LLt, alpha);

    // Using Laplace prior
	//cvFilter2D(Limg,panorama5LLt,kernelL,cvPoint(-1,1));

/*
    //Using the regularization proposed in the paper: "Fast and Robust Super-Resolution"

	allocateOnDemand( &Xregularized, cvSize(prev_panorama->width,prev_panorama->height), IPL_DEPTH_32F, 1 );

	FastRobustRegularization(Limg, Xregularized, 2, alpha); // p = 2

	panorama5LLt = convert_gray32_to_color(Xregularized);

*/

	/*
	//
	// Converting to Matrix lambda doesn't work, because of the size of the mosaicking
	//

	lambdaM = cvCreateMat(panorama5LLt->height,panorama5LLt->width,CV_32FC1);
	cvSetIdentity(lambdaM,cvRealScalar(lambda_it));

	cvMatMul(panorama5LLt,lambdaM, panorama5LLt2);
	*/

	//panorama5LLt2 = MulImagewithDouble(panorama5LLt, lambda_it);
	panorama5LLt2 = MulImagewithDouble(Limg, lambda_it); // Because I am using Bilateral Filter
	panorama5d2 = MulImagewithDouble(panorama5d, alpha_it);

   /**************************************************************************************/ 
   //
   //	Now updating the iteration of x(n+1). First I have to do: panorama5d - panorama5LLt
	//  Then I have to do : panorma5_2 = panorama5_1 + (panorama5d - panorama5LLt)
   //
   /**************************************************************************************/
	
	//allocateOnDemand( &panoramaSR, cvSize(prev_panorama->width,prev_panorama->height),  img1->depth, img1->nChannels );  
	allocateOnDemand( &panorama5_Delta, cvSize(prev_panorama->width,prev_panorama->height),  img1->depth, img1->nChannels );  

	//cvSub(panorama5d2,panorama5LLt2,panorama5_Delta,NULL);
	cvSub(panorama5LLt2,panorama5d2,panorama5_Delta,NULL); // Because I am using L1 norm
	cvAdd(panorama5_Delta,prev_panorama,panoramaSR,NULL);


	/***********************************************************************************
	Computing the error of the SR
	*****************************************************************************************/

	errorSR = cvNorm(panoramaSR, prev_panorama, CV_RELATIVE_L2, NULL);

	printf(" \n errorSR = %f\n", errorSR);

	//cvNamedWindow( "panoramaSR", 1 );
	//cvShowImage( "panoramaSR", panoramaSR );
	//cvWaitKey(10);

	cvReleaseMat( &kernelLLt );
	cvReleaseMat( &kernelL );
	cvReleaseMat( &kernelB );
	cvReleaseMat( &lambdaM );

	return;

}

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

extern IplImage* MulImagewithDouble(IplImage* img, double factor)
{
 IplImage* result = NULL;
 int  i, j, k;
 CvScalar z1, z2,zR;

 allocateOnDemand( &result, cvSize(img->width,img->height),  img->depth, img->nChannels );  
 for(i = 0; i < img->height; i++){ //rows
	 for( j = 0; j < img->width; j++) { //cols
		 z1 = cvGet2D(img,i,j);
		 /*
		 z2.val[0] = (z1.val[0])*(factor);
		 z2.val[1] = (z1.val[1])*(factor);
		 z2.val[2] = (z1.val[2])*(factor);
		 */
		  for ( k = 0; k < img->nChannels; k++){
		    z2.val[k] = (z1.val[k])*(factor);		 
		   }
		 cvSet2D(result,i,j,z2);
	 } // end for 
 } // end for
 return result;

}



// Computes the PSNR
// PSNR = 10log10(255^2N/norm2(x - xhat));

extern double PSNR(IplImage* X, IplImage* Xhat)
{
	int N;
	double PSNR;
	double temp;
	CvSize sizeX;

	sizeX = cvGetSize(X);
	N = (sizeX.height)*(sizeX.width);

	temp = 10*log10(255*255*N);
	PSNR = temp/cvNorm(X,Xhat,CV_L2,NULL);

	return PSNR;
}

// void DownSampleImage(IplImage* Img, IplImage* DownSampleImage, int D)
// This function downsamples the image by a factor of D, it does zero filling as
// in the paper: "Fast and Robust Multi-Frame Super-Resolution"Sina Farsiy, Dirk Robinson.
// The width and height of the image Img must be multiple of D
// This function in the equation is represented as D

extern void DownSampleImage(IplImage* Img, IplImage* DownSampleImage, int D)
{

   int i, j;
   int iA, jA;
   IplImage* temp = NULL;

   // added to follow the idea of the paper: " A Transform Approach to Super-Resolution Mosaicing
   //  of Compressed Images

   temp = cvCreateImage( cvSize(DownSampleImage->width,DownSampleImage->height),  Img->depth, Img->nChannels );  

	 for(i = 0; i < Img->height; i = i + D){ //rows
		 for( j = 0; j < Img->width; j = j + D) { //cols
				iA = i/D;
				jA = j/D;			
			//	 cvSet2D(DownSampleImage, iA , jA,cvGet2D(Img,i,j));  // Modified here
				cvSet2D(temp, iA , jA,cvGet2D(Img,i,j)); 
  			
		 } // end for 
	 } // end for
	 
	
    cvSmooth(temp,DownSampleImage, CV_GAUSSIAN, 5, 5, 2, 0); // Added

	cvReleaseImage(&temp); // Added

	return;
}
// extern void UpSampleImage(IplImage* Img, IplImage* DownSampleImage, int D)
// This function upsamples the image by a factor of D, it does zero filling as
// in the paper: "Fast and Robust Multi-Frame Super-Resolution"Sina Farsiy, Dirk Robinson.
// The width and height of the image Img must be multiple of D
// This function in the equation is represented as D^t

extern void UpSampleImage(IplImage* Img, IplImage* UpSampleImage, int D)
{

    int i, j;
	int iA, jA;

	 for(i = 0; i < UpSampleImage->height; i = i + D){ //rows
		 for( j = 0; j < UpSampleImage->width; j = j + D) { //cols
				iA = i/D;
				jA = j/D;			
				cvSet2D(UpSampleImage, i , j,cvGet2D(Img,iA,jA));
  			
		 } // end for 
	 } // end for
	
	return;
}

extern void UpSampleImageD(IplImage* Img, IplImage* UpSampleImageD)
{

	IplImage* temp = NULL;
	temp = cvCreateImage( cvSize(UpSampleImageD->width,UpSampleImageD->height),  Img->depth, Img->nChannels );  
   // allocateOnDemand( &temp, cvSize(UpSampleImageD->width,UpSampleImageD->height),  Img->depth, Img->nChannels );  

	cvResize(Img, temp,CV_INTER_CUBIC);

	// Now deblurring to do the downsampling

	//cvSmooth(temp,UpSampleImageD, CV_GAUSSIAN, 3, 0, 0, 0);
// June 6th, 2012 Commenting out the smooth to see if works 
    cvSmooth(temp,UpSampleImageD, CV_GAUSSIAN, 5, 5, 2, 0);
	//UpSampleImageD = cvCloneImage(temp);


	cvReleaseImage( &temp);


  //cvResize(Img, UpSampleImageD,CV_INTER_CUBIC);

	return;
}
/*
extern void UpSampleImageDcvg(IplImage* Img, IplImage* UpSampleImageD)
{

	IplImage* temp = NULL;
	temp = cvgCreateImage( cvSize(UpSampleImageD->width,UpSampleImageD->height),  Img->depth, Img->nChannels );  
   // allocateOnDemand( &temp, cvSize(UpSampleImageD->width,UpSampleImageD->height),  Img->depth, Img->nChannels );  

	cvgResize(Img, temp,CV_INTER_CUBIC);

	// Now deblurring to do the downsampling

	//cvSmooth(temp,UpSampleImageD, CV_GAUSSIAN, 3, 0, 0, 0);
	cvgSmooth(temp,UpSampleImageD, CV_GAUSSIAN, 5, 5, 2, 0);

	cvgReleaseImage( &temp);


  //cvResize(Img, UpSampleImageD,CV_INTER_CUBIC);

	return;
}
*/

extern void cvShowManyImages(char* title, int nArgs, ...)
{
    // img - Used for getting the arguments
    IplImage *img;

	va_list args;

    // DispImage - the image in which input images are to be copied
    IplImage *DispImage;

    int size;
    int i;
    int m, n;
    int x, y;

    // w - Maximum number of images in a row
    // h - Maximum number of images in a column
    int w, h;

    // scale - How much we have to resize the image
    float scale;
    int max;

    // If the number of arguments is lesser than 0 or greater than 12
    // return without displaying
    if(nArgs <= 0) {
        printf("Number of arguments too small....\n");
        return;
    }
    else if(nArgs > 12) {
        printf("Number of arguments too large....\n");
        return;
    }
    // Determine the size of the image,
    // and the number of rows/cols
    // from number of arguments
    else if (nArgs == 1) {
        w = h = 1;
        size = 300;
    }
    else if (nArgs == 2) {
        w = 2; h = 1;
        size = 300;
    }
    else if (nArgs == 3 || nArgs == 4) {
        w = 2; h = 2;
        size = 300;
    }
    else if (nArgs == 5 || nArgs == 6) {
        w = 3; h = 2;
        size = 200;
    }
    else if (nArgs == 7 || nArgs == 8) {
        w = 4; h = 2;
        size = 200;
    }
    else {
        w = 4; h = 3;
        size = 150;
    }

    // Create a new 3 channel image
    DispImage = cvCreateImage( cvSize(100 + size*w, 60 + size*h), 8, 3 );

    // Used to get the arguments passed
    
    va_start(args, nArgs);

    // Loop for nArgs number of arguments
    for (i = 0, m = 20, n = 20; i < nArgs; i++, m += (20 + size)) {

        // Get the Pointer to the IplImage
        img = va_arg(args, IplImage*);

        // Check whether it is NULL or not
        // If it is NULL, release the image, and return
        if(img == 0) {
            printf("Invalid arguments");
            cvReleaseImage(&DispImage);
            return;
        }

        // Find the width and height of the image
        x = img->width;
        y = img->height;

        // Find whether height or width is greater in order to resize the image
        max = (x > y)? x: y;

        // Find the scaling factor to resize the image
        scale = (float) ( (float) max / size );

        // Used to Align the images
        if( i % w == 0 && m!= 20) {
            m = 20;
            n+= 20 + size;
        }

        // Set the image ROI to display the current image
        cvSetImageROI(DispImage, cvRect(m, n, (int)( x/scale ), (int)( y/scale )));

        // Resize the input image and copy the it to the Single Big Image
        cvResize(img, DispImage,CV_INTER_CUBIC );

        // Reset the ROI in order to display the next image
        cvResetImageROI(DispImage);
    }

    // Create a new window, and show the Single Big Image
    cvNamedWindow( title, 1 );
    cvShowImage( title, DispImage);

    cvWaitKey(0);
    cvDestroyWindow(title);

    // End the number of arguments
    va_end(args);

    // Release the Image Memory
    cvReleaseImage(&DispImage);
}//You can use this function as in this sample program: 

// w and h are the width and height of all the frames
extern IplImage* CropImageROI(IplImage* src, double dx, double dy, double tetha, int w, int h)
{
	//CvSize size_src;
	int w_dst, h_dst;
	IplImage* dst = NULL;
	CvPoint2D32f P;
	int i, j;
	double wd, hd;
	struct RegionS S;
	struct image_border ROI;


   	//w = size_src.width;
	//h = size_src.height;

	wd = (double)w;
	hd = (double)h;
	/*
	ROI.P1.x = dx;
	ROI.P1.y = dy;
	ROI.P2.x = dx - hd*sin(tetha);
	ROI.P2.y = dy - hd*cos(tetha);
	ROI.P3.x = ROI.P2.x + wd*cos(tetha);
	ROI.P3.y = ROI.P2.y + wd*sin(tetha);
	ROI.P4.x = ROI.P1.x + wd*cos(tetha);
	ROI.P4.y = ROI.P1.y + wd*sin(tetha);
*/

	wd = (double)w;
	hd = (double)h;
	ROI.P1.x = dx;
	ROI.P1.y = dy;
	ROI.P2.x = dx - h*sin(tetha);
	ROI.P2.y = dy + h*cos(tetha);
	ROI.P3.x = ROI.P2.x + w*cos(tetha);
	ROI.P3.y = ROI.P2.y + w*sin(tetha);
	ROI.P4.x = ROI.P1.x + w*cos(tetha);
	ROI.P4.y = ROI.P1.y + w*sin(tetha);

	printf("\n h = %f, w = %f", hd, wd);
	printf("\n sin = %f, cos = %f",sin(tetha), cos(tetha));
	printf("\n P1.x = %f, P1.y = %f", ROI.P1.x, ROI.P1.y);
	printf("\n P2.x = %f, P2.y = %f", ROI.P2.x, ROI.P2.y);
	printf("\n P3.x = %f, P3.y = %f", ROI.P3.x, ROI.P3.y);
	printf("\n P4.x = %f, P4.y = %f", ROI.P4.x, ROI.P4.y);

	getchar();


	w_dst = (int)(abs(ROI.P4.x - ROI.P2.x));
	h_dst = (int)(abs(ROI.P3.y - ROI.P1.y));

	allocateOnDemand( &dst, cvSize(w_dst,h_dst),  src->depth, src->nChannels );  

	S = FindSRegion(ROI);

	for( i = 0; i < src->height; i++){
		for( j = 0; j < src->width; j++){
			P.x = j;
			P.y = i;
			  if(Belongs2S(P,S))cvSet2D(dst,i,j,cvGet2D(src,i,j));
		}
	}

	return dst;

}
extern void PrintHomography2File( CvMat* H, char* name)
{
	FILE *pFile; 
	int i,j;
	//pFile = fopen("Homographies.txt", "w");
	pFile = fopen(name, "w");

	if(pFile!= NULL)
	{
		for( i = 0; i < 3; i++){ //cols
			for(j = 0; j < 3; j ++){//rows
				fprintf (pFile, "%f \n",cvmGet(H,j,i));
			}
		}
	    fclose (pFile);
	}

	return;
}



/*
This function implements the derivate of the huber function
*/
extern double DerivateHubbertFunction(double x, float alpha)
{
 double px;
 
 if(fabs(x) < alpha) px = 2*x;
 else px = 2*alpha*signx(x);

 return px;

}

extern double HubbertFunction(double x, float alpha)
{
 double px;
 
 if(fabs(x) < alpha) px = x*x;
 else px = 2*alpha*fabs(x) - alpha*alpha;

 return px;

}

extern float signx(double x)
{
  float result;

  if(x > 0.0) result = -1.0;
  if(x == 0.0)  result = 0.0;
  if(x < 0.0) result = - 1.0;

  return result; 
}
extern void HubbertRegularization(IplImage* input, IplImage* output, float alpha)
{
  CvScalar z_input;
  CvScalar z_output;
  CvScalar temps;
  CvScalar   zij_1,zij, zij1, zi1j_1, zi_1j1, zi_1j, zi1j, zi_1j_1, zi1j1;
  int i, j, k;
  float d1, d2, d3, d4;
  double temp;

  for(i = 0; i < input->height; i++){//row
	  for(j = 0; j < input->width; j++){ //col
		  if( i - 1 >= 0 && i + 1 < input->height && j - 1 >= 0 && j + 1 < input->width)
		  {
			  zij_1 =  cvGet2D(input,i,j - 1); // i, j -1
			  zij = cvGet2D(input,i,j); // i,j 
			  zij1 =  cvGet2D(input,i,j + 1);// i, j + 1
			  zi1j_1 = cvGet2D(input,i + 1,j - 1);// i + 1, j - 1
			  zi_1j1 =  cvGet2D(input,i - 1,j + 1);// i - 1, j + 1
			  zi_1j = cvGet2D(input,i - 1,j); // i - 1, j
			  zi1j = cvGet2D(input,i + 1,j); // i + 1, j
			  zi_1j_1 =  cvGet2D(input,i - 1,j - 1);// i - 1, j - 1
			  zi1j1 = cvGet2D(input,i + 1,j + 1); // i + 1, j
              
			  for ( k = 0; k < input->nChannels; k++){
				   d1 = zij_1.val[k] -2*zij.val[k] + zij1.val[k];
				   d2 = 0.5*zi1j_1.val[k] - zij.val[k] + 0.5*zi_1j1.val[k];
				   d3 = zi_1j.val[k] - 2*zij.val[k] + zi1j.val[k];
			       d4 = 0.5*zi_1j_1.val[k] - zij.val[k] + 0.5*zi1j1.val[k];
			  	  temps.val[k] = DerivateHubbertFunction(d1,alpha) + DerivateHubbertFunction(d2,alpha) 
				  + DerivateHubbertFunction(d3,alpha) + DerivateHubbertFunction(d4,alpha);
				   temps.val[k] = (temps.val[k]);
				   cvSet2D(output,i,j,temps);
			  }
		  }
		  // more cases
			  // case P1
		  else if(i == 0 && j == 0){
			  zij = cvGet2D(input,i,j); // i,j 
			  zij1 =  cvGet2D(input,i,j + 1);// i, j + 1
			  zi1j = cvGet2D(input,i + 1,j); // i + 1, j
			  zi1j1 = cvGet2D(input,i + 1,j + 1); // i + 1, j              
			  for ( k = 0; k < input->nChannels; k++){
				   d1 = -2*zij.val[k] + zij1.val[k];
				   d2 =  -zij.val[k];
				   d3 = -2*zij.val[k] + zi1j.val[k];
			       d4 = -zij.val[k] + 0.5*zi1j1.val[k];
			  	  temps.val[k] = DerivateHubbertFunction(d1,alpha) + DerivateHubbertFunction(d2,alpha) 
				  + DerivateHubbertFunction(d3,alpha) + DerivateHubbertFunction(d4,alpha);		
				   temps.val[k] = (temps.val[k]);
				   cvSet2D(output,i,j,temps);
			  }
		  }
		  // case P2
		  else if(i == 0 && j == (input->width - 1)){
			  zij_1 =  cvGet2D(input,i,j - 1); // i, j -1
			  zij = cvGet2D(input,i,j); // i,j 
			  zi1j_1 = cvGet2D(input,i + 1,j - 1);// i + 1, j - 1
			  zi1j = cvGet2D(input,i + 1,j); // i + 1, j		  
			
			  	  for ( k = 0; k < input->nChannels; k++){
				   d1 = zij_1.val[k] -2*zij.val[k];
				   d2 = 0.5*zi1j_1.val[k] - zij.val[k];
				   d3 = -2*zij.val[k] + zi1j.val[k];
			       d4 =  -zij.val[k];
			  	  temps.val[k] = DerivateHubbertFunction(d1,alpha) + DerivateHubbertFunction(d2,alpha) 
				  + DerivateHubbertFunction(d3,alpha) + DerivateHubbertFunction(d4,alpha);
			
				   temps.val[k] = (temps.val[k]);
				   cvSet2D(output,i,j,temps);
			  }

		  }
		   // case P3
		  else if(i == (input->height - 1) && j == (input->width - 1)){
			  zij_1 =  cvGet2D(input,i,j - 1); // i, j -1
			  zij = cvGet2D(input,i,j); // i,j 
			  zi_1j = cvGet2D(input,i - 1,j); // i - 1, j
			  zi_1j_1 =  cvGet2D(input,i - 1,j - 1);// i - 1, j - 1
			  
			
			  	  for ( k = 0; k < input->nChannels; k++){
				   d1 = zij_1.val[k] -2*zij.val[k];
				   d2 =  -zij.val[k];
				   d3 = zi_1j.val[k] - 2*zij.val[k];
			       d4 = 0.5*zi_1j_1.val[k] - zij.val[k];

			  	  temps.val[k] = DerivateHubbertFunction(d1,alpha) + DerivateHubbertFunction(d2,alpha) 
				  + DerivateHubbertFunction(d3,alpha) + DerivateHubbertFunction(d4,alpha);
			
				   temps.val[k] = (temps.val[k]);
				   cvSet2D(output,i,j,temps);
				  }

		  }
		  // case P4
		  else if(i == (input->height - 1) && j == 0){
			  zij = cvGet2D(input,i,j); // i,j 
			  zij1 =  cvGet2D(input,i,j + 1);// i, j + 1
			  zi_1j1 =  cvGet2D(input,i - 1,j + 1);// i - 1, j + 1
			  zi_1j = cvGet2D(input,i - 1,j); // i - 1, j
			             
			  for ( k = 0; k < input->nChannels; k++){
				   d1 = -2*zij.val[k] + zij1.val[k];
				   d2 = -zij.val[k] + 0.5*zi_1j1.val[k];
				   d3 = zi_1j.val[k] - 2*zij.val[k];
			       d4 = zij.val[k];
			  	  temps.val[k] = DerivateHubbertFunction(d1,alpha) + DerivateHubbertFunction(d2,alpha) 
				  + DerivateHubbertFunction(d3,alpha) + DerivateHubbertFunction(d4,alpha);
				   temps.val[k] = (temps.val[k]);
				   cvSet2D(output,i,j,temps);
			  }
		  }
		  // case LS
		  else if(i > 0 && i < (input->height - 1) && j == 0) {
			 
			  zij = cvGet2D(input,i,j); // i,j 
			  zij1 =  cvGet2D(input,i,j + 1);// i, j + 1
			  zi_1j1 =  cvGet2D(input,i - 1,j + 1);// i - 1, j + 1
			  zi_1j = cvGet2D(input,i - 1,j); // i - 1, j
			  zi1j = cvGet2D(input,i + 1,j); // i + 1, j
			  zi1j1 = cvGet2D(input,i + 1,j + 1); // i + 1, j
              
			  for ( k = 0; k < input->nChannels; k++){
				   d1 = -2*zij.val[k] + zij1.val[k];
				   d2 = -zij.val[k] + 0.5*zi_1j1.val[k];
				   d3 = zi_1j.val[k] - 2*zij.val[k] + zi1j.val[k];
			       d4 = -zij.val[k] + 0.5*zi1j1.val[k];
			  	  temps.val[k] = DerivateHubbertFunction(d1,alpha) + DerivateHubbertFunction(d2,alpha) 
				  + DerivateHubbertFunction(d3,alpha) + DerivateHubbertFunction(d4,alpha);
				   temps.val[k] = (temps.val[k]);
				   cvSet2D(output,i,j,temps);
			  }
		  }

		  // case US
		  else if(i == 0 && j > 0 && j < (input->width - 1)) {
			  zij_1 =  cvGet2D(input,i,j - 1); // i, j -1
			  zij = cvGet2D(input,i,j); // i,j 
			  zij1 =  cvGet2D(input,i,j + 1);// i, j + 1
			  zi1j_1 = cvGet2D(input,i + 1,j - 1);// i + 1, j - 1
			  zi1j = cvGet2D(input,i + 1,j); // i + 1, j
			  zi1j1 = cvGet2D(input,i + 1,j + 1); // i + 1, j
              
			  for ( k = 0; k < input->nChannels; k++){
				   d1 = zij_1.val[k] -2*zij.val[k] + zij1.val[k];
				   d2 = 0.5*zi1j_1.val[k] - zij.val[k];
				   d3 = -2*zij.val[k] + zi1j.val[k];
			       d4 = -zij.val[k] + 0.5*zi1j1.val[k];
			  	  temps.val[k] = DerivateHubbertFunction(d1,alpha) + DerivateHubbertFunction(d2,alpha) 
				  + DerivateHubbertFunction(d3,alpha) + DerivateHubbertFunction(d4,alpha);
				   temps.val[k] = (temps.val[k]);
				   cvSet2D(output,i,j,temps);
			  }

		  }
		    // case RS
		  else if(i > 0 && i < (input->height - 1)  && j == (input->width - 1)) {
			  zij_1 =  cvGet2D(input,i,j - 1); // i, j -1
			  zij = cvGet2D(input,i,j); // i,j 
			  zi1j_1 = cvGet2D(input,i + 1,j - 1);// i + 1, j - 1
			  zi_1j = cvGet2D(input,i - 1,j); // i - 1, j
			  zi1j = cvGet2D(input,i + 1,j); // i + 1, j
			  zi_1j_1 =  cvGet2D(input,i - 1,j - 1);// i - 1, j - 1
			               
			  for ( k = 0; k < input->nChannels; k++){
				   d1 = zij_1.val[k] -2*zij.val[k];
				   d2 = 0.5*zi1j_1.val[k] - zij.val[k];
				   d3 = zi_1j.val[k] - 2*zij.val[k] + zi1j.val[k];
			       d4 = 0.5*zi_1j_1.val[k] - zij.val[k];
			  	  temps.val[k] = DerivateHubbertFunction(d1,alpha) + DerivateHubbertFunction(d2,alpha) 
				  + DerivateHubbertFunction(d3,alpha) + DerivateHubbertFunction(d4,alpha);
				   temps.val[k] = (temps.val[k]);
				   cvSet2D(output,i,j,temps);
			  }

		  }
		  // case DS
		  else if(i == (input->height - 1)  && j > 0 && j < (input->width - 1)) {
			  zij_1 =  cvGet2D(input,i,j - 1); // i, j -1
			  zij = cvGet2D(input,i,j); // i,j 
			  zij1 =  cvGet2D(input,i,j + 1);// i, j + 1
			  zi_1j1 =  cvGet2D(input,i - 1,j + 1);// i - 1, j + 1
			  zi_1j = cvGet2D(input,i - 1,j); // i - 1, j
			  zi_1j_1 =  cvGet2D(input,i - 1,j - 1);// i - 1, j - 1
			  
              
			  for ( k = 0; k < input->nChannels; k++){
				   d1 = zij_1.val[k] -2*zij.val[k] + zij1.val[k];
				   d2 = -zij.val[k] + 0.5*zi_1j1.val[k];
				   d3 = zi_1j.val[k] - 2*zij.val[k];
			       d4 = 0.5*zi_1j_1.val[k] - zij.val[k];
			  	  temps.val[k] = DerivateHubbertFunction(d1,alpha) + DerivateHubbertFunction(d2,alpha) 
				  + DerivateHubbertFunction(d3,alpha) + DerivateHubbertFunction(d4,alpha);
				   temps.val[k] = (temps.val[k]);
				   cvSet2D(output,i,j,temps);
			  }
		  }

	  }//end for
    } // end for

  return;
}


extern void HubbertFunctionCliques(IplImage* input, IplImage* output, float alpha)
{
  CvScalar z_input;
  CvScalar z_output;
  CvScalar temps;
  CvScalar   zij_1,zij, zij1, zi1j_1, zi_1j1, zi_1j, zi1j, zi_1j_1, zi1j1;
  int i, j, k;
  float d1, d2, d3, d4;
  double temp;

  for(i = 0; i < input->height; i++){//row
	  for(j = 0; j < input->width; j++){ //col
		  if( i - 1 >= 0 && i + 1 < input->height && j - 1 >= 0 && j + 1 < input->width)
		  {
			  zij_1 =  cvGet2D(input,i,j - 1); // i, j -1
			  zij = cvGet2D(input,i,j); // i,j 
			  zij1 =  cvGet2D(input,i,j + 1);// i, j + 1
			  zi1j_1 = cvGet2D(input,i + 1,j - 1);// i + 1, j - 1
			  zi_1j1 =  cvGet2D(input,i - 1,j + 1);// i - 1, j + 1
			  zi_1j = cvGet2D(input,i - 1,j); // i - 1, j
			  zi1j = cvGet2D(input,i + 1,j); // i + 1, j
			  zi_1j_1 =  cvGet2D(input,i - 1,j - 1);// i - 1, j - 1
			  zi1j1 = cvGet2D(input,i + 1,j + 1); // i + 1, j
              
			  for ( k = 0; k < input->nChannels; k++){
				   d1 = zij_1.val[k] -2*zij.val[k] + zij1.val[k];
				   d2 = 0.5*zi1j_1.val[k] - zij.val[k] + 0.5*zi_1j1.val[k];
				   d3 = zi_1j.val[k] - 2*zij.val[k] + zi1j.val[k];
			       d4 = 0.5*zi_1j_1.val[k] - zij.val[k] + 0.5*zi1j1.val[k];
			  	  temps.val[k] = HubbertFunction(d1,alpha) + HubbertFunction(d2,alpha) 
				  + HubbertFunction(d3,alpha) + HubbertFunction(d4,alpha);
				  // temps.val[k] = (temps.val[k]);
				   cvSet2D(output,i,j,temps);
			  }
		  }
		  // more cases
			  // case P1
		  else if(i == 0 && j == 0){
			  zij = cvGet2D(input,i,j); // i,j 
			  zij1 =  cvGet2D(input,i,j + 1);// i, j + 1
			  zi1j = cvGet2D(input,i + 1,j); // i + 1, j
			  zi1j1 = cvGet2D(input,i + 1,j + 1); // i + 1, j              
			  for ( k = 0; k < input->nChannels; k++){
				   d1 = -2*zij.val[k] + zij1.val[k];
				   d2 =  -zij.val[k];
				   d3 = -2*zij.val[k] + zi1j.val[k];
			       d4 = -zij.val[k] + 0.5*zi1j1.val[k];
			  	  temps.val[k] = HubbertFunction(d1,alpha) + HubbertFunction(d2,alpha) 
				  + HubbertFunction(d3,alpha) + HubbertFunction(d4,alpha);		
				   //temps.val[k] = (temps.val[k]);
				   cvSet2D(output,i,j,temps);
			  }
		  }
		  // case P2
		  else if(i == 0 && j == (input->width - 1)){
			  zij_1 =  cvGet2D(input,i,j - 1); // i, j -1
			  zij = cvGet2D(input,i,j); // i,j 
			  zi1j_1 = cvGet2D(input,i + 1,j - 1);// i + 1, j - 1
			  zi1j = cvGet2D(input,i + 1,j); // i + 1, j		  
			
			  	  for ( k = 0; k < input->nChannels; k++){
				   d1 = zij_1.val[k] -2*zij.val[k];
				   d2 = 0.5*zi1j_1.val[k] - zij.val[k];
				   d3 = -2*zij.val[k] + zi1j.val[k];
			       d4 =  -zij.val[k];
			  	  temps.val[k] = HubbertFunction(d1,alpha) + HubbertFunction(d2,alpha) 
				  + HubbertFunction(d3,alpha) + HubbertFunction(d4,alpha);
			
				   //temps.val[k] = (temps.val[k]);
				   cvSet2D(output,i,j,temps);
			  }

		  }
		   // case P3
		  else if(i == (input->height - 1) && j == (input->width - 1)){
			  zij_1 =  cvGet2D(input,i,j - 1); // i, j -1
			  zij = cvGet2D(input,i,j); // i,j 
			  zi_1j = cvGet2D(input,i - 1,j); // i - 1, j
			  zi_1j_1 =  cvGet2D(input,i - 1,j - 1);// i - 1, j - 1
			  
			
			  	  for ( k = 0; k < input->nChannels; k++){
				   d1 = zij_1.val[k] -2*zij.val[k];
				   d2 =  -zij.val[k];
				   d3 = zi_1j.val[k] - 2*zij.val[k];
			       d4 = 0.5*zi_1j_1.val[k] - zij.val[k];

			  	  temps.val[k] = HubbertFunction(d1,alpha) + HubbertFunction(d2,alpha) 
				  + HubbertFunction(d3,alpha) + HubbertFunction(d4,alpha);
			
				  // temps.val[k] = (temps.val[k]);
				   cvSet2D(output,i,j,temps);
				  }

		  }
		  // case P4
		  else if(i == (input->height - 1) && j == 0){
			  zij = cvGet2D(input,i,j); // i,j 
			  zij1 =  cvGet2D(input,i,j + 1);// i, j + 1
			  zi_1j1 =  cvGet2D(input,i - 1,j + 1);// i - 1, j + 1
			  zi_1j = cvGet2D(input,i - 1,j); // i - 1, j
			             
			  for ( k = 0; k < input->nChannels; k++){
				   d1 = -2*zij.val[k] + zij1.val[k];
				   d2 = -zij.val[k] + 0.5*zi_1j1.val[k];
				   d3 = zi_1j.val[k] - 2*zij.val[k];
			       d4 = zij.val[k];
			  	  temps.val[k] = HubbertFunction(d1,alpha) + HubbertFunction(d2,alpha) 
				  + HubbertFunction(d3,alpha) + HubbertFunction(d4,alpha);
				  // temps.val[k] = (temps.val[k]);
				   cvSet2D(output,i,j,temps);
			  }
		  }
		  // case LS
		  else if(i > 0 && i < (input->height - 1) && j == 0) {
			 
			  zij = cvGet2D(input,i,j); // i,j 
			  zij1 =  cvGet2D(input,i,j + 1);// i, j + 1
			  zi_1j1 =  cvGet2D(input,i - 1,j + 1);// i - 1, j + 1
			  zi_1j = cvGet2D(input,i - 1,j); // i - 1, j
			  zi1j = cvGet2D(input,i + 1,j); // i + 1, j
			  zi1j1 = cvGet2D(input,i + 1,j + 1); // i + 1, j
              
			  for ( k = 0; k < input->nChannels; k++){
				   d1 = -2*zij.val[k] + zij1.val[k];
				   d2 = -zij.val[k] + 0.5*zi_1j1.val[k];
				   d3 = zi_1j.val[k] - 2*zij.val[k] + zi1j.val[k];
			       d4 = -zij.val[k] + 0.5*zi1j1.val[k];
			  	  temps.val[k] = HubbertFunction(d1,alpha) + HubbertFunction(d2,alpha) 
				  + HubbertFunction(d3,alpha) + HubbertFunction(d4,alpha);
				   //temps.val[k] = (temps.val[k]);
				   cvSet2D(output,i,j,temps);
			  }
		  }

		  // case US
		  else if(i == 0 && j > 0 && j < (input->width - 1)) {
			  zij_1 =  cvGet2D(input,i,j - 1); // i, j -1
			  zij = cvGet2D(input,i,j); // i,j 
			  zij1 =  cvGet2D(input,i,j + 1);// i, j + 1
			  zi1j_1 = cvGet2D(input,i + 1,j - 1);// i + 1, j - 1
			  zi1j = cvGet2D(input,i + 1,j); // i + 1, j
			  zi1j1 = cvGet2D(input,i + 1,j + 1); // i + 1, j
              
			  for ( k = 0; k < input->nChannels; k++){
				   d1 = zij_1.val[k] -2*zij.val[k] + zij1.val[k];
				   d2 = 0.5*zi1j_1.val[k] - zij.val[k];
				   d3 = -2*zij.val[k] + zi1j.val[k];
			       d4 = -zij.val[k] + 0.5*zi1j1.val[k];
			  	  temps.val[k] = HubbertFunction(d1,alpha) + HubbertFunction(d2,alpha) 
				  + HubbertFunction(d3,alpha) + HubbertFunction(d4,alpha);
				   //temps.val[k] = (temps.val[k]);
				   cvSet2D(output,i,j,temps);
			  }

		  }
		    // case RS
		  else if(i > 0 && i < (input->height - 1)  && j == (input->width - 1)) {
			  zij_1 =  cvGet2D(input,i,j - 1); // i, j -1
			  zij = cvGet2D(input,i,j); // i,j 
			  zi1j_1 = cvGet2D(input,i + 1,j - 1);// i + 1, j - 1
			  zi_1j = cvGet2D(input,i - 1,j); // i - 1, j
			  zi1j = cvGet2D(input,i + 1,j); // i + 1, j
			  zi_1j_1 =  cvGet2D(input,i - 1,j - 1);// i - 1, j - 1
			               
			  for ( k = 0; k < input->nChannels; k++){
				   d1 = zij_1.val[k] -2*zij.val[k];
				   d2 = 0.5*zi1j_1.val[k] - zij.val[k];
				   d3 = zi_1j.val[k] - 2*zij.val[k] + zi1j.val[k];
			       d4 = 0.5*zi_1j_1.val[k] - zij.val[k];
			  	  temps.val[k] = HubbertFunction(d1,alpha) + HubbertFunction(d2,alpha) 
				  + HubbertFunction(d3,alpha) + HubbertFunction(d4,alpha);
				   //temps.val[k] = (temps.val[k]);
				   cvSet2D(output,i,j,temps);
			  }

		  }
		  // case DS
		  else if(i == (input->height - 1)  && j > 0 && j < (input->width - 1)) {
			  zij_1 =  cvGet2D(input,i,j - 1); // i, j -1
			  zij = cvGet2D(input,i,j); // i,j 
			  zij1 =  cvGet2D(input,i,j + 1);// i, j + 1
			  zi_1j1 =  cvGet2D(input,i - 1,j + 1);// i - 1, j + 1
			  zi_1j = cvGet2D(input,i - 1,j); // i - 1, j
			  zi_1j_1 =  cvGet2D(input,i - 1,j - 1);// i - 1, j - 1
			  
              
			  for ( k = 0; k < input->nChannels; k++){
				   d1 = zij_1.val[k] -2*zij.val[k] + zij1.val[k];
				   d2 = -zij.val[k] + 0.5*zi_1j1.val[k];
				   d3 = zi_1j.val[k] - 2*zij.val[k];
			       d4 = 0.5*zi_1j_1.val[k] - zij.val[k];
			  	  temps.val[k] = HubbertFunction(d1,alpha) + HubbertFunction(d2,alpha) 
				  + HubbertFunction(d3,alpha) + HubbertFunction(d4,alpha);
				   //temps.val[k] = (temps.val[k]);
				   cvSet2D(output,i,j,temps);
			  }
		  }

	  }//end for
    } // end for

  return;
}

extern void FastRobustRegularization(IplImage* Xinit, IplImage* Xregularized, int p, float alpha)
{

	IplImage* Xregularized_temp = NULL;
	int l, m;

	CvSize size_Reg;

	size_Reg = cvGetSize(Xregularized);



	//allocateOnDemand( &Xregularized_temp, size_Reg, IPL_DEPTH_32F, 1 );
	allocateOnDemand( &Xregularized_temp, size_Reg, Xinit->depth, Xinit->nChannels );
	
	cvZero(Xregularized_temp);


        for( l = 0; l < (p + 1);  l++){// changing according with the PhD Dissertation
			for( m = 0; m  < (p + 1); m++){
				printf("\n Working with regularization for l = %d, and m = %d", l, m);
				//Regularizationv4(Xinit, Xregularized_temp, m, l, alpha);
				Regularizationv5(Xinit, Xregularized_temp, m, l, alpha);
				
				//printf("\n Xregularized(height, width) = %d, %d \n Xregularized_temp(height, width) = %d, %d", Xregularized->height,Xregularized->width, Xregularized_temp->height, Xregularized_temp->width);
				//getchar();
			    cvAdd(Xregularized, Xregularized_temp,Xregularized,NULL);	

				//cvNamedWindow( "Xregularized", 1 );
				//cvShowImage( "Xregularized", Xregularized );
				//cvWaitKey(0);

				cvZero(Xregularized_temp);			
			}
		}
		return;
}



extern void Regularizationv4(IplImage* X, IplImage* Xregularized, int m, int l, float alpha)
{
	IplImage* XShifted_l = NULL, *XShifted_m = NULL, *XShifted_lminus = NULL, *XShifted_lminusf = NULL, *XShifted_mminus = NULL;
	IplImage* X_Sub_Shifted = NULL, * X_Sub_Shifted_Signed = NULL;
	IplImage* XX_Sub_Shifted = NULL, *XpaddedR = NULL, *XpaddedRf = NULL;
	IplImage* XShifted_mf = NULL, *X_Sub_Shiftedf = NULL, *X_Sub_Shifted_Signedf = NULL, *XShifted_mminusf = NULL, *XX_Sub_Shiftedf = NULL;
	IplImage* Xregularizedf = NULL, *XX_Sub_Shiftedff = NULL;
	//IplImage* Xregularized_padded = NULL;
	CvSize size, sizeX, size_padded;
   
	int i_test, j_test;
	double temp;
	CvMat *alphaM = NULL;

		
    CvScalar z_XpaddedRf, z_XShifted_mf, z_X_Sub_Shiftedf, z_X_Sub_Shifted_Signedf, z_Xregularized;
	

//	size.width = X->width + l; // x direcction
//	size.height = X->height + m; // y direcction

	size.width = X->width; // x direcction
	size.height = X->height; // y direcction
	
	sizeX.width = X->width; // x direcction
	sizeX.height = X->height; // y direcction

	size_padded.width = Xregularized->width;
	size_padded.height = Xregularized->height;

	allocateOnDemand( &XShifted_l, size, X->depth, X->nChannels );
	allocateOnDemand( &XpaddedR, size, X->depth, X->nChannels );
	allocateOnDemand( &XpaddedRf, size, IPL_DEPTH_32F, 1 );
	allocateOnDemand( &XShifted_m, size, X->depth, X->nChannels );
	allocateOnDemand( &XShifted_mf, size, IPL_DEPTH_32F, 1 );
	allocateOnDemand( &XShifted_lminus, size, X->depth, X->nChannels );
	allocateOnDemand( &XShifted_lminusf, size, IPL_DEPTH_32F, 1 );
	allocateOnDemand( &XShifted_mminus, size, X->depth, X->nChannels );
	allocateOnDemand( &XShifted_mminusf, size, IPL_DEPTH_32F, 1 );
	allocateOnDemand( &X_Sub_Shifted, size, X->depth, X->nChannels );
	allocateOnDemand( &X_Sub_Shiftedf, size, IPL_DEPTH_32F, 1 );
	allocateOnDemand( &X_Sub_Shifted_Signed, size, X->depth, X->nChannels );
	allocateOnDemand( &X_Sub_Shifted_Signedf, size, IPL_DEPTH_32F, 1 );
	allocateOnDemand( &XX_Sub_Shifted, size, X->depth, X->nChannels );
	allocateOnDemand( &XX_Sub_Shiftedf, size, IPL_DEPTH_32F, 1 );
	allocateOnDemand( &XX_Sub_Shiftedff, size, IPL_DEPTH_32F, 1 );
	allocateOnDemand( &Xregularizedf, size, IPL_DEPTH_32F, 1 );
//	allocateOnDemand( &Xregularized_padded, size_padded, ref->depth, ref->nChannels );
	

	printf(" \n Step 1 \n");
	ShiftXdirecction(X, XShifted_l, l);

	//cvNamedWindow( "XShifted_l", 1 );
	//cvShowImage( "XShifted_l", XShifted_l );
	//cvWaitKey(0);

	printf(" Step 2 \n");
	ShiftYdirecction(XShifted_l, XShifted_m, m);
	//cvNamedWindow( "XShifted_m", 1 );
	//cvShowImage( "XShifted_m", XShifted_m );
	//cvWaitKey(0);

	
	//padR allows to do the substraction because the sizes are different
	padR(X, XpaddedR);
	
	//cvNamedWindow( "XpaddedR", 1 );
	//cvShowImage( "XpaddedR", XpaddedR );
	//cvWaitKey(0);
/*
	cvNamedWindow( "XShifted_m", 1 );
	cvShowImage( "XShifted_m", XShifted_m );
	cvWaitKey(0);

	cvSaveImage("XpaddedR.jpg", XpaddedR);
	*/

	printf(" Step 3 \n");

	XpaddedRf = convert_to_gray32(XpaddedR);
	XShifted_mf = convert_to_gray32(XShifted_m);
	
	cvSub(XpaddedRf,XShifted_mf, X_Sub_Shiftedf,NULL);

	//cvNamedWindow( "X_Sub_Shiftedf", 1 );
	//cvShowImage( "X_Sub_Shiftedf", X_Sub_Shiftedf );
	//cvWaitKey(0);
	


	printf(" Step 4 \n");
	
	SignImage(X_Sub_Shiftedf, X_Sub_Shifted_Signedf);

	//cvNamedWindow( "X_Sub_Shifted_Signedf", 1 );
	//cvShowImage( "X_Sub_Shifted_Signedf", X_Sub_Shifted_Signedf );
	//cvWaitKey(0);
	
	i_test = rand()%(size.height);
	j_test = rand()%(size.width);

	z_XpaddedRf = cvGet2D(XpaddedRf,i_test,j_test);  
	z_XShifted_mf = cvGet2D(XShifted_mf,i_test,j_test);
	z_X_Sub_Shiftedf = cvGet2D(X_Sub_Shiftedf,i_test,j_test);
	z_X_Sub_Shifted_Signedf = cvGet2D(X_Sub_Shifted_Signedf,i_test,j_test);

/*
	printf(" i_test = %d \t j_test = %d \n",i_test, j_test);
	printf(" z_XpaddedRf.val[0] = %f \n", z_XpaddedRf.val[0]);
	printf(" z_XShifted_mf.val[0] = %f \n", z_XShifted_mf.val[0]);	
	printf(" z_X_Sub_Shiftedf.val[0] = %f \n", z_X_Sub_Shiftedf.val[0]);
	printf(" z_X_Sub_Shifted_Signedf.val[0] = %f \n", z_X_Sub_Shifted_Signedf.val[0]);
*/
	//getchar();
	
	//cvNamedWindow( "X_Sub_Shifted_Signed", 1 );
	//cvShowImage( "X_Sub_Shifted_Signed", X_Sub_Shifted_Signed );
	//cvWaitKey(0);

	printf("\n Shifting back ...\n");
	printf(" Step 5 \n");
	ShiftXdirecction(X_Sub_Shifted_Signedf, XShifted_lminusf, -l);

	//cvNamedWindow( "XShifted_lminusf", 1 );
	//cvShowImage( "XShifted_lminusf", XShifted_lminusf );
	//cvWaitKey(0);

	printf(" Step 6 \n");	
	ShiftYdirecction( XShifted_lminusf, XShifted_mminusf, -m);

	//cvNamedWindow( "XShifted_mminusf", 1 );
	//cvShowImage( "XShifted_mminusf", XShifted_mminusf );
	//cvWaitKey(0);

	printf(" Step 7 \n");	

	cvSub(X_Sub_Shifted_Signedf,XShifted_mminusf, XX_Sub_Shiftedf, NULL);

	//cvNamedWindow( "XX_Sub_Shiftedf", 1 );
	//cvShowImage( "XX_Sub_Shiftedf", XX_Sub_Shiftedf );
	//cvWaitKey(0);
	
	printf(" Step 8 \n");	
		
	m = fabs(m);
	l = fabs(l);
	temp =  pow(alpha, (m+l));
	//MulImagewithDoublev2(XX_Sub_Shiftedf,Xregularized, pow(alpha, (m+l)));

	alphaM = cvCreateMat(XX_Sub_Shiftedf->height,XX_Sub_Shiftedf->width,CV_32FC1);
	cvSetIdentity(alphaM,cvRealScalar(temp));
	
	cvMatMul(XX_Sub_Shiftedf,alphaM, Xregularized);
	

	//cvNamedWindow( "Xregularizedf", 1 );
	//cvShowImage( "Xregularizedf", Xregularizedf );
	//cvWaitKey(0);

	//padR(Xregularized, Xregularized_padded);

	//printf("\n 3 ...\n");
	//printf("\n Xregularizedf(height,width) = %d, %d\t  \n", Xregularizedf->height, Xregularizedf->width);
	printf("\n Xregularized(height,width) = %d, %d\t  \n", Xregularized->height, Xregularized->width);
	//getchar();

	z_Xregularized = cvGet2D(Xregularized,45,45);
	

	printf(" z_Xregularized.val[0] = %f \n", z_Xregularized.val[0]);

    // cvNamedWindow( "Xregularized", 1 );
	// cvShowImage( "Xregularized", Xregularized );
	// cvWaitKey(0);
	cvReleaseMat(&alphaM);
	
	return;
}

extern void Regularizationv5(IplImage* X, IplImage* Xregularized, int m, int l, double alpha)
{
	IplImage* XShifted_l = NULL, *XShifted_m = NULL, *XShifted_lminus = NULL, *XShifted_lminusf = NULL, *XShifted_mminus = NULL;
	IplImage* X_Sub_Shifted = NULL, * X_Sub_Shifted_Signed = NULL;
	IplImage* XX_Sub_Shifted = NULL, *XpaddedR = NULL, *XpaddedRf = NULL;
	IplImage* XShifted_mf = NULL, *X_Sub_Shiftedf = NULL, *X_Sub_Shifted_Signedf = NULL, *XShifted_mminusf = NULL, *XX_Sub_Shiftedf = NULL;
	IplImage* Xregularizedf = NULL, *XX_Sub_Shiftedff = NULL;
	//IplImage* Xregularized_padded = NULL;
	CvSize size, sizeX, size_padded;
   
	int i_test, j_test;
	double temp;
	//CvMat *alphaM = NULL;

		
    CvScalar z_XpaddedRf, z_XShifted_mf, z_X_Sub_Shiftedf, z_X_Sub_Shifted_Signedf, z_Xregularized;
	

//	size.width = X->width + l; // x direcction
//	size.height = X->height + m; // y direcction

	size.width = X->width; // x direcction
	size.height = X->height; // y direcction
	
	sizeX.width = X->width; // x direcction
	sizeX.height = X->height; // y direcction

	size_padded.width = Xregularized->width;
	size_padded.height = Xregularized->height;

	allocateOnDemand( &XShifted_l, size, X->depth, X->nChannels );
	allocateOnDemand( &XpaddedR, size, X->depth, X->nChannels );
	allocateOnDemand( &XpaddedRf, size, IPL_DEPTH_32F, 1 );
	allocateOnDemand( &XShifted_m, size, X->depth, X->nChannels );
	allocateOnDemand( &XShifted_mf, size, IPL_DEPTH_32F, 1 );
	allocateOnDemand( &XShifted_lminus, size, X->depth, X->nChannels );
	allocateOnDemand( &XShifted_lminusf, size, IPL_DEPTH_32F, 1 );
	allocateOnDemand( &XShifted_mminus, size, X->depth, X->nChannels );
	allocateOnDemand( &XShifted_mminusf, size, IPL_DEPTH_32F, 1 );
	allocateOnDemand( &X_Sub_Shifted, size, X->depth, X->nChannels );
	allocateOnDemand( &X_Sub_Shiftedf, size, IPL_DEPTH_32F, 1 );
	allocateOnDemand( &X_Sub_Shifted_Signed, size, X->depth, X->nChannels );
	allocateOnDemand( &X_Sub_Shifted_Signedf, size, IPL_DEPTH_32F, 1 );
	allocateOnDemand( &XX_Sub_Shifted, size, X->depth, X->nChannels );
	allocateOnDemand( &XX_Sub_Shiftedf, size, IPL_DEPTH_32F, 1 );
	allocateOnDemand( &XX_Sub_Shiftedff, size, IPL_DEPTH_32F, 1 );
	allocateOnDemand( &Xregularizedf, size, IPL_DEPTH_32F, 1 );
//	allocateOnDemand( &Xregularized_padded, size_padded, ref->depth, ref->nChannels );
	

	printf(" \n Step 1 \n");
	ShiftXdirecction(X, XShifted_l, l);

	//cvNamedWindow( "XShifted_l", 1 );
	//cvShowImage( "XShifted_l", XShifted_l );
	//cvWaitKey(0);

	printf(" Step 2 \n");
	ShiftYdirecction(XShifted_l, XShifted_m, m);
	//cvNamedWindow( "XShifted_m", 1 );
	//cvShowImage( "XShifted_m", XShifted_m );
	//cvWaitKey(0);

	
	//padR allows to do the substraction because the sizes are different
	padR(X, XpaddedR);
	
	//cvNamedWindow( "XpaddedR", 1 );
	//cvShowImage( "XpaddedR", XpaddedR );
	//cvWaitKey(0);
/*
	cvNamedWindow( "XShifted_m", 1 );
	cvShowImage( "XShifted_m", XShifted_m );
	cvWaitKey(0);

	cvSaveImage("XpaddedR.jpg", XpaddedR);
	*/

	printf(" Step 3 \n");

	//XpaddedRf = convert_to_gray32(XpaddedR);
	//XShifted_mf = convert_to_gray32(XShifted_m);
	
	cvSub(XpaddedR,XShifted_m, X_Sub_Shifted,NULL);

	//cvNamedWindow( "X_Sub_Shiftedf", 1 );
	//cvShowImage( "X_Sub_Shiftedf", X_Sub_Shiftedf );
	//cvWaitKey(0);
	
	printf(" Step 4 \n");
	
	SignImage(X_Sub_Shifted, X_Sub_Shifted_Signed);

	//cvNamedWindow( "X_Sub_Shifted_Signedf", 1 );
	//cvShowImage( "X_Sub_Shifted_Signedf", X_Sub_Shifted_Signedf );
	//cvWaitKey(0);
	
	/*
	i_test = rand()%(size.height);
	j_test = rand()%(size.width);

	z_XpaddedRf = cvGet2D(XpaddedRf,i_test,j_test);  
	z_XShifted_mf = cvGet2D(XShifted_mf,i_test,j_test);
	z_X_Sub_Shiftedf = cvGet2D(X_Sub_Shiftedf,i_test,j_test);
	z_X_Sub_Shifted_Signedf = cvGet2D(X_Sub_Shifted_Signedf,i_test,j_test);
	*/

/*
	printf(" i_test = %d \t j_test = %d \n",i_test, j_test);
	printf(" z_XpaddedRf.val[0] = %f \n", z_XpaddedRf.val[0]);
	printf(" z_XShifted_mf.val[0] = %f \n", z_XShifted_mf.val[0]);	
	printf(" z_X_Sub_Shiftedf.val[0] = %f \n", z_X_Sub_Shiftedf.val[0]);
	printf(" z_X_Sub_Shifted_Signedf.val[0] = %f \n", z_X_Sub_Shifted_Signedf.val[0]);
*/
	//getchar();
	
	//cvNamedWindow( "X_Sub_Shifted_Signed", 1 );
	//cvShowImage( "X_Sub_Shifted_Signed", X_Sub_Shifted_Signed );
	//cvWaitKey(0);

	printf("\n Shifting back ...\n");
	printf(" Step 5 \n");
	ShiftXdirecction(X_Sub_Shifted_Signed, XShifted_lminus, -l);

	//cvNamedWindow( "XShifted_lminusf", 1 );
	//cvShowImage( "XShifted_lminusf", XShifted_lminusf );
	//cvWaitKey(0);

	printf(" Step 6 \n");	
	ShiftYdirecction( XShifted_lminus, XShifted_mminus, -m);

	//cvNamedWindow( "XShifted_mminusf", 1 );
	//cvShowImage( "XShifted_mminusf", XShifted_mminusf );
	//cvWaitKey(0);

	printf(" Step 7 \n");	

	cvSub(X_Sub_Shifted_Signed,XShifted_mminus, XX_Sub_Shifted, NULL);

	//cvNamedWindow( "XX_Sub_Shiftedf", 1 );
	//cvShowImage( "XX_Sub_Shiftedf", XX_Sub_Shiftedf );
	//cvWaitKey(0);
	
	printf(" Step 8 \n");	
		
	m = fabs(m);
	l = fabs(l);
	temp =  pow(alpha, (m+l));

//	MulImagewithDoublev2(XX_Sub_Shifted,Xregularized, temp);
         Xregularized = MulImagewithDouble(XX_Sub_Shifted, temp);
		 /*
	alphaM = cvCreateMat(XX_Sub_Shiftedf->height,XX_Sub_Shiftedf->width,CV_32FC1);
	cvSetIdentity(alphaM,cvRealScalar(temp));
	
	cvMatMul(XX_Sub_Shiftedf,alphaM, Xregularized);
	*/

	//cvNamedWindow( "Xregularizedf", 1 );
	//cvShowImage( "Xregularizedf", Xregularizedf );
	//cvWaitKey(0);

	//padR(Xregularized, Xregularized_padded);

	//printf("\n 3 ...\n");
	//printf("\n Xregularizedf(height,width) = %d, %d\t  \n", Xregularizedf->height, Xregularizedf->width);
	printf("\n Xregularized(height,width) = %d, %d\t  \n", Xregularized->height, Xregularized->width);
	//getchar();

	z_Xregularized = cvGet2D(Xregularized,45,45);
	

	//printf(" z_Xregularized.val[0] = %f \n", z_Xregularized.val[0]);

//	cvNamedWindow( "Xregularized", 1 );
	//cvShowImage( "Xregularized", Xregularized );
	//cvWaitKey(0);
	//cvReleaseMat(&alphaM);
	
	return;
}

// This function computes the sign of an Image
// It follows the same idea of the Sign function of MATLAB

extern void SignImage(IplImage* Img, IplImage* SignImg)
{
	CvScalar z, zR;
        int i, j;
		int k;

	 for(i = 0; i < Img->height; i++){ //rows
		 for( j = 0; j < Img->width; j++) { //cols
			 z = cvGet2D(Img,i,j);

			 for (k = 0; k < Img->nChannels; k++){
			  if(z.val[k] > 0.0) zR.val[k] = 1;
			  if(z.val[k] == 0.0) zR.val[k] = 0;
			  if(z.val[k] < 0.0) zR.val[k] = 0;
			 }			             
			 cvSet2D(SignImg,i,j,zR);
		 } // end for 
	 } // end for

	return;

}


// ShiftXdirecction(IplImage* Img, IplImage* ShiftImage, int l) 
// Shifts the image Img l pixels in the x direcction
// Be careful with the size of ShiftImage -> have to be the same as Img
// In the algorithm is noted as Sx^l

extern void ShiftXdirecction(IplImage* Img, IplImage* ShiftImage, int l)
{

    int i, j; 

	 for(i = 0; i < Img->height; i++){ //rows
		 for( j = 0; j < Img->width; j++) { //cols
			if((j + l) < ShiftImage->width && (j+l) >= 0){
				// cvSet2D(ShiftImage, i, j,cvGet2D(Img,i,j + l));
				 cvSet2D(ShiftImage, i, j + l,cvGet2D(Img,i,j));
  			} //end of if
		 } // end for 
	 } // end for
	
	return;
}


// ShiftYdirecction(IplImage* Img, IplImage* ShiftImage, int m) 
// Shifts the image Img m pixels in the y direcction
// Be careful with the size of ShiftImage -> has to be the same as Img
// In the algorithm is noted as Sy^m

extern void ShiftYdirecction(IplImage* Img, IplImage* ShiftImage, int m)
{

    int i, j;

	for(i = 0; i < Img->height; i++){ //rows
		 for( j = 0; j < Img->width; j++) { //cols
			if((i + m )< Img->height && (i+m) >= 0 ){
				// cvSet2D(ShiftImage, i , j,cvGet2D(Img,i + m,j));
				 cvSet2D(ShiftImage, i + m , j,cvGet2D(Img,i,j));
  			} //end of if
		 } // end for 
	 } // end for
	
	return;
}

extern IplImage* convert_gray32_to_color( IplImage* gray32 )
{
	IplImage* gray8, * color_img;
	int r, c;

	gray8 = cvCreateImage( cvGetSize(gray32), IPL_DEPTH_8U, 1 );
	color_img = cvCreateImage( cvGetSize(gray32), IPL_DEPTH_8U, 3 );

	// Always there will be one channel
	cvConvertScale( gray32, gray8, 255.0, 0 );
	cvCvtColor( gray8, color_img, CV_GRAY2BGR );
	

	cvReleaseImage( &gray8 );
	return color_img;
}

// extern void padR(IplImage* img, IplImage* imgpadR, Rpixels)
// This functon adds R pixels to the right of the image
extern void padR(IplImage* img, IplImage* imgpadR)
{

	int i, j;
	 for(i = 0; i < img->height; i++){ //rows
		 for( j = 0; j < img->width; j++) { //cols
				if(i < imgpadR->height && j < imgpadR->width)
					{cvSet2D(imgpadR, i , j,cvGet2D(img,i,j));}
  			
		 } // end for 
	 } // end for
	return;
}

// Pad npixels the image
extern IplImage* padcn( IplImage* img, int n)
{
/* padcn reduces n pixels in the columns, at the begging  and at the end
*/

  IplImage* paddedimg = NULL;
  int i, j;
  

  paddedimg = cvCreateImage(cvSize(img->width-2*n,img->height),img->depth,img->nChannels);
  cvZero(paddedimg);
  
 
  for(i = 0; i < paddedimg->height; i++){// rows
	  for(j = 0; j < paddedimg->width; j ++){ // cols
         cvSet2D(paddedimg,i,j,cvGet2D(img,i,j+n));
	  }
  }
 
 return paddedimg;

}

/* padcn reduces n pixels in the columns, at the begging  and at the end
*/
/*
extern IplImage* padcncvg( IplImage* img, int n)
{


  IplImage* paddedimg = NULL;
  int i, j;
  

  paddedimg = cvCreateImage(cvSize(img->width-2*n,img->height),img->depth,img->nChannels);
  cvZero(paddedimg);
  
 
  for(i = 0; i < paddedimg->height; i++){// rows
	  for(j = 0; j < paddedimg->width; j ++){ // cols
         cvSet2D(paddedimg,i,j,cvGet2D(img,i,j+n));
	  }
  }
 
 return paddedimg;

}
*/


extern float ComputeLambda(IplImage* X, float alpha, int p)
{
	int m,l,k,i;
	int size = 2;
	IplImage* Sy_m = NULL;
	IplImage* Sx_l = NULL;
	IplImage* Xlm = NULL;
	float temp;
	float median_lambda;
	float lambdav[10];
	double* lambdap = NULL;
	double* sigma1v = NULL;
	float sigma1;
	
	float lambda = 0.0;

	Sy_m = cvCreateImage(cvGetSize(X),X->depth, X->nChannels);
	Sx_l = cvCreateImage(cvGetSize(X),X->depth, X->nChannels);
	Xlm = cvCreateImage(cvGetSize(X),X->depth, X->nChannels);

	size = (p+1)*(p+1);

	printf("size = %d \n", size);

	lambdap = calloc( size, sizeof( double ) );


    printf("lambda = %f \n", lambda);
    k = 0; 
	for(m = 0; m <= p; m++){
		for(l = 0; l <=p; l++){
		
           ShiftYdirecction(X,Sy_m,m);
		   ShiftXdirecction(Sy_m,Sx_l,l);
			cvSub(X,Sx_l,Xlm,NULL);
			temp = cvNorm(Xlm,NULL,CV_L1,NULL);
			//printf("temp = %f \n", temp);
			
			lambdav[k] = (pow(alpha, (float)(m+l)))*temp;
			lambdap[k] = (pow(alpha, (float)(m+l)))*temp;
			//lambda += (pow(alpha, (float)(m+l)))*temp;
			lambda += lambdav[k];
			//printf("lambda = %f \t lambdav[%d] = %f \n", lambda, k, lambdav[k]);
			printf("lambdap[%d] = %f \t lambdav[%d] = %f \n", k, lambdap[k], k, lambdav[k] );
			k++;
			}
	}
    
   //gsl_sort(lambdav,1,k+1);
	//median_lambda = median_select(&lambdav[0], k+1);
	printf(" k = %d \n", k);
	median_lambda = median_select(lambdap, k);
	printf("median_lambda = %f\n", median_lambda);
	gsl_sort(lambdap, 1, k);
   // printf(" k%2 = %d \n", k%2);
	//printf(" (k+1)%2 = %d \n", (k+1)%2);

	
	//median_lambda = gsl_stats_median_from_sorted_data(lambdap,1, k);
	//printf("lambdap[%d] = %f \t lambdav[%d] = %f \n", 4, lambdap[4], 4, lambdav[4] );
	//printf("median_lambda = %f\n", median_lambda);
	if( k%2 == 0) median_lambda = (lambdap[k/2 -1] + lambdap[k/2 + 1])*0.5;
	else median_lambda = lambdap[(k-1)/2];
	printf("median_lambda final = %f\n", median_lambda);
	
	// Finding sigma1
	sigma1v = calloc( size, sizeof( double ) );
	for(i = 0; i < size; i++){ 
	sigma1v[i] = fabs(lambda - median_lambda);
	}

	gsl_sort(sigma1v, 1, size);
	if( size%2 == 0) sigma1 = (sigma1v[size/2 -1] + sigma1v[size/2 + 1])*0.5;
	else sigma1 = sigma1v[(size-1)/2];
	printf("sigma1 = %f\n", sigma1);
	
	


  free( lambdap );
  free( sigma1v );
  cvReleaseImage(&Sx_l);
  cvReleaseImage(&Sy_m);
  cvReleaseImage(&Xlm);
 

  return lambda;
} 

extern double BilateralRegularization(IplImage* Xinit, int p, float alpha)
{
	int m, l;
	IplImage* Sy_m = NULL;
	IplImage* Sx_l = NULL;
	IplImage* temp = NULL;
	double tempd = 0.0;
	double temp1;
	float alpha_m_l;
	//double result;
	
	allocateOnDemand( &Sy_m, cvGetSize(Xinit), Xinit->depth, Xinit->nChannels);
	allocateOnDemand( &Sx_l, cvGetSize(Xinit), Xinit->depth, Xinit->nChannels);
	allocateOnDemand( &temp, cvGetSize(Xinit), Xinit->depth, Xinit->nChannels);
	Sy_m = cvCreateImage(cvGetSize(Xinit), Xinit->depth, Xinit->nChannels);
	Sx_l = cvCreateImage(cvGetSize(Xinit), Xinit->depth, Xinit->nChannels);
	temp = cvCreateImage(cvGetSize(Xinit), Xinit->depth, Xinit->nChannels);

    
	for(l = 0; l <= p; l++){
	 for( m = 0; m <= p; m++){
		 ShiftYdirecction(Xinit, Sy_m, m);
		 ShiftXdirecction(Sy_m, Sx_l, l);
		 cvSub(Xinit, Sx_l, temp, NULL);
		 temp1 = cvNorm(temp,NULL, CV_L1, NULL);
		 alpha_m_l = pow((double)alpha,(double)(m+l));
		 tempd += alpha_m_l*temp1;
	 }
	}
  //result = tempd;

	return tempd;
}
// This function defines the region S for the case1 in the aim to find the right blending

extern struct RegionS1 FindSRegionS1(struct borderS1 borderS1)
{
 struct RegionS1 S;
 double vP12[3], vPI1[3], vP3S[3], vP4S[3],vP5S[3], vP6S[3], vPI3[3], vP42[3];
 double vR1[3],vR2[3],vR3[3],vR4[3], vR5[3], vR6[3], vR7[3], vR8[3];

// start
 vP12[0] = borderS1.P12.x;
 vP12[1] = borderS1.P12.y;
 vP12[2] = 1;

 vPI1[0] = borderS1.PI1.x;
 vPI1[1] = borderS1.PI1.y;
 vPI1[2] = 1;

 vP3S[0] = borderS1.P3S.x;
 vP3S[1] = borderS1.P3S.y;
 vP3S[2] = 1;

 vP4S[0] = borderS1.P4S.x;
 vP4S[1] = borderS1.P4S.y;
 vP4S[2] = 1;
 
 vP5S[0] = borderS1.P5S.x;
 vP5S[1] = borderS1.P5S.y;
 vP5S[2] = 1;
 
 vP6S[0] = borderS1.P6S.x;
 vP6S[1] = borderS1.P6S.y;
 vP6S[2] = 1;

 vPI3[0] = borderS1.PI3.x;
 vPI3[1] = borderS1.PI3.y;
 vPI3[2] = 1;
 
 vP42[0] = borderS1.P42.x;
 vP42[1] = borderS1.P42.y;
 vP42[2] = 1;

 crossProduct(vR1,vP12,vPI1);
 crossProduct(vR2,vPI1,vP3S);
 crossProduct(vR3,vP3S,vP4S);
 crossProduct(vR4,vP4S,vP5S);
 crossProduct(vR5,vP5S,vP6S);
 crossProduct(vR6,vP6S,vPI3);
 crossProduct(vR7,vPI3,vP42);
 crossProduct(vR8,vP42,vP12);

 S.L1.x = (vR1[0])/(vR1[1]); //a   a*x + b*y + c = 0;
 S.L1.y = (vR1[1])/(vR1[1]);  // b
 S.L1.z = (vR1[2])/(vR1[1]);  //c

 S.L2.x = (vR2[0])/(vR2[1]);
 S.L2.y = (vR2[1])/(vR2[1]);
 S.L2.z = (vR2[2])/(vR2[1]);

 S.L3.x = (vR3[0])/(vR3[1]);
 S.L3.y = (vR3[1])/(vR3[1]);
 S.L3.z = (vR3[2])/(vR3[1]);

 S.L4.x = (vR4[0])/(vR4[1]);
 S.L4.y = (vR4[1])/(vR4[1]);
 S.L4.z = (vR4[2])/(vR4[1]);

 S.L5.x = (vR5[0])/(vR5[1]);
 S.L5.y = (vR5[1])/(vR5[1]);
 S.L5.z = (vR5[2])/(vR5[1]);
 
 S.L6.x = (vR6[0])/(vR6[1]);
 S.L6.y = (vR6[1])/(vR6[1]);
 S.L6.z = (vR6[2])/(vR6[1]);
 
 S.L7.x = (vR7[0])/(vR7[1]);
 S.L7.y = (vR7[1])/(vR7[1]);
 S.L7.z = (vR7[2])/(vR7[1]);
 
 S.L8.x = (vR8[0])/(vR8[1]);
 S.L8.y = (vR8[1])/(vR8[1]);
 S.L8.z = (vR8[2])/(vR8[1]);

  return S;
}

extern struct borderS1 FindBorderS1(struct image_border ROI_frame1, struct image_border ROI_frame2)
{
    struct borderS1 S1;
	double Line_P11_P21[3], Line_P21_P31[3];
	double Line_P12_P22[3], Line_P32_P42[3];
	double vector_P11[3], vector_P21[3], vector_P31[3], vector_P41[3];
	double vector_P12[3], vector_P22[3], vector_P32[3], vector_P42[3];
	double PI1[3], PI3[3];

	int i;

	float delta_x, delta_y;
	float delta1[2], delta2[2], delta3[2], delta4[2];
	float P3S[2], P4S[2], P5S[2], P6S[2];
	
	CvPoint P_3S, P_4S, P_5S, P_6S;
	CvPoint P_I1, P_I3, P_11, P_21, P_31, P_41, P_12, P_22, P_32, P_42;
	
// Finding the mask that will be used in the blending

	 vector_P11[0] = ROI_frame1.P1.x;
	 vector_P11[1] = ROI_frame1.P1.y;
	 vector_P11[2] = 1;

	 vector_P21[0] = ROI_frame1.P2.x;
	 vector_P21[1] = ROI_frame1.P2.y;
	 vector_P21[2] = 1;

	 vector_P31[0] = ROI_frame1.P3.x;
	 vector_P31[1] = ROI_frame1.P3.y;
	 vector_P31[2] = 1;


	 vector_P12[0] = ROI_frame2.P1.x;
	 vector_P12[1] = ROI_frame2.P1.y;
	 vector_P12[2] = 1;

	 vector_P22[0] = ROI_frame2.P2.x;
	 vector_P22[1] = ROI_frame2.P2.y;
	 vector_P22[2] = 1;

	 vector_P32[0] = ROI_frame2.P3.x;
	 vector_P32[1] = ROI_frame2.P3.y;
	 vector_P32[2] = 1;

	 vector_P42[0] = ROI_frame2.P4.x;
	 vector_P42[1] = ROI_frame2.P4.y;
	 vector_P42[2] = 1;


	 /***********************************************
	 *     Lines
	 ***************************************************/

     crossProduct(Line_P11_P21, vector_P11,vector_P21);
	//Normalizing 
	 for(i = 0; i < 3; i++) Line_P11_P21[i] = Line_P11_P21[i]/Line_P11_P21[2];

	 crossProduct(Line_P21_P31, vector_P21,vector_P31);
	//Normalizing 
	 for(i = 0; i < 3; i++) Line_P21_P31[i] = Line_P21_P31[i]/Line_P21_P31[2];

	 crossProduct(Line_P12_P22, vector_P12,vector_P22);
	 //Normalizing
	 for(i = 0; i < 3; i++) Line_P12_P22[i] = Line_P12_P22[i]/Line_P12_P22[2];

	  crossProduct(Line_P32_P42, vector_P32,vector_P42);
	 //Normalizing
	 for(i = 0; i < 3; i++) Line_P32_P42[i] = Line_P32_P42[i]/Line_P32_P42[2];

	 /***********************************************
	 *     Intersections
	 ***************************************************/

	 crossProduct(PI1, Line_P11_P21, Line_P12_P22);
	 //Normalizing
	 for(i = 0; i < 3; i++) PI1[i] = PI1[i]/PI1[2];

	 crossProduct(PI3, Line_P21_P31, Line_P32_P42);
	 //Normalizing
	 for(i = 0; i < 3; i++) PI3[i] = PI3[i]/PI3[2];

	 P_11.x = ROI_frame1.P1.x;
	 P_21.x = ROI_frame1.P2.x;
	 P_31.x = ROI_frame1.P3.x;
	 P_41.x = ROI_frame1.P4.x;

	 P_11.y = ROI_frame1.P1.y;
	 P_21.y = ROI_frame1.P2.y;
	 P_31.y = ROI_frame1.P3.y;
	 P_41.y = ROI_frame1.P4.y;

	 P_12.x = ROI_frame2.P1.x;
	 P_22.x = ROI_frame2.P2.x;
	 P_32.x = ROI_frame2.P3.x;
	 P_42.x = ROI_frame2.P4.x;

	 P_12.y = ROI_frame2.P1.y;
	 P_22.y = ROI_frame2.P2.y;
	 P_32.y = ROI_frame2.P3.y;
	 P_42.y = ROI_frame2.P4.y;
	 
	P_I1.x = PI1[0];
	P_I1.y = PI1[1];

	delta_y = 30; // 10 pixels 
	delta_x = 30; // 10 pixels

	P_I3.x = PI3[0];
	P_I3.y = PI3[1];

	// Moving the I3 point to have a better blending mask
  // PI3[0] = PI3[0] - delta_x; 
 //  PI3[1] = PI3[1]; 

   /***********************************************
   *     Finding the points P3S, P4S, P5S and P6S
   ***************************************************/



    delta1[0] = 0;
	delta1[1] = delta_y;

	delta2[0] = delta_x;
	delta2[1] = delta_y;

	delta3[0] = delta_x;
	delta3[1] = -delta_y;

	delta4[0] = -delta_x;// it was 0
	delta4[1] = -delta_y;
   
	P3S[0] =  PI1[0] + delta1[0];
	P3S[1] =  PI1[1] + delta1[1];

	P4S[0] =  P_11.x + delta2[0];
	P4S[1] =  P_11.y + delta2[1];

	P5S[0] =  P_42.x + delta3[0];
	P5S[1] =  P_42.y + delta3[1];

	P6S[0] =  PI3[0] + delta4[0];
	P6S[1] =  PI3[1] + delta4[1];

	P_3S.x = P3S[0];
	P_3S.y = P3S[1];

	P_4S.x = P4S[0];
	P_4S.y = P4S[1];

	P_5S.x = P5S[0];
	P_5S.y = P5S[1];

	P_6S.x = P6S[0];
	P_6S.y = P6S[1];

	S1.P12.x = ROI_frame2.P1.x;
	S1.P12.y = ROI_frame2.P1.y;
	S1.P3S.x = P3S[0];
	S1.P3S.y = P3S[1];
	S1.P42.x = ROI_frame2.P4.x;
	S1.P42.y = ROI_frame2.P4.y;
	S1.P4S.x = P4S[0];
	S1.P4S.y = P4S[1];
	S1.P5S.x = P5S[0];
	S1.P5S.y = P5S[1];
	S1.P6S.x = P6S[0];
	S1.P6S.y = P6S[1];
	S1.PI1.x = PI1[0];
	S1.PI1.y = PI1[1];
	S1.PI3.x = PI3[0];
	S1.PI3.y = PI3[1];
	
	return S1;

}

extern int Belongs2S1(CvPoint2D32f point, struct RegionS1 S1, struct borderS1 bS1)
{
  int result = 0;
  float xq2; // is the x coordinates of the interception of the y value in the Line L2
  float xq6; // is the x coordinates of the interception of the y value in the Line L4
  float yq2; // is the y coordinates of the interception of the x value in the Line L2
  float yq4; // is the y coordinates of the interception of the x value in the Line L4
  float yq1; // is the y coordinates of the interception of the x value in the Line L1
  float yq3; // is the y coordinates of the interception of the x value in the Line L3
  float yq5; // is the y coordinates of the interception of the x value in the Line L5
  float yq6; // is the y coordinates of the interception of the x value in the Line L6
  float yq7; // is the y coordinates of the interception of the x value in the Line L7
  float yq8; // is the y coordinates of the interception of the x value in the Line L8
  float yq9; // is the y coordinates of the interception of the x value in the Line L42P5S
  float yq10; // is the y coordinates of the interception of the x value in the Line L4SP3S
  

  float x, y; // the x and y coordinates of the point

  float vLP5S_P42[3], vLP4S_P3S[3];
  float LP5S_P42[3], LP4S_P3S[3];
  float vP5S[3], vP42[3], vP3S[3], vP4S[3];

  int state = 0;

  vP5S[0] = bS1.P5S.x;
  vP5S[1] = bS1.P5S.y;
  vP5S[2] = 1.0;

  vP42[0] = bS1.P42.x;
  vP42[1] = bS1.P42.y;
  vP42[2] = 1.0;

  x = point.x;
  y = point.y;

  crossProduct(vLP5S_P42,vP42, vP5S);
  LP5S_P42[0] = vLP5S_P42[0] / vLP5S_P42[2];
  LP5S_P42[1] = vLP5S_P42[1] / vLP5S_P42[2];
  LP5S_P42[2] = vLP5S_P42[2] / vLP5S_P42[2];

  vP3S[0] = bS1.P3S.x;
  vP3S[1] = bS1.P3S.y;
  vP3S[2] = 1.0;

  vP4S[0] = bS1.P4S.x;
  vP4S[1] = bS1.P4S.y;
  vP4S[2] = 1.0;

  crossProduct(vLP4S_P3S,vP4S, vP3S);
  LP4S_P3S[0] = vLP4S_P3S[0] / vLP4S_P3S[2];
  LP4S_P3S[1] = vLP4S_P3S[1] / vLP4S_P3S[2];
  LP4S_P3S[2] = vLP4S_P3S[2] / vLP4S_P3S[2];

  S1.L9.x = LP5S_P42[0];
  S1.L9.y = LP5S_P42[1];
  S1.L9.z = LP5S_P42[2];

  S1.L10.x = LP4S_P3S[0];
  S1.L10.y = LP4S_P3S[1];
  S1.L10.z = LP4S_P3S[2];

  
  xq2 = - (S1.L2.z)/(S1.L2.x) - ((S1.L2.y)/(S1.L2.x))*(point.y);
  xq6 = - (S1.L6.z)/(S1.L6.x) - ((S1.L6.y)/(S1.L6.x))*(point.y);
   yq1 = - (S1.L1.z)/(S1.L1.y) - ((S1.L1.x)/(S1.L1.y))*(point.x);
   yq3 = - (S1.L3.z)/(S1.L3.y) - ((S1.L3.x)/(S1.L3.y))*(point.x);
   yq2 = - (S1.L2.z)/(S1.L2.y) - ((S1.L2.x)/(S1.L2.y))*(point.x);
   yq4 = - (S1.L4.z)/(S1.L4.y) - ((S1.L4.x)/(S1.L4.y))*(point.x);
   yq5 = - (S1.L5.z)/(S1.L5.y) - ((S1.L5.x)/(S1.L5.y))*(point.x);
   yq6 = - (S1.L6.z)/(S1.L6.y) - ((S1.L6.x)/(S1.L6.y))*(point.x);
   yq7 = - (S1.L7.z)/(S1.L7.y) - ((S1.L7.x)/(S1.L7.y))*(point.x);
   yq8 = - (S1.L8.z)/(S1.L8.y) - ((S1.L8.x)/(S1.L8.y))*(point.x);
   yq9 = - (S1.L9.z)/(S1.L9.y) - ((S1.L9.x)/(S1.L9.y))*(point.x);
   yq10 = - (S1.L10.z)/(S1.L10.y) - ((S1.L10.x)/(S1.L10.y))*(point.x);
/*
   printf(" yq1 = %f \n", yq1);
   printf(" xq2 = %f \n", bS1.PI1.x );
   printf(" yq3 = %f \n", yq3);
   printf(" yq4 = %f \n", yq4);
   printf(" yq5 = %f \n", yq5);
   printf(" xq6 = %f \n", bS1.PI3.x );
   printf(" yq7 = %f \n", yq7);
   printf(" yq8 = %f \n", yq8);
   printf(" yq9 = %f \n", yq9);
   printf(" yq10 = %f \n", yq10);
   */
 //  printf("\ state %d \n", state); 

  /*
   if( x >= bS1.P12.x && x <= MAX(bS1.PI1.x, bS1.P3S.x) && y >= MIN(bS1.P12.y, bS1.PI1.y) && y <= bS1.P4S.y)
	   state = 0;
   else if( x > bS1.P12.x && x <= bS1.P5S.x && y > bS1.P12.y && y <= bS1.P42.y )
	   state = 1;
   else if( x >= bS1.P42.x && x <= bS1.P6S.x && y >= MIN(bS1.P5S.y, bS1.P6S.y) && y <= MAX(bS1.P42.y, bS1.PI3.y))
	   state = 2;
   else 
	   state = 3;

 */

  if( x >= bS1.P12.x && x <= MAX(bS1.PI1.x, bS1.P3S.x) && y >=  yq1 && y <= yq3 && y <= yq10 && y <= yq8 )
	   state = 0;
   else if( x > bS1.P12.x && x <= bS1.P5S.x && y > yq10 &&  y >= yq4 && y <= yq9 && y <= yq8 )
	   state = 1;
   //else if( x >= bS1.P42.x && x < bS1.P6S.x &&  y >= yq5  && y <= yq7 && y > yq9)
  else if( x >= bS1.P42.x &&  y > yq6 && y >= yq5  && y <= yq7 && y > yq9)
	   state = 2;
   else 
	   state = 3;

   switch(state) {
		case 0: {
		//	 if( y >  yq1 && x <= bS1.PI1.x && y < yq3 && y < yq10 && y < yq8 && x > bS1.P12.x ){
   				result = 1;
		//	 }
		
			break;
				}
		case 1: {
			// if( y > yq10 &&  y > yq4 && y < yq9 && y < yq8){
			  result = 1;//
			//	}
			break;
				}
		case 2: {
			//  if( y > yq5  && y < yq7 && x <= bS1.PI3.x && y > yq9){
			result = 1;
			//  }//
			break;
				}
        case 3: {
			result = 0;
			break;
				}
   }
 
/*
   if( y > yq5  && y < yq7 && x <= bS1.PI3.x && y > yq9){
   result = 1;//case_s1 = 3; 
   }
   else{
   result = 0;
   }
*/
  return result; // 0: the point doesn't belong to S, and 1 otherwise
}

extern IplImage** build_gaussb_pyr_blending( IplImage* base, int levels )
{
	IplImage** gaussb_pyr;
	int i, o;
	

    //levels = log( MIN( base->width, base->height ) ) / log(2) + 1;

	gaussb_pyr = calloc( levels, sizeof( IplImage** ) );
	for( i = 0; i < levels; i++ )
		gaussb_pyr[i] = calloc( levels, sizeof( IplImage* ) );

	
	for( o = 0; o < levels; o++ )
		{
			if( o == 0 )
				gaussb_pyr[o] = cvCloneImage(base);

			 else 
			 {  
			
				gaussb_pyr[o] = downsample_blending( gaussb_pyr[o-1] );				
				//cvSmooth(gaussb_pyr[o],gaussb_pyr[o], CV_GAUSSIAN, 5, 5, 0, 0 );
			 }
	   }
	
	return gaussb_pyr;
}

extern IplImage** build_laplace_lpyr_blending( IplImage* base, int levels )
{
	IplImage** laplace_pyr;
	IplImage** gaussb_pyr;
	IplImage* temp;
	int i, o;
	int levels2;
	int r, c, n; // to compute the dimmensions of each level
	CvScalar s;

    //levels2 = levels;
		
	//gaussb_pyr = build_gaussb3_pyr( base );

	gaussb_pyr = build_gaussb_pyr_blending(base, levels);

	laplace_pyr = calloc( levels, sizeof( IplImage** ) );
	for( i = 0; i < levels; i++ ){
		laplace_pyr[i] = cvCreateImage( cvGetSize(gaussb_pyr[i]), base->depth, base->nChannels);
	}

	

	for( o = 0; o < levels - 1; o++ )
		{
				//r = gaussb_pyr[o]->height;
				//c = gaussb_pyr[o]->width;
				//printf("\ r = %d \t c =  %d \n", r,c);
				temp = cvCreateImage(cvGetSize(gaussb_pyr[o]),base->depth, base->nChannels);
			    //temp = expandl2(gaussb_pyr[ o + 1 ],r,c);
				temp = expand_blending(gaussb_pyr[o+1]);
				//printf("\n expandl(gaussb_pyr[o+1]) is %d x %d x %d", temp->height , temp ->width, temp ->nChannels);
				//printf("\n gaussb_pyr[o] is %d x %d x %d", gaussb_pyr[o]->height , gaussb_pyr[o] ->width, gaussb_pyr[o] ->nChannels);
				//printf("\n laplace_pyr[o] is %d x %d x %d", laplace_pyr[o]->height , laplace_pyr[o] ->width, laplace_pyr[o] ->nChannels);
				cvSub(gaussb_pyr[o],temp,laplace_pyr[o],NULL );
				cvReleaseImage( &temp );
	   }

	laplace_pyr[levels-1] = cvCloneImage(gaussb_pyr[levels-1]);

	releaseb_pyr( &gaussb_pyr, levels );
	 
	return laplace_pyr;
}

extern IplImage* BlendImageswithMask(IplImage* im1, IplImage* im2, IplImage* im3, int levels)
{
	int i,j,k,l;
	CvScalar s,t,u,v,w,x,y,z;
	IplImage** laplace_pyr_im1;
	IplImage** gaussb_pyr_im1;
	IplImage** laplace_pyr_im2;
	IplImage** gaussb_pyr_im2;
	IplImage** laplace_pyr_im3;
	IplImage** gaussb_pyr_im3;
	IplImage** gaussb_pyr_GRN;
	IplImage** laplace_pyr_C;
	IplImage* C = NULL;
	CvSize size_img;


	gaussb_pyr_im1 = build_gaussb_pyr_blending(im1, levels);
	gaussb_pyr_im2 = build_gaussb_pyr_blending(im2, levels);
	gaussb_pyr_im3 = build_gaussb_pyr_blending(im3, levels);

	laplace_pyr_im1 = build_laplace_lpyr_blending(im1, levels);
	laplace_pyr_im2 = build_laplace_lpyr_blending(im2, levels);
	laplace_pyr_im3 = build_laplace_lpyr_blending(im3, levels);

	for(l = 0; l < gaussb_pyr_im3[0]->nChannels; l++) s.val[l] = 1;
	printf(" I am here 1\n");



	gaussb_pyr_GRN = calloc( levels, sizeof( IplImage** ) );
	for( k = 0; k < levels; k++ ){
		size_img.height = gaussb_pyr_im3[k]->height;
		size_img.width = gaussb_pyr_im3[k]->width;
		gaussb_pyr_GRN[k] = cvCreateImage( size_img, im3->depth, im3->nChannels);
	}
/*
	gaussb_pyr_GRN = calloc( levels, sizeof( IplImage** ) );
	for( k = 0; k < levels; k++ ){
		gaussb_pyr_GRN[k] = calloc( levels, sizeof( IplImage* ) );
	}
	*/
    
	// The following code does GRN = 1 - GR; in the MATLAB code
printf(" I am here 2\n");
	for ( l = 0; l < levels; l++){
		for(i = 0; i < gaussb_pyr_im3[l]->height; i++){
			for(j = 0; j < gaussb_pyr_im3[l]->width; j++){
				//printf(" I am here 2A\n");
				u = cvGet2D(gaussb_pyr_im3[l],i,j);
				for(k = 0; k < gaussb_pyr_im3[k]->nChannels; k++) t.val[k] = s.val[k] - u.val[k];
				//printf(" I am here 2B\n");
                cvSet2D(gaussb_pyr_GRN[l],i,j,t);
			}
		}
	}
/******************************************************************
  MATLAB:
	for i = 1:level
    LC(:,:,i) = GR(:,:,i) .* LA(:,:,i) + GRN(:,:,i) .* LB(:,:,i);
    end
********************************************************************/
printf(" I am here 3\n");
	laplace_pyr_C = calloc( levels, sizeof( IplImage** ) );
	for( k = 0; k < levels; k++ ){
		size_img.height = gaussb_pyr_im1[k]->height;
		size_img.width = gaussb_pyr_im1[k]->width;
		laplace_pyr_C[k] = cvCreateImage( size_img , im1->depth, im1->nChannels);
	}
	for ( l = 0; l < levels; l++){
		for(i = 0; i < laplace_pyr_C[l]->height; i++){
			for(j = 0; j < laplace_pyr_C[l]->width; j++){
				v = cvGet2D(gaussb_pyr_im3[l],i,j); //GR
				w = cvGet2D(laplace_pyr_im1[l],i,j);//LA
				x = cvGet2D(gaussb_pyr_GRN[l],i,j); //GRN
				y = cvGet2D(laplace_pyr_im2[l],i,j); //LB
				for(k = 0; k < gaussb_pyr_im3[k]->nChannels; k++) z.val[k] = v.val[k] * w.val[k] + x.val[k] * y.val[k];
				cvSet2D(laplace_pyr_C[l],i,j,z);
			}
		}
	}

/**************************************************
  MATLAB:
	C = reconstruct(LC);
*****************************************************/
	size_img.height = im1->height;
	size_img.width = im1->width;
	C = cvCreateImage(size_img,im1->depth, im1->nChannels);

	printf(" doing the reconstruction ...\n");
    C = ReconstructPyramid(laplace_pyr_C, levels);
	printf(" reconstruction done ...\n");
      
  releaseb_pyr( &gaussb_pyr_im1, levels );
  releaseb_pyr( &gaussb_pyr_im2, levels );
  releaseb_pyr( &gaussb_pyr_im3, levels );
  releaseb_pyr( &laplace_pyr_im1, levels );
  releaseb_pyr( &laplace_pyr_im2, levels );
  releaseb_pyr( &laplace_pyr_im3, levels );
   releaseb_pyr( &gaussb_pyr_GRN, levels );
  releaseb_pyr( &laplace_pyr_C, levels );

  return C;
}

extern IplImage* ReconstructPyramid(IplImage** laplace_pyr, int levels)
{
  IplImage* C = NULL;
  IplImage* out_expand = NULL;
  IplImage* out = NULL;
  int m,n, i, j, k, l;
  double s;
  CvScalar t,u,v,w,x,y,z;
  CvSize size_out, size_C, size_out_expand;

/**************************************************
  MATLAB:
	[m,n, level] = size(input_pyramid);
    s = 1/power(2,level-1);
	out = input_pyramid(1:m*s,1:n*s,level);

	This part of the code just clone the last level of the pyramid
*****************************************************/
  m = laplace_pyr[0]->height;
  n = laplace_pyr[0]->width;
  s = 1 / pow(2, levels - 1);
/*
  size_out.height = floor(m*s);
  size_out.width = floor(n*s);

  out = cvCreateImage(size_out, laplace_pyr[0]->depth, laplace_pyr[0]->nChannels);

  for(i = 0; i < m*s; i++){
	  for(j = 0; j < n*s; j++){
		  cvSet2D(out, i, j, cvGet2D(laplace_pyr[levels],i,j));
	  }
  }
 */
  
  size_out.height =  laplace_pyr[levels-1]->height;
  size_out.width = laplace_pyr[levels-1]->width;
  out = cvCreateImage(size_out, laplace_pyr[0]->depth, laplace_pyr[0]->nChannels);
  out = cvCloneImage(laplace_pyr[levels-1]);

/**************************************************
  MATLAB:
	for i = level-1:-1:1
    s = 1/power(2,i-1);
    out = expand(out) + input_pyramid(1:m*s,1:n*s,i);      
	end
*****************************************************/

  for(l = levels - 2; l >= 0; l--){
	  //s = 1 / pow(2, l - 1);
	  printf(" level = %d", l);
	  size_out_expand.height = laplace_pyr[l]->height;
	  size_out_expand.width = laplace_pyr[l]->width;
	  out_expand = cvCreateImage(size_out_expand, laplace_pyr[0]->depth, laplace_pyr[0]->nChannels);
	  out_expand = expand_blending(out);
	  size_C.height = out_expand->height;
	  size_C.width = out_expand->width;
	  C = cvCreateImage(size_C, laplace_pyr[0]->depth, laplace_pyr[0]->nChannels);
	  for( i = 0; i < out_expand->height; i++){
		  for(j = 0; j < out_expand->width; j++){
			t = cvGet2D(out_expand, i, j); //expand(out)
			u = cvGet2D(laplace_pyr[l],i,j); //input_pyramid(1:m*s,1:n*s,i); 
			for(k = 0; k < laplace_pyr[0]->nChannels; k++) v.val[k] = t.val[k] + u.val[k];// adding all the channels
            cvSet2D(C,i,j,v);  //out
		  }
	  }
	  size_out.height = C->height;
	  size_out.width = C->width;
	  out = cvCreateImage(size_out, laplace_pyr[0]->depth, laplace_pyr[0]->nChannels);
	  out = cvCloneImage(C);
	  cvReleaseImage(& C );      
  }

  
  cvReleaseImage(& out_expand);

  return out;

}
extern IplImage* expand_blending(IplImage* input)
{ 


	int i, j, k, l;
	CvMat* Kernel;
	CvSize size_output;
	IplImage *output = NULL;
	IplImage *output_conv = NULL;
	//IplImage *temp = NULL;
	CvScalar t,u,v;

/**************************************************
  MATLAB:
		[m,n] = size(input);
		out(2*m,2*n) = 0;
		out(1:2:2*m, 1:2:2*n) = input;
*****************************************************/
	//printf(" %A input (%d, %d) \n", 2*(input->height), 2*(input->width));		 

	size_output.height = 2*input->height;
	size_output.width = 2*input->width;

	output = cvCreateImage(size_output, input->depth, input->nChannels);

/*
	for(i = 0; i < input->height; i++ ){
		for(j = 0; j < input->width; j++){
			cvSet2D(output,2*i, 2*j, cvGet2D(input,i,j));
		}

	}
*/
	for(i = 0; i < output->height; i+=2 ){
		for(j = 0; j < output->width; j+=2){
			cvSet2D(output,i, j, cvGet2D(input,i/2,j/2));
		}

	}
	/*
	//t = cvGet2D(input,10,9);
	//u = cvGet2D(input,9,10);
	//printf(" input(10,9) = %f\t input(9,10) = %f", t.val[0], u.val[0]);
	//getchar();

	allocateOnDemand(&temp, size_output, input->depth, input->nChannels);
	temp = convert_gray32_to_color_blending(output);

	cvNamedWindow( "output", 1 );
	cvShowImage( "output", temp );
	cvWaitKey(0);
	cvSaveImage("output.jpg",temp);
	*/


	Kernel = cvCreateMat(5,5,CV_32FC1);

	cvmSet(Kernel, 0,0, 0.0025);
	cvmSet(Kernel, 0,1, 0.0125);
	cvmSet(Kernel, 0,2, 0.02);
	cvmSet(Kernel, 0,3, 0.0125);
	cvmSet(Kernel, 0,4, 0.0025);
	cvmSet(Kernel, 1,0, 0.0125);
	cvmSet(Kernel, 1,1, 0.0625);
	cvmSet(Kernel, 1,2, 0.1);
	cvmSet(Kernel, 1,3, 0.0625);
	cvmSet(Kernel, 1,4, 0.0125);
	cvmSet(Kernel, 2,0, 0.02);
	cvmSet(Kernel, 2,1, 0.1);
	cvmSet(Kernel, 2,2, 0.16);
	cvmSet(Kernel, 2,3, 0.1);
	cvmSet(Kernel, 2,4, 0.02);
	cvmSet(Kernel, 3,0, 0.0125);
	cvmSet(Kernel, 3,1, 0.0625);
	cvmSet(Kernel, 3,2, 0.1);
	cvmSet(Kernel, 3,3, 0.0625);
	cvmSet(Kernel, 3,4, 0.0125);
	cvmSet(Kernel, 4,0, 0.0025);
	cvmSet(Kernel, 4,1, 0.0125);
	cvmSet(Kernel, 4,2, 0.02);
	cvmSet(Kernel, 4,3, 0.0125);
	cvmSet(Kernel, 4,4, 0.0025);


	output_conv = cvCreateImage(size_output,input->depth, input->nChannels);
	cvFilter2D(output, output_conv, Kernel, cvPoint(-1,-1));

	output_conv = MulImagewithDouble(output_conv, 4.0);

	
  cvReleaseImage( &output );
  cvReleaseMat( &Kernel );

	
  return output_conv;

   

}

void speedy_convolution( const CvMat* A, // Size: M1xN1
						 const CvMat* B, //Size: M2xN2
						 CvMat* C // size ( A->rows+B->rows-1 )x( A->cols+B->cols-1 )
						 )
{

    int dft_M = cvGetOptimalDFTSize( A->rows+B->rows-1 );
    int dft_N = cvGetOptimalDFTSize( A->cols+B->cols-1 );

    CvMat* dft_A = cvCreateMat( dft_M, dft_N, A->type );
    CvMat* dft_B = cvCreateMat( dft_M, dft_N, B->type );
    CvMat tmp;

    // copy A to dft_A and pad dft_A with zeros
    //
    cvGetSubRect( dft_A, &tmp, cvRect(0,0,A->cols,A->rows));
    cvCopy( A, &tmp, 0 );
    cvGetSubRect( 
      dft_A,
      &tmp,
      cvRect( A->cols, 0, dft_A->cols-A->cols, A->rows )
    );
    cvZero( &tmp );

    // no need to pad bottom part of dft_A with zeros because of
    // use nonzero_rows parameter in cvDFT() call below
    //
    cvDFT( dft_A, dft_A, CV_DXT_FORWARD, A->rows );

    // repeat the same with the second array
    //
    cvGetSubRect( dft_B, &tmp, cvRect(0,0,B->cols,B->rows) );
    cvCopy( B, &tmp, 0 );
    cvGetSubRect(
      dft_B, 
      &tmp, 
      cvRect( B->cols, 0, dft_B->cols-B->cols, B->rows )
    );
    cvZero( &tmp );

    // no need to pad bottom part of dft_B with zeros because of
    // use nonzero_rows parameter in cvDFT() call below
    //
    cvDFT( dft_B, dft_B, CV_DXT_FORWARD, B->rows );

    // or CV_DXT_MUL_CONJ to get correlation rather than convolution 
    //
    cvMulSpectrums( dft_A, dft_B, dft_A, 0 );

    // calculate only the top part
    //
    cvDFT( dft_A, dft_A, CV_DXT_INV_SCALE, C->rows ); 
    cvGetSubRect( dft_A, &tmp, cvRect(0,0,C->cols,C->rows) );

    cvCopy( &tmp, C, 0 );
	cvReleaseMat( dft_A );
	cvReleaseMat( dft_B );
}


extern IplImage* convert2gray32( IplImage* img )
{
	IplImage* gray8, * gray32;
	int r, c;

	gray8 = cvCreateImage( cvGetSize(img), IPL_DEPTH_8U, 1 );
	gray32 = cvCreateImage( cvGetSize(img), IPL_DEPTH_32F, 1 );

	if( img->nChannels == 1 )
		gray8 = cvClone( img );
	else
		cvCvtColor( img, gray8, CV_RGB2GRAY );
	cvConvertScale( gray8, gray32, 1.0, 0 );

	cvReleaseImage( &gray8 );
	return gray32;
}

extern IplImage* convert_gray32_to_color_blending( IplImage* gray32 )
{
	IplImage* gray8, * color_img;
	int r, c;

	gray8 = cvCreateImage( cvGetSize(gray32), IPL_DEPTH_8U, 1 );
	color_img = cvCreateImage( cvGetSize(gray32), IPL_DEPTH_8U, 3 );

	// Always there will be one channel
	cvConvertScale( gray32, gray8, 1.0, 0 );
	cvCvtColor( gray8, color_img, CV_GRAY2BGR );
	

	cvReleaseImage( &gray8 );
	return color_img;
}

extern IplImage* convert_gray32_to_color_DICOM( IplImage* gray32 )
{
	IplImage* gray8, * color_img;
	int r, c;

	gray8 = cvCreateImage( cvGetSize(gray32), IPL_DEPTH_8U, 1 );
	color_img = cvCreateImage( cvGetSize(gray32), IPL_DEPTH_8U, 1 );

	// Always there will be one channel
	cvConvertScale( gray32, gray8, 1.0, 0 );
	cvCvtColor( gray8, color_img, CV_GRAY2BGR );
	

	cvReleaseImage( &gray8 );
	return color_img;
}

extern IplImage* downsample_blending( IplImage* img )
{
   int i,j,k,l,m,n;
   CvMat* Kernel;
   IplImage* sym_input = NULL;
   IplImage* out = NULL;
   IplImage* temp_out = NULL;

   CvSize size_sym_input;
   CvScalar t,u,v;


	IplImage* smaller = cvCreateImage( cvSize(img->width / 2, img->height / 2),
		img->depth, img->nChannels );
	
     



	
	Kernel = cvCreateMat(5,5,CV_32FC1);

	cvmSet(Kernel, 0,0, 0.0025);
	cvmSet(Kernel, 0,1, 0.0125);
	cvmSet(Kernel, 0,2, 0.02);
	cvmSet(Kernel, 0,3, 0.0125);
	cvmSet(Kernel, 0,4, 0.0025);
	cvmSet(Kernel, 1,0, 0.0125);
	cvmSet(Kernel, 1,1, 0.0625);
	cvmSet(Kernel, 1,2, 0.1);
	cvmSet(Kernel, 1,3, 0.0625);
	cvmSet(Kernel, 1,4, 0.0125);
	cvmSet(Kernel, 2,0, 0.02);
	cvmSet(Kernel, 2,1, 0.1);
	cvmSet(Kernel, 2,2, 0.16);
	cvmSet(Kernel, 2,3, 0.1);
	cvmSet(Kernel, 2,4, 0.02);
	cvmSet(Kernel, 3,0, 0.0125);
	cvmSet(Kernel, 3,1, 0.0625);
	cvmSet(Kernel, 3,2, 0.1);
	cvmSet(Kernel, 3,3, 0.0625);
	cvmSet(Kernel, 3,4, 0.0125);
	cvmSet(Kernel, 4,0, 0.0025);
	cvmSet(Kernel, 4,1, 0.0125);
	cvmSet(Kernel, 4,2, 0.02);
	cvmSet(Kernel, 4,3, 0.0125);
	cvmSet(Kernel, 4,4, 0.0025);

	size_sym_input.height = img->height + 4;
	size_sym_input.width = img->width + 4;

	m = img->height;
	n = img->width;

	allocateOnDemand(&sym_input, size_sym_input, img->depth, img->nChannels );
	cvZero( sym_input );

/**************************************************
    MATLAB:
	sym_input(3:m+2,3:n+2) = input;
*****************************************************/

	for( i = 2; i < img->height + 2; i++){
		for( j = 2; j < img->width + 2; j++){
			cvSet2D(sym_input,i,j,cvGet2D(img, i - 2, j - 2));
		}
	}
/**************************************************
    MATLAB:
	sym_input(1,3:n+2) = 2*input(1,:)-input(3,:);
*****************************************************/
	for( j = 2; j < img->width + 2; j++){
		for ( k = 0; k < img->width; k++){
			t = cvGet2D(img,0,k);
			u = cvGet2D(img,2,k);
			for(l = 0; l < img->nChannels; l++)	v.val[l] = 2*t.val[l] -u.val[l];
			cvSet2D(sym_input,0,j, v);
		}
	}

/**************************************************
    MATLAB:
	sym_input(2,3:n+2) = 2*input(1,:)-input(2,:);
*****************************************************/
	for( j = 2; j < img->width + 2; j++){
		for ( k = 0; k < img->width; k++){
			t = cvGet2D(img,0,k);
			u = cvGet2D(img,1,k);
			for(l = 0; l < img->nChannels; l++)	v.val[l] = 2*t.val[l] -u.val[l];
			cvSet2D(sym_input,1,j, v);
		}
	}

/**************************************************
    MATLAB:
	sym_input(m+4,3:n+2) = 2*input(m,:)-input(m-2,:);
*****************************************************/
	for( j = 2; j < img->width + 2; j++){
		for ( k = 0; k < img->width; k++){
			t = cvGet2D(img,m-1,k);
			u = cvGet2D(img,m-3,k);
			for(l = 0; l < img->nChannels; l++)	v.val[l] = 2*t.val[l] -u.val[l];
			cvSet2D(sym_input,m+3,j, v);
		}
	}

/**************************************************
    MATLAB:
	sym_input(m+3,3:n+2) = 2*input(m,:)-input(m-1,:);
*****************************************************/
	for( j = 2; j < img->width + 2; j++){
		for ( k = 0; k < img->width; k++){
			t = cvGet2D(img,m-1,k);
			u = cvGet2D(img,m-2,k);
			for(l = 0; l < img->nChannels; l++)	v.val[l] = 2*t.val[l] -u.val[l];
			cvSet2D(sym_input,m+2,j, v);
		}
	}

/**************************************************
    MATLAB:
	sym_input(3:m+2,1) = 2*input(:,1)-input(:,3);
*****************************************************/
	for( i = 2; j < img->height + 2; j++){
		for ( k = 0; k < img->height; k++){
			t = cvGet2D(img,k,0);
			u = cvGet2D(img,k,2);
			for(l = 0; l < img->nChannels; l++)	v.val[l] = 2*t.val[l] -u.val[l];
			cvSet2D(sym_input,i,0, v);
		}
	}

/**************************************************
    MATLAB:
	sym_input(3:m+2,2) = 2*input(:,1)-input(:,2);
*****************************************************/
	for( i = 2; j < img->height + 2; j++){
		for ( k = 0; k < img->height; k++){
			t = cvGet2D(img,k,0);
			u = cvGet2D(img,k,1);
			for(l = 0; l < img->nChannels; l++)	v.val[l] = 2*t.val[l] -u.val[l];
			cvSet2D(sym_input,i,1, v);
		}
	}

/**************************************************
    MATLAB:
	sym_input(3:m+2,n+4) = 2*input(:,n)-input(:,n-2);
*****************************************************/
	for( i = 2; j < img->height + 2; j++){
		for ( k = 0; k < img->height; k++){
			t = cvGet2D(img,k, n - 1);
			u = cvGet2D(img,k, n - 3);
			for(l = 0; l < img->nChannels; l++)	v.val[l] = 2*t.val[l] -u.val[l];
			cvSet2D(sym_input,i,n+3, v);
		}
	}

/**************************************************
    MATLAB:
	sym_input(3:m+2,n+3) = 2*input(:,n)-input(:,n-1);
*****************************************************/
	for( i = 2; j < img->height + 2; j++){
		for ( k = 0; k < img->height; k++){
			t = cvGet2D(img,k, n - 1);
			u = cvGet2D(img,k, n - 2);
			for(l = 0; l < img->nChannels; l++)	v.val[l] = 2*t.val[l] -u.val[l];
			cvSet2D(sym_input,i,n+2, v);
		}
	}

   allocateOnDemand(&temp_out, size_sym_input, img->depth, img->nChannels );
   cvZero( temp_out );

   cvFilter2D(sym_input, temp_out, Kernel, cvPoint(-1,-1));

   for(i = 2; i < temp_out->height-2; i+=2){
	   for(j = 2; j < temp_out->width-2; j+=2){
		   k = (i - 2)/2;
		   l = (j - 2)/2;
		   cvSet2D(smaller, k, l, cvGet2D(temp_out,i,j));
	   }
   }
	


	return smaller;
}
extern IplImage* FindRegionR1(struct image_border ROI_frame1, struct image_border ROI_frame2, IplImage* panorama)
{
	int i, j;
	struct borderS1 Sub_S1;
	struct RegionS1 S1_Region;
	CvSize panorama_size;
	IplImage* panorama_R1 = NULL;
	CvScalar z;
	CvPoint2D32f PS1;

    Sub_S1 = FindBorderS1(ROI_frame1, ROI_frame2);
	S1_Region = FindSRegionS1(Sub_S1);

	panorama_size.height = panorama->height;
	panorama_size.width = panorama->width;

	for(i = 0; i < panorama->nChannels; i++) { z.val[i] = 255.0;}
	allocateOnDemand(&panorama_R1, panorama_size, panorama->depth, panorama->nChannels);

	 for(i = 0; i < panorama->height; i ++){ //rows
	    for(j = 0; j < panorama->width; j++){ //cols
			  PS1.x = j;
	          PS1.y = i;
			   if(Belongs2S1(PS1,S1_Region, Sub_S1))cvSet2D(panorama_R1,i,j,z);
	     }//end for
	  }// end for

	  return panorama_R1;
}
// CreateCircularMask creates a circular mask of the dimension of img
extern IplImage* CreateCircularMask(IplImage* img)
{
	int i, j, k;
	CvSize img_size;
	IplImage* mask = NULL;
	CvScalar z;
	
	float x, y, r;
	float h,w,max_r;

   	img_size.height = img->height;
	img_size.width = img->width;

	//for(i = 0; i < img->nChannels; i++) { z.val[i] = 255.0;}
	allocateOnDemand(&mask, img_size, IPL_DEPTH_32F, 1 );
	h = (float)img->height;
	w = (float)img->width;
	h = h/2;
	w = w/2;
	max_r = pow(pow(h,2.0) + pow(w,2.0),0.5); 

	 for(i = 0; i < img->height; i ++){ //rows
	    for(j = 0; j < img->width; j++){ //cols
			x = (float)j - ((float)img->width)/2;
			y = ((float)img->height)/2 - (float)i;
	        r = pow(pow(x,2.0) + pow(y,2.0),0.5);
			z.val[0] = 255 - r*255/max_r; // assuming that the image is float and only one channel
			cvSet2D(mask,i,j,z);			   
	     }//end for
	  }// end for

	  return mask;
}
// Triangular mask is a mask where the pixel level decay from 1 to 0 linearly
extern IplImage* CreateTriangularMask(IplImage* img)
{
	int i, j, k;
	CvSize img_size;
	IplImage* mask = NULL;
	CvScalar z;
	
	float x, y, xp, yp;
	float h,w,Wx,Wy;

   	img_size.height = img->height;
	img_size.width = img->width;

	//for(i = 0; i < img->nChannels; i++) { z.val[i] = 255.0;}
	allocateOnDemand(&mask, img_size, img->depth, img->nChannels);

	h = (float)img->height;
	w = (float)img->width;
	//h = h/2;
	//w = w/2;
	
	 for(i = 0; i < img->height; i ++){ //rows
	    for(j = 0; j < img->width; j++){ //cols
			xp = (float)j - ((float)img->width)/2;
			yp = ((float)img->height)/2 - (float)i;

			if( xp >=0 ) Wx = -(2/w)*xp + 1;
			else Wx = (2/w)*xp + 1;
        
			if( yp >= 0 ) Wy = -(2/h)*yp + 1;
            else Wy = (2/h)*yp + 1;
               
			z.val[0] = 255*Wx*Wy; // assuming that the image is float and only one channel
			cvSet2D(mask,i,j,z);			   
	     }//end for
	  }// end for

	  return mask;
}

// Starting the functions that will use different access to the image data, they use pointer access
// I will test how fast it can go.

extern IplImage* downsample_blending_p( IplImage* img )
{
	// img has only 1 channel and is float
   int i,j,k,l,m,n;
   CvMat* Kernel;
   IplImage* sym_input = NULL;
   IplImage* out = NULL;
   IplImage* temp_out = NULL;

   CvSize size_sym_input;
   CvScalar t,u,v;
   float tf, uf, vf;
   


	IplImage* smaller = cvCreateImage( cvSize(img->width / 2, img->height / 2),
		img->depth, img->nChannels );
	
     

	
	Kernel = cvCreateMat(5,5,CV_32FC1);

	cvmSet(Kernel, 0,0, 0.0025);
	cvmSet(Kernel, 0,1, 0.0125);
	cvmSet(Kernel, 0,2, 0.02);
	cvmSet(Kernel, 0,3, 0.0125);
	cvmSet(Kernel, 0,4, 0.0025);
	cvmSet(Kernel, 1,0, 0.0125);
	cvmSet(Kernel, 1,1, 0.0625);
	cvmSet(Kernel, 1,2, 0.1);
	cvmSet(Kernel, 1,3, 0.0625);
	cvmSet(Kernel, 1,4, 0.0125);
	cvmSet(Kernel, 2,0, 0.02);
	cvmSet(Kernel, 2,1, 0.1);
	cvmSet(Kernel, 2,2, 0.16);
	cvmSet(Kernel, 2,3, 0.1);
	cvmSet(Kernel, 2,4, 0.02);
	cvmSet(Kernel, 3,0, 0.0125);
	cvmSet(Kernel, 3,1, 0.0625);
	cvmSet(Kernel, 3,2, 0.1);
	cvmSet(Kernel, 3,3, 0.0625);
	cvmSet(Kernel, 3,4, 0.0125);
	cvmSet(Kernel, 4,0, 0.0025);
	cvmSet(Kernel, 4,1, 0.0125);
	cvmSet(Kernel, 4,2, 0.02);
	cvmSet(Kernel, 4,3, 0.0125);
	cvmSet(Kernel, 4,4, 0.0025);

	size_sym_input.height = img->height + 4;
	size_sym_input.width = img->width + 4;

	m = img->height;
	n = img->width;

	allocateOnDemand(&sym_input, size_sym_input, img->depth, img->nChannels );
	cvZero( sym_input );

/**************************************************
    MATLAB:
	sym_input(3:m+2,3:n+2) = input;
*****************************************************/

	for( i = 2; i < img->height + 2; i++){
		for( j = 2; j < img->width + 2; j++){
		//	cvSet2D(sym_input,i,j,cvGet2D(img, i - 2, j - 2));
			((float *)(sym_input->imageData + i*sym_input->widthStep))[j]=((float *)(img->imageData + (i-2)*img->widthStep))[j-2];

		}
	}
/**************************************************
    MATLAB:
	sym_input(1,3:n+2) = 2*input(1,:)-input(3,:);
*****************************************************/
	for( j = 2; j < img->width + 2; j++){
		for ( k = 0; k < img->width; k++){
			//t = cvGet2D(img,0,k);
			//u = cvGet2D(img,2,k);
			tf = ((float *)(img->imageData + 0*img->widthStep))[k];
			uf = ((float *)(img->imageData + 2*img->widthStep))[k];
			//for(l = 0; l < img->nChannels; l++)	v.val[l] = 2*t.val[l] -u.val[l];
			vf = 2*tf - uf;
			//cvSet2D(sym_input,0,j, v);
           ((float *)(sym_input->imageData + 0*sym_input->widthStep))[j] = vf;
		}
	}

/**************************************************
    MATLAB:
	sym_input(2,3:n+2) = 2*input(1,:)-input(2,:);
*****************************************************/
	for( j = 2; j < img->width + 2; j++){
		for ( k = 0; k < img->width; k++){
			//t = cvGet2D(img,0,k);
			//u = cvGet2D(img,1,k);
			tf = ((float *)(img->imageData + 0*img->widthStep))[k];
			uf = ((float *)(img->imageData + 1*img->widthStep))[k];
			//for(l = 0; l < img->nChannels; l++)	v.val[l] = 2*t.val[l] -u.val[l];
			vf = 2*tf - uf;
			//cvSet2D(sym_input,1,j, v);
			((float *)(sym_input->imageData + 1*sym_input->widthStep))[j] = vf;
		}
	}

/**************************************************
    MATLAB:
	sym_input(m+4,3:n+2) = 2*input(m,:)-input(m-2,:);
*****************************************************/
	for( j = 2; j < img->width + 2; j++){
		for ( k = 0; k < img->width; k++){
			//t = cvGet2D(img,m-1,k);
			//u = cvGet2D(img,m-3,k);
			tf = ((float *)(img->imageData + (m-1)*img->widthStep))[k];
			uf = ((float *)(img->imageData + (m-3)*img->widthStep))[k];
			//for(l = 0; l < img->nChannels; l++)	v.val[l] = 2*t.val[l] -u.val[l];
			vf = 2*tf - uf;
			//cvSet2D(sym_input,m+3,j, v);
			((float *)(sym_input->imageData + (m+3)*sym_input->widthStep))[j] = vf;
		}
	}

/**************************************************
    MATLAB:
	sym_input(m+3,3:n+2) = 2*input(m,:)-input(m-1,:);
*****************************************************/
	for( j = 2; j < img->width + 2; j++){
		for ( k = 0; k < img->width; k++){
			//t = cvGet2D(img,m-1,k);
			//u = cvGet2D(img,m-2,k);
			tf = ((float *)(img->imageData + (m-1)*img->widthStep))[k];
			uf = ((float *)(img->imageData + (m-2)*img->widthStep))[k];
			//for(l = 0; l < img->nChannels; l++)	v.val[l] = 2*t.val[l] -u.val[l];
			vf = 2*tf - uf;
			//cvSet2D(sym_input,m+2,j, v);
			((float *)(sym_input->imageData + (m+2)*sym_input->widthStep))[j] = vf;
		}
	}

/**************************************************
    MATLAB:
	sym_input(3:m+2,1) = 2*input(:,1)-input(:,3);
*****************************************************/
	for( i = 2; j < img->height + 2; j++){
		for ( k = 0; k < img->height; k++){
			//t = cvGet2D(img,k,0);
			//u = cvGet2D(img,k,2);
			tf = ((float *)(img->imageData + k*img->widthStep))[0];
			uf = ((float *)(img->imageData + k*img->widthStep))[2];
			//for(l = 0; l < img->nChannels; l++)	v.val[l] = 2*t.val[l] -u.val[l];
			vf = 2*tf - uf;
			//cvSet2D(sym_input,i,0, v);
			((float *)(sym_input->imageData + (i)*sym_input->widthStep))[0] = vf;
		}
	}

/**************************************************
    MATLAB:
	sym_input(3:m+2,2) = 2*input(:,1)-input(:,2);
*****************************************************/
	for( i = 2; j < img->height + 2; j++){
		for ( k = 0; k < img->height; k++){
			//t = cvGet2D(img,k,0);
			//u = cvGet2D(img,k,1);
			//for(l = 0; l < img->nChannels; l++)	v.val[l] = 2*t.val[l] -u.val[l];
			//cvSet2D(sym_input,i,1, v);
			tf = ((float *)(img->imageData + k*img->widthStep))[0];
			uf = ((float *)(img->imageData + k*img->widthStep))[1];
			vf = 2*tf - uf;
			((float *)(sym_input->imageData + (i)*sym_input->widthStep))[1] = vf;
		}
	}

/**************************************************
    MATLAB:
	sym_input(3:m+2,n+4) = 2*input(:,n)-input(:,n-2);
*****************************************************/
	for( i = 2; j < img->height + 2; j++){
		for ( k = 0; k < img->height; k++){
			/*
			t = cvGet2D(img,k, n - 1);
			u = cvGet2D(img,k, n - 3);
			for(l = 0; l < img->nChannels; l++)	v.val[l] = 2*t.val[l] -u.val[l];
			cvSet2D(sym_input,i,n+3, v);
			*/
			tf = ((float *)(img->imageData + k*img->widthStep))[n-1];
			uf = ((float *)(img->imageData + k*img->widthStep))[n-3];
			vf = 2*tf - uf;
			((float *)(sym_input->imageData + (i)*sym_input->widthStep))[n+3] = vf;

		}
	}

/**************************************************
    MATLAB:
	sym_input(3:m+2,n+3) = 2*input(:,n)-input(:,n-1);
*****************************************************/
	for( i = 2; j < img->height + 2; j++){
		for ( k = 0; k < img->height; k++){/*
			t = cvGet2D(img,k, n - 1);
			u = cvGet2D(img,k, n - 2);
			for(l = 0; l < img->nChannels; l++)	v.val[l] = 2*t.val[l] -u.val[l];
			cvSet2D(sym_input,i,n+2, v);*/
			tf = ((float *)(img->imageData + k*img->widthStep))[n-1];
			uf = ((float *)(img->imageData + k*img->widthStep))[n-2];
			vf = 2*tf - uf;
			((float *)(sym_input->imageData + (i)*sym_input->widthStep))[n+2] = vf;
		}
	}

   allocateOnDemand(&temp_out, size_sym_input, img->depth, img->nChannels );
   cvZero( temp_out );

   cvFilter2D(sym_input, temp_out, Kernel, cvPoint(-1,-1));

   for(i = 2; i < temp_out->height-2; i+=2){
	   for(j = 2; j < temp_out->width-2; j+=2){
		   k = (i - 2)/2;
		   l = (j - 2)/2;
		  // cvSet2D(smaller, k, l, cvGet2D(temp_out,i,j));
          ((float *)(smaller->imageData + (k)*smaller->widthStep))[l] = ((float *)(temp_out->imageData + (i)*temp_out->widthStep))[j];
	   }
   }
	


	return smaller;
}


extern IplImage* expand_blending_p(IplImage* input)
{ 


	int i, j, k, l;
	CvMat* Kernel;
	CvSize size_output;
	IplImage *output = NULL;
	IplImage *output_conv = NULL;
	//IplImage *temp = NULL;
	//CvScalar t,u,v;
	float vf;

/**************************************************
  MATLAB:
		[m,n] = size(input);
		out(2*m,2*n) = 0;
		out(1:2:2*m, 1:2:2*n) = input;
*****************************************************/
	//printf(" %A input (%d, %d) \n", 2*(input->height), 2*(input->width));		 

	size_output.height = 2*input->height;
	size_output.width = 2*input->width;

	output = cvCreateImage(size_output, input->depth, input->nChannels);


	for(i = 0; i < output->height; i+=2 ){
		for(j = 0; j < output->width; j+=2){
			//cvSet2D(output,i, j, cvGet2D(input,i/2,j/2));
			((float *)(output->imageData + i*output->widthStep))[j]=((float *)(input->imageData + (i/2)*input->widthStep))[j/2];
		}
	}

	Kernel = cvCreateMat(5,5,CV_32FC1);

	cvmSet(Kernel, 0,0, 0.0025);
	cvmSet(Kernel, 0,1, 0.0125);
	cvmSet(Kernel, 0,2, 0.02);
	cvmSet(Kernel, 0,3, 0.0125);
	cvmSet(Kernel, 0,4, 0.0025);
	cvmSet(Kernel, 1,0, 0.0125);
	cvmSet(Kernel, 1,1, 0.0625);
	cvmSet(Kernel, 1,2, 0.1);
	cvmSet(Kernel, 1,3, 0.0625);
	cvmSet(Kernel, 1,4, 0.0125);
	cvmSet(Kernel, 2,0, 0.02);
	cvmSet(Kernel, 2,1, 0.1);
	cvmSet(Kernel, 2,2, 0.16);
	cvmSet(Kernel, 2,3, 0.1);
	cvmSet(Kernel, 2,4, 0.02);
	cvmSet(Kernel, 3,0, 0.0125);
	cvmSet(Kernel, 3,1, 0.0625);
	cvmSet(Kernel, 3,2, 0.1);
	cvmSet(Kernel, 3,3, 0.0625);
	cvmSet(Kernel, 3,4, 0.0125);
	cvmSet(Kernel, 4,0, 0.0025);
	cvmSet(Kernel, 4,1, 0.0125);
	cvmSet(Kernel, 4,2, 0.02);
	cvmSet(Kernel, 4,3, 0.0125);
	cvmSet(Kernel, 4,4, 0.0025);


	output_conv = cvCreateImage(size_output,input->depth, input->nChannels);
	cvFilter2D(output, output_conv, Kernel, cvPoint(-1,-1));

//	output_conv = MulImagewithDouble(output_conv, 4.0);

	for(i = 0; i < output_conv->height; i++ ){
		for(j = 0; j < output_conv->width; j++){
			vf = ((float *)(output_conv->imageData + i*output_conv->widthStep))[j];
			((float *)(output_conv->imageData + i*output_conv->widthStep))[j] = vf*4.0;
		}
	}

	
  cvReleaseImage( &output );
  cvReleaseMat( &Kernel );

	
  return output_conv;
}

extern IplImage** build_gaussb_pyr_blending_p( IplImage* base, int levels )
{
	IplImage** gaussb_pyr;
	int i, o;
	
    //levels = log( MIN( base->width, base->height ) ) / log(2) + 1;

	gaussb_pyr = calloc( levels, sizeof( IplImage** ) );
	for( i = 0; i < levels; i++ )
		gaussb_pyr[i] = calloc( levels, sizeof( IplImage* ) );

	
	for( o = 0; o < levels; o++ )
		{
			if( o == 0 )
				gaussb_pyr[o] = cvCloneImage(base);

			 else 
			 {  
			
				gaussb_pyr[o] = downsample_blending_p( gaussb_pyr[o-1] );				
				
			 }
	   }
	
	return gaussb_pyr;
}

extern IplImage** build_laplace_lpyr_blending_p( IplImage* base, int levels )
{
	IplImage** laplace_pyr;
	IplImage** gaussb_pyr;
	IplImage* temp;
	int i, o;
	int levels2;
	int r, c, n; // to compute the dimmensions of each level
	CvScalar s;

    //levels2 = levels;
		
	//gaussb_pyr = build_gaussb3_pyr( base );

	gaussb_pyr = build_gaussb_pyr_blending(base, levels);

	laplace_pyr = calloc( levels, sizeof( IplImage** ) );
	for( i = 0; i < levels; i++ ){
		laplace_pyr[i] = cvCreateImage( cvGetSize(gaussb_pyr[i]), base->depth, base->nChannels);
	}

	

	for( o = 0; o < levels - 1; o++ )
		{
			
				temp = cvCreateImage(cvGetSize(gaussb_pyr[o]),base->depth, base->nChannels);
			   	temp = expand_blending_p(gaussb_pyr[o+1]);
				cvSub(gaussb_pyr[o],temp,laplace_pyr[o],NULL );
				cvReleaseImage( &temp );
	   }

	laplace_pyr[levels-1] = cvCloneImage(gaussb_pyr[levels-1]);

	releaseb_pyr( &gaussb_pyr, levels );
	 
	return laplace_pyr;
}

extern IplImage* BlendImageswithMask_p(IplImage* im1, IplImage* im2, IplImage* im3, int levels)
{
	int i,j,k,l;
	CvScalar s,t,u,v,w,x,y,z;
	float sf, tf, uf, vf, wf, xf, yf,zf;
	IplImage** laplace_pyr_im1;
	IplImage** gaussb_pyr_im1;
	IplImage** laplace_pyr_im2;
	IplImage** gaussb_pyr_im2;
	IplImage** laplace_pyr_im3;
	IplImage** gaussb_pyr_im3;
	IplImage** gaussb_pyr_GRN;
	IplImage** laplace_pyr_C;
	IplImage* C = NULL;
	CvSize size_img;


	gaussb_pyr_im1 = build_gaussb_pyr_blending_p(im1, levels);
	gaussb_pyr_im2 = build_gaussb_pyr_blending_p(im2, levels);
	gaussb_pyr_im3 = build_gaussb_pyr_blending_p(im3, levels);

	laplace_pyr_im1 = build_laplace_lpyr_blending_p(im1, levels);
	laplace_pyr_im2 = build_laplace_lpyr_blending_p(im2, levels);
	laplace_pyr_im3 = build_laplace_lpyr_blending_p(im3, levels);

	//for(l = 0; l < gaussb_pyr_im3[0]->nChannels; l++) s.val[l] = 1;
	printf(" I am here 1\n");



	gaussb_pyr_GRN = calloc( levels, sizeof( IplImage** ) );
	for( k = 0; k < levels; k++ ){
		size_img.height = gaussb_pyr_im3[k]->height;
		size_img.width = gaussb_pyr_im3[k]->width;
		gaussb_pyr_GRN[k] = cvCreateImage( size_img, im3->depth, im3->nChannels);
	}
/*
	gaussb_pyr_GRN = calloc( levels, sizeof( IplImage** ) );
	for( k = 0; k < levels; k++ ){
		gaussb_pyr_GRN[k] = calloc( levels, sizeof( IplImage* ) );
	}
	*/
    
	// The following code does GRN = 1 - GR; in the MATLAB code
printf(" I am here 2\n");
	for ( l = 0; l < levels; l++){
		for(i = 0; i < gaussb_pyr_im3[l]->height; i++){
			for(j = 0; j < gaussb_pyr_im3[l]->width; j++){
				//printf(" I am here 2A\n");
				//u = cvGet2D(gaussb_pyr_im3[l],i,j);
				//for(k = 0; k < gaussb_pyr_im3[k]->nChannels; k++) t.val[k] = s.val[k] - u.val[k];
				//printf(" I am here 2B\n");
                //cvSet2D(gaussb_pyr_GRN[l],i,j,t);
				uf = ((float *)(gaussb_pyr_im3[l]->imageData + i*gaussb_pyr_im3[l]->widthStep))[j];
				((float *)(gaussb_pyr_GRN[l]->imageData + i*gaussb_pyr_GRN[l]->widthStep))[j] = 1.0 - uf;
			}
		}
	}
/******************************************************************
  MATLAB:
	for i = 1:level
    LC(:,:,i) = GR(:,:,i) .* LA(:,:,i) + GRN(:,:,i) .* LB(:,:,i);
    end
********************************************************************/
printf(" I am here 3\n");
	laplace_pyr_C = calloc( levels, sizeof( IplImage** ) );
	for( k = 0; k < levels; k++ ){
		size_img.height = gaussb_pyr_im1[k]->height;
		size_img.width = gaussb_pyr_im1[k]->width;
		laplace_pyr_C[k] = cvCreateImage( size_img , im1->depth, im1->nChannels);
	}
	for ( l = 0; l < levels; l++){
		for(i = 0; i < laplace_pyr_C[l]->height; i++){
			for(j = 0; j < laplace_pyr_C[l]->width; j++){/*
				v = cvGet2D(gaussb_pyr_im3[l],i,j); //GR
				w = cvGet2D(laplace_pyr_im1[l],i,j);//LA
				x = cvGet2D(gaussb_pyr_GRN[l],i,j); //GRN
				y = cvGet2D(laplace_pyr_im2[l],i,j); //LB
				for(k = 0; k < gaussb_pyr_im3[k]->nChannels; k++) z.val[k] = v.val[k] * w.val[k] + x.val[k] * y.val[k];
				cvSet2D(laplace_pyr_C[l],i,j,z);*/
				vf = ((float *)(gaussb_pyr_im3[l]->imageData + i*gaussb_pyr_im3[l]->widthStep))[j]; // GR
				wf = ((float *)(laplace_pyr_im1[l]->imageData + i*laplace_pyr_im1[l]->widthStep))[j]; // LA
				xf = ((float *)(gaussb_pyr_GRN[l]->imageData + i*gaussb_pyr_GRN[l]->widthStep))[j]; // GRN
				yf = ((float *)(laplace_pyr_im2[l]->imageData + i*laplace_pyr_im2[l]->widthStep))[j]; // LB
				((float *)(laplace_pyr_C[l]->imageData + i*laplace_pyr_C[l]->widthStep))[j] =  vf* wf + xf* yf;
			}
		}
	}

/**************************************************
  MATLAB:
	C = reconstruct(LC);
*****************************************************/
	size_img.height = im1->height;
	size_img.width = im1->width;
	C = cvCreateImage(size_img,im1->depth, im1->nChannels);

	printf(" doing the reconstruction ...\n");
    C = ReconstructPyramid_p(laplace_pyr_C, levels);
	printf(" reconstruction done ...\n");
      
  releaseb_pyr( &gaussb_pyr_im1, levels );
  releaseb_pyr( &gaussb_pyr_im2, levels );
  releaseb_pyr( &gaussb_pyr_im3, levels );
  releaseb_pyr( &laplace_pyr_im1, levels );
  releaseb_pyr( &laplace_pyr_im2, levels );
  releaseb_pyr( &laplace_pyr_im3, levels );
   releaseb_pyr( &gaussb_pyr_GRN, levels );
  releaseb_pyr( &laplace_pyr_C, levels );

  return C;
}

extern IplImage* ReconstructPyramid_p(IplImage** laplace_pyr, int levels)
{
  IplImage* C = NULL;
  IplImage* out_expand = NULL;
  IplImage* out = NULL;
  int m,n, i, j, k, l;
  double s;
  CvScalar t,u,v,w,x,y,z;
  float tf, uf;
  CvSize size_out, size_C, size_out_expand;

/**************************************************
  MATLAB:
	[m,n, level] = size(input_pyramid);
    s = 1/power(2,level-1);
	out = input_pyramid(1:m*s,1:n*s,level);

	This part of the code just clone the last level of the pyramid
*****************************************************/
  m = laplace_pyr[0]->height;
  n = laplace_pyr[0]->width;
  s = 1 / pow(2, levels - 1);
/*
  size_out.height = floor(m*s);
  size_out.width = floor(n*s);

  out = cvCreateImage(size_out, laplace_pyr[0]->depth, laplace_pyr[0]->nChannels);

  for(i = 0; i < m*s; i++){
	  for(j = 0; j < n*s; j++){
		  cvSet2D(out, i, j, cvGet2D(laplace_pyr[levels],i,j));
	  }
  }
 */
  
  size_out.height =  laplace_pyr[levels-1]->height;
  size_out.width = laplace_pyr[levels-1]->width;
  out = cvCreateImage(size_out, laplace_pyr[0]->depth, laplace_pyr[0]->nChannels);
  out = cvCloneImage(laplace_pyr[levels-1]);

/**************************************************
  MATLAB:
	for i = level-1:-1:1
    s = 1/power(2,i-1);
    out = expand(out) + input_pyramid(1:m*s,1:n*s,i);      
	end
*****************************************************/

  for(l = levels - 2; l >= 0; l--){
	  //s = 1 / pow(2, l - 1);
	  printf(" level = %d", l);
	  size_out_expand.height = laplace_pyr[l]->height;
	  size_out_expand.width = laplace_pyr[l]->width;
	  out_expand = cvCreateImage(size_out_expand, laplace_pyr[0]->depth, laplace_pyr[0]->nChannels);
	  out_expand = expand_blending(out);
	  size_C.height = out_expand->height;
	  size_C.width = out_expand->width;
	  C = cvCreateImage(size_C, laplace_pyr[0]->depth, laplace_pyr[0]->nChannels);
	  for( i = 0; i < out_expand->height; i++){
		  for(j = 0; j < out_expand->width; j++){/*
			t = cvGet2D(out_expand, i, j); //expand(out)
			u = cvGet2D(laplace_pyr[l],i,j); //input_pyramid(1:m*s,1:n*s,i); 
			for(k = 0; k < laplace_pyr[0]->nChannels; k++) v.val[k] = t.val[k] + u.val[k];// adding all the channels
            cvSet2D(C,i,j,v);  //out*/
			tf = ((float *)(out_expand->imageData + i*out_expand->widthStep))[j]; //expand(out)
			uf = ((float *)(laplace_pyr[l]->imageData + i*laplace_pyr[l]->widthStep))[j]; //input_pyramid(1:m*s,1:n*s,i); 
			((float *)(C->imageData + i*C->widthStep))[j] = tf + uf;
		  }
	  }
	  size_out.height = C->height;
	  size_out.width = C->width;
	  out = cvCreateImage(size_out, laplace_pyr[0]->depth, laplace_pyr[0]->nChannels);
	  out = cvCloneImage(C);
	  cvReleaseImage(& C );      
  }

  
  cvReleaseImage(& out_expand);

  return out;

}
// refine_mask_p is a pointer implementation (to speed up the calculations) to determine the mask1 and mask2
// panorama5 is the prev panorama
void refine_mask_p(IplImage* panorama5, IplImage* dst_mask_im1, IplImage* dst_mask_im2)
{
	int i, j;
	float sf, tf, uf, avef;
   
	for(i = 0; i < panorama5->height; i ++){ //rows
		  for(j = 0; j < panorama5->width; j++){ //cols
			  
			  /*s = cvGet2D(dst_mask_im1, i, j);
			  t = cvGet2D(dst_mask_im2, i, j);*/
			  uf = ((float *)(panorama5->imageData + i*panorama5->widthStep))[j];
			  if(uf == 255){
			  sf = ((float *)(dst_mask_im1->imageData + i*dst_mask_im1->widthStep))[j];
			  tf = ((float *)(dst_mask_im2->imageData + i*dst_mask_im2->widthStep))[j];

			  
			  if(sf > tf){
				  /*cvSet2D(dst_mask_im1,i,j,z);// assign value to 255
				  cvSet2D(dst_mask_im2,i,j, u); //assign value to 0
				  */
					((float *)(dst_mask_im1->imageData + i*dst_mask_im1->widthStep))[j] = 255;
					((float *)(dst_mask_im2->imageData + i*dst_mask_im2->widthStep))[j] = 0;
			  }
			  else if(sf <= tf){
				  /*cvSet2D(dst_mask_im1,i,j,u); //assign value to 0
				  cvSet2D(dst_mask_im2,i,j,z); //assing value to 255*/
					((float *)(dst_mask_im1->imageData + i*dst_mask_im1->widthStep))[j] = 0;
					((float *)(dst_mask_im2->imageData + i*dst_mask_im2->widthStep))[j] = 255;
			  }
			  /*else {
				 	  if( sf != 255 ){ 
					  ((float *)(dst_mask_im1->imageData + i*dst_mask_im1->widthStep))[j] = 0;
				  }
				  else
					   ((float *)(dst_mask_im1->imageData + i*dst_mask_im1->widthStep))[j] = sf;

				  if( tf != 255 ) {
					  ((float *)(dst_mask_im2->imageData + i*dst_mask_im2->widthStep))[j] = 0;
				  }
				  else
					  ((float *)(dst_mask_im2->imageData + i*dst_mask_im2->widthStep))[j] = tf;
			  }*/
			 }//end of if (uf != 0)
			  else{
					((float *)(dst_mask_im1->imageData + i*dst_mask_im1->widthStep))[j] = 0;
					((float *)(dst_mask_im2->imageData + i*dst_mask_im2->widthStep))[j] = 0;
			  }
		  }//end for
     }// end for

return;

}
// Find the extreme borders of the mosaic x_min, y_min, and x_max_y_max
// based on this finds the P1, P2, P3, and P4 that determine the border of the mosaic
extern struct image_border  FindCornersMosaic(struct image_border ROI_frame1, struct image_border ROI_frame2)
{
	float x1_min, x2_max, x1_max, x2_min, x_max, x_min;
	float y1_min, y2_max, y1_max, y2_min, y_max, y_min;
	struct image_border border_mosaic;
	int w, h;

	x1_min = MIN(ROI_frame1.P1.x, MIN(ROI_frame1.P2.x,MIN(ROI_frame1.P3.x,ROI_frame1.P4.x)));
	x1_max = MAX(ROI_frame1.P1.x, MAX(ROI_frame1.P2.x,MAX(ROI_frame1.P3.x,ROI_frame1.P4.x)));
	y1_min = MIN(ROI_frame1.P1.y, MIN(ROI_frame1.P2.y,MIN(ROI_frame1.P3.y,ROI_frame1.P4.y)));
	y1_max = MAX(ROI_frame1.P1.y, MAX(ROI_frame1.P2.y,MAX(ROI_frame1.P3.y,ROI_frame1.P4.y)));

    x2_min = MIN(ROI_frame2.P1.x, MIN(ROI_frame2.P2.x,MIN(ROI_frame2.P3.x,ROI_frame2.P4.x)));
	x2_max = MAX(ROI_frame2.P1.x, MAX(ROI_frame2.P2.x,MAX(ROI_frame2.P3.x,ROI_frame2.P4.x)));
	y2_min = MIN(ROI_frame2.P1.y, MIN(ROI_frame2.P2.y,MIN(ROI_frame2.P3.y,ROI_frame2.P4.y)));
	y2_max = MAX(ROI_frame2.P1.y, MAX(ROI_frame2.P2.y,MAX(ROI_frame2.P3.y,ROI_frame2.P4.y)));

	x_max = MAX(x1_max, x2_max);
	y_max = MAX(y1_max, y2_max);
	x_min = MIN(x1_min, x2_min);
	y_min = MIN(y1_min, y2_min);

    
	w = floor(x_max - x_min);
	h = floor(y_max - y_min);
	border_mosaic.P1.x = x_min;
	border_mosaic.P1.y = y_min;
	border_mosaic.P3.x = x_max;
	border_mosaic.P3.y = y_max;
	border_mosaic.P2.x = x_min + w;
	border_mosaic.P2.y = y_min;
	border_mosaic.P4.x = x_min;
	border_mosaic.P4.y = y_max;

	return border_mosaic;

  return;
}
// This function evaluates if the point belongs to the mosaic, the mosaic is a binary version of the real mosaic

extern int Belongs2Mosaic(CvPoint2D32f point, IplImage* mosaic )
{
 int result;
 int i, j;
 float vf;
 
		 i = floor(point.y); // row
		 j = floor(point.x); // col
		 vf = ((float *)(mosaic->imageData + i*mosaic->widthStep))[j];
		 if ( vf == 255) result = 1;
		 else result = 0;

 return result;
}

// This function find a binary image where white (255) is where the mosaic is present and 0 where isn't
extern IplImage* FindMosaicRegion(IplImage* prev_mosaic,struct RegionS new_imag_region)
{
   int i, j;
   float uf;
   CvPoint2D32f P;
   IplImage* mosaic = NULL;
   mosaic = cvCreateImage(cvGetSize(prev_mosaic), IPL_DEPTH_32F, 1 );
   for(i = 0; i < prev_mosaic->height; i++){
	   for(j = 0; j < prev_mosaic->width; j++){
		   P.x = j; //cols
		   P.y = i; //rows
		   uf = ((float *)(prev_mosaic->imageData + i*prev_mosaic->widthStep))[j];
		   if(uf == 255 || Belongs2S(P,new_imag_region)) 
			   ((float *)(mosaic->imageData + i*mosaic->widthStep))[j] = 255;
	   }
   }
  return mosaic;
}

// This function find a binary image where white (255) is where the mosaic is present and 0 where isn't
extern void voidFindMosaicRegion(IplImage* mosaic,struct RegionS new_imag_region)
{
   int i, j;
   float uf;
   CvPoint2D32f P;
   
   for(i = 0; i < mosaic->height; i++){
	   for(j = 0; j < mosaic->width; j++){
		   P.x = j; //cols
		   P.y = i; //rows
		   uf = ((float *)(mosaic->imageData + i*mosaic->widthStep))[j];
		   if(uf == 255 || Belongs2S(P,new_imag_region)) 
			   ((float *)(mosaic->imageData + i*mosaic->widthStep))[j] = 255;
	   }
   }
  return;
}
// for the first frame

extern IplImage* FindInitMosaicRegion(IplImage* dst1,struct image_border new_imag_border)
{
   int i, j;
   float uf;
   CvPoint2D32f P;
   IplImage* mosaic = NULL;
   mosaic = cvCreateImage(cvGetSize(dst1), IPL_DEPTH_32F, 1 );
   for(i = 0; i < mosaic->height; i++){
	   for(j = 0; j < mosaic->width; j++){
		   P.x = j; //cols
		   P.y = i; //rows
		   if(Belongs2S1v2(P,new_imag_border)) {
			  // printf(" P = (%f,%f) \n", P.x, P.y);
			  // getchar();
			   ((float *)(mosaic->imageData + i*mosaic->widthStep))[j] = 255;
		   }
	   }
   }
  return mosaic;
}
/*
extern int FindEdgeMosaic(CvPoint2D32f point, struct RegionS border1,struct RegionS border2 )
{
 int result;
 if(Belongs2S(point, border1) || Belongs2S(point, border2)) result = 1;
 else result = 0;

 return result;
}
*/

extern int Belongs2S1v2(CvPoint2D32f point, struct image_border borderS1)
{
  int result;
  float xq2; // is the x coordinates of the interception of the y value in the Line L2
  float xq4; // is the x coordinates of the interception of the y value in the Line L4
  float yq1; // is the y coordinates of the interception of the x value in the Line L1
  float yq3; // is the x coordinates of the interception of the x value in the Line L3
  float x, y; // the x and y coordinates of the point

  x = point.x;
  y = point.y;
  
  if( (y >= (borderS1.P1.y)) && (y <= (borderS1.P4.y)) && (x >= (borderS1.P1.x)) && (x <= borderS1.P2.x)){
  result = 1; // the point belongs to S
  } // end if
  else{
	  result = 0; // the point doesn't belong to borderS1;
  }

  return result; // 0: the point doesn't belong to borderS1, and 1 otherwise
}
// FindIntersectionEdgeMosaic finds the edge interesection between the mosaic and the new image
// this function is useful to eliminate vignetig or some artifacts that the image could have
// after the Multiband blending

extern void FindIntersectionEdgeMosaic(IplImage* mosaic, struct RegionS RegionNewImage)
{ 
	int i, j;
	float vf;
	CvPoint2D32f P, P1, P2, P3, P4;

	for(i = 0; i < mosaic->height; i++){
		for(j = 0; j < mosaic->width; j++){
			P.y = i;
			P.x = j;
			if( Belongs2S(P,RegionNewImage) && Belongs2Mosaic(P,mosaic)){
				P1.y = i;
				P1.x = j - 1;
				P2.y = i - 1;
				P2.x = j;
				P3.y = i + 1;
				P3.x = j;
				P4.y = i;
				P4.x = j + 1;
				if(!Belongs2S(P1,RegionNewImage) || !Belongs2S(P2,RegionNewImage)|| !Belongs2S(P3,RegionNewImage) || !Belongs2S(P4,RegionNewImage))
					((float *)(mosaic->imageData + i*mosaic->widthStep))[j] = 128;
			}//end firs if (belongs2S
		}
	} 
 return;
}

extern IplImage* FindIntersectionImageMosaic(IplImage* mosaic, struct RegionS RegionNewImage)
{ 
	int i, j;
	float vf;
	CvPoint2D32f P, P1, P2, P3, P4;
	IplImage* result_img = NULL;

	result_img = cvCreateImage(cvGetSize(mosaic), mosaic->depth, mosaic->nChannels);

	for(i = 0; i < mosaic->height; i++){
		for(j = 0; j < mosaic->width; j++){
			P.y = i;
			P.x = j;
			if( Belongs2S(P,RegionNewImage) && Belongs2Mosaic(P,mosaic)){
				((float *)(result_img->imageData + i*result_img->widthStep))[j] = 255;
			}//end firs if (belongs2S
		}
	} 
 return result_img;
}
// Find the edge of the mosaic,
// the input is a binary image of mosaic
//extern void FindEdgeMosaic(IplImage* mosaic)
extern IplImage* FindEdgeMosaic(IplImage* mosaic)
{ 
	int i, j, delta;
	float vf;
	float upf, dwnf, rgtf, lftf;
	CvPoint2D32f P, P1, P2, P3, P4;
	IplImage* EdgeMosaic = NULL;

	delta = 3;
	

	EdgeMosaic = cvCreateImage(cvGetSize(mosaic), mosaic->depth, mosaic->nChannels);

	for(i = 0; i < mosaic->height; i++){
		for(j = 0; j < mosaic->width; j++){
			vf = ((float *)(mosaic->imageData + i*mosaic->widthStep))[j];
			if (vf == 255){
				upf =  ((float *)(mosaic->imageData + (i - delta)*mosaic->widthStep))[j];
				dwnf = ((float *)(mosaic->imageData + (i + delta)*mosaic->widthStep))[j];
				rgtf = ((float *)(mosaic->imageData + (i)*mosaic->widthStep))[j + delta];
				lftf = ((float *)(mosaic->imageData + (i)*mosaic->widthStep))[j - delta];
				if( (upf == 0) || (dwnf == 0) || (rgtf == 0) || (lftf == 0)){
				((float *)(EdgeMosaic->imageData + i*EdgeMosaic->widthStep))[j] = 128;
				}
				else 
					((float *)(EdgeMosaic->imageData + i*EdgeMosaic->widthStep))[j] = vf;
			
			}
		}
	} 
 return EdgeMosaic;
}

// Find the edge of the mosaic,
// the input is a binary image of mosaic
extern void FindEdgeMosaic_void(IplImage* mosaic)
{ 
	int i, j, delta;
	float vf;
	float upf, dwnf, rgtf, lftf;
	CvPoint2D32f P, P1, P2, P3, P4;
	delta = 2;
		
	for(i = 0; i < mosaic->height; i++){
		for(j = 0; j < mosaic->width; j++){
			vf = ((float *)(mosaic->imageData + i*mosaic->widthStep))[j];
			if (vf == 255){
				upf =  ((float *)(mosaic->imageData + (i - delta)*mosaic->widthStep))[j];
				dwnf = ((float *)(mosaic->imageData + (i + delta)*mosaic->widthStep))[j];
				rgtf = ((float *)(mosaic->imageData + (i)*mosaic->widthStep))[j + delta];
				lftf = ((float *)(mosaic->imageData + (i)*mosaic->widthStep))[j - delta];
				if( (upf == 0) || (dwnf == 0) || (rgtf == 0) || (lftf == 0)){
				((float *)(mosaic->imageData + i*mosaic->widthStep))[j] = 128;
				}
				else 
					((float *)(mosaic->imageData + i*mosaic->widthStep))[j] = vf;
			
			}
		}
	} 
 return;
}



// This function computes the median of a kernel nxn
// for this case n = 3, so we will use a column vector of 9 elements,
//

extern double ComputeMedianKernel(double* kernel, int n)
{
 
  double median;
  int index;
  gsl_sort(kernel, 1, n);

  if(n%2 == 0)  median = (kernel[n/2 - 1] + kernel[n/2])/2;
  else median = kernel[(n-1)/2];

  return median;
}

//extern float median_vector(double* vector);
// This function applies a median filter to the edge of the mosaic

extern void RefineEdgeMosaic(IplImage* mosaic, IplImage* edge_mosaic)
{ 
	int i, j, k, l, m, n;
	float mf,ef,kf,medianf;
	double* vector;
	int p,q,w;
	
	p = 3;
	q = (p-1)/2;
	w = p*p;
    
	for(i = 0; i < mosaic->height; i++){
		for(j = 0; j < mosaic->width; j++){
			mf = ((float *)(mosaic->imageData + i*mosaic->widthStep))[j];
			ef = ((float *)(edge_mosaic->imageData + i*edge_mosaic->widthStep))[j];
			if (ef == 128){
				if ( (i - 1) >= 0 && (i + 1) < mosaic->height && (j - 1) >= 0 && (j + 1) < mosaic->width){
				  //   printf(" 1A \t ");

					vector = (double *)malloc(w*sizeof(double));
					 m = 0;
					for(k = -q; k <= q; k++){
						for( l = -q; l <= q; l++){
							kf = ((float *)(mosaic->imageData + (i+k)*mosaic->widthStep))[j+l];
							if (kf != 0){
							   //  printf(" \n 1B \t ");
									vector[m] = (double)kf;
									m++;
									//printf("mf = %f \t ef = %f \t m = %d \t i = %d \t j = %d ", mf, ef, m, i, j);
								//	getchar();
							}
						}
					}
					/*
					// for debugging
					if ( i == 56 && j == 16){
						printf("mf = %f \t ef = %f \t m = %d", mf, ef, m);
						for( n = 0; n < m; n++) { 
							//printf(" vector [%d] = %f", n, vector[n]);
							medianf = (float)ComputeMedianKernel(vector,m); 
							printf(" medianf = %f \n", medianf);
						}
					}// end of if debugging
					*/
                   ((float *)(mosaic->imageData + i*mosaic->widthStep))[j] = (float)ComputeMedianKernel(vector,m);
					free(vector);
				}
			}
		}
	} 
 return;
}
// This function refines (reduces by delta pixels) the mosaic, this constraction is based on the edge_mosaic information
extern void RefineEdgeBinaryMosaic(IplImage* mosaic, IplImage* edge_mosaic, int delta)
{ 
	int i, j;
	//int delta;
	float mf,ef;
	float upf, dwnf, rgtf, lftf;
	
	
	//delta = 2;
    
	for(i = 0; i < mosaic->height; i++){
		for(j = 0; j < mosaic->width; j++){
			mf = ((float *)(mosaic->imageData + i*mosaic->widthStep))[j];
			ef = ((float *)(edge_mosaic->imageData + i*edge_mosaic->widthStep))[j];
			
			if (ef == 128){
				upf =  ((float *)(mosaic->imageData + (i - delta)*mosaic->widthStep))[j];
				dwnf = ((float *)(mosaic->imageData + (i + delta)*mosaic->widthStep))[j];
				rgtf = ((float *)(mosaic->imageData + (i)*mosaic->widthStep))[j + delta];
				lftf = ((float *)(mosaic->imageData + (i)*mosaic->widthStep))[j - delta];
				if( (upf == 0) || (dwnf == 0) || (rgtf == 0) || (lftf == 0)){
				((float *)(mosaic->imageData + i*mosaic->widthStep))[j] = 0;
				}
				else 
					((float *)(mosaic->imageData + i*mosaic->widthStep))[j] = mf;
			}
		}
	}
	 
 return;
}

//extern IplImage* MosaicWithBlending(IplImage* prev_mosaic, IplImage* prev_binaryf, IplImage* mosaic, IplImage* circ_mask_u8, struct image_border border1, struct image_border border2, struct RegionS Snew, int status)
//extern void MosaicWithBlending(IplImage* prev_mosaic, IplImage* prev_binaryf, IplImage* mosaic, IplImage* binary_panoramaf, IplImage* circ_mask_u8, IplImage* C, struct image_border border1, struct image_border border2, struct RegionS Snew, int status)
extern IplImage* MosaicWithBlending(IplImage* prev_mosaic, IplImage* prev_binaryf, IplImage* mosaic, IplImage* binary_panoramaf, IplImage* circ_mask_u8, struct image_border border1, struct image_border border2, struct RegionS Snew, int status)
{
 //  IplImage* binary_panoramaf = NULL;
   IplImage* C = NULL, *im1 = NULL, *im2 = NULL, *im3 = NULL, *new_mask_u8 = NULL;
   //IplImage* im1 = NULL, *im2 = NULL, *im3 = NULL, *new_mask_u8 = NULL;
   IplImage* dst_mask_im1f = NULL, *dst_mask_im2f = NULL, *dst_mask_im1_u8 = NULL, *dst_mask_im2_u8 = NULL/*,  *dst1f = NULL*/;
   IplImage* EdgeMosaicf = NULL, *panorama5b_u8 = NULL;
   IplImage* C_u8 = NULL;
   struct image_border ROI_mosaic;


  //allocateOnDemand( &binary_panoramaf, cvGetSize(prev_mosaic), IPL_DEPTH_32F, 1 );
 
  allocateOnDemand( &im1, cvGetSize(prev_mosaic), IPL_DEPTH_32F, 1 );
  allocateOnDemand( &im2, cvGetSize(prev_mosaic), IPL_DEPTH_32F, 1 );
  allocateOnDemand( &im3, cvGetSize(prev_mosaic), IPL_DEPTH_32F, 1 );
  allocateOnDemand( &dst_mask_im1f, cvGetSize(prev_mosaic), IPL_DEPTH_32F, 1 );
  allocateOnDemand( &dst_mask_im2f, cvGetSize(prev_mosaic), IPL_DEPTH_32F, 1 );
  allocateOnDemand( &dst_mask_im1_u8, cvGetSize(prev_mosaic), IPL_DEPTH_8U, 3 );
  allocateOnDemand( &dst_mask_im2_u8, cvGetSize(prev_mosaic), IPL_DEPTH_8U, 3 );
  allocateOnDemand( &C_u8, cvGetSize(prev_mosaic), IPL_DEPTH_8U, 3 );
 // allocateOnDemand( &dst1f, cvGetSize(prev_mosaic), IPL_DEPTH_32F, 1 );
   

  im1 = convert2gray32(prev_mosaic); // is the image where we get the values for the blending
  im2 = convert2gray32(mosaic);	// is the initial image mosaic
  

  if( status == 1){
  printf(" status = 1 ... \n");
  WarpImageUnderQuad(circ_mask_u8,dst_mask_im2_u8,border2);
  WarpImageUnderQuad(circ_mask_u8,dst_mask_im1_u8,border1);
  
  prev_binaryf = FindInitMosaicRegion(im1, border1);  // previous binary mosaic
  
  binary_panoramaf = FindMosaicRegion(prev_binaryf, Snew);
  
  dst_mask_im1f = convert2gray32(dst_mask_im1_u8);
  dst_mask_im2f = convert2gray32(dst_mask_im2_u8);

  // refining the mask
  
  
  refine_mask_p(binary_panoramaf, dst_mask_im1f, dst_mask_im2f);
  dst_mask_im1_u8 = convert_gray32_to_color_blending(dst_mask_im1f);
  
  im3 = convert_to_gray32(dst_mask_im1_u8);
  }
 
  if(status == 2){
	printf(" status = 2 ... \n");
	ROI_mosaic = FindCornersMosaic(border1, border2);
    WarpImageUnderQuad(circ_mask_u8,dst_mask_im2_u8,border2);
    WarpImageUnderQuad(circ_mask_u8,dst_mask_im1_u8,ROI_mosaic);
	dst_mask_im1f = convert2gray32(dst_mask_im1_u8);
    dst_mask_im2f = convert2gray32(dst_mask_im2_u8);
	//allocateOnDemand( &prev_binaryf, cvGetSize(prev_mosaic), IPL_DEPTH_32F, 1 );
	//binary_panoramaf = FindMosaicRegion(prev_binaryf, Snew);
	//binary_panoramaf = cvCloneImage(prev_binaryf);
	allocateOnDemand( &EdgeMosaicf, cvGetSize(prev_mosaic), IPL_DEPTH_32F, 1 );
   	EdgeMosaicf = FindEdgeMosaic(binary_panoramaf);
	RefineEdgeBinaryMosaic(binary_panoramaf, EdgeMosaicf,1);
	refine_mask_p(binary_panoramaf, dst_mask_im1f, dst_mask_im2f);
	dst_mask_im1_u8 = convert_gray32_to_color_blending(dst_mask_im1f);
    im3 = convert_to_gray32(dst_mask_im1_u8);
    // allocateOnDemand( &panorama5b_u8, cvGetSize(prev_mosaic), IPL_DEPTH_8U, 3 );
	// panorama5b_u8 = convert_gray32_to_color_blending(binary_panoramaf);

	// cvSaveImage("dst_mask_mosaic_i.jpg",dst_mask_im1_u8);
    // cvSaveImage("panorama5_prev_i.jpg",prev_mosaic);
     //cvSaveImage("panorama5_i.jpg",mosaic);
     //cvSaveImage("panorama5b_u8_i.jpg", panorama5b_u8);
  }
  
  C = cvCreateImage(cvGetSize(mosaic), IPL_DEPTH_32F, 1 );
 
  C = BlendImageswithMask_p(im1, im2, im3, 4);

  C_u8 = convert_gray32_to_color_blending(C);

 cvNamedWindow( "result_blending_inside", 1 );
 cvShowImage( "result_blending_inside", C_u8 );
 cvWaitKey(0);

   return C;
  //return;
}

//extern void MosaicWithBlending(IplImage* prev_mosaic, IplImage* prev_binaryf, IplImage* mosaic, IplImage* binary_panoramaf, IplImage* circ_mask_u8, IplImage* C, struct image_border border1, struct image_border border2, struct RegionS Snew, int status)
extern void void_MosaicWithBlending(IplImage* prev_mosaic, IplImage* prev_binaryf, IplImage* mosaic, IplImage* binary_panoramaf, IplImage* C, IplImage* circ_mask_u8, struct image_border border1, struct image_border border2, struct RegionS Snew, int status)
{
 //  IplImage* binary_panoramaf = NULL;
   //IplImage* C = NULL, *im1 = NULL, *im2 = NULL, *im3 = NULL, *new_mask_u8 = NULL;
   IplImage* im1 = NULL, *im2 = NULL, *im3 = NULL, *new_mask_u8 = NULL;
   IplImage* dst_mask_im1f = NULL, *dst_mask_im2f = NULL, *dst_mask_im1_u8 = NULL, *dst_mask_im2_u8 = NULL/*,  *dst1f = NULL*/;
   IplImage* EdgeMosaicf = NULL, *panorama5b_u8 = NULL;
   IplImage* C_u8 = NULL;


  //allocateOnDemand( &binary_panoramaf, cvGetSize(prev_mosaic), IPL_DEPTH_32F, 1 );
 
  allocateOnDemand( &im1, cvGetSize(prev_mosaic), IPL_DEPTH_32F, 1 );
  allocateOnDemand( &im2, cvGetSize(prev_mosaic), IPL_DEPTH_32F, 1 );
  allocateOnDemand( &im3, cvGetSize(prev_mosaic), IPL_DEPTH_32F, 1 );
  allocateOnDemand( &dst_mask_im1f, cvGetSize(prev_mosaic), IPL_DEPTH_32F, 1 );
  allocateOnDemand( &dst_mask_im2f, cvGetSize(prev_mosaic), IPL_DEPTH_32F, 1 );
  allocateOnDemand( &dst_mask_im1_u8, cvGetSize(prev_mosaic), IPL_DEPTH_8U, 3 );
  allocateOnDemand( &dst_mask_im2_u8, cvGetSize(prev_mosaic), IPL_DEPTH_8U, 3 );
  allocateOnDemand( &C_u8, cvGetSize(prev_mosaic), IPL_DEPTH_8U, 3 );
 // allocateOnDemand( &dst1f, cvGetSize(prev_mosaic), IPL_DEPTH_32F, 1 );
   

  im1 = convert2gray32(prev_mosaic); // is the image where we get the values for the blending
  im2 = convert2gray32(mosaic);	// is the initial image mosaic
  

  if( status == 1){
  printf(" status = 1 ... \n");
  WarpImageUnderQuad(circ_mask_u8,dst_mask_im2_u8,border2);
  WarpImageUnderQuad(circ_mask_u8,dst_mask_im1_u8,border1);
  
  prev_binaryf = FindInitMosaicRegion(im1, border1);  // previous binary mosaic
  
  binary_panoramaf = FindMosaicRegion(prev_binaryf, Snew);
  
  dst_mask_im1f = convert2gray32(dst_mask_im1_u8);
  dst_mask_im2f = convert2gray32(dst_mask_im2_u8);

  // refining the mask
  
  
  refine_mask_p(binary_panoramaf, dst_mask_im1f, dst_mask_im2f);
  dst_mask_im1_u8 = convert_gray32_to_color_blending(dst_mask_im1f);
  
  im3 = convert_to_gray32(dst_mask_im1_u8);
  }
 
  if(status == 2){
	printf(" status = 2 ... \n");
    WarpImageUnderQuad(circ_mask_u8,dst_mask_im2_u8,border2);
    WarpImageUnderQuad(circ_mask_u8,dst_mask_im1_u8,border1);
	dst_mask_im1f = convert2gray32(dst_mask_im1_u8);
    dst_mask_im2f = convert2gray32(dst_mask_im2_u8);
	//allocateOnDemand( &prev_binaryf, cvGetSize(prev_mosaic), IPL_DEPTH_32F, 1 );
	//binary_panoramaf = FindMosaicRegion(prev_binaryf, Snew);
	//binary_panoramaf = cvCloneImage(prev_binaryf);
	allocateOnDemand( &EdgeMosaicf, cvGetSize(prev_mosaic), IPL_DEPTH_32F, 1 );
   	EdgeMosaicf = FindEdgeMosaic(binary_panoramaf);
	RefineEdgeBinaryMosaic(binary_panoramaf, EdgeMosaicf,1);
	refine_mask_p(binary_panoramaf, dst_mask_im1f, dst_mask_im2f);
	dst_mask_im1_u8 = convert_gray32_to_color_blending(dst_mask_im1f);
    im3 = convert_to_gray32(dst_mask_im1_u8);
     allocateOnDemand( &panorama5b_u8, cvGetSize(prev_mosaic), IPL_DEPTH_8U, 3 );
	 panorama5b_u8 = convert_gray32_to_color_blending(binary_panoramaf);

	 cvSaveImage("dst_mask_mosaic_i.jpg",dst_mask_im1_u8,0); 
     cvSaveImage("panorama5_prev_i.jpg",prev_mosaic,0);
     cvSaveImage("panorama5_i.jpg",mosaic,0);
     cvSaveImage("panorama5b_u8_i.jpg", panorama5b_u8,0);

	 /*
	 cvSaveImage("dst_mask_mosaic_i.jpg",dst_mask_im1_u8); 
     cvSaveImage("panorama5_prev_i.jpg",prev_mosaic);
     cvSaveImage("panorama5_i.jpg",mosaic);
     cvSaveImage("panorama5b_u8_i.jpg", panorama5b_u8);
	 */
  }
  
  //C = cvCreateImage(cvGetSize(mosaic), IPL_DEPTH_32F, 1 );
 
  C = BlendImageswithMask_p(im1, im2, im3, 4);

  C_u8 = convert_gray32_to_color_blending(C);

 cvNamedWindow( "result_blending_inside", 1 );
 cvShowImage( "result_blending_inside", C_u8 );
 cvWaitKey(0);


  return;
}



extern IplImage* Convert2ColumnImage(IplImage* img)
{
	IplImage* img_colum = NULL;
	CvSize size_img, size_ColumnImage;
	int m, n, i, j, k;
	float px_img, px_ColumnImage;
   
    size_img = cvGetSize(img);

	m = size_img.height;
	n = size_img.width;

	size_ColumnImage.height = m*n;
	size_ColumnImage.width = 1;

	//img_colum = cvCreateImage(size_ColumnImage, img->depth, img->nChannels);
	img_colum = cvCreateImage(size_ColumnImage, img->depth, img->nChannels);
    
	for( i = 0; i < img->height; i ++){
		for( j = 0; j < img->width; j++){
			px_ColumnImage = ((float *)(img->imageData + i*img->widthStep))[j];
			((float *)(img_colum->imageData + (i + m*j)*img_colum->widthStep))[0] = px_ColumnImage;
		}
	}

	return img_colum;
}

extern void Convert2ColumnMatrix(CvMat* matrix, CvMat* matrix_column)
{
	
	int i, j, k;
	int m = matrix->rows;
	int n = matrix->cols;

	float *matrix_data = matrix->data.fl;
	float *matrix_column_data = matrix_column->data.fl;
	
    
	for( j = 0; j < matrix->cols; j ++){
		for( i = 0; i < matrix->rows; i++){
           	matrix_column_data[0*1 + i + j*m] = matrix_data[i*n + j];
		}
	}

	return;
}
// Converts a column image into a matrix image
extern IplImage* Convert2MatrixImage(IplImage* img,int m, int n){
    int i,j,k;
	float temp;
	IplImage* MatrixImage = NULL;

	MatrixImage = cvCreateImage(cvSize(n,m), img->depth, img->nChannels);

	for( j = 0; j < n; j ++){ //columns
		for( i = 0; i < m; i++){ //rows
			k = i + m*j;
			temp = ((float *)(img->imageData + k*img->widthStep))[0];
           ((float *)(MatrixImage->imageData + i*MatrixImage->widthStep))[j] =  temp;
		}
	}
   return MatrixImage;

}

extern void ConvertColumn2MatrixCvMat(CvMat* vector, CvMat* Matrix, int m, int n){
    int i,j,k;
	float temp;
	

	float* matrix_data = Matrix->data.fl;
	float* matrix_column_data = vector->data.fl;
	
	

	for( j = 0; j < n; j ++){
		for( i = 0; i < m; i++){
			matrix_data[i*n + j] =	matrix_column_data[0*1 + i + j*m];
		}
	}
   return;

}

// This function computes the vectors: J, Jc, A, ep, epc, g and gc
extern void mrqcof(IplImage* inpimg, IplImage* inpinvimg, IplImage* J, IplImage* Jc, IplImage* ep, IplImage* epc, IplImage* gc, IplImage* A)
{
  IplImage* Jct = NULL;
  IplImage* Jt = NULL;
  IplImage* g = NULL;
  int m, n,i,j;

  /************************************************************************/
  //  Computing the Jacobian
  /****************************************************************************/
 printf("\n Computing the Jacobian ...\n" );

 cvSobel(inpinvimg,J,1, 1, 3);


  //*********************************************************************************************//
  //  Converting the Jimgs to column images										//
  //*********************************************************************************************//
    
  m = inpimg->height;
  n = inpimg->width;
   
  Jc = Convert2ColumnImage(J);
	

/***********************************************************************************/
//			Computing the Heissian for every Jimg           				    	//
//				A = JJ'																	//
/***********************************************************************************/
    printf("\n Calulating the transpose for the column images ...\n" );
	Jct = cvCreateImage(cvSize(m*n,1),inpimg->depth, inpimg->nChannels);
	cvTranspose(Jc, Jct);


	printf("\n Calulating the Hessian Matrix for every image ...\n" );
    cvMul(Jc, Jct, A,1);
	

/***********************************************************************************/
//			Computing the epk = yk - DBkWkR[x]k																//
/***********************************************************************************/
	printf("\n Calulating the error epk  ...\n" );

    // Check size of img1_inv

	cvSub(inpimg, inpinvimg, ep,0);
	   
	
   printf("\n Converting to column image ... \n" );
   epc = Convert2ColumnImage(ep);
 

/***********************************************************************************/
//			Computing gk (oneda, betha for the book "Numerical Recipes in C"       //
/***********************************************************************************/
    
  printf("\n Computing gk's ... \n" );
  Jt = cvCreateImage(cvSize(m,n),inpimg->depth, inpimg->nChannels);
  g = cvCreateImage(cvSize(n,m), inpimg->depth, inpimg->nChannels);

  cvTranspose(J, Jt);

  cvMul(Jt,ep,g,1);
  gc = Convert2ColumnImage(g);

  cvReleaseImage(&Jct);
  cvReleaseImage(&Jt);
  cvReleaseImage(&g);


  return;
  
 }

extern void mrqcof_modified(IplImage* img1, IplImage* img2, IplImage* img3, IplImage* img4, IplImage* img5, IplImage* prev_panorama, IplImage* panorama5d, IplImage* panorama5_Delta,
					struct image_border ROI_frame1, struct image_border ROI_frame2, struct image_border ROI_frame3, struct image_border ROI_frame4, struct image_border ROI_frame5,
					double* lambda_itp)//, int D)
					{

//IplImage* panoramaSR = NULL;
IplImage* img1_inv = NULL, *img2_inv = NULL, *img3_inv=NULL, *img4_inv=NULL, *img5_inv=NULL;
IplImage* img1_inv_temp1 = NULL, *img2_inv_temp1 = NULL, *img3_inv_temp1=NULL, *img4_inv_temp1=NULL, *img5_inv_temp1=NULL;
IplImage* img1_inv_temp2 = NULL, *img2_inv_temp2 = NULL, *img3_inv_temp2=NULL, *img4_inv_temp2=NULL, *img5_inv_temp2=NULL;

IplImage* dimg1=NULL, *dimg2=NULL, *dimg3=NULL, *dimg4=NULL, *dimg5=NULL;
IplImage* Ddimg1=NULL, *Ddimg2=NULL, *Ddimg3=NULL, *Ddimg4=NULL, *Ddimg5=NULL;
IplImage* ddst1 = NULL, *ddst2 = NULL, *ddst3 = NULL, *ddst4 = NULL, *ddst5=NULL;

IplImage* panorama5d2 = NULL; //after multiplication with alpha
IplImage* panorama5LLt = NULL;
IplImage* panorama5LLt2 = NULL; //after multiplication

IplImage* Xregularized = NULL; // To use with the algorithm Fast and Robust Regularization
IplImage* Limg = NULL;

CvMat* kernelLLt = cvCreateMat(3,3,CV_32FC1);
CvMat* kernelB = cvCreateMat(3,3,CV_32FC1);
CvMat* kernelL = cvCreateMat(3,3,CV_32FC1);

CvMat* lambdaM = NULL;

double lambda_1, lambda_2, lambda_3, lambda_4, lambda_5;
double lambda_num, lambda_den;
double lambdad_1, lambdad_2, lambdad_3, lambdad_4, lambdad_5;
double lambda_it;
double alpha_it = 1.1127;// was 7, 1, 1.1127
double errorSR;

float alpha = 0.0;

CvPoint2D32f P; // P for the mosaic

int i, j, D;

struct RegionS  S1, S2, S3, S4, S5;

CvSize sizeD1, sizeD2, sizeD3, sizeD4, sizeD5;
CvSize size; //size of the frames

//FILE *pFile;




	/// First part:  Interpolation : resizing the image to 2 times the original size

  D = 2;

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

	size.height = img1->height;
	size.width = img1->width;

/**************************************************************************************/ 
   //   Starting a new iteration (it = 3)
   //   
	//  First Step: Getting the imgs from the mosaic image
   //
   /**************************************************************************************/
/*
	allocateOnDemand( &img1_inv, cvSize(img2->width,img2->height),  img1->depth, img1->nChannels );
    WarpImageUnderQuadInv(prev_panorama,img1_inv,ROI_frame1);

    allocateOnDemand( &img2_inv, cvSize(img2->width,img2->height),  img1->depth, img1->nChannels );
    WarpImageUnderQuadInv(prev_panorama,img2_inv,ROI_frame2);

	allocateOnDemand( &img3_inv, cvSize(img2->width,img2->height),  img1->depth, img1->nChannels );
	WarpImageUnderQuadInv(prev_panorama,img3_inv,ROI_frame3);

	allocateOnDemand( &img4_inv, cvSize(img2->width,img2->height),  img1->depth, img1->nChannels );
	WarpImageUnderQuadInv(prev_panorama,img4_inv,ROI_frame4);

	allocateOnDemand( &img5_inv, cvSize(img2->width,img2->height),  img1->depth, img1->nChannels );
	WarpImageUnderQuadInv(prev_panorama,img5_inv,ROI_frame5);
*/

	allocateOnDemand( &img1_inv_temp1, sizeD1,  img1->depth, img1->nChannels );
    WarpImageUnderQuadInv(prev_panorama,img1_inv_temp1,ROI_frame1);

    allocateOnDemand( &img2_inv_temp1, sizeD1,  img1->depth, img1->nChannels );
    WarpImageUnderQuadInv(prev_panorama,img2_inv_temp1,ROI_frame2);

	allocateOnDemand( &img3_inv_temp1, sizeD1,  img1->depth, img1->nChannels );
	WarpImageUnderQuadInv(prev_panorama,img3_inv_temp1,ROI_frame3);

	allocateOnDemand( &img4_inv_temp1, sizeD1,  img1->depth, img1->nChannels );
	WarpImageUnderQuadInv(prev_panorama,img4_inv_temp1,ROI_frame4);

	allocateOnDemand( &img5_inv_temp1, sizeD1,  img1->depth, img1->nChannels );
	WarpImageUnderQuadInv(prev_panorama,img5_inv_temp1,ROI_frame5);

	// Using a Gaussian Filter to perform an Antia-alias filtering
	
	allocateOnDemand( &img1_inv_temp2, sizeD1,  img1->depth, img1->nChannels );
    allocateOnDemand( &img2_inv_temp2, sizeD1,  img1->depth, img1->nChannels );
    allocateOnDemand( &img3_inv_temp2, sizeD1,  img1->depth, img1->nChannels );
	allocateOnDemand( &img4_inv_temp2, sizeD1,  img1->depth, img1->nChannels );
	allocateOnDemand( &img5_inv_temp2, sizeD1,  img1->depth, img1->nChannels );
/*
	cvSmooth(img1_inv_temp1,img1_inv_temp2, CV_GAUSSIAN, 3, 0, 0, 0);
	cvSmooth(img2_inv_temp1,img2_inv_temp2, CV_GAUSSIAN, 3, 0, 0, 0);
	cvSmooth(img3_inv_temp1,img3_inv_temp2, CV_GAUSSIAN, 3, 0, 0, 0);
	cvSmooth(img4_inv_temp1,img4_inv_temp2, CV_GAUSSIAN, 3, 0, 0, 0);
	cvSmooth(img5_inv_temp1,img5_inv_temp2, CV_GAUSSIAN, 3, 0, 0, 0);
*/
	// Using Filter 2D:
    //This kernel is the same as the used in the paper: " A Transform-Domain Approach to Super-resolution Mosaicing of Compressed Images"

	cvmSet(kernelB,0,0,0.1016);
	cvmSet(kernelB,0,1,0.1172);
	cvmSet(kernelB,0,2,0.1016);
	cvmSet(kernelB,1,0,0.1172);
	cvmSet(kernelB,1,1,0.1250);
	cvmSet(kernelB,1,2,0.1172);
	cvmSet(kernelB,2,0,0.1016);
	cvmSet(kernelB,2,1,0.1172);
	cvmSet(kernelB,2,2,0.1016);

	cvFilter2D(img1_inv_temp1,img1_inv_temp2,kernelB,cvPoint(-1,1));
	cvFilter2D(img2_inv_temp1,img2_inv_temp2,kernelB,cvPoint(-1,1));
	cvFilter2D(img3_inv_temp1,img3_inv_temp2,kernelB,cvPoint(-1,1));
	cvFilter2D(img4_inv_temp1,img4_inv_temp2,kernelB,cvPoint(-1,1));
	cvFilter2D(img5_inv_temp1,img5_inv_temp2,kernelB,cvPoint(-1,1));
	
  // Downsampling using cubic interpolation

	allocateOnDemand( &img1_inv, size,  img1->depth, img1->nChannels );
    allocateOnDemand( &img2_inv, size,  img1->depth, img1->nChannels );
    allocateOnDemand( &img3_inv, size,  img1->depth, img1->nChannels );
	allocateOnDemand( &img4_inv, size,  img1->depth, img1->nChannels );
	allocateOnDemand( &img5_inv, size,  img1->depth, img1->nChannels );
	
	cvResize(img1_inv_temp2, img1_inv, CV_INTER_CUBIC);
	cvResize(img2_inv_temp2, img2_inv, CV_INTER_CUBIC);
	cvResize(img3_inv_temp2, img3_inv, CV_INTER_CUBIC);
	cvResize(img4_inv_temp2, img4_inv, CV_INTER_CUBIC);
	cvResize(img5_inv_temp2, img5_inv, CV_INTER_CUBIC);

	/*
// Downsampling using my own code

	DownSampleImage(img1_inv_temp2, img1_inv, 2);
	DownSampleImage(img2_inv_temp2, img2_inv, 2);
	DownSampleImage(img3_inv_temp2, img3_inv, 2);
	DownSampleImage(img4_inv_temp2, img4_inv, 2);
	DownSampleImage(img5_inv_temp2, img5_inv, 2);
*/
/*
	cvSaveImage("DBWRx1.jpg", img1_inv);
	cvSaveImage("DBWRx2.jpg", img2_inv);
	cvSaveImage("DBWRx3.jpg", img3_inv);
	cvSaveImage("DBWRx4.jpg", img4_inv);
	cvSaveImage("DBWRx5.jpg", img5_inv);


	cvNamedWindow( "img1_inv", 1 );
	cvShowImage( "img1_inv", img1_inv);
	cvWaitKey(0);

	cvNamedWindow( "img2_inv", 1 );
	cvShowImage( "img2_inv", img2_inv);
	cvWaitKey(0);

	cvNamedWindow( "img3_inv", 1 );
	cvShowImage( "img3_inv", img3_inv );
	cvWaitKey(0);

	cvNamedWindow( "img4_inv", 1 );
	cvShowImage( "img4_inv", img4_inv );
	cvWaitKey(0);

	cvNamedWindow( "img5_inv", 1 );
	cvShowImage( "img5_inv", img5_inv );
	cvWaitKey(0);
*/	

   /**************************************************************************************/ 
   //    
   //   
   //  Second Step: Compute the frame difference between the original low frames and the LR reconstructed ones
   //
   /**************************************************************************************/

  /************************************************************************/
  //  Doing the frame substracion
  /****************************************************************************/
 //  printf("\n Doing the frame substracion ...\n" );


  allocateOnDemand( &dimg1, cvSize(img2->width,img2->height),  img1->depth, img1->nChannels );
  allocateOnDemand( &dimg2, cvSize(img2->width,img2->height),  img1->depth, img1->nChannels );
  allocateOnDemand( &dimg3, cvSize(img2->width,img2->height),  img1->depth, img1->nChannels );
  allocateOnDemand( &dimg4, cvSize(img2->width,img2->height),  img1->depth, img1->nChannels );
  allocateOnDemand( &dimg5, cvSize(img2->width,img2->height),  img1->depth, img1->nChannels );
/*
   printf("\n size_img1 = (%d , %d) \t size_img1_inv = (%d, %d)", img1->height, img1->width, img1_inv->height, img1_inv->width);
   printf("\n size_dimg1 = (%d , %d) ", dimg1->height, dimg1->width);

   getchar();
*/

  cvSub(img1,img1_inv,dimg1,NULL);
  cvSub(img2,img2_inv,dimg2,NULL);
  cvSub(img3,img3_inv,dimg3,NULL);
  cvSub(img4,img4_inv,dimg4,NULL);
  cvSub(img5,img5_inv,dimg5,NULL);

 
  //*********************************************************************************************//
  //  Doing the interpolation of the images differences											//
  //*********************************************************************************************//
    //printf("\n Doing the frame interpolation of the difference of frames ...\n" );
    allocateOnDemand( &Ddimg1, sizeD1,  img1->depth, img1->nChannels );  
	allocateOnDemand( &Ddimg2, sizeD2,  img1->depth, img1->nChannels );  
	allocateOnDemand( &Ddimg3, sizeD3,  img1->depth, img1->nChannels );  
	allocateOnDemand( &Ddimg4, sizeD4,  img1->depth, img1->nChannels );  
	allocateOnDemand( &Ddimg5, sizeD5,  img1->depth, img1->nChannels );  

	/*
  	cvResize(dimg1, Ddimg1,CV_INTER_CUBIC);
	cvResize(dimg2, Ddimg2,CV_INTER_CUBIC);
	cvResize(dimg3, Ddimg3,CV_INTER_CUBIC);
	cvResize(dimg4, Ddimg4,CV_INTER_CUBIC);
	cvResize(dimg5, Ddimg5,CV_INTER_CUBIC);
	*/

	UpSampleImageD(dimg1, Ddimg1);
	UpSampleImageD(dimg2, Ddimg2);
	UpSampleImageD(dimg3, Ddimg3);
	UpSampleImageD(dimg4, Ddimg4);
	UpSampleImageD(dimg5, Ddimg5);

/***********************************************************************************/
//			 Doing the Mosaicking of the Error Images								//
//																					//
/***********************************************************************************/
	//printf("\n Doing the Mosaicking of the Error Images ...\n" );

	allocateOnDemand( &ddst1,  cvSize(prev_panorama->width,prev_panorama->height),  img1->depth, img1->nChannels );  
	allocateOnDemand( &ddst2,  cvSize(prev_panorama->width,prev_panorama->height),  img1->depth, img1->nChannels );  
	allocateOnDemand( &ddst3,  cvSize(prev_panorama->width,prev_panorama->height),  img1->depth, img1->nChannels );  
	allocateOnDemand( &ddst4,  cvSize(prev_panorama->width,prev_panorama->height),  img1->depth, img1->nChannels );  
	allocateOnDemand( &ddst5,  cvSize(prev_panorama->width,prev_panorama->height),  img1->depth, img1->nChannels );  
	
	/*
	allocateOnDemand( &ddst1,  cvSize(1360, 640),  img1->depth, img1->nChannels );  
	allocateOnDemand( &ddst2,  cvSize(1360, 640),  img1->depth, img1->nChannels );  
	allocateOnDemand( &ddst3,  cvSize(1360, 640),  img1->depth, img1->nChannels );  
	allocateOnDemand( &ddst4,  cvSize(1360, 640),  img1->depth, img1->nChannels );  
	allocateOnDemand( &ddst5,  cvSize(1360, 640),  img1->depth, img1->nChannels );  
*/

    WarpImageUnderQuad(Ddimg1,ddst1,ROI_frame1);
	//cvNamedWindow( "Perspective_Warpdd1", 1 );
	//cvShowImage( "Perspective_Warpdd1", ddst1);
	//cvWaitKey(10);


  WarpImageUnderQuad(Ddimg2,ddst2,ROI_frame2);

 // cvNamedWindow( "Perspective_Warpdd2", 1 );
 // cvShowImage( "Perspective_Warpdd2", ddst2 );
  //cvWaitKey(10);


 WarpImageUnderQuad(Ddimg3,ddst3,ROI_frame3);
 //cvNamedWindow( "Perspective_Warpdd3", 1 );
 //cvShowImage( "Perspective_Warpdd3", ddst3 );
 //cvWaitKey(10);


 WarpImageUnderQuad(Ddimg4,ddst4,ROI_frame4);
 //cvNamedWindow( "Perspective_Warpdd4", 1 );
 //cvShowImage( "Perspective_Warpdd4", ddst4 );
 //cvWaitKey(10);

 
 WarpImageUnderQuad(Ddimg5,ddst5,ROI_frame5);
 //cvNamedWindow( "Perspective_Warpdd5", 1 );
 ///cvShowImage( "Perspective_Warpdd5", ddst5 );
 //cvWaitKey(10);

// allocateOnDemand( &panorama5d,  cvSize(prev_panorama->width, prev_panorama->height),  img1->depth, img1->nChannels );  

 S2 = FindSRegion(ROI_frame2);

	  for(i = 0; i < panorama5d->height; i ++){ //rows
		  for(j = 0; j < panorama5d->width; j++){ //cols
			  P.x = j;
			  P.y = i;
			  if(Belongs2S(P,S2))cvSet2D(panorama5d,i,j,cvGet2D(ddst2,i,j));
			  else cvSet2D(panorama5d,i,j,cvGet2D(ddst1,i,j));			  
		  }//end for
	  }// end for

	

////////////////////////////
// Finding the region S3 //
////////////////////////////

   S3 = FindSRegion(ROI_frame3);

	  for(i = 0; i < panorama5d->height; i ++){ //rows
		  for(j = 0; j < panorama5d->width; j++){ //cols
			  P.x = j;
			  P.y = i;
			  if(Belongs2S(P,S3))cvSet2D(panorama5d,i,j,cvGet2D(ddst3,i,j));
			  else cvSet2D(panorama5d,i,j,cvGet2D(panorama5d,i,j));			  
		  }//end for
	  }// end for



 ////////////////////////////
// Finding the region S4 //
////////////////////////////

   S4 = FindSRegion(ROI_frame4);

	  for(i = 0; i < panorama5d->height; i ++){ //rows
		  for(j = 0; j < panorama5d->width; j++){ //cols
			  P.x = j;
			  P.y = i;
			  if(Belongs2S(P,S4))cvSet2D(panorama5d,i,j,cvGet2D(ddst4,i,j));
			  else cvSet2D(panorama5d,i,j,cvGet2D(panorama5d,i,j));			  
		  }//end for
	  }// end for


	    ////////////////////////////
		// Finding the region S4 //
		////////////////////////////

   S5 = FindSRegion(ROI_frame5);


	  for(i = 0; i < panorama5d->height; i ++){ //rows
		  for(j = 0; j < panorama5d->width; j++){ //cols
			  P.x = j;
			  P.y = i;
			  if(Belongs2S(P,S5))cvSet2D(panorama5d,i,j,cvGet2D(ddst5,i,j));
			  else cvSet2D(panorama5d,i,j,cvGet2D(panorama5d,i,j));			  
		  }//end for
	  }// end for

	// cvNamedWindow( "panorama5d", 1 );
	 //cvShowImage( "panorama5d", panorama5d );
	 //cvWaitKey(10);



   /**************************************************************************************/
   //
   // Computing the normalizing parameter lambda^(n) 
   //
   /**************************************************************************************/
    
	  lambda_1 = cvNorm(img1, img1_inv, CV_L2, NULL);
	 // printf("\n lambda_1 = %f", lambda_1);
	  lambda_2 = cvNorm(img2, img2_inv, CV_L2, NULL);
	 // printf("\n lambda_2 = %f", lambda_2);
	  lambda_3 = cvNorm(img3, img3_inv, CV_L2, NULL);
	 // printf("\n lambda_3 = %f", lambda_3);
	  lambda_4 = cvNorm(img4, img4_inv, CV_L2, NULL);
	 // printf("\n lambda_4 = %f", lambda_4);
	  lambda_5 = cvNorm(img5, img5_inv, CV_L2, NULL);
	 // printf("\n lambda_5 = %f", lambda_5);

   /**************************************************************************************/
   //
   // Computing the Laplace of the x^(n) image x(n) comes to be the panorama5 variables (mosaic of the previous state)
   //
   /**************************************************************************************/
   allocateOnDemand( &Limg, cvSize(prev_panorama->width,prev_panorama->height),  img1->depth, img1->nChannels );  
   //cvLaplace(prev_panorama, Limg,3);   
/*
// Uncomment this to use Laplace Prior Function

   // Using in stead cvFilter2D

   	cvmSet(kernelL,0,0,0.0000);
	cvmSet(kernelL,0,1,0.2500);
	cvmSet(kernelL,0,2,0.0000);
	cvmSet(kernelL,1,0,0.2500);
	cvmSet(kernelL,1,1,-1.000);
	cvmSet(kernelL,1,2,0.2500);
	cvmSet(kernelL,2,0,0.0000);
	cvmSet(kernelL,2,1,0.2500);
	cvmSet(kernelL,2,2,0.0000);

   cvFilter2D(prev_panorama, Limg, kernelL, cvPoint(-1,-1));
*/
 // Uncomment this to use Huber Prior Function
  // Using Huber Prior Function to find lambda

     alpha = 1.75;//was 1.75

     HubbertFunctionCliques(prev_panorama, Limg, alpha);

  // cvNamedWindow( "Limg", 1 );
  // cvShowImage( "Limg", Limg );
  // cvWaitKey(0);


   /**************************************************************************************/
   //
   // Computing the K norm(x^(n),2) , K = 1, 2 ...5
   //
   /**************************************************************************************/

  lambda_num = pow((lambda_1 + lambda_2 + lambda_3 + lambda_4 + lambda_5),2);
  lambdad_1 = cvNorm(Limg, NULL, CV_L2, NULL);
 

    /**************************************************************************************/
   //
   // Computing the lambda(n), K = 5, because I am just working with 5 frames
   //
   /**************************************************************************************/
 
  lambda_den = pow((5*lambdad_1),2);

    
  lambda_it = lambda_num/lambda_den;

  
  // printf("\n lambda_num = %f", lambda_num);
  // printf("\n lambda_den = %f", lambda_den);
  // printf("\n lambda_it = %f", lambda_it);

   *lambda_itp =  lambda_it;

  // printf("\n lambda_it = %f", *lambda_itp);

  // pFile = fopen(name, "w");

   //fprintf(pFile, "%f\n", lambda_it);

  // fclose(pFile);
   
  // lambda_it = 1.1127;

  // printf("\n Using alpha from GCV = %f", alpha_it);

    /**************************************************************************************/
   //
   // Computing the lambda(n)*L'*L*x(n)
   //
   /**************************************************************************************/

   /*
    cvmSet(kernelLLt,0,0,1.000*lambda_it);
	cvmSet(kernelLLt,0,1,-4.000*lambda_it);
	cvmSet(kernelLLt,0,2,1.000*lambda_it);
	cvmSet(kernelLLt,1,0,-4.000*lambda_it);
	cvmSet(kernelLLt,1,1,18.000*lambda_it);
	cvmSet(kernelLLt,1,2,-4.000*lambda_it);
	cvmSet(kernelLLt,2,0,1.000*lambda_it);
	cvmSet(kernelLLt,2,1,-4.000*lambda_it);
	cvmSet(kernelLLt,2,2,1.000*lambda_it);
*/
	cvmSet(kernelLLt,0,0,1.000);
	cvmSet(kernelLLt,0,1,-4.000);
	cvmSet(kernelLLt,0,2,1.000);
	cvmSet(kernelLLt,1,0,-4.000);
	cvmSet(kernelLLt,1,1,18.000);
	cvmSet(kernelLLt,1,2,-4.000);
	cvmSet(kernelLLt,2,0,1.000);
	cvmSet(kernelLLt,2,1,-4.000);
	cvmSet(kernelLLt,2,2,1.000);

	allocateOnDemand( &panorama5LLt, cvSize(prev_panorama->width,prev_panorama->height),  img1->depth, img1->nChannels );  
	allocateOnDemand( &panorama5LLt2, cvSize(prev_panorama->width,prev_panorama->height),  img1->depth, img1->nChannels );  
	allocateOnDemand( &panorama5d2, cvSize(prev_panorama->width,prev_panorama->height),  img1->depth, img1->nChannels ); 

 //	printf("\n Limg->height = %d \t Limg->width = %d", Limg->height, Limg->width);
 //	printf("\n prev_panorama->height = %d \t prev_panorama->width = %d", prev_panorama->height, prev_panorama->width);

  //  Uncomment to use Huber Regularization

    // Using Huber Regularization here as well
	HubbertRegularization(Limg, panorama5LLt, alpha);
/*
  // Uncomment to use Laplace Regularization
    // Using Laplace prior
	cvFilter2D(Limg,panorama5LLt,kernelL,cvPoint(-1,1));
	cvFilter2D(panorama5LLt,panorama5LLt2, kernelL,cvPoint(-1,1)); // Because os L^t*L*x
*/	

/*
    //Using the regularization proposed in the paper: "Fast and Robust Super-Resolution"

	allocateOnDemand( &Xregularized, cvSize(prev_panorama->width,prev_panorama->height), IPL_DEPTH_32F, 1 );

	FastRobustRegularization(Limg, Xregularized, 2, alpha); // p = 2

	panorama5LLt = convert_gray32_to_color(Xregularized);

*/

	/*
	//
	// Converting to Matrix lambda doesn't work, because of the size of the mosaicking
	//

	lambdaM = cvCreateMat(panorama5LLt->height,panorama5LLt->width,CV_32FC1);
	cvSetIdentity(lambdaM,cvRealScalar(lambda_it));

	cvMatMul(panorama5LLt,lambdaM, panorama5LLt2);
	*/

	panorama5LLt2 = MulImagewithDouble(panorama5LLt, lambda_it);// Use this for Huber Regularization
	
	//panorama5LLt2 = MulImagewithDouble(panorama5LLt2, lambda_it); // Use this for Laplace Regularization
	
	panorama5d2 = MulImagewithDouble(panorama5d, alpha_it);

   /**************************************************************************************/ 
   //
   //	Now updating the iteration of x(n+1). First I have to do: panorama5d - panorama5LLt
   //   Then I have to do : panorma5_2 = panorama5_1 + (panorama5d - panorama5LLt)
   //
   /**************************************************************************************/
	
	//allocateOnDemand( &panoramaSR, cvSize(prev_panorama->width,prev_panorama->height),  img1->depth, img1->nChannels );  
	//allocateOnDemand( &panorama5_Delta, cvSize(prev_panorama->width,prev_panorama->height),  img1->depth, img1->nChannels );  

	cvSub(panorama5d2,panorama5LLt2,panorama5_Delta,NULL);
	//cvAdd(panorama5_Delta,prev_panorama,panoramaSR,NULL);


	/***********************************************************************************
	Computing the error of the SR
	*****************************************************************************************/

	//errorSR = cvNorm(panoramaSR, prev_panorama, CV_RELATIVE_L2, NULL);

//	printf(" \n errorSR = %f\n", errorSR);

	//cvNamedWindow( "panoramaSR", 1 );
	//cvShowImage( "panoramaSR", panoramaSR );
	//cvWaitKey(10);

	cvReleaseMat( &kernelLLt );
	cvReleaseMat( &kernelL );
	cvReleaseMat( &kernelB );
	cvReleaseMat( &lambdaM );

	return;

}

// Implementation of LevenbergMarquardt Algorithm for Non Linear Systems
// x should be the current panorama
// inputs: p0 -> Initital SR Mosaicking...
// output: p, which is the SR mosaicking
//extern void LevenbergMarquardtNL(IplImage* p0, IplImage* p, IplImage *A, IplImage* gc, IplImage* pc, IplImage* x)
extern void LevenbergMarquardtNL(IplImage* img1, IplImage* img2, IplImage* img3, IplImage* img4, IplImage* img5, struct image_border ROI_frame1, struct image_border ROI_frame2, struct image_border ROI_frame3, struct image_border ROI_frame4, struct image_border ROI_frame5, IplImage* p0, IplImage* p)
{
    IplImage *A = NULL, *ep = NULL, *J = NULL, *epc = NULL, *Jc = NULL, *gc = NULL, *pc = NULL;
	IplImage* Ainv = NULL, * Jct = NULL, *Jft = NULL, *g = NULL;
	IplImage* delta_p = NULL, *ep_new = NULL, *epf_new = NULL, *pf = NULL, *p0f = NULL, *p0cf = NULL;
	IplImage* epf = NULL, *Jf = NULL, *delta_pt = NULL, *pnew = NULL;
	IplImage* prev_panorama = NULL, *prev_panorama_u8 = NULL;
	IplImage* temp_pro_img = NULL, *p_mn = NULL;


	CvSize size_panorama;
	double norminf_g;
	float max_Adiag, v, pro, temp_pro_float;
	double epsilon1, epsilon2, epsilon3, tau, mu;
	double norm_delta_p, norm_p, norm_ep, norm_ep_new;
	double* lambda_it;
	int stop, status; // will be zero or one
	int k, kmax;
	int i, j, m, n;
	


    tau = .001;
	epsilon1 = epsilon2 = epsilon3 = 1e-15;
	kmax = 10;
	size_panorama = cvGetSize(p);
	m = size_panorama.height;
	n = size_panorama.width;

	A = cvCreateImage(cvSize(m*n, m*n),  IPL_DEPTH_32F, 1 );
	Ainv = cvCreateImage(cvSize(m*n, m*n),  IPL_DEPTH_32F, 1 );
	delta_p = cvCreateImage(cvSize(1, m*n),  IPL_DEPTH_32F, 1 );
	epc = cvCreateImage(cvSize(1,m*n),  IPL_DEPTH_32F, 1 );
	Jc = cvCreateImage(cvSize(1,m*n),  IPL_DEPTH_32F, 1 );
	ep = cvCreateImage(size_panorama,  IPL_DEPTH_8U, 3 );
	ep_new = cvCreateImage(size_panorama,  IPL_DEPTH_8U, 3 );
	epf_new = cvCreateImage(size_panorama,  IPL_DEPTH_32F, 1 );
	pf = cvCreateImage(cvSize(1,m*n),  IPL_DEPTH_32F, 1 );
	pnew = cvCreateImage(cvSize(1,m*n),  IPL_DEPTH_32F, 1 );
	p0f = cvCreateImage(cvSize(n,m),  IPL_DEPTH_32F, 1 );
	p0cf = cvCreateImage(cvSize(1,m*n),  IPL_DEPTH_32F, 1 );
	J = cvCreateImage(size_panorama,  IPL_DEPTH_8U, 3 );
	p_mn = cvCreateImage(size_panorama,   IPL_DEPTH_32F, 1 );

	lambda_it = calloc( kmax, sizeof( double ) );

    // Computing ep, and J
	// I have to change the formats for the images in this function
   mrqcof_modified(img1, img2, img3, img4, img5, p0, ep, J, ROI_frame1, ROI_frame2, ROI_frame3, ROI_frame4, ROI_frame5, lambda_it);

    // Converting the images from u8 to floats
   allocateOnDemand( &epf, size_panorama, IPL_DEPTH_32F, 1 );
   allocateOnDemand( &Jf, size_panorama, IPL_DEPTH_32F, 1 );
   allocateOnDemand( &Jft, size_panorama, IPL_DEPTH_32F, 1 ); //m = n
   allocateOnDemand( &Jct, cvSize(m*n,1), IPL_DEPTH_32F, 1 );
   epf = convert2gray32(ep);
   Jf =  convert2gray32(J);

   epc = Convert2ColumnImage(epf);
   Jc = Convert2ColumnImage(Jf);

   printf("Computing Jct ... \n" );

   cvTranspose(Jc, Jct);
   cvTranspose(Jf, Jft);



    printf("Computing g ... \n" );
	allocateOnDemand( &g, size_panorama, IPL_DEPTH_32F, 1 );
	cvMul(Jft, epf,g,1);
	gc = Convert2ColumnImage(g);
	norminf_g = cvNorm(gc,0, CV_C, 0);
    
	printf("Computing A ... \n" );
	multvector_vectort(Jf, Jft, A);
    

	printf("Computing mu ... \n" );
	max_Adiag = MaxValueDiagonal(A);
	mu = tau*max_Adiag;

	printf("Allocating memory ... \n" );

    allocateOnDemand( &delta_pt, cvSize(m*n,1), IPL_DEPTH_32F, 1 );
	allocateOnDemand( &temp_pro_img, cvSize(1,m*n), IPL_DEPTH_32F, 1 );
	allocateOnDemand( &prev_panorama, size_panorama, IPL_DEPTH_32F, 1 );
	allocateOnDemand( &prev_panorama_u8, size_panorama, IPL_DEPTH_8U, 3 );
			  
	
    p0f = convert2gray32(p0);
	p0cf = Convert2ColumnImage(p0f);

	if( norminf_g <= epsilon1) stop  = 1;
	else stop = 0;
    
	k = 0; 
	v = 2.0;
	pf = cvCloneImage(p0cf);
	printf("Entering in the loop ... \n" );
	while( ~stop && k < kmax){
      k++;
	  do{
		  ContructAuI(A,mu);
		  // computing delta
		  printf("Computing delta ... \n" );
		  cvInvert(A,Ainv, CV_SVD);
		  cvMul(Ainv,gc,delta_p,1);
          
		  cvTranspose(delta_p, delta_pt);
		
		  printf("Computing norms ... \n" );
		  norm_p = cvNorm(pf,0,CV_L2,0);
		  norm_delta_p = cvNorm(delta_p, 0, CV_L2, 0);

		  printf("norm_delta_p = %f \t epsilon2*norm_p = %f =  \n", norm_delta_p, epsilon2*norm_p );

		  if(norm_delta_p <= epsilon2*norm_p)
			  stop = 1;
		  else{
               cvAdd(pf,delta_p, pnew,0); // pnew becomes the prev_panorama, but pnew is in column format,...
			    // I have to change to matrix format
			   //panorama5d, is f(pnew) and panorama5_Delta is the Jacobian
			   // the output is ep and J, the rest are inputs
               
			   prev_panorama = Convert2MatrixImage(pnew, m, n);
			   prev_panorama_u8 = convert_gray32_to_color_blending(prev_panorama);
               
			   norm_ep = cvNorm(epf,0,CV_L2,0);
			   cvZero(ep);
			   cvZero(J);
			   cvZero(epf);
			   cvZero(Jf);
               
			   printf("Computing the news ep and Js ... \n" );
			   mrqcof_modified(img1, img2, img3, img4, img5, prev_panorama_u8, ep_new, J, ROI_frame1, ROI_frame2, ROI_frame3, ROI_frame4, ROI_frame5, lambda_it);

			    epf_new = convert2gray32(ep_new);
				
				norm_ep_new = cvNorm(epf_new, 0, CV_L2, 0);

				 
               printf("Computing pro ... \n" );
			      temp_pro_img = MulImagewithDouble(delta_p, mu);
				  cvAdd(temp_pro_img,gc,temp_pro_img,0);
				  temp_pro_float = multvectort_vector(delta_pt, temp_pro_img);
				  pro = (norm_ep*norm_ep - norm_ep_new*norm_ep_new)/temp_pro_float;

				  if( pro > 0.0){
                   pf = cvCloneImage(pnew);
				   printf("Computing A ... \n" );
				   Jf =  convert2gray32(J);
				   cvTranspose(Jf,Jft);
				   multvector_vectort(Jf, Jft, A);
				   // ep = x - fp
				   epf = cvCloneImage(epf_new);
				   norm_ep = norm_ep_new;

				   printf("Computing g ... \n" );
	               cvMul(Jft, epf,g,1);
				   gc = Convert2ColumnImage(g);
				   norminf_g = cvNorm(gc,0, CV_C, 0);

				   if (norminf_g <= epsilon1 || norm_ep*norm_ep < epsilon3) stop = 1;
				   else stop = 0;

				   mu = mu*MAX(0.10, 1 - (2*pro - 1,3)*(2*pro - 1,3)*(2*pro - 1,3)); //modified to be more accurate
				  }
				  else{
					  mu *= v;
					  v *= 5;
				  }

		  }

	  }while(pro <= 0.0 || ~stop);
	}
    p_mn = Convert2MatrixImage(pf, m, n);

    p = convert_gray32_to_color_blending(p_mn);

    cvReleaseImage(&A);
    cvReleaseImage(&Ainv);
    cvReleaseImage(&delta_p);
	cvReleaseImage(&epc);
	cvReleaseImage(&Jc);
	cvReleaseImage(&ep);
	cvReleaseImage(&ep_new);
	cvReleaseImage(&epf_new);
	cvReleaseImage(&pf);
	cvReleaseImage(&pnew);
	cvReleaseImage(&p0f);
	cvReleaseImage(&p0cf);
	cvReleaseImage(&J);
	cvReleaseImage(&p_mn);


	free(lambda_it);
	
	return;
}


// Implementation of LevenbergMarquardt Algorithm for Non Linear Systems
// x should be the current panorama
// inputs: p0 -> Initital SR Mosaicking...
// output: p, which is the SR mosaicking
//extern void LevenbergMarquardtNL(IplImage* p0, IplImage* p, IplImage *A, IplImage* gc, IplImage* pc, IplImage* x)

extern void LevenbergMarquardtNL_Matrix(IplImage* img1, IplImage* img2, IplImage* img3, IplImage* img4, IplImage* img5, struct image_border ROI_frame1, struct image_border ROI_frame2, struct image_border ROI_frame3, struct image_border ROI_frame4, struct image_border ROI_frame5, IplImage* p0, IplImage* p)
{
    CvMat *A = NULL, *ep = NULL, *J = NULL, *epc = NULL, *Jc = NULL, *gc = NULL, *pc = NULL;
	CvMat* Ainv = NULL, * Jct = NULL, *Jft = NULL, *g = NULL, *Jtf = NULL;
	CvMat* delta_p = NULL, *ep_new = NULL, *epf_new = NULL, *pf = NULL, *p0f = NULL, *p0cf = NULL;
	CvMat* epf = NULL, *Jf = NULL, *delta_pt = NULL, *pnew = NULL, *pnew_matrix = NULL;
	IplImage* prev_panorama = NULL, *prev_panorama_u8 = NULL, *ep_img = NULL, *J_img = NULL, *Jimgf = NULL;
	IplImage* p0f_img = NULL, *epf_new_imgf = NULL, *p_mn_img = NULL, *ep_imgf = NULL, *J_imgf = NULL;
	CvMat* temp_pro_img = NULL, *p_mn = NULL;


	CvSize size_panorama;
	double norminf_g;
	float max_Adiag, v, pro, temp_pro_float;
	double epsilon1, epsilon2, epsilon3, tau, mu;
	double norm_delta_p, norm_p, norm_ep, norm_ep_new;
	double* lambda_it;
	int stop, status; // will be zero or one
	int k, kmax;
	int i, j, m, n;
	

    tau = .001;
	epsilon1 = epsilon2 = epsilon3 = 1e-3;
	kmax = 1;
	size_panorama = cvGetSize(p);
	m = size_panorama.height;
	n = size_panorama.width;

	A = cvCreateMat(m*n, m*n, CV_32FC1);//row, cols, type
	Ainv = cvCreateMat(m*n, m*n, CV_32FC1);
	delta_p = cvCreateMat(m*n, 1,CV_32FC1 );
	epc = cvCreateMat(m*n, 1, CV_32FC1 );
	Jc = cvCreateMat(m*n, 1, CV_32FC1 );
	ep = cvCreateMat(m, n, CV_32FC1);
	ep_new = cvCreateMat(m, n, CV_32FC1);
	epf_new = cvCreateMat(m, m, CV_32FC1);
	pf = cvCreateMat(m*n, 1, CV_32FC1);
	pnew = cvCreateMat(m*n, 1, CV_32FC1);
	pnew_matrix = cvCreateMat(m, n, CV_32FC1);
	p0f = cvCreateMat(m, n, CV_32FC1);
	p0cf = cvCreateMat(m*n, 1, CV_32FC1);
	J = cvCreateMat(m, n, CV_32FC1);
	p_mn = cvCreateMat(m, n, CV_32FC1); 
	epf = cvCreateMat( m, n, CV_32FC1); 
	Jf = cvCreateMat( m, n, CV_32FC1); 
	Jft = cvCreateMat( n, m, CV_32FC1); 
	Jtf = cvCreateMat( n, m, CV_32FC1); 
	Jct = cvCreateMat( 1, m*n, CV_32FC1); 
	g = cvCreateMat( m, n, CV_32FC1); 
	gc = cvCreateMat( m*n, 1, CV_32FC1); 

	delta_pt = cvCreateMat( 1, m*n, CV_32FC1); 
	temp_pro_img = cvCreateMat( m*n, 1, CV_32FC1); 
	
	allocateOnDemand( &ep_img, cvSize(n,m), p0->depth, p0->nChannels );
	allocateOnDemand( &J_img, cvSize(n,m), p0->depth, p0->nChannels );

	lambda_it = calloc( kmax, sizeof( double ) );

    // Computing ep, and J which are the outputs 
	// I have to change the formats for the images in this function
   mrqcof_modified(img1, img2, img3, img4, img5, p0, ep_img, J_img, ROI_frame1, ROI_frame2, ROI_frame3, ROI_frame4, ROI_frame5, lambda_it);

    // Converting the images from u8 to floats
   allocateOnDemand( &ep_imgf, size_panorama, IPL_DEPTH_32F, 1 );
   allocateOnDemand( &J_imgf, size_panorama, IPL_DEPTH_32F, 1 );
   ep_imgf = convert2gray32(ep_img);
   J_imgf = convert2gray32(J_img);

   convertIplImage2CvMat(ep_imgf, epf);
   convertIplImage2CvMat(J_imgf, Jf);

   printf("Converting images to matrix format data type ... \n" );
   
   Convert2ColumnMatrix(epf, epc);
   Convert2ColumnMatrix(Jf, Jc);

   printf("Computing Jct ... \n" );

   cvTranspose(Jc, Jct);
  
   cvTranspose(Jf, Jft);
  
    printf("Computing g ... \n" );
	
	cvMatMul(Jft, epf,g);
	Convert2ColumnMatrix(g, gc);
	norminf_g = cvNorm(gc,0, CV_C, 0);
    
	printf("Computing A ... \n" );
	//multvector_vectort(Jf, Jft, A);
	//multvector_vectort_cvMat(Jf, J
	cvMatMul(Jc,Jct,A);
    

	printf("Computing mu ... \n" );
	max_Adiag = MaxValueDiagonalMatrix(A);
	mu = tau*max_Adiag;

	printf("Allocating memory ... \n" );

   
	allocateOnDemand( &prev_panorama, size_panorama, IPL_DEPTH_32F, 1 );
	allocateOnDemand( &p0f_img, size_panorama, IPL_DEPTH_32F, 1 );
	allocateOnDemand( &prev_panorama_u8, size_panorama, IPL_DEPTH_8U, 3 );
	allocateOnDemand( &epf_new_imgf, size_panorama, IPL_DEPTH_32F, 1 );
	allocateOnDemand( &Jimgf, size_panorama, IPL_DEPTH_32F, 1 );
			  
	
   p0f_img = convert2gray32(p0);
   convertIplImage2CvMat(p0f_img, p0f);
   Convert2ColumnMatrix(p0f, p0cf);


	if( norminf_g <= epsilon1) stop  = 1;
	else stop = 0;
    
	k = 0; 
	v = 2.0;
	pf = cvCloneMat(p0cf);

	 ConvertColumn2MatrixCvMat(pf, pnew_matrix, m,n);
	 convertCvMat2IplImage(pnew_matrix, prev_panorama);
	 prev_panorama_u8 = convert_gray32_to_color_blending(prev_panorama);

	 cvNamedWindow( "prev_panoramaA", 1 );
     cvShowImage( "prev_panoramaA", prev_panorama_u8 );
	 cvWaitKey(0);



	printf("Entering in the loop ... \n" );
	while( !stop && k < kmax){
      k = k + 1;
	  do{
		  printf(" iteration = %d \n", k);
		  printf(" stop = %d \n", stop);
		  ContructAuI_Matrix(A,mu);
		  // computing delta
		  printf("Computing delta ... \n" );
		  cvSolve(A, gc, delta_p, CV_LU);

		 		
		  printf("Computing norms ... \n" );
		  norm_p = cvNorm(pf,0,CV_L2,0);
		  norm_delta_p = cvNorm(delta_p, 0, CV_L2, 0);

		  printf("norm_delta_p = %f \t epsilon2*norm_p = %f =  \n", norm_delta_p, epsilon2*norm_p );

		  if(norm_delta_p <= epsilon2*norm_p)
			  stop = 1;
		  else{
               cvAdd(pf,delta_p, pnew,0); // pnew becomes the prev_panorama, but pnew is in column format,...
			    // I have to change to matrix format
			   //panorama5d, is f(pnew) and panorama5_Delta is the Jacobian
			   // the output is ep and J, the rest are inputs
               //ConvertColumn2MatrixCvMat(pnew,  pnew_matrix, m,n);
			   ConvertColumn2MatrixCvMat(pnew, pnew_matrix, m,n);
			   convertCvMat2IplImage(pnew_matrix, prev_panorama);
			   prev_panorama_u8 = convert_gray32_to_color_blending(prev_panorama);
               
			   norm_ep = cvNorm(epf,0,CV_L2,0);
			   cvZero(ep);
			   cvZero(J);
			   cvZero(epf);
			   cvZero(Jf);
               
			   printf("Computing the news ep and Js ... \n" );

			   cvNamedWindow( "prev_panoramaB", 1 );
			   cvShowImage( "prev_panoramaB", prev_panorama_u8 );
			   cvWaitKey(0);

			   mrqcof_modified(img1, img2, img3, img4, img5, prev_panorama_u8, ep_img, J_img, ROI_frame1, ROI_frame2, ROI_frame3, ROI_frame4, ROI_frame5, lambda_it);

			    epf_new_imgf = convert2gray32(ep_img);
				convertIplImage2CvMat(epf_new_imgf, epf_new);
	
				norm_ep_new = cvNorm(epf_new, 0, CV_L2, 0);
  			 
                  printf("Computing pro ... \n" );
			      MulMatrixwithDouble(delta_p, (float)mu, temp_pro_img);
				  cvAdd(temp_pro_img,gc,temp_pro_img,0);
				  printf("Computing deltat ... \n" );  
				  cvTranspose(delta_p, delta_pt);
				  temp_pro_float = multvectort_vector_CvMat(delta_pt, temp_pro_img);
				  pro = (norm_ep_new*norm_ep_new - norm_ep*norm_ep)/temp_pro_float;
				  printf("pro = %f ... \n", pro );

				  if( pro > 0.0){
                   pf = cvCloneMat(pnew);
				   printf("(inside pro > 0.0): Computing A ... \n" );
				   Jimgf = convert2gray32(J_img);
				   convertIplImage2CvMat(Jimgf, Jf);

				   Convert2ColumnMatrix(Jf, Jc);

				   printf("Computing Jct ... \n" );
				   cvTranspose(Jc, Jct);
				   cvTranspose(Jf, Jft);

				   
				  // Convert2ColumnMatrix(g, gc);
				   cvTranspose(Jf,Jft);
				   cvMatMul(Jc, Jct, A);
				   // ep = x - fp
				   epf = cvCloneMat(epf_new);
				   norm_ep = norm_ep_new;

				   printf("Computing g ... \n" );
	               cvMatMul(Jft, epf,g,1);
				   Convert2ColumnMatrix(g, gc);
				   norminf_g = cvNorm(gc,0, CV_C, 0);

				   if (norminf_g <= epsilon1 || norm_ep*norm_ep <= epsilon3) stop = 1;
				   else stop = 0;
                   printf("stop = %d ... \n", stop );
				   mu = mu*MAX(0.33, 1 - (2*pro - 1)*(2*pro - 1)*(2*pro - 1)); //modified to be more accurate
				  }
				  else{
					  mu *= v;
					  v *= 2;//was 5
				  }
		  }
       printf("pro = %f \t stop = %d... \t !stop = %d \n", pro, stop, !stop );
       printf("pro <= 0.0 && !stop = %d \n", (pro <= 0.0 && !stop) );
	  }while(pro <= 0.0 && !stop);
	  printf("Leaving the loop :).. \n" );
	}
    ConvertColumn2MatrixCvMat(pf,p_mn, m, n);

	allocateOnDemand( &p_mn_img, size_panorama, IPL_DEPTH_32F, 1 );
	convertCvMat2IplImage(p_mn, p_mn_img);


    p = convert_gray32_to_color_blending(p_mn_img);

    cvReleaseMat(&A);
    cvReleaseMat(&Ainv);
    cvReleaseMat(&delta_p);
	cvReleaseMat(&epc);
	cvReleaseMat(&Jc);
	cvReleaseMat(&ep);
	cvReleaseMat(&ep_new);
	cvReleaseMat(&epf_new);
	cvReleaseMat(&pf);
	cvReleaseMat(&pnew);
	cvReleaseMat(&p0f);
	cvReleaseMat(&p0cf);
	cvReleaseMat(&J);
	cvReleaseMat(&p_mn);
	cvReleaseMat(&epf);
	cvReleaseMat(&Jf);
	cvReleaseMat(&Jtf);
	cvReleaseMat(&Jct);
	cvReleaseMat(&g);
	cvReleaseMat(&gc);
	cvReleaseMat(&Jtf);
	cvReleaseMat(&delta_pt);
	cvReleaseMat(&temp_pro_img);


	free(lambda_it);
	
	return;
}





extern double MaxValueDiagonal(IplImage* A)
{
  int i,j;
  float max_value;
  float temp_value;
  max_value = ((float *)(A->imageData + (0)*A->widthStep))[0];
  for( i = 0; i < A->height; i ++){
	  for(j = 0; j < A->width; j++){
		  if( i == j){ // Working with the diagonal elements
			temp_value =  ((float *)(A->imageData + i*A->widthStep))[j];
			max_value = MAX(temp_value, max_value);
		  }
	  }
  }
  return max_value;

}

extern double MaxValueDiagonalMatrix(CvMat* A)
{
  int i,j;
  float max_value;
  float temp_value;
  int n = A->cols;
  float* A_data = A->data.fl;

  max_value = A_data[0*n + 0];
  for( i = 0; i < A->rows; i ++){
	  for(j = 0; j < A->cols; j++){
		  if( i == j){ // Working with the diagonal elements
			temp_value =  A_data[i*n + j];
			max_value = MAX(temp_value, max_value);
		  }
	  }
  }
  return max_value;

}
//Constructs the matrix A + muI
extern void ContructAuI(IplImage* A, double mu)
{
  int i,j;
  float temp_value;

  for( i = 0; i < A->height; i ++){
	  for(j = 0; j < A->width; j++){
		  if (i == j){ // Working with the diagonal elements
			 temp_value = ((float *)(A->imageData + i*A->widthStep))[j];
			 temp_value += (float)mu;
			((float *)(A->imageData + i*A->widthStep))[j] = temp_value;		
		 }
	  }
  }
  return;

}

//Constructs the matrix A + muI
extern void ContructAuI_Matrix(CvMat* A, double mu)
{
  int i,j;
  float temp_value;
  int n = A->cols;
  float* A_data = A->data.fl;

  for( i = 0; i < A->rows; i ++){
	  for(j = 0; j < A->cols; j++){
		  if (i == j){ // Working with the diagonal elements
			 temp_value = A_data[i*n + j];
			 temp_value += (float)mu;
			 A_data[i*n + j] = temp_value;		
		 }
	  }
  }
  return;

}

extern float multvectort_vector(IplImage* vectort, IplImage* vector)
{
   int i, j;
   float result;
   float value_vectort, value_vector;

   result = 0.0;
   for( i = 0; i < vectort->width; i++){
       value_vectort = ((float *)(vectort->imageData + 0*vectort->widthStep))[i];
	   value_vector = ((float *)(vector->imageData + i*vector->widthStep))[0];	
	   result += value_vectort*value_vector;
   }

  return result;
}

extern float multvectort_vector_CvMat(CvMat* vectort, CvMat* vector)
{
   int i;
   float result;
   float value_vectort, value_vector;
   float* vectort_data = vectort->data.fl;
   float* vector_data = vector->data.fl;


   result = 0.0;
   for( i = 0; i < vectort->cols; i++){
	   value_vectort = vectort_data[i];
	   value_vector = vector_data[i];
	   result += value_vectort*value_vector;
   }

  return result;
}


extern void multvector_vectort(IplImage* vector, IplImage* vectort, IplImage* result)
{
   int i, j;

   float value_vectort, value_vector, resultf;

   for( i = 0; i < vector->height; i++){
	   for(j = 0; j < vectort->width; j++){
	   value_vectort = ((float *)(vectort->imageData + 0*vectort->widthStep))[j];
	   value_vector = ((float *)(vector->imageData + i*vector->widthStep))[0];	
	   resultf = value_vectort*value_vector;
	   ((float *)(result->imageData + i*result->widthStep))[j] = resultf;		
	   }
   }

  return;
}

extern void multvector_vectort_cvMat(CvMat* vector, CvMat* vectort, CvMat* result)
{
   int i, j, k;

   float value_vectort, value_vector, resultf;

   int n = result->cols;


   float* vector_data = vector->data.fl;
   float* vectort_data = vectort->data.fl;
   float* result_data = result->data.fl;

   for( i = 0; i < vector->rows; i++){
	   for(j = 0; j < vectort->cols; j++){
	   value_vector = vector_data[i];
	   value_vectort = vectort_data[j];
	   result_data[i*n + j] = value_vector*value_vectort;	   
	   }
   }

  return;
}

extern void convertIplImage2CvMat(IplImage* img, CvMat* matrix)
{
   int i, j;

   int n = matrix->cols;
   float *value_matrix = matrix->data.fl;

   float value_img;

   for( i = 0; i < img->height; i++){
	   for(j = 0; j < img->width; j++){
	   value_img = ((float *)(img->imageData + i*img->widthStep))[j];
	   value_matrix[i*n + j] = value_img;	   
	   }
   }

  return;
}

extern void convertCvMat2IplImage(CvMat* matrix, IplImage* img)
{
   int i, j;

   int n = matrix->cols;
   float *value_matrix = matrix->data.fl;

   float value_img;

   for( i = 0; i < img->height; i++){
	   for(j = 0; j < img->width; j++){
	   value_img = value_matrix[i*n + j];	   
	   ((float *)(img->imageData + i*img->widthStep))[j] = value_img;
	    }
   }

  return;
}

extern void MulMatrixwithDouble(CvMat* A, float factor, CvMat* Result)
{
 
 int  i, j, k;
 float temp;
 int n = A->cols;

 float* A_data = A->data.fl;

 
 for(i = 0; i < A->rows; i++){ //rows
	 for( j = 0; j < A->cols; j++) { //cols
		 temp = A_data[i*n + j];
		 temp *= factor;
		 A_data[i*n + j] = temp;		
	 } // end for 
 } // end for
 return;

}

extern void SRMosaicking_Gauss_Newton(IplImage* img1, IplImage* img2, IplImage* img3, IplImage* img4, IplImage* img5, IplImage* prev_panorama, 
					struct image_border ROI_frame1, struct image_border ROI_frame2, struct image_border ROI_frame3, struct image_border ROI_frame4, struct image_border ROI_frame5,
					IplImage* panoramaSR, double* lambda_itp,double* errorSRp, int it)//, int *status)//, int D)
					{

//IplImage* panoramaSR = NULL;
IplImage* img1_inv = NULL, *img2_inv = NULL, *img3_inv=NULL, *img4_inv=NULL, *img5_inv=NULL;
IplImage* img1_inv_temp1 = NULL, *img2_inv_temp1 = NULL, *img3_inv_temp1=NULL, *img4_inv_temp1=NULL, *img5_inv_temp1=NULL;
IplImage* img1_inv_temp2 = NULL, *img2_inv_temp2 = NULL, *img3_inv_temp2=NULL, *img4_inv_temp2=NULL, *img5_inv_temp2=NULL;

IplImage* dimg1=NULL, *dimg2=NULL, *dimg3=NULL, *dimg4=NULL, *dimg5=NULL;
IplImage* Ddimg1=NULL, *Ddimg2=NULL, *Ddimg3=NULL, *Ddimg4=NULL, *Ddimg5=NULL;
IplImage* ddst1 = NULL, *ddst2 = NULL, *ddst3 = NULL, *ddst4 = NULL, *ddst5=NULL;
IplImage* panorama5d = NULL, *delta_SRf = NULL, *delta_SR_u8, *panorama5_Deltaf = NULL, *panorama5df = NULL;
IplImage* panorama5d2 = NULL, *panorama5_J = NULL, *panorama5_Jf = NULL; //after multiplication with alpha
IplImage* panorama5LLt = NULL, *panorama5_H_inv_Jf = NULL, *panorama5_H_inv_J_u8 = NULL;
IplImage* panorama5LLt2 = NULL, *panorama5_Jnf = NULL, *gf = NULL; //after multiplication
IplImage* panorama5_Delta = NULL, *Sumdimgkf = NULL, *Sumdimgk_u8 = NULL, *Sumdimgkf_panorama_size = NULL;
IplImage* Xregularized = NULL, *panorama5_H_invf = NULL, *panorama5_H_inv_u8 = NULL, *Sumdimgkf_panorama_size_u8 = NULL; // To use with the algorithm Fast and Robust Regularization
IplImage* Limg = NULL, *panorama5_Jtf = NULL, *panorama5_Jt_u8 = NULL, *panorama5_Hf = NULL, *panorama5_H_u8 = NULL;
IplImage *prev_panoramaf = NULL, *panoramaSRf = NULL;

CvMat* kernelLLt = cvCreateMat(3,3,CV_32FC1);
CvMat* kernelB = cvCreateMat(3,3,CV_32FC1);
CvMat* kernelL = cvCreateMat(3,3,CV_32FC1);

CvMat* lambdaM = NULL;
CvMat* A = NULL, *delta_p = NULL, *delta_p_matrix = NULL,  *J = NULL, *Jc = NULL, * Jct = NULL,*g = NULL, *gc = NULL;

double lambda_1, lambda_2, lambda_3, lambda_4, lambda_5;
double lambda_num, lambda_den;
double lambdad_1, lambdad_2, lambdad_3, lambdad_4, lambdad_5;
double lambda_it;
double alpha_it = 1.1127;// was 7, 1, 1.1127
double errorSR = 0.0;

float alpha = 0.0;
float pix_value_panorama5_Deltaf, pix_value_prev_panoramaf, pixvalue_panorama5_Hf, pixvalue_panorama5_Jtf, pixvalue_panorama5_H_invf;

CvPoint2D32f P; // P for the mosaic

int i, j, D, m,n;

struct RegionS  S1, S2, S3, S4, S5;

CvSize sizeD1, sizeD2, sizeD3, sizeD4, sizeD5;
CvSize size; //size of the frames

//FILE *pFile;

cvZero(panoramaSR);


	/// First part:  Interpolation : resizing the image to 2 times the original size

  D = 2;

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

	size.height = img1->height;
	size.width = img1->width;

/**************************************************************************************/ 
   //   Starting a new iteration (it = 3)
   //   
	//  First Step: Getting the imgs from the mosaic image
   //
   /**************************************************************************************/
/*
	allocateOnDemand( &img1_inv, cvSize(img2->width,img2->height),  img1->depth, img1->nChannels );
    WarpImageUnderQuadInv(prev_panorama,img1_inv,ROI_frame1);

    allocateOnDemand( &img2_inv, cvSize(img2->width,img2->height),  img1->depth, img1->nChannels );
    WarpImageUnderQuadInv(prev_panorama,img2_inv,ROI_frame2);

	allocateOnDemand( &img3_inv, cvSize(img2->width,img2->height),  img1->depth, img1->nChannels );
	WarpImageUnderQuadInv(prev_panorama,img3_inv,ROI_frame3);

	allocateOnDemand( &img4_inv, cvSize(img2->width,img2->height),  img1->depth, img1->nChannels );
	WarpImageUnderQuadInv(prev_panorama,img4_inv,ROI_frame4);

	allocateOnDemand( &img5_inv, cvSize(img2->width,img2->height),  img1->depth, img1->nChannels );
	WarpImageUnderQuadInv(prev_panorama,img5_inv,ROI_frame5);
*/

	allocateOnDemand( &img1_inv_temp1, sizeD1,  img1->depth, img1->nChannels );
    WarpImageUnderQuadInv(prev_panorama,img1_inv_temp1,ROI_frame1);

    allocateOnDemand( &img2_inv_temp1, sizeD1,  img1->depth, img1->nChannels );
    WarpImageUnderQuadInv(prev_panorama,img2_inv_temp1,ROI_frame2);

	allocateOnDemand( &img3_inv_temp1, sizeD1,  img1->depth, img1->nChannels );
	WarpImageUnderQuadInv(prev_panorama,img3_inv_temp1,ROI_frame3);

	allocateOnDemand( &img4_inv_temp1, sizeD1,  img1->depth, img1->nChannels );
	WarpImageUnderQuadInv(prev_panorama,img4_inv_temp1,ROI_frame4);

	allocateOnDemand( &img5_inv_temp1, sizeD1,  img1->depth, img1->nChannels );
	WarpImageUnderQuadInv(prev_panorama,img5_inv_temp1,ROI_frame5);

	// Using a Gaussian Filter to perform an Antia-alias filtering
	
	allocateOnDemand( &img1_inv_temp2, sizeD1,  img1->depth, img1->nChannels );
    allocateOnDemand( &img2_inv_temp2, sizeD1,  img1->depth, img1->nChannels );
    allocateOnDemand( &img3_inv_temp2, sizeD1,  img1->depth, img1->nChannels );
	allocateOnDemand( &img4_inv_temp2, sizeD1,  img1->depth, img1->nChannels );
	allocateOnDemand( &img5_inv_temp2, sizeD1,  img1->depth, img1->nChannels );
/*
	cvSmooth(img1_inv_temp1,img1_inv_temp2, CV_GAUSSIAN, 3, 0, 0, 0);
	cvSmooth(img2_inv_temp1,img2_inv_temp2, CV_GAUSSIAN, 3, 0, 0, 0);
	cvSmooth(img3_inv_temp1,img3_inv_temp2, CV_GAUSSIAN, 3, 0, 0, 0);
	cvSmooth(img4_inv_temp1,img4_inv_temp2, CV_GAUSSIAN, 3, 0, 0, 0);
	cvSmooth(img5_inv_temp1,img5_inv_temp2, CV_GAUSSIAN, 3, 0, 0, 0);
*/
	// Using Filter 2D:
    //This kernel is the same as the used in the paper: " A Transform-Domain Approach to Super-resolution Mosaicing of Compressed Images"

	cvmSet(kernelB,0,0,0.1016);
	cvmSet(kernelB,0,1,0.1172);
	cvmSet(kernelB,0,2,0.1016);
	cvmSet(kernelB,1,0,0.1172);
	cvmSet(kernelB,1,1,0.1250);
	cvmSet(kernelB,1,2,0.1172);
	cvmSet(kernelB,2,0,0.1016);
	cvmSet(kernelB,2,1,0.1172);
	cvmSet(kernelB,2,2,0.1016);

	cvFilter2D(img1_inv_temp1,img1_inv_temp2,kernelB,cvPoint(-1,1));
	cvFilter2D(img2_inv_temp1,img2_inv_temp2,kernelB,cvPoint(-1,1));
	cvFilter2D(img3_inv_temp1,img3_inv_temp2,kernelB,cvPoint(-1,1));
	cvFilter2D(img4_inv_temp1,img4_inv_temp2,kernelB,cvPoint(-1,1));
	cvFilter2D(img5_inv_temp1,img5_inv_temp2,kernelB,cvPoint(-1,1));
	
  // Downsampling using cubic interpolation

	allocateOnDemand( &img1_inv, size,  img1->depth, img1->nChannels );
    allocateOnDemand( &img2_inv, size,  img1->depth, img1->nChannels );
    allocateOnDemand( &img3_inv, size,  img1->depth, img1->nChannels );
	allocateOnDemand( &img4_inv, size,  img1->depth, img1->nChannels );
	allocateOnDemand( &img5_inv, size,  img1->depth, img1->nChannels );
	
	cvResize(img1_inv_temp2, img1_inv, CV_INTER_CUBIC);
	cvResize(img2_inv_temp2, img2_inv, CV_INTER_CUBIC);
	cvResize(img3_inv_temp2, img3_inv, CV_INTER_CUBIC);
	cvResize(img4_inv_temp2, img4_inv, CV_INTER_CUBIC);
	cvResize(img5_inv_temp2, img5_inv, CV_INTER_CUBIC);

	/*
// Downsampling using my own code

	DownSampleImage(img1_inv_temp2, img1_inv, 2);
	DownSampleImage(img2_inv_temp2, img2_inv, 2);
	DownSampleImage(img3_inv_temp2, img3_inv, 2);
	DownSampleImage(img4_inv_temp2, img4_inv, 2);
	DownSampleImage(img5_inv_temp2, img5_inv, 2);
*/
/*
	cvSaveImage("DBWRx1.jpg", img1_inv);
	cvSaveImage("DBWRx2.jpg", img2_inv);
	cvSaveImage("DBWRx3.jpg", img3_inv);
	cvSaveImage("DBWRx4.jpg", img4_inv);
	cvSaveImage("DBWRx5.jpg", img5_inv);


	cvNamedWindow( "img1_inv", 1 );
	cvShowImage( "img1_inv", img1_inv);
	cvWaitKey(0);

	cvNamedWindow( "img2_inv", 1 );
	cvShowImage( "img2_inv", img2_inv);
	cvWaitKey(0);

	cvNamedWindow( "img3_inv", 1 );
	cvShowImage( "img3_inv", img3_inv );
	cvWaitKey(0);

	cvNamedWindow( "img4_inv", 1 );
	cvShowImage( "img4_inv", img4_inv );
	cvWaitKey(0);

	cvNamedWindow( "img5_inv", 1 );
	cvShowImage( "img5_inv", img5_inv );
	cvWaitKey(0);
*/	

   /**************************************************************************************/ 
   //    
   //   
   //  Second Step: Compute the frame difference between the original low frames and the LR reconstructed ones
   //
   /**************************************************************************************/

  /************************************************************************/
  //  Doing the frame substracion
  /****************************************************************************/
 //  printf("\n Doing the frame substracion ...\n" );


  allocateOnDemand( &dimg1, cvSize(img2->width,img2->height),  img1->depth, img1->nChannels );
  allocateOnDemand( &dimg2, cvSize(img2->width,img2->height),  img1->depth, img1->nChannels );
  allocateOnDemand( &dimg3, cvSize(img2->width,img2->height),  img1->depth, img1->nChannels );
  allocateOnDemand( &dimg4, cvSize(img2->width,img2->height),  img1->depth, img1->nChannels );
  allocateOnDemand( &dimg5, cvSize(img2->width,img2->height),  img1->depth, img1->nChannels );
/*
   printf("\n size_img1 = (%d , %d) \t size_img1_inv = (%d, %d)", img1->height, img1->width, img1_inv->height, img1_inv->width);
   printf("\n size_dimg1 = (%d , %d) ", dimg1->height, dimg1->width);

   getchar();
*/

  cvSub(img1,img1_inv,dimg1,NULL);
  cvSub(img2,img2_inv,dimg2,NULL);
  cvSub(img3,img3_inv,dimg3,NULL);
  cvSub(img4,img4_inv,dimg4,NULL);
  cvSub(img5,img5_inv,dimg5,NULL);

 
  //*********************************************************************************************//
  //  Doing the interpolation of the images differences											//
  //*********************************************************************************************//
    //printf("\n Doing the frame interpolation of the difference of frames ...\n" );
    allocateOnDemand( &Ddimg1, sizeD1,  img1->depth, img1->nChannels );  
	allocateOnDemand( &Ddimg2, sizeD2,  img1->depth, img1->nChannels );  
	allocateOnDemand( &Ddimg3, sizeD3,  img1->depth, img1->nChannels );  
	allocateOnDemand( &Ddimg4, sizeD4,  img1->depth, img1->nChannels );  
	allocateOnDemand( &Ddimg5, sizeD5,  img1->depth, img1->nChannels );  

	/*
  	cvResize(dimg1, Ddimg1,CV_INTER_CUBIC);
	cvResize(dimg2, Ddimg2,CV_INTER_CUBIC);
	cvResize(dimg3, Ddimg3,CV_INTER_CUBIC);
	cvResize(dimg4, Ddimg4,CV_INTER_CUBIC);
	cvResize(dimg5, Ddimg5,CV_INTER_CUBIC);
	*/

	UpSampleImageD(dimg1, Ddimg1);
	UpSampleImageD(dimg2, Ddimg2);
	UpSampleImageD(dimg3, Ddimg3);
	UpSampleImageD(dimg4, Ddimg4);
	UpSampleImageD(dimg5, Ddimg5);

/***********************************************************************************/
//			 Doing the Mosaicking of the Error Images								//
//																					//
/***********************************************************************************/
	//printf("\n Doing the Mosaicking of the Error Images ...\n" );

	allocateOnDemand( &ddst1,  cvSize(panoramaSR->width,panoramaSR->height),  img1->depth, img1->nChannels );  
	allocateOnDemand( &ddst2,  cvSize(panoramaSR->width,panoramaSR->height),  img1->depth, img1->nChannels );  
	allocateOnDemand( &ddst3,  cvSize(panoramaSR->width,panoramaSR->height),  img1->depth, img1->nChannels );  
	allocateOnDemand( &ddst4,  cvSize(panoramaSR->width,panoramaSR->height),  img1->depth, img1->nChannels );  
	allocateOnDemand( &ddst5,  cvSize(panoramaSR->width,panoramaSR->height),  img1->depth, img1->nChannels );  
	
	/*
	allocateOnDemand( &ddst1,  cvSize(1360, 640),  img1->depth, img1->nChannels );  
	allocateOnDemand( &ddst2,  cvSize(1360, 640),  img1->depth, img1->nChannels );  
	allocateOnDemand( &ddst3,  cvSize(1360, 640),  img1->depth, img1->nChannels );  
	allocateOnDemand( &ddst4,  cvSize(1360, 640),  img1->depth, img1->nChannels );  
	allocateOnDemand( &ddst5,  cvSize(1360, 640),  img1->depth, img1->nChannels );  
*/

    WarpImageUnderQuad(Ddimg1,ddst1,ROI_frame1);
	//cvNamedWindow( "Perspective_Warpdd1", 1 );
	//cvShowImage( "Perspective_Warpdd1", ddst1);
	//cvWaitKey(10);


  WarpImageUnderQuad(Ddimg2,ddst2,ROI_frame2);

 // cvNamedWindow( "Perspective_Warpdd2", 1 );
 // cvShowImage( "Perspective_Warpdd2", ddst2 );
  //cvWaitKey(10);


 WarpImageUnderQuad(Ddimg3,ddst3,ROI_frame3);
 //cvNamedWindow( "Perspective_Warpdd3", 1 );
 //cvShowImage( "Perspective_Warpdd3", ddst3 );
 //cvWaitKey(10);


 WarpImageUnderQuad(Ddimg4,ddst4,ROI_frame4);
 //cvNamedWindow( "Perspective_Warpdd4", 1 );
 //cvShowImage( "Perspective_Warpdd4", ddst4 );
 //cvWaitKey(10);

 
 WarpImageUnderQuad(Ddimg5,ddst5,ROI_frame5);
 //cvNamedWindow( "Perspective_Warpdd5", 1 );
 ///cvShowImage( "Perspective_Warpdd5", ddst5 );
 //cvWaitKey(10);

 allocateOnDemand( &panorama5d,  cvSize(prev_panorama->width, prev_panorama->height),  img1->depth, img1->nChannels );  

 S2 = FindSRegion(ROI_frame2);

	  for(i = 0; i < panorama5d->height; i ++){ //rows
		  for(j = 0; j < panorama5d->width; j++){ //cols
			  P.x = j;
			  P.y = i;
			  if(Belongs2S(P,S2))cvSet2D(panorama5d,i,j,cvGet2D(ddst2,i,j));
			  else cvSet2D(panorama5d,i,j,cvGet2D(ddst1,i,j));			  
		  }//end for
	  }// end for

	

////////////////////////////
// Finding the region S3 //
////////////////////////////

   S3 = FindSRegion(ROI_frame3);

	  for(i = 0; i < panorama5d->height; i ++){ //rows
		  for(j = 0; j < panorama5d->width; j++){ //cols
			  P.x = j;
			  P.y = i;
			  if(Belongs2S(P,S3))cvSet2D(panorama5d,i,j,cvGet2D(ddst3,i,j));
			  else cvSet2D(panorama5d,i,j,cvGet2D(panorama5d,i,j));			  
		  }//end for
	  }// end for



 ////////////////////////////
// Finding the region S4 //
////////////////////////////

   S4 = FindSRegion(ROI_frame4);

	  for(i = 0; i < panorama5d->height; i ++){ //rows
		  for(j = 0; j < panorama5d->width; j++){ //cols
			  P.x = j;
			  P.y = i;
			  if(Belongs2S(P,S4))cvSet2D(panorama5d,i,j,cvGet2D(ddst4,i,j));
			  else cvSet2D(panorama5d,i,j,cvGet2D(panorama5d,i,j));			  
		  }//end for
	  }// end for


	    ////////////////////////////
		// Finding the region S4 //
		////////////////////////////

   S5 = FindSRegion(ROI_frame5);


	  for(i = 0; i < panorama5d->height; i ++){ //rows
		  for(j = 0; j < panorama5d->width; j++){ //cols
			  P.x = j;
			  P.y = i;
			  if(Belongs2S(P,S5))cvSet2D(panorama5d,i,j,cvGet2D(ddst5,i,j));
			  else cvSet2D(panorama5d,i,j,cvGet2D(panorama5d,i,j));			  
		  }//end for
	  }// end for

	cvNamedWindow( "panorama5d", 1 );
	cvShowImage( "panorama5d", panorama5d );
	cvWaitKey(10);



   /**************************************************************************************/
   //
   // Computing the normalizing parameter lambda^(n) 
   //
   /**************************************************************************************/
    
	  lambda_1 = cvNorm(img1, img1_inv, CV_L2, NULL);
	 // printf("\n lambda_1 = %f", lambda_1);
	  lambda_2 = cvNorm(img2, img2_inv, CV_L2, NULL);
	 // printf("\n lambda_2 = %f", lambda_2);
	  lambda_3 = cvNorm(img3, img3_inv, CV_L2, NULL);
	 // printf("\n lambda_3 = %f", lambda_3);
	  lambda_4 = cvNorm(img4, img4_inv, CV_L2, NULL);
	 // printf("\n lambda_4 = %f", lambda_4);
	  lambda_5 = cvNorm(img5, img5_inv, CV_L2, NULL);
	 // printf("\n lambda_5 = %f", lambda_5);

   /**************************************************************************************/
   //
   // Computing the Laplace of the x^(n) image x(n) comes to be the panorama5 variables (mosaic of the previous state)
   //
   /**************************************************************************************/
   allocateOnDemand( &Limg, cvSize(prev_panorama->width,prev_panorama->height),  img1->depth, img1->nChannels );  
   //cvLaplace(prev_panorama, Limg,3);   
/*
// Uncomment this to use Laplace Prior Function

   // Using in stead cvFilter2D

   	cvmSet(kernelL,0,0,0.0000);
	cvmSet(kernelL,0,1,0.2500);
	cvmSet(kernelL,0,2,0.0000);
	cvmSet(kernelL,1,0,0.2500);
	cvmSet(kernelL,1,1,-1.000);
	cvmSet(kernelL,1,2,0.2500);
	cvmSet(kernelL,2,0,0.0000);
	cvmSet(kernelL,2,1,0.2500);
	cvmSet(kernelL,2,2,0.0000);

   cvFilter2D(prev_panorama, Limg, kernelL, cvPoint(-1,-1));
*/
 // Uncomment this to use Huber Prior Function
  // Using Huber Prior Function to find lambda

     alpha = 1.75;//was 1.75

     HubbertFunctionCliques(prev_panorama, Limg, alpha);

  // cvNamedWindow( "Limg", 1 );
  // cvShowImage( "Limg", Limg );
  // cvWaitKey(0);


   /**************************************************************************************/
   //
   // Computing the K norm(x^(n),2) , K = 1, 2 ...5
   //
   /**************************************************************************************/

  lambda_num = pow((lambda_1 + lambda_2 + lambda_3 + lambda_4 + lambda_5),2);
  lambdad_1 = cvNorm(Limg, NULL, CV_L2, NULL);
 

    /**************************************************************************************/
   //
   // Computing the lambda(n), K = 5, because I am just working with 5 frames
   //
   /**************************************************************************************/
 
  lambda_den = pow((5*lambdad_1),2);

    
  lambda_it = lambda_num/lambda_den;

  
  // printf("\n lambda_num = %f", lambda_num);
  // printf("\n lambda_den = %f", lambda_den);
  // printf("\n lambda_it = %f", lambda_it);

   *lambda_itp =  lambda_it;

   printf("lambda_it = %f \n", *lambda_itp);

   

  // pFile = fopen(name, "w");

   //fprintf(pFile, "%f\n", lambda_it);

  // fclose(pFile);
   
  // lambda_it = 1.1127;

  // printf("\n Using alpha from GCV = %f", alpha_it);

    /**************************************************************************************/
   //
   // Computing the lambda(n)*L'*L*x(n)
   //
   /**************************************************************************************/

   /*
    cvmSet(kernelLLt,0,0,1.000*lambda_it);
	cvmSet(kernelLLt,0,1,-4.000*lambda_it);
	cvmSet(kernelLLt,0,2,1.000*lambda_it);
	cvmSet(kernelLLt,1,0,-4.000*lambda_it);
	cvmSet(kernelLLt,1,1,18.000*lambda_it);
	cvmSet(kernelLLt,1,2,-4.000*lambda_it);
	cvmSet(kernelLLt,2,0,1.000*lambda_it);
	cvmSet(kernelLLt,2,1,-4.000*lambda_it);
	cvmSet(kernelLLt,2,2,1.000*lambda_it);
*/
	cvmSet(kernelLLt,0,0,1.000);
	cvmSet(kernelLLt,0,1,-4.000);
	cvmSet(kernelLLt,0,2,1.000);
	cvmSet(kernelLLt,1,0,-4.000);
	cvmSet(kernelLLt,1,1,18.000);
	cvmSet(kernelLLt,1,2,-4.000);
	cvmSet(kernelLLt,2,0,1.000);
	cvmSet(kernelLLt,2,1,-4.000);
	cvmSet(kernelLLt,2,2,1.000);

	allocateOnDemand( &panorama5LLt, cvSize(prev_panorama->width,prev_panorama->height),  img1->depth, img1->nChannels );  
	allocateOnDemand( &panorama5LLt2, cvSize(prev_panorama->width,prev_panorama->height),  img1->depth, img1->nChannels );  
	allocateOnDemand( &panorama5d2, cvSize(prev_panorama->width,prev_panorama->height),  img1->depth, img1->nChannels ); 

 //	printf("\n Limg->height = %d \t Limg->width = %d", Limg->height, Limg->width);
 //	printf("\n prev_panorama->height = %d \t prev_panorama->width = %d", prev_panorama->height, prev_panorama->width);

  //  Uncomment to use Huber Regularization

    // Using Huber Regularization here as well
	HubbertRegularization(Limg, panorama5LLt, alpha);
/*
  // Uncomment to use Laplace Regularization
    // Using Laplace prior
	cvFilter2D(Limg,panorama5LLt,kernelL,cvPoint(-1,1));
	cvFilter2D(panorama5LLt,panorama5LLt2, kernelL,cvPoint(-1,1)); // Because os L^t*L*x
*/	

/*
    //Using the regularization proposed in the paper: "Fast and Robust Super-Resolution"

	allocateOnDemand( &Xregularized, cvSize(prev_panorama->width,prev_panorama->height), IPL_DEPTH_32F, 1 );

	FastRobustRegularization(Limg, Xregularized, 2, alpha); // p = 2

	panorama5LLt = convert_gray32_to_color(Xregularized);

*/

	/*
	//
	// Converting to Matrix lambda doesn't work, because of the size of the mosaicking
	//

	lambdaM = cvCreateMat(panorama5LLt->height,panorama5LLt->width,CV_32FC1);
	cvSetIdentity(lambdaM,cvRealScalar(lambda_it));

	cvMatMul(panorama5LLt,lambdaM, panorama5LLt2);
	*/

	panorama5LLt2 = MulImagewithDouble(panorama5LLt, lambda_it);// Use this for Huber Regularization
	
	//panorama5LLt2 = MulImagewithDouble(panorama5LLt2, lambda_it); // Use this for Laplace Regularization
	
	panorama5d2 = MulImagewithDouble(panorama5d, alpha_it);

	allocateOnDemand( &panorama5_Delta, cvSize(prev_panorama->width,prev_panorama->height),  img1->depth, img1->nChannels );  

	cvSub(panorama5d2,panorama5LLt2,panorama5_Delta,NULL);

	// panorama5_Delta is the Jacobian including the regularization term

	//cvSub(panorama5d,panorama5LLt2,panorama5_Delta,NULL);

	printf(" it = %d \n", it);

	//printf("\n errorSRp[it - 2] > = errorSRp[it - 3] = %d \n", ((it == 1 )||(errorSRp[it - 2] >= errorSRp[it - 3])));

	//if(it == 1 || errorSRp[it - 2] >= errorSRp[it - 3] ){
	//if(it == 1 || *status == 1){
//     printf(" status = %d \n", *status);
   /**************************************************************************************/ 
   //
   //	Now updating the iteration of x(n+1). First I have to do: panorama5d - panorama5LLt
   //   Then I have to do : panorma5_2 = panorama5_1 + (panorama5d - panorama5LLt)
   //   This part computes the SR based on Steep Descent 
   /**************************************************************************************/
	//if(it == 1){
	//printf(" \ initializing the SR computation using SD \n");
	//allocateOnDemand( &panoramaSR, cvSize(prev_panorama->width,prev_panorama->height),  img1->depth, img1->nChannels );  
	//allocateOnDemand( &error_SR_img, cvSize(prev_panorama->width,prev_panorama->height),  img1->depth, img1->nChannels );  
	
	//cvAdd(panorama5_Delta,prev_panorama,panoramaSR,NULL);

	//error_SR_img = cv
	//}

	//cvNamedWindow( "panoramaSR", 1 );
	//cvShowImage( "panoramaSR", panoramaSR );
	//cvWaitKey(0);

	//cvSaveImage("panoramaSR_A.jpg",panoramaSR);
	//}
	//else{

	/***********************************************************************************
	Computing delta to update the SR for Gaussian Newton
	J = panorama5_Delta;
	rk = b - A(yk)xk = Sum(dimgk)k=1toK (K = 5 frames by now)
	*****************************************************************************************/
  //  printf(" status = %d \n", *status);
	printf(" Computing SR using Gauss Newton ..... \n");

	allocateOnDemand( &delta_SRf, cvSize(prev_panorama->width,prev_panorama->height),   IPL_DEPTH_32F, 1 );
	allocateOnDemand( &delta_SR_u8, cvSize(prev_panorama->width,prev_panorama->height),   IPL_DEPTH_32F, 1 );
	allocateOnDemand( &panorama5_Deltaf, cvSize(prev_panorama->width,prev_panorama->height),   IPL_DEPTH_32F, 1 );
	allocateOnDemand( &panorama5_Jf, cvSize(prev_panorama->width,prev_panorama->height),   IPL_DEPTH_32F, 1 );
	allocateOnDemand( &panorama5_J, cvSize(prev_panorama->width,prev_panorama->height),   img1->depth, img1->nChannels );
	allocateOnDemand( &panorama5df, cvSize(prev_panorama->width,prev_panorama->height),  IPL_DEPTH_32F, 1 );
	//allocateOnDemand( &Sumdimgkf, cvGetSize(dimg1), IPL_DEPTH_32F, 1 );
	allocateOnDemand( &Sumdimgkf, cvSize(prev_panorama->width,prev_panorama->height),  IPL_DEPTH_32F, 1 );
	allocateOnDemand( &Sumdimgk_u8, cvGetSize(dimg1),  dimg1->depth, dimg1->nChannels );
	allocateOnDemand( &Sumdimgkf_panorama_size, cvSize(prev_panorama->width,prev_panorama->height),  IPL_DEPTH_32F, 1 );
	allocateOnDemand( &Sumdimgkf_panorama_size_u8, cvSize(prev_panorama->width,prev_panorama->height),  img1->depth, img1->nChannels );
	allocateOnDemand( &prev_panoramaf, cvSize(prev_panorama->width,prev_panorama->height),   IPL_DEPTH_32F, 1 );
	allocateOnDemand( &panoramaSRf, cvSize(prev_panorama->width,prev_panorama->height),   IPL_DEPTH_32F, 1 );

	
		
	//convertIplImage2CvMat(panorama5_Deltaf, panorama5_Deltaf_matrix);
	//convertIplImage2CvMat(panorama5df, panorama5df_matrix);
	//cvSolve(panorama5_Deltaf_matrix, panorama5df_matrix, delta_SRf_matrix, CV_LU);

    // rk:
	/*
	cvAdd(dimg1, dimg2,Sumdimgk_u8,0);
	cvAdd(Sumdimgk_u8, dimg3, Sumdimgk_u8,0);
	cvAdd(Sumdimgk_u8, dimg4, Sumdimgk_u8,0);
	cvAdd(Sumdimgk_u8, dimg5, Sumdimgk_u8,0);

	Sumdimgkf = convert2gray32(Sumdimgk_u8);*/
	//Sumdimgkf = convert2gray32(panorama5_Delta);

	/*
   // So far this is the best method that I found, because
	panorama5_Deltaf = convert2gray32(panorama5_Delta);
	prev_panoramaf = convert2gray32(prev_panorama);
	for( i = 0; i < panorama5_Deltaf->height; i++){
		for(j = 0; j < panorama5_Deltaf->width; j++){
			if(!is_too_edge_like(panorama5_Deltaf, i, j, 10)){
				pix_value_panorama5_Deltaf  = ((float *)(panorama5_Deltaf->imageData + i*panorama5_Deltaf->widthStep))[j];
				pix_value_prev_panoramaf = ((float *)(prev_panoramaf->imageData + i*prev_panoramaf->widthStep))[j];
				((float *)(panoramaSRf->imageData + i*panoramaSRf->widthStep))[j] = pix_value_panorama5_Deltaf + pix_value_prev_panoramaf;
			}
			else
			{
				pix_value_prev_panoramaf = ((float *)(prev_panoramaf->imageData + i*prev_panoramaf->widthStep))[j];
				((float *)(panoramaSRf->imageData + i*panoramaSRf->widthStep))[j] = pix_value_prev_panoramaf;
			}
		}
	}
	 
	panoramaSR = convert_gray32_to_color_blending(panoramaSRf);
*/


	
	//J: panorama5_Delta;
	//panorama5_Jf = convert2gray32(panorama5_Delta);
	panorama5_Jf = convert2gray32(panorama5d);

	// Computing the Hessian:
	// H =  Jt*J
	printf(" Computing the Hessian ... \n");
	allocateOnDemand( &panorama5_Jtf, cvSize(prev_panorama->width,prev_panorama->height),  IPL_DEPTH_32F, 1 );
	allocateOnDemand( &panorama5_Jt_u8, cvSize(prev_panorama->width,prev_panorama->height),  img1->depth, img1->nChannels );
	allocateOnDemand( &panorama5_Hf, cvSize(prev_panorama->width,prev_panorama->height),  IPL_DEPTH_32F, 1 );
	allocateOnDemand( &panorama5_H_u8, cvSize(prev_panorama->width,prev_panorama->height),  img1->depth, img1->nChannels );
	allocateOnDemand( &panorama5_H_invf, cvSize(prev_panorama->width,prev_panorama->height),  IPL_DEPTH_32F, 1 );
	allocateOnDemand( &panorama5_H_inv_u8, cvSize(prev_panorama->width,prev_panorama->height),  img1->depth, img1->nChannels );
	allocateOnDemand( &panorama5_H_inv_Jf, cvSize(prev_panorama->width,prev_panorama->height),  IPL_DEPTH_32F, 1 );
	allocateOnDemand( &panorama5_H_inv_J_u8, cvSize(prev_panorama->width,prev_panorama->height),  img1->depth, img1->nChannels );
	allocateOnDemand( &panorama5_Jnf, cvSize(prev_panorama->width,prev_panorama->height),  IPL_DEPTH_32F, 1 );
	allocateOnDemand( &gf, cvSize(prev_panorama->width,prev_panorama->height),  IPL_DEPTH_32F, 1 );

	cvTranspose(panorama5_Jf, panorama5_Jtf);

	cvMul(panorama5_Jf, panorama5_Jtf,panorama5_Hf,1.0);

	//panorama5_Jnf =MulImagewithDouble(panorama5_Jf, -1.0);

	panorama5_H_u8 = convert_gray32_to_color_blending(panorama5_Hf);

	for( i = 0; i < panorama5_Jf->height; i++){
		for(j = 0; j < panorama5_Jf->width; j++){
			pixvalue_panorama5_Hf = ((float *)(panorama5_Hf->imageData + i*panorama5_Hf->widthStep))[j];
			if( pixvalue_panorama5_Hf != 0.0){
				pixvalue_panorama5_Jtf = ((float *)(panorama5_Jf->imageData + i*panorama5_Jf->widthStep))[j];
				pixvalue_panorama5_H_invf = pixvalue_panorama5_Jtf/pixvalue_panorama5_Hf;
				((float *)(panorama5_H_invf->imageData + i*panorama5_H_invf->widthStep))[j] = pixvalue_panorama5_H_invf;
			}
		}
	}
			

//	ContructAuI(panorama5_Hf, 0.01);
	

	//cvInvert(panorama5_Hf,panorama5_H_invf, CV_SVD);

//	cvMul(panorama5_H_invf, panorama5_Jf, panorama5_H_inv_Jf,1.0);
	    
//	panorama5_H_inv_J_u8 = convert_gray32_to_color_blending(panorama5_H_invf);
/*
	cvNamedWindow( "panorama5_H_u8", 1 );
	cvShowImage( "panorama5_H_u8", panorama5_H_u8 );
	cvWaitKey(0);

	cvNamedWindow( "panorama5_Hinv_u8", 1 );
	cvShowImage( "panorama5_Hinv_u8", panorama5_H_inv_J_u8 );
	cvWaitKey(0);

	cvNamedWindow( "panorama5_Delta", 1 );
	cvShowImage( "panorama5_Delta", panorama5_Delta );
	cvWaitKey(0);
*/	

	//pad2size_panorama(Sumdimgkf, Sumdimgkf_panorama_size);

	//Sumdimgkf_panorama_size_u8 = convert_gray32_to_color_blending(Sumdimgkf_panorama_size);

//	cvSaveImage("panorama5_H_u8.jpg",panorama5_H_u8);
//	cvSaveImage("panorama5_J_u8.jpg",panorama5d);
	//cvSaveImage("Sumdimgkf_panorama_size_u8.jpg",Sumdimgkf_panorama_size_u8);
	//cvSaveImage("Sumdimgk_u8.jpg",Sumdimgk_u8);
	

	

	//ContructAuI(panorama5_Jf, 0.01*lambda_it);

    /*******************************************************************/
	// Solving the least square problem:  argmin_d || J*d - rk ||
	//******************************************************************/
	
	//cvSolve(panorama5_Jf, panorama5_Deltaf, delta_SRf, CV_LU);
	//cvSolve(panorama5_H_invf, panorama5_Deltaf, delta_SRf, CV_LU);

	cvMul(panorama5_H_invf, panorama5_Deltaf, delta_SRf, 1);


	//cvMul(panorama5_Jnf, Sumdimgkf_panorama_size, gf,1.0);
	//cvSolve(panorama5_Jf, Sumdimgkf_panorama_size, delta_SRf, CV_SVD);
/*
	m = panorama5_Delta->height;
	n = panorama5_Delta->width;

	A = cvCreateMat(m*n, m*n, CV_32FC1);//row, cols, type
	delta_p = cvCreateMat(m*n, 1,CV_32FC1 );
	delta_p_matrix = cvCreateMat(m, n,CV_32FC1 );

	J = cvCreateMat(m, n, CV_32FC1);
	Jc = cvCreateMat(m*n, 1, CV_32FC1 );
	Jct = cvCreateMat( 1, m*n, CV_32FC1); 
	g = cvCreateMat( m, n, CV_32FC1); 
	gc = cvCreateMat( m*n, 1, CV_32FC1); 

	convertIplImage2CvMat(panorama5_Jf, J);
	Convert2ColumnMatrix(J, Jc);
	cvTranspose(Jc, Jct);
	cvMatMul(Jc,Jct,A);

	convertIplImage2CvMat(Sumdimgkf_panorama_size, g);
	Convert2ColumnMatrix(g, gc);

	cvSolve(A, gc, delta_p, CV_SVD_SYM );

	ConvertColumn2MatrixCvMat(delta_p, delta_p_matrix, m,n);
    convertCvMat2IplImage(delta_p_matrix, delta_SRf);
*/
	//cvSolve(panorama5_Hf,gf,delta_SRf, CV_SVD);
	
	delta_SR_u8 = convert_gray32_to_color_blending(delta_SRf);
	cvAdd(prev_panorama, delta_SR_u8, panoramaSR,0);
	//}
    
	//cvNamedWindow( "delta_SR_u8", 1 );
	//cvShowImage( "delta_SR_u8", delta_SR_u8 );
	//cvWaitKey(0);

    

	cvNamedWindow( "panoramaSR", 1 );
	cvShowImage( "panoramaSR", panoramaSR );
	cvWaitKey(0);


	//cvSaveImage("panoramaSR_B.jpg",panoramaSR);

	/***********************************************************************************
	Computing the error of the norm(SR)
	*****************************************************************************************/
	
	//printf(" Computing errorSR = %f \t *errorSRp = %f n", errorSR, *errorSRp);

	//if (it >= 2){

	errorSR = cvNorm(panoramaSR, prev_panorama, CV_RELATIVE_L2, NULL);

	errorSRp[it-1] = errorSR;
	printf(" errorSR = %5.7f \n", errorSR);

    //if( it>2)	printf(" Computing errorSR = %f \t errorSRp[it-1] = %f \t errorSRp[it -2] = %f, it = %d \n", errorSR, errorSRp[it-1], errorSRp[it-2], it);
	//printf(" Computing errorSR = %f \t errorSRp[it-1] = %f \t  it = %d \n", errorSR, errorSRp[it-1],  it);

	//if( errorSRp[it-1] < errorSRp[it - 2] ) *status = 0;
	//else *status = 1;
	////}
	//else {
	//	errorSR = cvNorm(panoramaSR, prev_panorama, CV_RELATIVE_L2, NULL);
	//	errorSRp[it-1] = errorSR;
	//	*status = 1;
	//}


	

	//cvNamedWindow( "delta_SR", 1 );
	//cvShowImage( "delta_SR", delta_SR_u8 );
	//cvWaitKey(0);

	//cvSaveImage("delta_SR.jpg",delta_SR_u8);

	cvReleaseMat( &kernelLLt );
	cvReleaseMat( &kernelL );
	cvReleaseMat( &kernelB );
	cvReleaseMat( &lambdaM );

	return;

}

extern void SRMosaicking_Levenberg_Marquardt(IplImage* img1, IplImage* img2, IplImage* img3, IplImage* img4, IplImage* img5, IplImage* prev_panorama, 
					struct image_border ROI_frame1, struct image_border ROI_frame2, struct image_border ROI_frame3, struct image_border ROI_frame4, struct image_border ROI_frame5,
					IplImage* panoramaSR, double* lambda_itp,double* errorSRp, int it)//, int *status)//, int D)
					{

//IplImage* panoramaSR = NULL;
IplImage* img1_inv = NULL, *img2_inv = NULL, *img3_inv=NULL, *img4_inv=NULL, *img5_inv=NULL;
IplImage* img1_inv_temp1 = NULL, *img2_inv_temp1 = NULL, *img3_inv_temp1=NULL, *img4_inv_temp1=NULL, *img5_inv_temp1=NULL;
IplImage* img1_inv_temp2 = NULL, *img2_inv_temp2 = NULL, *img3_inv_temp2=NULL, *img4_inv_temp2=NULL, *img5_inv_temp2=NULL;

IplImage* dimg1=NULL, *dimg2=NULL, *dimg3=NULL, *dimg4=NULL, *dimg5=NULL;
IplImage* Ddimg1=NULL, *Ddimg2=NULL, *Ddimg3=NULL, *Ddimg4=NULL, *Ddimg5=NULL;
IplImage* ddst1 = NULL, *ddst2 = NULL, *ddst3 = NULL, *ddst4 = NULL, *ddst5=NULL;
IplImage* panorama5d = NULL, *delta_SRf = NULL, *delta_SR_u8, *panorama5_Deltaf = NULL, *panorama5df = NULL, *delta_SR_u8_SRivam = NULL;
IplImage* prev_panoramaf = NULL, *panorama5_Hf = NULL;
IplImage* panorama5d2 = NULL; //after multiplication with alpha
IplImage* panorama5LLt = NULL;
IplImage* panorama5LLt2 = NULL; //after multiplication
IplImage* panorama5_Delta = NULL;
IplImage* Xregularized = NULL; // To use with the algorithm Fast and Robust Regularization
IplImage* Limg = NULL;

CvMat* kernelLLt = cvCreateMat(3,3,CV_32FC1);
CvMat* kernelB = cvCreateMat(3,3,CV_32FC1);
CvMat* kernelL = cvCreateMat(3,3,CV_32FC1);

CvMat* lambdaM = NULL;

double lambda_1, lambda_2, lambda_3, lambda_4, lambda_5;
double lambda_num, lambda_den;
double lambdad_1, lambdad_2, lambdad_3, lambdad_4, lambdad_5;
double lambda_it;
double alpha_it = 1.1127;// was 7, 1, 1.1127
double errorSR = 0.0;

float alpha = 0.0;

CvPoint2D32f P; // P for the mosaic

int i, j, D;

struct RegionS  S1, S2, S3, S4, S5;

CvSize sizeD1, sizeD2, sizeD3, sizeD4, sizeD5;
CvSize size; //size of the frames

//FILE *pFile;

cvZero(panoramaSR);


	/// First part:  Interpolation : resizing the image to 2 times the original size

  D = 2;

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

	size.height = img1->height;
	size.width = img1->width;

/**************************************************************************************/ 
   //   Starting a new iteration (it = 3)
   //   
	//  First Step: Getting the imgs from the mosaic image
   //
   /**************************************************************************************/
/*
	allocateOnDemand( &img1_inv, cvSize(img2->width,img2->height),  img1->depth, img1->nChannels );
    WarpImageUnderQuadInv(prev_panorama,img1_inv,ROI_frame1);

    allocateOnDemand( &img2_inv, cvSize(img2->width,img2->height),  img1->depth, img1->nChannels );
    WarpImageUnderQuadInv(prev_panorama,img2_inv,ROI_frame2);

	allocateOnDemand( &img3_inv, cvSize(img2->width,img2->height),  img1->depth, img1->nChannels );
	WarpImageUnderQuadInv(prev_panorama,img3_inv,ROI_frame3);

	allocateOnDemand( &img4_inv, cvSize(img2->width,img2->height),  img1->depth, img1->nChannels );
	WarpImageUnderQuadInv(prev_panorama,img4_inv,ROI_frame4);

	allocateOnDemand( &img5_inv, cvSize(img2->width,img2->height),  img1->depth, img1->nChannels );
	WarpImageUnderQuadInv(prev_panorama,img5_inv,ROI_frame5);
*/

	allocateOnDemand( &img1_inv_temp1, sizeD1,  img1->depth, img1->nChannels );
    WarpImageUnderQuadInv(prev_panorama,img1_inv_temp1,ROI_frame1);

    allocateOnDemand( &img2_inv_temp1, sizeD1,  img1->depth, img1->nChannels );
    WarpImageUnderQuadInv(prev_panorama,img2_inv_temp1,ROI_frame2);

	allocateOnDemand( &img3_inv_temp1, sizeD1,  img1->depth, img1->nChannels );
	WarpImageUnderQuadInv(prev_panorama,img3_inv_temp1,ROI_frame3);

	allocateOnDemand( &img4_inv_temp1, sizeD1,  img1->depth, img1->nChannels );
	WarpImageUnderQuadInv(prev_panorama,img4_inv_temp1,ROI_frame4);

	allocateOnDemand( &img5_inv_temp1, sizeD1,  img1->depth, img1->nChannels );
	WarpImageUnderQuadInv(prev_panorama,img5_inv_temp1,ROI_frame5);

	// Using a Gaussian Filter to perform an Antia-alias filtering
	
	allocateOnDemand( &img1_inv_temp2, sizeD1,  img1->depth, img1->nChannels );
    allocateOnDemand( &img2_inv_temp2, sizeD1,  img1->depth, img1->nChannels );
    allocateOnDemand( &img3_inv_temp2, sizeD1,  img1->depth, img1->nChannels );
	allocateOnDemand( &img4_inv_temp2, sizeD1,  img1->depth, img1->nChannels );
	allocateOnDemand( &img5_inv_temp2, sizeD1,  img1->depth, img1->nChannels );
/*
	cvSmooth(img1_inv_temp1,img1_inv_temp2, CV_GAUSSIAN, 3, 0, 0, 0);
	cvSmooth(img2_inv_temp1,img2_inv_temp2, CV_GAUSSIAN, 3, 0, 0, 0);
	cvSmooth(img3_inv_temp1,img3_inv_temp2, CV_GAUSSIAN, 3, 0, 0, 0);
	cvSmooth(img4_inv_temp1,img4_inv_temp2, CV_GAUSSIAN, 3, 0, 0, 0);
	cvSmooth(img5_inv_temp1,img5_inv_temp2, CV_GAUSSIAN, 3, 0, 0, 0);
*/
	// Using Filter 2D:
    //This kernel is the same as the used in the paper: " A Transform-Domain Approach to Super-resolution Mosaicing of Compressed Images"

	cvmSet(kernelB,0,0,0.1016);
	cvmSet(kernelB,0,1,0.1172);
	cvmSet(kernelB,0,2,0.1016);
	cvmSet(kernelB,1,0,0.1172);
	cvmSet(kernelB,1,1,0.1250);
	cvmSet(kernelB,1,2,0.1172);
	cvmSet(kernelB,2,0,0.1016);
	cvmSet(kernelB,2,1,0.1172);
	cvmSet(kernelB,2,2,0.1016);

	cvFilter2D(img1_inv_temp1,img1_inv_temp2,kernelB,cvPoint(-1,1));
	cvFilter2D(img2_inv_temp1,img2_inv_temp2,kernelB,cvPoint(-1,1));
	cvFilter2D(img3_inv_temp1,img3_inv_temp2,kernelB,cvPoint(-1,1));
	cvFilter2D(img4_inv_temp1,img4_inv_temp2,kernelB,cvPoint(-1,1));
	cvFilter2D(img5_inv_temp1,img5_inv_temp2,kernelB,cvPoint(-1,1));
	
  // Downsampling using cubic interpolation

	allocateOnDemand( &img1_inv, size,  img1->depth, img1->nChannels );
    allocateOnDemand( &img2_inv, size,  img1->depth, img1->nChannels );
    allocateOnDemand( &img3_inv, size,  img1->depth, img1->nChannels );
	allocateOnDemand( &img4_inv, size,  img1->depth, img1->nChannels );
	allocateOnDemand( &img5_inv, size,  img1->depth, img1->nChannels );
	
	cvResize(img1_inv_temp2, img1_inv, CV_INTER_CUBIC);
	cvResize(img2_inv_temp2, img2_inv, CV_INTER_CUBIC);
	cvResize(img3_inv_temp2, img3_inv, CV_INTER_CUBIC);
	cvResize(img4_inv_temp2, img4_inv, CV_INTER_CUBIC);
	cvResize(img5_inv_temp2, img5_inv, CV_INTER_CUBIC);

	/*
// Downsampling using my own code

	DownSampleImage(img1_inv_temp2, img1_inv, 2);
	DownSampleImage(img2_inv_temp2, img2_inv, 2);
	DownSampleImage(img3_inv_temp2, img3_inv, 2);
	DownSampleImage(img4_inv_temp2, img4_inv, 2);
	DownSampleImage(img5_inv_temp2, img5_inv, 2);
*/
/*
	cvSaveImage("DBWRx1.jpg", img1_inv);
	cvSaveImage("DBWRx2.jpg", img2_inv);
	cvSaveImage("DBWRx3.jpg", img3_inv);
	cvSaveImage("DBWRx4.jpg", img4_inv);
	cvSaveImage("DBWRx5.jpg", img5_inv);


	cvNamedWindow( "img1_inv", 1 );
	cvShowImage( "img1_inv", img1_inv);
	cvWaitKey(0);

	cvNamedWindow( "img2_inv", 1 );
	cvShowImage( "img2_inv", img2_inv);
	cvWaitKey(0);

	cvNamedWindow( "img3_inv", 1 );
	cvShowImage( "img3_inv", img3_inv );
	cvWaitKey(0);

	cvNamedWindow( "img4_inv", 1 );
	cvShowImage( "img4_inv", img4_inv );
	cvWaitKey(0);

	cvNamedWindow( "img5_inv", 1 );
	cvShowImage( "img5_inv", img5_inv );
	cvWaitKey(0);
*/	

   /**************************************************************************************/ 
   //    
   //   
   //  Second Step: Compute the frame difference between the original low frames and the LR reconstructed ones
   //
   /**************************************************************************************/

  /************************************************************************/
  //  Doing the frame substracion
  /****************************************************************************/
 //  printf("\n Doing the frame substracion ...\n" );


  allocateOnDemand( &dimg1, cvSize(img2->width,img2->height),  img1->depth, img1->nChannels );
  allocateOnDemand( &dimg2, cvSize(img2->width,img2->height),  img1->depth, img1->nChannels );
  allocateOnDemand( &dimg3, cvSize(img2->width,img2->height),  img1->depth, img1->nChannels );
  allocateOnDemand( &dimg4, cvSize(img2->width,img2->height),  img1->depth, img1->nChannels );
  allocateOnDemand( &dimg5, cvSize(img2->width,img2->height),  img1->depth, img1->nChannels );
/*
   printf("\n size_img1 = (%d , %d) \t size_img1_inv = (%d, %d)", img1->height, img1->width, img1_inv->height, img1_inv->width);
   printf("\n size_dimg1 = (%d , %d) ", dimg1->height, dimg1->width);

   getchar();
*/

  cvSub(img1,img1_inv,dimg1,NULL);
  cvSub(img2,img2_inv,dimg2,NULL);
  cvSub(img3,img3_inv,dimg3,NULL);
  cvSub(img4,img4_inv,dimg4,NULL);
  cvSub(img5,img5_inv,dimg5,NULL);

 
  //*********************************************************************************************//
  //  Doing the interpolation of the images differences											//
  //*********************************************************************************************//
    //printf("\n Doing the frame interpolation of the difference of frames ...\n" );
    allocateOnDemand( &Ddimg1, sizeD1,  img1->depth, img1->nChannels );  
	allocateOnDemand( &Ddimg2, sizeD2,  img1->depth, img1->nChannels );  
	allocateOnDemand( &Ddimg3, sizeD3,  img1->depth, img1->nChannels );  
	allocateOnDemand( &Ddimg4, sizeD4,  img1->depth, img1->nChannels );  
	allocateOnDemand( &Ddimg5, sizeD5,  img1->depth, img1->nChannels );  

	/*
  	cvResize(dimg1, Ddimg1,CV_INTER_CUBIC);
	cvResize(dimg2, Ddimg2,CV_INTER_CUBIC);
	cvResize(dimg3, Ddimg3,CV_INTER_CUBIC);
	cvResize(dimg4, Ddimg4,CV_INTER_CUBIC);
	cvResize(dimg5, Ddimg5,CV_INTER_CUBIC);
	*/

	UpSampleImageD(dimg1, Ddimg1);
	UpSampleImageD(dimg2, Ddimg2);
	UpSampleImageD(dimg3, Ddimg3);
	UpSampleImageD(dimg4, Ddimg4);
	UpSampleImageD(dimg5, Ddimg5);

/***********************************************************************************/
//			 Doing the Mosaicking of the Error Images								//
//																					//
/***********************************************************************************/
	//printf("\n Doing the Mosaicking of the Error Images ...\n" );

	allocateOnDemand( &ddst1,  cvSize(panoramaSR->width,panoramaSR->height),  img1->depth, img1->nChannels );  
	allocateOnDemand( &ddst2,  cvSize(panoramaSR->width,panoramaSR->height),  img1->depth, img1->nChannels );  
	allocateOnDemand( &ddst3,  cvSize(panoramaSR->width,panoramaSR->height),  img1->depth, img1->nChannels );  
	allocateOnDemand( &ddst4,  cvSize(panoramaSR->width,panoramaSR->height),  img1->depth, img1->nChannels );  
	allocateOnDemand( &ddst5,  cvSize(panoramaSR->width,panoramaSR->height),  img1->depth, img1->nChannels );  
	
	/*
	allocateOnDemand( &ddst1,  cvSize(1360, 640),  img1->depth, img1->nChannels );  
	allocateOnDemand( &ddst2,  cvSize(1360, 640),  img1->depth, img1->nChannels );  
	allocateOnDemand( &ddst3,  cvSize(1360, 640),  img1->depth, img1->nChannels );  
	allocateOnDemand( &ddst4,  cvSize(1360, 640),  img1->depth, img1->nChannels );  
	allocateOnDemand( &ddst5,  cvSize(1360, 640),  img1->depth, img1->nChannels );  
*/

    WarpImageUnderQuad(Ddimg1,ddst1,ROI_frame1);
	//cvNamedWindow( "Perspective_Warpdd1", 1 );
	//cvShowImage( "Perspective_Warpdd1", ddst1);
	//cvWaitKey(10);


  WarpImageUnderQuad(Ddimg2,ddst2,ROI_frame2);

 // cvNamedWindow( "Perspective_Warpdd2", 1 );
 // cvShowImage( "Perspective_Warpdd2", ddst2 );
  //cvWaitKey(10);


 WarpImageUnderQuad(Ddimg3,ddst3,ROI_frame3);
 //cvNamedWindow( "Perspective_Warpdd3", 1 );
 //cvShowImage( "Perspective_Warpdd3", ddst3 );
 //cvWaitKey(10);


 WarpImageUnderQuad(Ddimg4,ddst4,ROI_frame4);
 //cvNamedWindow( "Perspective_Warpdd4", 1 );
 //cvShowImage( "Perspective_Warpdd4", ddst4 );
 //cvWaitKey(10);

 
 WarpImageUnderQuad(Ddimg5,ddst5,ROI_frame5);
 //cvNamedWindow( "Perspective_Warpdd5", 1 );
 ///cvShowImage( "Perspective_Warpdd5", ddst5 );
 //cvWaitKey(10);

 allocateOnDemand( &panorama5d,  cvSize(prev_panorama->width, prev_panorama->height),  img1->depth, img1->nChannels );  

 S2 = FindSRegion(ROI_frame2);

	  for(i = 0; i < panorama5d->height; i ++){ //rows
		  for(j = 0; j < panorama5d->width; j++){ //cols
			  P.x = j;
			  P.y = i;
			  if(Belongs2S(P,S2))cvSet2D(panorama5d,i,j,cvGet2D(ddst2,i,j));
			  else cvSet2D(panorama5d,i,j,cvGet2D(ddst1,i,j));			  
		  }//end for
	  }// end for

	

////////////////////////////
// Finding the region S3 //
////////////////////////////

   S3 = FindSRegion(ROI_frame3);

	  for(i = 0; i < panorama5d->height; i ++){ //rows
		  for(j = 0; j < panorama5d->width; j++){ //cols
			  P.x = j;
			  P.y = i;
			  if(Belongs2S(P,S3))cvSet2D(panorama5d,i,j,cvGet2D(ddst3,i,j));
			  else cvSet2D(panorama5d,i,j,cvGet2D(panorama5d,i,j));			  
		  }//end for
	  }// end for



 ////////////////////////////
// Finding the region S4 //
////////////////////////////

   S4 = FindSRegion(ROI_frame4);

	  for(i = 0; i < panorama5d->height; i ++){ //rows
		  for(j = 0; j < panorama5d->width; j++){ //cols
			  P.x = j;
			  P.y = i;
			  if(Belongs2S(P,S4))cvSet2D(panorama5d,i,j,cvGet2D(ddst4,i,j));
			  else cvSet2D(panorama5d,i,j,cvGet2D(panorama5d,i,j));			  
		  }//end for
	  }// end for


	    ////////////////////////////
		// Finding the region S4 //
		////////////////////////////

   S5 = FindSRegion(ROI_frame5);


	  for(i = 0; i < panorama5d->height; i ++){ //rows
		  for(j = 0; j < panorama5d->width; j++){ //cols
			  P.x = j;
			  P.y = i;
			  if(Belongs2S(P,S5))cvSet2D(panorama5d,i,j,cvGet2D(ddst5,i,j));
			  else cvSet2D(panorama5d,i,j,cvGet2D(panorama5d,i,j));			  
		  }//end for
	  }// end for

	//cvNamedWindow( "panorama5d", 1 );
	//cvShowImage( "panorama5d", panorama5d );
	//cvWaitKey(10);



   /**************************************************************************************/
   //
   // Computing the normalizing parameter lambda^(n) 
   //
   /**************************************************************************************/
    
	  lambda_1 = cvNorm(img1, img1_inv, CV_L2, NULL);
	 // printf("\n lambda_1 = %f", lambda_1);
	  lambda_2 = cvNorm(img2, img2_inv, CV_L2, NULL);
	 // printf("\n lambda_2 = %f", lambda_2);
	  lambda_3 = cvNorm(img3, img3_inv, CV_L2, NULL);
	 // printf("\n lambda_3 = %f", lambda_3);
	  lambda_4 = cvNorm(img4, img4_inv, CV_L2, NULL);
	 // printf("\n lambda_4 = %f", lambda_4);
	  lambda_5 = cvNorm(img5, img5_inv, CV_L2, NULL);
	 // printf("\n lambda_5 = %f", lambda_5);

   /**************************************************************************************/
   //
   // Computing the Laplace of the x^(n) image x(n) comes to be the panorama5 variables (mosaic of the previous state)
   //
   /**************************************************************************************/
   allocateOnDemand( &Limg, cvSize(prev_panorama->width,prev_panorama->height),  img1->depth, img1->nChannels );  
   //cvLaplace(prev_panorama, Limg,3);   
/*
// Uncomment this to use Laplace Prior Function

   // Using in stead cvFilter2D

   	cvmSet(kernelL,0,0,0.0000);
	cvmSet(kernelL,0,1,0.2500);
	cvmSet(kernelL,0,2,0.0000);
	cvmSet(kernelL,1,0,0.2500);
	cvmSet(kernelL,1,1,-1.000);
	cvmSet(kernelL,1,2,0.2500);
	cvmSet(kernelL,2,0,0.0000);
	cvmSet(kernelL,2,1,0.2500);
	cvmSet(kernelL,2,2,0.0000);

   cvFilter2D(prev_panorama, Limg, kernelL, cvPoint(-1,-1));
*/
 // Uncomment this to use Huber Prior Function
  // Using Huber Prior Function to find lambda

     alpha = 1.75;//was 1.75

     HubbertFunctionCliques(prev_panorama, Limg, alpha);

  // cvNamedWindow( "Limg", 1 );
  // cvShowImage( "Limg", Limg );
  // cvWaitKey(0);


   /**************************************************************************************/
   //
   // Computing the K norm(x^(n),2) , K = 1, 2 ...5
   //
   /**************************************************************************************/

  lambda_num = pow((lambda_1 + lambda_2 + lambda_3 + lambda_4 + lambda_5),2);
  lambdad_1 = cvNorm(Limg, NULL, CV_L2, NULL);
 

    /**************************************************************************************/
   //
   // Computing the lambda(n), K = 5, because I am just working with 5 frames
   //
   /**************************************************************************************/
 
  lambda_den = pow((5*lambdad_1),2);

    
  lambda_it = lambda_num/lambda_den;

  
  // printf("\n lambda_num = %f", lambda_num);
  // printf("\n lambda_den = %f", lambda_den);
  // printf("\n lambda_it = %f", lambda_it);

   *lambda_itp =  lambda_it;

   printf("lambda_it = %f \n", *lambda_itp);

   

  // pFile = fopen(name, "w");

   //fprintf(pFile, "%f\n", lambda_it);

  // fclose(pFile);
   
  // lambda_it = 1.1127;

  // printf("\n Using alpha from GCV = %f", alpha_it);

    /**************************************************************************************/
   //
   // Computing the lambda(n)*L'*L*x(n)
   //
   /**************************************************************************************/

   /*
    cvmSet(kernelLLt,0,0,1.000*lambda_it);
	cvmSet(kernelLLt,0,1,-4.000*lambda_it);
	cvmSet(kernelLLt,0,2,1.000*lambda_it);
	cvmSet(kernelLLt,1,0,-4.000*lambda_it);
	cvmSet(kernelLLt,1,1,18.000*lambda_it);
	cvmSet(kernelLLt,1,2,-4.000*lambda_it);
	cvmSet(kernelLLt,2,0,1.000*lambda_it);
	cvmSet(kernelLLt,2,1,-4.000*lambda_it);
	cvmSet(kernelLLt,2,2,1.000*lambda_it);
*/
	cvmSet(kernelLLt,0,0,1.000);
	cvmSet(kernelLLt,0,1,-4.000);
	cvmSet(kernelLLt,0,2,1.000);
	cvmSet(kernelLLt,1,0,-4.000);
	cvmSet(kernelLLt,1,1,18.000);
	cvmSet(kernelLLt,1,2,-4.000);
	cvmSet(kernelLLt,2,0,1.000);
	cvmSet(kernelLLt,2,1,-4.000);
	cvmSet(kernelLLt,2,2,1.000);

	allocateOnDemand( &panorama5LLt, cvSize(prev_panorama->width,prev_panorama->height),  img1->depth, img1->nChannels );  
	allocateOnDemand( &panorama5LLt2, cvSize(prev_panorama->width,prev_panorama->height),  img1->depth, img1->nChannels );  
	allocateOnDemand( &panorama5d2, cvSize(prev_panorama->width,prev_panorama->height),  img1->depth, img1->nChannels ); 

 //	printf("\n Limg->height = %d \t Limg->width = %d", Limg->height, Limg->width);
 //	printf("\n prev_panorama->height = %d \t prev_panorama->width = %d", prev_panorama->height, prev_panorama->width);

  //  Uncomment to use Huber Regularization

    // Using Huber Regularization here as well
	HubbertRegularization(Limg, panorama5LLt, alpha);
/*
  // Uncomment to use Laplace Regularization
    // Using Laplace prior
	cvFilter2D(Limg,panorama5LLt,kernelL,cvPoint(-1,1));
	cvFilter2D(panorama5LLt,panorama5LLt2, kernelL,cvPoint(-1,1)); // Because os L^t*L*x
*/	

/*
    //Using the regularization proposed in the paper: "Fast and Robust Super-Resolution"

	allocateOnDemand( &Xregularized, cvSize(prev_panorama->width,prev_panorama->height), IPL_DEPTH_32F, 1 );

	FastRobustRegularization(Limg, Xregularized, 2, alpha); // p = 2

	panorama5LLt = convert_gray32_to_color(Xregularized);

*/

	/*
	//
	// Converting to Matrix lambda doesn't work, because of the size of the mosaicking
	//

	lambdaM = cvCreateMat(panorama5LLt->height,panorama5LLt->width,CV_32FC1);
	cvSetIdentity(lambdaM,cvRealScalar(lambda_it));

	cvMatMul(panorama5LLt,lambdaM, panorama5LLt2);
	*/

	panorama5LLt2 = MulImagewithDouble(panorama5LLt, lambda_it);// Use this for Huber Regularization
	
	//panorama5LLt2 = MulImagewithDouble(panorama5LLt2, lambda_it); // Use this for Laplace Regularization
	
	panorama5d2 = MulImagewithDouble(panorama5d, alpha_it);

	allocateOnDemand( &panorama5_Delta, cvSize(prev_panorama->width,prev_panorama->height),  img1->depth, img1->nChannels );  

	cvSub(panorama5d2,panorama5LLt2,panorama5_Delta,NULL);

	printf(" it = %d \n", it);

	printf("\n errorSRp[it - 2] > = errorSRp[it - 3] = %d \n", ((it == 1 )||(errorSRp[it - 2] >= errorSRp[it - 3])));

	if(it == 1 || errorSRp[it - 2] >= errorSRp[it - 3] ){
	//if(it == 1 || *status == 1){
//     printf(" status = %d \n", *status);
   /**************************************************************************************/ 
   //
   //	Now updating the iteration of x(n+1). First I have to do: panorama5d - panorama5LLt
   //   Then I have to do : panorma5_2 = panorama5_1 + (panorama5d - panorama5LLt)
   //   This part computes the SR based on Steep Descent 
   /**************************************************************************************/
	
	//allocateOnDemand( &panoramaSR, cvSize(prev_panorama->width,prev_panorama->height),  img1->depth, img1->nChannels );  
	
	cvAdd(panorama5_Delta,prev_panorama,panoramaSR,NULL);

	//cvNamedWindow( "panoramaSR", 1 );
	//cvShowImage( "panoramaSR", panoramaSR );
	//cvWaitKey(0);

	//cvSaveImage("panoramaSR_A.jpg",panoramaSR);
	}
	else{

	/***********************************************************************************
	Computing delta to update the SR for Gaussian Newton
	J = panorama5_Delta;
	rk = b - A(yk)xk = panorama5d
	*****************************************************************************************/
  //  printf(" status = %d \n", *status);
	printf(" Computing delta ..... \n");

	allocateOnDemand( &delta_SRf, cvSize(prev_panorama->width,prev_panorama->height),   IPL_DEPTH_32F, 1 );
	allocateOnDemand( &delta_SR_u8, cvSize(prev_panorama->width,prev_panorama->height),   IPL_DEPTH_32F, 1 );
	//allocateOnDemand( &delta_SR_u8, cvSize(prev_panorama->width,prev_panorama->height),   img1->depth, img1->nChannels );
	// To be able to work with DICOM Images
	allocateOnDemand( &delta_SR_u8_SRivam, cvSize(prev_panorama->width,prev_panorama->height),  IPL_DEPTH_8U, 1 ); 
	allocateOnDemand( &panorama5_Deltaf, cvSize(prev_panorama->width,prev_panorama->height),   IPL_DEPTH_32F, 1 );
	allocateOnDemand( &panorama5df, cvSize(prev_panorama->width,prev_panorama->height),   IPL_DEPTH_32F, 1 );
	// new:
	allocateOnDemand( &prev_panoramaf, cvSize(prev_panorama->width,prev_panorama->height),   IPL_DEPTH_32F, 1 );
	allocateOnDemand( &panorama5_Hf, cvSize(prev_panorama->width,prev_panorama->height),   IPL_DEPTH_32F, 1 );
		


	//convertIplImage2CvMat(panorama5_Deltaf, panorama5_Deltaf_matrix);
	//convertIplImage2CvMat(panorama5df, panorama5df_matrix);
	//cvSolve(panorama5_Deltaf_matrix, panorama5df_matrix, delta_SRf_matrix, CV_LU);

	panorama5_Deltaf = convert2gray32(panorama5_Delta);
	panorama5df = convert2gray32(panorama5d);
	prev_panoramaf = convert2gray32(prev_panorama);
	 
	//cvSolve(panorama5_Deltaf, panorama5df, delta_SRf, CV_LU);

	cvMulTransposed(panorama5_Deltaf, panorama5_Hf, 1, 0, 1.00);

	cvSolve(panorama5_Hf, panorama5_Deltaf, delta_SRf, CV_SVD);

	//delta_SR_u8 = convert_gray32_to_color_blending(delta_SRf);

	//printf("Computing the Addition of both images ... \n");

	cvConvertScale( delta_SRf, delta_SR_u8_SRivam, 1.0, 0 );

	//delta_SR_u8_SRivam = convert_gray32_to_color_DICOM(delta_SRf);
	
	//printf("prev_panorama->nChannels = %d \t delta_SR_u8->nChannels = %d \t  panoramaSR->nChannels = %d \n",prev_panorama->nChannels, delta_SR_u8_SRivam->nChannels, panoramaSR->nChannels );
	//printf("getchar() \n");
	//getchar();
    
	//cvScale(delta_SR_u8,delta_SR_u8_SRivam,1,0);
	// cvAdd(prev_panorama, delta_SR_u8, panoramaSR,0);

	
	
	
	cvAdd(prev_panorama, delta_SR_u8_SRivam, panoramaSR,0);

	printf("Finished ... \n");
	}

	//cvNamedWindow( "panoramaSR", 1 );
	//cvShowImage( "panoramaSR", panoramaSR );
	//cvWaitKey(0);

	//cvSaveImage("panoramaSR_B.jpg",panoramaSR);

	/***********************************************************************************
	Computing the error of the norm(SR)
	*****************************************************************************************/
	
	//printf(" Computing errorSR = %f \t *errorSRp = %f n", errorSR, *errorSRp);

	//if (it >= 2){

	errorSR = cvNorm(panoramaSR, prev_panorama, CV_RELATIVE_L2, NULL);
	//errorSR = cvNorm(panoramaSR, prev_panorama, CV_L2, NULL);

	errorSRp[it-1] = errorSR;
	printf(" errorSR = %f \n", errorSR);

    //if( it>2)	printf(" Computing errorSR = %f \t errorSRp[it-1] = %f \t errorSRp[it -2] = %f, it = %d \n", errorSR, errorSRp[it-1], errorSRp[it-2], it);
	//printf(" Computing errorSR = %f \t errorSRp[it-1] = %f \t  it = %d \n", errorSR, errorSRp[it-1],  it);

	//if( errorSRp[it-1] < errorSRp[it - 2] ) *status = 0;
	//else *status = 1;
	////}
	//else {
	//	errorSR = cvNorm(panoramaSR, prev_panorama, CV_RELATIVE_L2, NULL);
	//	errorSRp[it-1] = errorSR;
	//	*status = 1;
	//}


	

	//cvNamedWindow( "delta_SR", 1 );
	//cvShowImage( "delta_SR", delta_SR_u8 );
	//cvWaitKey(0);

	//cvSaveImage("delta_SR.jpg",delta_SR_u8);

	cvReleaseMat( &kernelLLt );
	cvReleaseMat( &kernelL );
	cvReleaseMat( &kernelB );
	cvReleaseMat( &lambdaM );

	return;

}

extern void pad2size_panorama(IplImage* input_imgf, IplImage* output_imgf){
    int i, j;
	float input_value;

	for(i = 0; i < input_imgf->height; i++){
		for(j = 0; j < input_imgf->width; j++){
           input_value = ((float *)(input_imgf->imageData + i*input_imgf->widthStep))[j];
		   ((float *)(output_imgf->imageData + i*output_imgf->widthStep))[j] = input_value;
		}
	}

	return;
}

// Implements the Quasi Newton Method, Davidson-Fletcher-Powell Method (DFP)
extern void SRMosaicking_Quasi_Newton_DFP(IplImage* img1, IplImage* img2, IplImage* img3, IplImage* img4, IplImage* img5, IplImage* prev_panorama, 
					struct image_border ROI_frame1, struct image_border ROI_frame2, struct image_border ROI_frame3, struct image_border ROI_frame4, struct image_border ROI_frame5,
					IplImage* panoramaSR, IplImage* prev_GradientMosaic, IplImage* GradientMosaic, IplImage* prev_pseudoHessianMosaic, IplImage* pseudoHessianMosaic, double* lambda_itp,double* errorSRp, int it)
					{

//IplImage* panoramaSR = NULL;
IplImage* img1_inv = NULL, *img2_inv = NULL, *img3_inv=NULL, *img4_inv=NULL, *img5_inv=NULL;
IplImage* img1_inv_temp1 = NULL, *img2_inv_temp1 = NULL, *img3_inv_temp1=NULL, *img4_inv_temp1=NULL, *img5_inv_temp1=NULL;
IplImage* img1_inv_temp2 = NULL, *img2_inv_temp2 = NULL, *img3_inv_temp2=NULL, *img4_inv_temp2=NULL, *img5_inv_temp2=NULL;

IplImage* dimg1=NULL, *dimg2=NULL, *dimg3=NULL, *dimg4=NULL, *dimg5=NULL;
IplImage* Ddimg1=NULL, *Ddimg2=NULL, *Ddimg3=NULL, *Ddimg4=NULL, *Ddimg5=NULL;
IplImage* ddst1 = NULL, *ddst2 = NULL, *ddst3 = NULL, *ddst4 = NULL, *ddst5=NULL;
IplImage* panorama5d = NULL, *delta_SRf = NULL, *delta_SR_u8, *panorama5_Deltaf = NULL, *panorama5df = NULL;
IplImage* panorama5d2 = NULL; //after multiplication with alpha
IplImage* panorama5LLt = NULL;
IplImage* panorama5LLt2 = NULL; //after multiplication
IplImage* panorama5_Delta = NULL;
IplImage* Xregularized = NULL; // To use with the algorithm Fast and Robust Regularization
IplImage* Limg = NULL;

// New staff here
IplImage *nGradientMosaic =  NULL, *delta_Gradient =  NULL, *delta_Gradientf =  NULL, *rk =  NULL, *rkf =  NULL;
IplImage *rkt =  NULL, *rkft =  NULL, *delta_Gradient_t =  NULL, *num1 =  NULL, *num2 =  NULL, *den1 =  NULL;
IplImage *den2 =  NULL, *iden1 =  NULL, *iden2 =  NULL, *fact1 =  NULL, *fact2 =  NULL;
IplImage *panorama5_Deltaft =  NULL, *npseudoHessianMosaic =  NULL,  *npseudoHessianMosaicf =  NULL, *pseudoHessianMosaicf =  NULL;
IplImage* prev_pseudoHessianMosaicf = NULL;

CvMat* kernelLLt = cvCreateMat(3,3,CV_32FC1);
CvMat* kernelB = cvCreateMat(3,3,CV_32FC1);
CvMat* kernelL = cvCreateMat(3,3,CV_32FC1);

CvMat* lambdaM = NULL;

double lambda_1, lambda_2, lambda_3, lambda_4, lambda_5;
double lambda_num, lambda_den;
double lambdad_1, lambdad_2, lambdad_3, lambdad_4, lambdad_5;
double lambda_it;
double alpha_it = 1.1127;// was 7, 1, 1.1127
double errorSR = 0.0;

float alpha = 0.0;

CvPoint2D32f P; // P for the mosaic

int i, j, D;

struct RegionS  S1, S2, S3, S4, S5;

CvSize sizeD1, sizeD2, sizeD3, sizeD4, sizeD5;
CvSize size; //size of the frames

//FILE *pFile;

cvZero(panoramaSR);


// Computing: alpha*Qk*g^k
allocateOnDemand( &panorama5d, cvSize(prev_panorama->width,prev_panorama->height),  img1->depth, img1->nChannels );  
if( it == 1) panorama5d = cvCloneImage(prev_GradientMosaic);
else cvMul(prev_pseudoHessianMosaic, prev_GradientMosaic, panorama5d, 1);

   

   // computing x^(k+1)
	printf(" Computing x(k+1) .... \n");

	cvAdd(prev_panorama, panorama5d, panoramaSR, NULL);

	cvNamedWindow( "panoramaSR", 1 );
	cvShowImage( "panoramaSR", panoramaSR );
	cvWaitKey(0);

	// Computing x^(k+1) - x^(k) 

	printf(" Computing delta_x(k+1) .... \n");

	allocateOnDemand( &panorama5_Delta, cvSize(prev_panorama->width,prev_panorama->height),  img1->depth, img1->nChannels );  
	cvSub(panoramaSR,prev_panorama,panorama5_Delta,NULL);

	

	// Computing: g^(k+1) = Jf(x^(k+1))
	// nGradientMosaic = g^(k+1)

	printf(" Computing the new gradient g(k+1) .... \n");
    //allocateOnDemand( &nGradientMosaic, cvSize(prev_panorama->width,prev_panorama->height),  img1->depth, img1->nChannels );  

	Gradient_Mosaic(img1, img2, img3, img4, img5, panoramaSR, ROI_frame1,ROI_frame2, ROI_frame3, ROI_frame4, ROI_frame5, GradientMosaic, lambda_itp);

	// Computing: delta_g(k) = g(k+1) - g(k)

	printf(" Computing the delta_g(k) .... \n");
	allocateOnDemand( &delta_Gradient, cvSize(prev_panorama->width,prev_panorama->height),  img1->depth, img1->nChannels );  
    cvSub(GradientMosaic, prev_GradientMosaic, delta_Gradient,0);

	//cvNamedWindow( "pseudoHessianMosaic", 1 );
	//cvShowImage( "pseudoHessianMosaic", pseudoHessianMosaic );
	//cvWaitKey(0);

	//cvNamedWindow( "delta_Gradient", 1 );
	//cvShowImage( "delta_Gradient", delta_Gradient );
	//cvWaitKey(0);
	
	// Computing: rk = Qk*(delta_g(k))
	allocateOnDemand( &rk, cvSize(prev_panorama->width,prev_panorama->height),  img1->depth, img1->nChannels );  
    printf(" Computing the r(k) .... \n");
	if( it == 1)	rk = cvCloneImage(delta_Gradient);
	else  cvMul(prev_pseudoHessianMosaic, delta_Gradient, rk, 1);
    
   
    cvNamedWindow( "rk", 1 );
	cvShowImage( "rk", rk );
	cvWaitKey(0);

    
   // Computing: Q(k+1) = Q(k) - rk(rk)^t/ rk^t*delta_g(k) + delta_x(k)*delta_x(k)^t/(delta_x(k)^t*delta_g(k))
  // All the images has to be converted to float images

	printf(" Computing Q(k+1) ..... \n");

	allocateOnDemand( &rkf, cvSize(prev_panorama->width,prev_panorama->height), IPL_DEPTH_32F, 1 ); 
	allocateOnDemand( &delta_Gradientf, cvSize(prev_panorama->width,prev_panorama->height),   IPL_DEPTH_32F, 1 );
	allocateOnDemand( &panorama5_Deltaf, cvSize(prev_panorama->width,prev_panorama->height),   IPL_DEPTH_32F, 1 );
	allocateOnDemand( &panorama5_Deltaft, cvSize(prev_panorama->width,prev_panorama->height),   IPL_DEPTH_32F, 1 );
	//allocateOnDemand( &npseudoHessianMosaicf, cvSize(prev_panorama->width,prev_panorama->height),   IPL_DEPTH_32F, 1 );
	allocateOnDemand( &pseudoHessianMosaicf, cvSize(prev_panorama->width,prev_panorama->height),   IPL_DEPTH_32F, 1 );
	allocateOnDemand( &prev_pseudoHessianMosaicf, cvSize(prev_panorama->width,prev_panorama->height),   IPL_DEPTH_32F, 1 );
	allocateOnDemand( &rkft, cvSize(prev_panorama->width,prev_panorama->height),   IPL_DEPTH_32F, 1 );
	allocateOnDemand( &den1, cvSize(prev_panorama->width,prev_panorama->height),   IPL_DEPTH_32F, 1 );
	allocateOnDemand( &iden1, cvSize(prev_panorama->width,prev_panorama->height),   IPL_DEPTH_32F, 1 );
	allocateOnDemand( &num1, cvSize(prev_panorama->width,prev_panorama->height),   IPL_DEPTH_32F, 1 );
	allocateOnDemand( &num2, cvSize(prev_panorama->width,prev_panorama->height),   IPL_DEPTH_32F, 1 );
	allocateOnDemand( &den2, cvSize(prev_panorama->width,prev_panorama->height),   IPL_DEPTH_32F, 1 );
	allocateOnDemand( &iden2, cvSize(prev_panorama->width,prev_panorama->height),   IPL_DEPTH_32F, 1 );
	allocateOnDemand( &fact1, cvSize(prev_panorama->width,prev_panorama->height),   IPL_DEPTH_32F, 1 );
	allocateOnDemand( &fact2, cvSize(prev_panorama->width,prev_panorama->height),   IPL_DEPTH_32F, 1 );


	rkf = convert2gray32(rk);
	delta_Gradientf = convert2gray32(delta_Gradient);
	panorama5_Deltaf = convert2gray32(panorama5_Delta);
	printf(" Converting pseudoHessian to float ..... \n");
	
	prev_pseudoHessianMosaicf = convert2gray32(prev_pseudoHessianMosaic);

	printf(" Converting to transpose images ..... \n");

	cvTranspose(rkf, rkft);
	
    printf(" Converting to transpose images ..... \n");
	cvTranspose(panorama5_Deltaf, panorama5_Deltaft); //delta_x
	//cvTranspose(delta_Gradient, delta_Gradient_t);

    
    printf(" Computing factor1 ..... \n");
    cvMul(rkf, rkft, num1,1);
	//cvMulTransposed(rkf, num1, 1, 0, 1);
	cvMul(rkft, delta_Gradientf, den1,1);
	cvInvert(den1, iden1, CV_LU);
	cvMul(num1, iden1, fact1, 1);

	printf(" Computing factor2 ..... \n");
	cvMul(panorama5_Deltaf, panorama5_Deltaft, num2,1);
	
	cvMul(panorama5_Deltaft, delta_Gradientf, den2,1);
	
	cvInvert(den2, iden2, CV_LU);
	
	cvMul(num2, iden2, fact2, 1);

    printf(" Computing the new pseudoHessian ..... \n");
	if(it == 1)ContructAuI(prev_pseudoHessianMosaicf, 1.0);
	
	cvSub(prev_pseudoHessianMosaicf, fact1, pseudoHessianMosaicf, 0);

	cvAdd(pseudoHessianMosaicf, fact2, pseudoHessianMosaicf, 0);

	pseudoHessianMosaic = convert_gray32_to_color_blending(pseudoHessianMosaicf);

	cvNamedWindow( "pseudoHessianMosaic", 1 );
	cvShowImage( "pseudoHessianMosaic", pseudoHessianMosaic );
	cvWaitKey(0);

	/***********************************************************************************
	Computing the error of the norm(SR)
	*****************************************************************************************/
	/*
	//printf(" Computing errorSR = %f \t *errorSRp = %f n", errorSR, *errorSRp);

	//if (it >= 2){

	errorSR = cvNorm(panoramaSR, prev_panorama, CV_RELATIVE_L2, NULL);

	errorSRp[it-1] = errorSR;
	printf(" errorSR = %f \n", errorSR);

  	*/

	cvReleaseMat( &kernelLLt );
	cvReleaseMat( &kernelL );
	cvReleaseMat( &kernelB );
	cvReleaseMat( &lambdaM );



	return;

}

extern void Gradient_Mosaic(IplImage* img1, IplImage* img2, IplImage* img3, IplImage* img4, IplImage* img5, IplImage* prev_panorama, 
					struct image_border ROI_frame1, struct image_border ROI_frame2, struct image_border ROI_frame3, struct image_border ROI_frame4, struct image_border ROI_frame5,
					IplImage* GradientMosaic, double* lambda_itp)//, int *status)//, int D)
					{

//IplImage* panoramaSR = NULL;
IplImage* img1_inv = NULL, *img2_inv = NULL, *img3_inv=NULL, *img4_inv=NULL, *img5_inv=NULL;
IplImage* img1_inv_temp1 = NULL, *img2_inv_temp1 = NULL, *img3_inv_temp1=NULL, *img4_inv_temp1=NULL, *img5_inv_temp1=NULL;
IplImage* img1_inv_temp2 = NULL, *img2_inv_temp2 = NULL, *img3_inv_temp2=NULL, *img4_inv_temp2=NULL, *img5_inv_temp2=NULL;

IplImage* dimg1=NULL, *dimg2=NULL, *dimg3=NULL, *dimg4=NULL, *dimg5=NULL;
IplImage* Ddimg1=NULL, *Ddimg2=NULL, *Ddimg3=NULL, *Ddimg4=NULL, *Ddimg5=NULL;
IplImage* ddst1 = NULL, *ddst2 = NULL, *ddst3 = NULL, *ddst4 = NULL, *ddst5=NULL;
IplImage* panorama5d = NULL, *delta_SRf = NULL, *delta_SR_u8, *panorama5_Deltaf = NULL, *panorama5df = NULL;
IplImage* panorama5d2 = NULL; //after multiplication with alpha
IplImage* panorama5LLt = NULL;
IplImage* panorama5LLt2 = NULL; //after multiplication
IplImage* panorama5_Delta = NULL;
IplImage* Xregularized = NULL; // To use with the algorithm Fast and Robust Regularization
IplImage* Limg = NULL;

CvMat* kernelLLt = cvCreateMat(3,3,CV_32FC1);
CvMat* kernelB = cvCreateMat(3,3,CV_32FC1);
CvMat* kernelL = cvCreateMat(3,3,CV_32FC1);

CvMat* lambdaM = NULL;

double lambda_1, lambda_2, lambda_3, lambda_4, lambda_5;
double lambda_num, lambda_den;
double lambdad_1, lambdad_2, lambdad_3, lambdad_4, lambdad_5;
double lambda_it;
double alpha_it = 1.1127;// was 7, 1, 1.1127
double errorSR = 0.0;

float alpha = 0.0;

CvPoint2D32f P; // P for the mosaic

int i, j, D;

struct RegionS  S1, S2, S3, S4, S5;

CvSize sizeD1, sizeD2, sizeD3, sizeD4, sizeD5;
CvSize size; //size of the frames

//FILE *pFile;

cvZero(GradientMosaic);


	/// First part:  Interpolation : resizing the image to 2 times the original size

  D = 2;

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

	size.height = img1->height;
	size.width = img1->width;

/**************************************************************************************/ 
   //   Starting a new iteration (it = 3)
   //   
	//  First Step: Getting the imgs from the mosaic image
   //
   /**************************************************************************************/
/*
	allocateOnDemand( &img1_inv, cvSize(img2->width,img2->height),  img1->depth, img1->nChannels );
    WarpImageUnderQuadInv(prev_panorama,img1_inv,ROI_frame1);

    allocateOnDemand( &img2_inv, cvSize(img2->width,img2->height),  img1->depth, img1->nChannels );
    WarpImageUnderQuadInv(prev_panorama,img2_inv,ROI_frame2);

	allocateOnDemand( &img3_inv, cvSize(img2->width,img2->height),  img1->depth, img1->nChannels );
	WarpImageUnderQuadInv(prev_panorama,img3_inv,ROI_frame3);

	allocateOnDemand( &img4_inv, cvSize(img2->width,img2->height),  img1->depth, img1->nChannels );
	WarpImageUnderQuadInv(prev_panorama,img4_inv,ROI_frame4);

	allocateOnDemand( &img5_inv, cvSize(img2->width,img2->height),  img1->depth, img1->nChannels );
	WarpImageUnderQuadInv(prev_panorama,img5_inv,ROI_frame5);
*/

	allocateOnDemand( &img1_inv_temp1, sizeD1,  img1->depth, img1->nChannels );
    WarpImageUnderQuadInv(prev_panorama,img1_inv_temp1,ROI_frame1);

    allocateOnDemand( &img2_inv_temp1, sizeD1,  img1->depth, img1->nChannels );
    WarpImageUnderQuadInv(prev_panorama,img2_inv_temp1,ROI_frame2);

	allocateOnDemand( &img3_inv_temp1, sizeD1,  img1->depth, img1->nChannels );
	WarpImageUnderQuadInv(prev_panorama,img3_inv_temp1,ROI_frame3);

	allocateOnDemand( &img4_inv_temp1, sizeD1,  img1->depth, img1->nChannels );
	WarpImageUnderQuadInv(prev_panorama,img4_inv_temp1,ROI_frame4);

	allocateOnDemand( &img5_inv_temp1, sizeD1,  img1->depth, img1->nChannels );
	WarpImageUnderQuadInv(prev_panorama,img5_inv_temp1,ROI_frame5);

	// Using a Gaussian Filter to perform an Antia-alias filtering
	
	allocateOnDemand( &img1_inv_temp2, sizeD1,  img1->depth, img1->nChannels );
    allocateOnDemand( &img2_inv_temp2, sizeD1,  img1->depth, img1->nChannels );
    allocateOnDemand( &img3_inv_temp2, sizeD1,  img1->depth, img1->nChannels );
	allocateOnDemand( &img4_inv_temp2, sizeD1,  img1->depth, img1->nChannels );
	allocateOnDemand( &img5_inv_temp2, sizeD1,  img1->depth, img1->nChannels );
/*
	cvSmooth(img1_inv_temp1,img1_inv_temp2, CV_GAUSSIAN, 3, 0, 0, 0);
	cvSmooth(img2_inv_temp1,img2_inv_temp2, CV_GAUSSIAN, 3, 0, 0, 0);
	cvSmooth(img3_inv_temp1,img3_inv_temp2, CV_GAUSSIAN, 3, 0, 0, 0);
	cvSmooth(img4_inv_temp1,img4_inv_temp2, CV_GAUSSIAN, 3, 0, 0, 0);
	cvSmooth(img5_inv_temp1,img5_inv_temp2, CV_GAUSSIAN, 3, 0, 0, 0);
*/
	// Using Filter 2D:
    //This kernel is the same as the used in the paper: " A Transform-Domain Approach to Super-resolution Mosaicing of Compressed Images"

	cvmSet(kernelB,0,0,0.1016);
	cvmSet(kernelB,0,1,0.1172);
	cvmSet(kernelB,0,2,0.1016);
	cvmSet(kernelB,1,0,0.1172);
	cvmSet(kernelB,1,1,0.1250);
	cvmSet(kernelB,1,2,0.1172);
	cvmSet(kernelB,2,0,0.1016);
	cvmSet(kernelB,2,1,0.1172);
	cvmSet(kernelB,2,2,0.1016);

	cvFilter2D(img1_inv_temp1,img1_inv_temp2,kernelB,cvPoint(-1,1));
	cvFilter2D(img2_inv_temp1,img2_inv_temp2,kernelB,cvPoint(-1,1));
	cvFilter2D(img3_inv_temp1,img3_inv_temp2,kernelB,cvPoint(-1,1));
	cvFilter2D(img4_inv_temp1,img4_inv_temp2,kernelB,cvPoint(-1,1));
	cvFilter2D(img5_inv_temp1,img5_inv_temp2,kernelB,cvPoint(-1,1));
	
  // Downsampling using cubic interpolation

	allocateOnDemand( &img1_inv, size,  img1->depth, img1->nChannels );
    allocateOnDemand( &img2_inv, size,  img1->depth, img1->nChannels );
    allocateOnDemand( &img3_inv, size,  img1->depth, img1->nChannels );
	allocateOnDemand( &img4_inv, size,  img1->depth, img1->nChannels );
	allocateOnDemand( &img5_inv, size,  img1->depth, img1->nChannels );
	
	cvResize(img1_inv_temp2, img1_inv, CV_INTER_CUBIC);
	cvResize(img2_inv_temp2, img2_inv, CV_INTER_CUBIC);
	cvResize(img3_inv_temp2, img3_inv, CV_INTER_CUBIC);
	cvResize(img4_inv_temp2, img4_inv, CV_INTER_CUBIC);
	cvResize(img5_inv_temp2, img5_inv, CV_INTER_CUBIC);

	/*
// Downsampling using my own code

	DownSampleImage(img1_inv_temp2, img1_inv, 2);
	DownSampleImage(img2_inv_temp2, img2_inv, 2);
	DownSampleImage(img3_inv_temp2, img3_inv, 2);
	DownSampleImage(img4_inv_temp2, img4_inv, 2);
	DownSampleImage(img5_inv_temp2, img5_inv, 2);
*/
/*
	cvSaveImage("DBWRx1.jpg", img1_inv);
	cvSaveImage("DBWRx2.jpg", img2_inv);
	cvSaveImage("DBWRx3.jpg", img3_inv);
	cvSaveImage("DBWRx4.jpg", img4_inv);
	cvSaveImage("DBWRx5.jpg", img5_inv);


	cvNamedWindow( "img1_inv", 1 );
	cvShowImage( "img1_inv", img1_inv);
	cvWaitKey(0);

	cvNamedWindow( "img2_inv", 1 );
	cvShowImage( "img2_inv", img2_inv);
	cvWaitKey(0);

	cvNamedWindow( "img3_inv", 1 );
	cvShowImage( "img3_inv", img3_inv );
	cvWaitKey(0);

	cvNamedWindow( "img4_inv", 1 );
	cvShowImage( "img4_inv", img4_inv );
	cvWaitKey(0);

	cvNamedWindow( "img5_inv", 1 );
	cvShowImage( "img5_inv", img5_inv );
	cvWaitKey(0);
*/	

   /**************************************************************************************/ 
   //    
   //   
   //  Second Step: Compute the frame difference between the original low frames and the LR reconstructed ones
   //
   /**************************************************************************************/

  /************************************************************************/
  //  Doing the frame substracion
  /****************************************************************************/
 //  printf("\n Doing the frame substracion ...\n" );


  allocateOnDemand( &dimg1, cvSize(img2->width,img2->height),  img1->depth, img1->nChannels );
  allocateOnDemand( &dimg2, cvSize(img2->width,img2->height),  img1->depth, img1->nChannels );
  allocateOnDemand( &dimg3, cvSize(img2->width,img2->height),  img1->depth, img1->nChannels );
  allocateOnDemand( &dimg4, cvSize(img2->width,img2->height),  img1->depth, img1->nChannels );
  allocateOnDemand( &dimg5, cvSize(img2->width,img2->height),  img1->depth, img1->nChannels );
/*
   printf("\n size_img1 = (%d , %d) \t size_img1_inv = (%d, %d)", img1->height, img1->width, img1_inv->height, img1_inv->width);
   printf("\n size_dimg1 = (%d , %d) ", dimg1->height, dimg1->width);

   getchar();
*/

  cvSub(img1,img1_inv,dimg1,NULL);
  cvSub(img2,img2_inv,dimg2,NULL);
  cvSub(img3,img3_inv,dimg3,NULL);
  cvSub(img4,img4_inv,dimg4,NULL);
  cvSub(img5,img5_inv,dimg5,NULL);

 
  //*********************************************************************************************//
  //  Doing the interpolation of the images differences											//
  //*********************************************************************************************//
    //printf("\n Doing the frame interpolation of the difference of frames ...\n" );
    allocateOnDemand( &Ddimg1, sizeD1,  img1->depth, img1->nChannels );  
	allocateOnDemand( &Ddimg2, sizeD2,  img1->depth, img1->nChannels );  
	allocateOnDemand( &Ddimg3, sizeD3,  img1->depth, img1->nChannels );  
	allocateOnDemand( &Ddimg4, sizeD4,  img1->depth, img1->nChannels );  
	allocateOnDemand( &Ddimg5, sizeD5,  img1->depth, img1->nChannels );  

	/*
  	cvResize(dimg1, Ddimg1,CV_INTER_CUBIC);
	cvResize(dimg2, Ddimg2,CV_INTER_CUBIC);
	cvResize(dimg3, Ddimg3,CV_INTER_CUBIC);
	cvResize(dimg4, Ddimg4,CV_INTER_CUBIC);
	cvResize(dimg5, Ddimg5,CV_INTER_CUBIC);
	*/

	UpSampleImageD(dimg1, Ddimg1);
	UpSampleImageD(dimg2, Ddimg2);
	UpSampleImageD(dimg3, Ddimg3);
	UpSampleImageD(dimg4, Ddimg4);
	UpSampleImageD(dimg5, Ddimg5);

/***********************************************************************************/
//			 Doing the Mosaicking of the Error Images								//
//																					//
/***********************************************************************************/
	//printf("\n Doing the Mosaicking of the Error Images ...\n" );

	allocateOnDemand( &ddst1,  cvSize(GradientMosaic->width,GradientMosaic->height),  img1->depth, img1->nChannels );  
	allocateOnDemand( &ddst2,  cvSize(GradientMosaic->width,GradientMosaic->height),  img1->depth, img1->nChannels );  
	allocateOnDemand( &ddst3,  cvSize(GradientMosaic->width,GradientMosaic->height),  img1->depth, img1->nChannels );  
	allocateOnDemand( &ddst4,  cvSize(GradientMosaic->width,GradientMosaic->height),  img1->depth, img1->nChannels );  
	allocateOnDemand( &ddst5,  cvSize(GradientMosaic->width,GradientMosaic->height),  img1->depth, img1->nChannels );  
	
	/*
	allocateOnDemand( &ddst1,  cvSize(1360, 640),  img1->depth, img1->nChannels );  
	allocateOnDemand( &ddst2,  cvSize(1360, 640),  img1->depth, img1->nChannels );  
	allocateOnDemand( &ddst3,  cvSize(1360, 640),  img1->depth, img1->nChannels );  
	allocateOnDemand( &ddst4,  cvSize(1360, 640),  img1->depth, img1->nChannels );  
	allocateOnDemand( &ddst5,  cvSize(1360, 640),  img1->depth, img1->nChannels );  
*/

    WarpImageUnderQuad(Ddimg1,ddst1,ROI_frame1);
	//cvNamedWindow( "Perspective_Warpdd1", 1 );
	//cvShowImage( "Perspective_Warpdd1", ddst1);
	//cvWaitKey(10);


  WarpImageUnderQuad(Ddimg2,ddst2,ROI_frame2);
 // cvNamedWindow( "Perspective_Warpdd2", 1 );
 // cvShowImage( "Perspective_Warpdd2", ddst2 );
  //cvWaitKey(10);


 WarpImageUnderQuad(Ddimg3,ddst3,ROI_frame3);
 //cvNamedWindow( "Perspective_Warpdd3", 1 );
 //cvShowImage( "Perspective_Warpdd3", ddst3 );
 //cvWaitKey(10);


 WarpImageUnderQuad(Ddimg4,ddst4,ROI_frame4);
 //cvNamedWindow( "Perspective_Warpdd4", 1 );
 //cvShowImage( "Perspective_Warpdd4", ddst4 );
 //cvWaitKey(10);

 
 WarpImageUnderQuad(Ddimg5,ddst5,ROI_frame5);
 //cvNamedWindow( "Perspective_Warpdd5", 1 );
 ///cvShowImage( "Perspective_Warpdd5", ddst5 );
 //cvWaitKey(10);

 allocateOnDemand( &panorama5d,  cvSize(prev_panorama->width, prev_panorama->height),  img1->depth, img1->nChannels );  

 S2 = FindSRegion(ROI_frame2);

	  for(i = 0; i < panorama5d->height; i ++){ //rows
		  for(j = 0; j < panorama5d->width; j++){ //cols
			  P.x = j;
			  P.y = i;
			  if(Belongs2S(P,S2))cvSet2D(panorama5d,i,j,cvGet2D(ddst2,i,j));
			  else cvSet2D(panorama5d,i,j,cvGet2D(ddst1,i,j));			  
		  }//end for
	  }// end for

	

////////////////////////////
// Finding the region S3 //
////////////////////////////

   S3 = FindSRegion(ROI_frame3);

	  for(i = 0; i < panorama5d->height; i ++){ //rows
		  for(j = 0; j < panorama5d->width; j++){ //cols
			  P.x = j;
			  P.y = i;
			  if(Belongs2S(P,S3))cvSet2D(panorama5d,i,j,cvGet2D(ddst3,i,j));
			  else cvSet2D(panorama5d,i,j,cvGet2D(panorama5d,i,j));			  
		  }//end for
	  }// end for



 ////////////////////////////
// Finding the region S4 //
////////////////////////////

   S4 = FindSRegion(ROI_frame4);

	  for(i = 0; i < GradientMosaic->height; i ++){ //rows
		  for(j = 0; j < GradientMosaic->width; j++){ //cols
			  P.x = j;
			  P.y = i;
			  if(Belongs2S(P,S4))cvSet2D(panorama5d,i,j,cvGet2D(ddst4,i,j));
			  else cvSet2D(panorama5d,i,j,cvGet2D(panorama5d,i,j));			  
		  }//end for
	  }// end for


	    ////////////////////////////
		// Finding the region S4 //
		////////////////////////////

   S5 = FindSRegion(ROI_frame5);


	  for(i = 0; i < panorama5d->height; i ++){ //rows
		  for(j = 0; j < panorama5d->width; j++){ //cols
			  P.x = j;
			  P.y = i;
			  if(Belongs2S(P,S5))cvSet2D(panorama5d,i,j,cvGet2D(ddst5,i,j));
			  else cvSet2D(panorama5d,i,j,cvGet2D(panorama5d,i,j));			  
		  }//end for
	  }// end for

	//cvNamedWindow( "GradientMosaic", 1 );
	//cvShowImage( "GradientMosaic", GradientMosaic );
	//cvWaitKey(0);
	//GradientMosaic = cvCloneImage(panorama5d);

	  
   /**************************************************************************************/
   //
   // Computing the normalizing parameter lambda^(n) 
   //
   /**************************************************************************************/
    
	  lambda_1 = cvNorm(img1, img1_inv, CV_L2, NULL);
	 // printf("\n lambda_1 = %f", lambda_1);
	  lambda_2 = cvNorm(img2, img2_inv, CV_L2, NULL);
	 // printf("\n lambda_2 = %f", lambda_2);
	  lambda_3 = cvNorm(img3, img3_inv, CV_L2, NULL);
	 // printf("\n lambda_3 = %f", lambda_3);
	  lambda_4 = cvNorm(img4, img4_inv, CV_L2, NULL);
	 // printf("\n lambda_4 = %f", lambda_4);
	  lambda_5 = cvNorm(img5, img5_inv, CV_L2, NULL);
	 // printf("\n lambda_5 = %f", lambda_5);

   /**************************************************************************************/
   //
   // Computing the Laplace of the x^(n) image x(n) comes to be the panorama5 variables (mosaic of the previous state)
   //
   /**************************************************************************************/
   allocateOnDemand( &Limg, cvSize(prev_panorama->width,prev_panorama->height),  img1->depth, img1->nChannels );  
   //cvLaplace(prev_panorama, Limg,3);   
/*
// Uncomment this to use Laplace Prior Function

   // Using in stead cvFilter2D

   	cvmSet(kernelL,0,0,0.0000);
	cvmSet(kernelL,0,1,0.2500);
	cvmSet(kernelL,0,2,0.0000);
	cvmSet(kernelL,1,0,0.2500);
	cvmSet(kernelL,1,1,-1.000);
	cvmSet(kernelL,1,2,0.2500);
	cvmSet(kernelL,2,0,0.0000);
	cvmSet(kernelL,2,1,0.2500);
	cvmSet(kernelL,2,2,0.0000);

   cvFilter2D(prev_panorama, Limg, kernelL, cvPoint(-1,-1));
*/
 // Uncomment this to use Huber Prior Function
  // Using Huber Prior Function to find lambda

     alpha = 1.75;//was 1.75

     HubbertFunctionCliques(prev_panorama, Limg, alpha);

  // cvNamedWindow( "Limg", 1 );
  // cvShowImage( "Limg", Limg );
  // cvWaitKey(0);


   /**************************************************************************************/
   //
   // Computing the K norm(x^(n),2) , K = 1, 2 ...5
   //
   /**************************************************************************************/

  lambda_num = pow((lambda_1 + lambda_2 + lambda_3 + lambda_4 + lambda_5),2);
  lambdad_1 = cvNorm(Limg, NULL, CV_L2, NULL);
 

    /**************************************************************************************/
   //
   // Computing the lambda(n), K = 5, because I am just working with 5 frames
   //
   /**************************************************************************************/
 
  lambda_den = pow((5*lambdad_1),2);

    
  lambda_it = lambda_num/lambda_den;

  
  // printf("\n lambda_num = %f", lambda_num);
  // printf("\n lambda_den = %f", lambda_den);
  // printf("\n lambda_it = %f", lambda_it);

   *lambda_itp =  lambda_it;

   printf("\n lambda_it = %f \n", *lambda_itp);

  // pFile = fopen(name, "w");

   //fprintf(pFile, "%f\n", lambda_it);

  // fclose(pFile);
   
  // lambda_it = 1.1127;

  // printf("\n Using alpha from GCV = %f", alpha_it);

    /**************************************************************************************/
   //
   // Computing the lambda(n)*L'*L*x(n)
   //
   /**************************************************************************************/

   /*
    cvmSet(kernelLLt,0,0,1.000*lambda_it);
	cvmSet(kernelLLt,0,1,-4.000*lambda_it);
	cvmSet(kernelLLt,0,2,1.000*lambda_it);
	cvmSet(kernelLLt,1,0,-4.000*lambda_it);
	cvmSet(kernelLLt,1,1,18.000*lambda_it);
	cvmSet(kernelLLt,1,2,-4.000*lambda_it);
	cvmSet(kernelLLt,2,0,1.000*lambda_it);
	cvmSet(kernelLLt,2,1,-4.000*lambda_it);
	cvmSet(kernelLLt,2,2,1.000*lambda_it);
*/
	cvmSet(kernelLLt,0,0,1.000);
	cvmSet(kernelLLt,0,1,-4.000);
	cvmSet(kernelLLt,0,2,1.000);
	cvmSet(kernelLLt,1,0,-4.000);
	cvmSet(kernelLLt,1,1,18.000);
	cvmSet(kernelLLt,1,2,-4.000);
	cvmSet(kernelLLt,2,0,1.000);
	cvmSet(kernelLLt,2,1,-4.000);
	cvmSet(kernelLLt,2,2,1.000);

	allocateOnDemand( &panorama5LLt, cvSize(prev_panorama->width,prev_panorama->height),  img1->depth, img1->nChannels );  
	allocateOnDemand( &panorama5LLt2, cvSize(prev_panorama->width,prev_panorama->height),  img1->depth, img1->nChannels );  
	allocateOnDemand( &panorama5d2, cvSize(prev_panorama->width,prev_panorama->height),  img1->depth, img1->nChannels ); 

 //	printf("\n Limg->height = %d \t Limg->width = %d", Limg->height, Limg->width);
 //	printf("\n prev_panorama->height = %d \t prev_panorama->width = %d", prev_panorama->height, prev_panorama->width);

  //  Uncomment to use Huber Regularization

    // Using Huber Regularization here as well
	HubbertRegularization(Limg, panorama5LLt, alpha);
/*
  // Uncomment to use Laplace Regularization
    // Using Laplace prior
	cvFilter2D(Limg,panorama5LLt,kernelL,cvPoint(-1,1));
	cvFilter2D(panorama5LLt,panorama5LLt2, kernelL,cvPoint(-1,1)); // Because os L^t*L*x
*/	

/*
    //Using the regularization proposed in the paper: "Fast and Robust Super-Resolution"

	allocateOnDemand( &Xregularized, cvSize(prev_panorama->width,prev_panorama->height), IPL_DEPTH_32F, 1 );

	FastRobustRegularization(Limg, Xregularized, 2, alpha); // p = 2

	panorama5LLt = convert_gray32_to_color(Xregularized);

*/

	/*
	//
	// Converting to Matrix lambda doesn't work, because of the size of the mosaicking
	//

	lambdaM = cvCreateMat(panorama5LLt->height,panorama5LLt->width,CV_32FC1);
	cvSetIdentity(lambdaM,cvRealScalar(lambda_it));

	cvMatMul(panorama5LLt,lambdaM, panorama5LLt2);
	*/

	panorama5LLt2 = MulImagewithDouble(panorama5LLt, lambda_it);// Use this for Huber Regularization
	
	//panorama5LLt2 = MulImagewithDouble(panorama5LLt2, lambda_it); // Use this for Laplace Regularization
	
	panorama5d2 = MulImagewithDouble(panorama5d, alpha_it);

   /**************************************************************************************/ 
   //
   //	Now updating the iteration of x(n+1). First I have to do: panorama5d - panorama5LLt
   //   Then I have to do : panorma5_2 = panorama5_1 + (panorama5d - panorama5LLt)
   //
   /**************************************************************************************/
	
	//allocateOnDemand( &panoramaSR, cvSize(prev_panorama->width,prev_panorama->height),  img1->depth, img1->nChannels );  
	//allocateOnDemand( &panorama5_Delta, cvSize(prev_panorama->width,prev_panorama->height),  img1->depth, img1->nChannels );  

//	cvSub(panorama5d2,panorama5LLt2,panorama5_Delta,NULL);
	cvSub(panorama5d2,panorama5LLt2,GradientMosaic, NULL);

  return;

}

extern void SRMosaicking_Quasi_Newton_DFP_pixel32(IplImage* img1, IplImage* img2, IplImage* img3, IplImage* img4, IplImage* img5, IplImage* prev_panorama, 
					struct image_border ROI_frame1, struct image_border ROI_frame2, struct image_border ROI_frame3, struct image_border ROI_frame4, struct image_border ROI_frame5,
					IplImage* panoramaSR, IplImage* prev_GradientMosaic, IplImage* GradientMosaic, IplImage* prev_pseudoHessianMosaic, IplImage* pseudoHessianMosaic, double* lambda_itp,double* errorSRp, int it)
					{

//IplImage* panoramaSR = NULL;
IplImage* img1_inv = NULL, *img2_inv = NULL, *img3_inv=NULL, *img4_inv=NULL, *img5_inv=NULL;
IplImage* img1_inv_temp1 = NULL, *img2_inv_temp1 = NULL, *img3_inv_temp1=NULL, *img4_inv_temp1=NULL, *img5_inv_temp1=NULL;
IplImage* img1_inv_temp2 = NULL, *img2_inv_temp2 = NULL, *img3_inv_temp2=NULL, *img4_inv_temp2=NULL, *img5_inv_temp2=NULL;

IplImage* dimg1=NULL, *dimg2=NULL, *dimg3=NULL, *dimg4=NULL, *dimg5=NULL;
IplImage* Ddimg1=NULL, *Ddimg2=NULL, *Ddimg3=NULL, *Ddimg4=NULL, *Ddimg5=NULL;
IplImage* ddst1 = NULL, *ddst2 = NULL, *ddst3 = NULL, *ddst4 = NULL, *ddst5=NULL;
IplImage* panorama5d = NULL, *delta_SRf = NULL, *delta_SR_u8, *panorama5_Deltaf = NULL, *panorama5df = NULL;
IplImage* panorama5d2 = NULL; //after multiplication with alpha
IplImage* panorama5LLt = NULL;
IplImage* panorama5LLt2 = NULL; //after multiplication
IplImage* panorama5_Delta = NULL;
IplImage* Xregularized = NULL; // To use with the algorithm Fast and Robust Regularization
IplImage* Limg = NULL;

// New staff here
IplImage *nGradientMosaic =  NULL, *delta_Gradient =  NULL, *delta_Gradientf =  NULL, *rk =  NULL, *rkf =  NULL;
IplImage *rkt =  NULL, *rkft =  NULL, *delta_Gradient_t =  NULL, *num1 =  NULL, *num2 =  NULL, *den1 =  NULL;
IplImage *den2 =  NULL, *iden1 =  NULL, *iden2 =  NULL, *fact1 =  NULL, *fact2 =  NULL;
IplImage *panorama5_Deltaft =  NULL, *npseudoHessianMosaic =  NULL,  *npseudoHessianMosaicf =  NULL, *pseudoHessianMosaicf =  NULL;
IplImage* prev_pseudoHessianMosaicf = NULL;

CvMat* kernelLLt = cvCreateMat(3,3,CV_32FC1);
CvMat* kernelB = cvCreateMat(3,3,CV_32FC1);
CvMat* kernelL = cvCreateMat(3,3,CV_32FC1);

CvMat* lambdaM = NULL;

double lambda_1, lambda_2, lambda_3, lambda_4, lambda_5;
double lambda_num, lambda_den;
double lambdad_1, lambdad_2, lambdad_3, lambdad_4, lambdad_5;
double lambda_it;
double alpha_it = 1.1127;// was 7, 1, 1.1127
double errorSR = 0.0;

float alpha = 0.0;
float pixel32_Qk, pixel32_Qkt, pixel32_delta_gk, pixel32_delta_gkt, pixel32_delta_xk, pixel32_delta_xkt, pixel32_rk, pixel32_rkt, factor1, factor2, pixel32_Qk1;

CvPoint2D32f P; // P for the mosaic

int i, j, D;

struct RegionS  S1, S2, S3, S4, S5;

CvSize sizeD1, sizeD2, sizeD3, sizeD4, sizeD5;
CvSize size; //size of the frames

//FILE *pFile;

cvZero(panoramaSR);


// Computing: alpha*Qk*g^k
allocateOnDemand( &panorama5d, cvSize(prev_panorama->width,prev_panorama->height),  img1->depth, img1->nChannels );  
if( it == 1) panorama5d = cvCloneImage(prev_GradientMosaic);
else cvMul(prev_pseudoHessianMosaic, prev_GradientMosaic, panorama5d, 1);

   

   // computing x^(k+1)
	printf(" Computing x(k+1) .... \n");

	cvAdd(prev_panorama, panorama5d, panoramaSR, NULL);

	cvNamedWindow( "panoramaSR", 1 );
	cvShowImage( "panoramaSR", panoramaSR );
	cvWaitKey(0);

	// Computing x^(k+1) - x^(k) 

	printf(" Computing delta_x(k+1) .... \n");

	allocateOnDemand( &panorama5_Delta, cvSize(prev_panorama->width,prev_panorama->height),  img1->depth, img1->nChannels );  
	cvSub(panoramaSR,prev_panorama,panorama5_Delta,NULL);

	

	// Computing: g^(k+1) = Jf(x^(k+1))
	// nGradientMosaic = g^(k+1)

	printf(" Computing the new gradient g(k+1) .... \n");
    //allocateOnDemand( &nGradientMosaic, cvSize(prev_panorama->width,prev_panorama->height),  img1->depth, img1->nChannels );  

	Gradient_Mosaic(img1, img2, img3, img4, img5, panoramaSR, ROI_frame1,ROI_frame2, ROI_frame3, ROI_frame4, ROI_frame5, GradientMosaic, lambda_itp);

	// Computing: delta_g(k) = g(k+1) - g(k)

	printf(" Computing the delta_g(k) .... \n");
	allocateOnDemand( &delta_Gradient, cvSize(prev_panorama->width,prev_panorama->height),  img1->depth, img1->nChannels );  
    cvSub(GradientMosaic, prev_GradientMosaic, delta_Gradient,0);

	//cvNamedWindow( "pseudoHessianMosaic", 1 );
	//cvShowImage( "pseudoHessianMosaic", pseudoHessianMosaic );
	//cvWaitKey(0);

	//cvNamedWindow( "delta_Gradient", 1 );
	//cvShowImage( "delta_Gradient", delta_Gradient );
	//cvWaitKey(0);
	
	// Computing: rk = Qk*(delta_g(k))
	//allocateOnDemand( &rk, cvSize(prev_panorama->width,prev_panorama->height),  img1->depth, img1->nChannels );  
	//allocateOnDemand( &rkf, cvSize(prev_panorama->width,prev_panorama->height), IPL_DEPTH_32F, 1 ); 
	allocateOnDemand( &delta_Gradientf, cvSize(prev_panorama->width,prev_panorama->height),   IPL_DEPTH_32F, 1 );
	allocateOnDemand( &panorama5_Deltaf, cvSize(prev_panorama->width,prev_panorama->height),   IPL_DEPTH_32F, 1 );
	//allocateOnDemand( &panorama5_Deltaft, cvSize(prev_panorama->width,prev_panorama->height),   IPL_DEPTH_32F, 1 );
	//allocateOnDemand( &npseudoHessianMosaicf, cvSize(prev_panorama->width,prev_panorama->height),   IPL_DEPTH_32F, 1 );
	allocateOnDemand( &pseudoHessianMosaicf, cvSize(prev_panorama->width,prev_panorama->height),   IPL_DEPTH_32F, 1 );
	allocateOnDemand( &prev_pseudoHessianMosaicf, cvSize(prev_panorama->width,prev_panorama->height),   IPL_DEPTH_32F, 1 );
	//allocateOnDemand( &rkft, cvSize(prev_panorama->width,prev_panorama->height),   IPL_DEPTH_32F, 1 );
	//allocateOnDemand( &den1, cvSize(prev_panorama->width,prev_panorama->height),   IPL_DEPTH_32F, 1 );
	//allocateOnDemand( &iden1, cvSize(prev_panorama->width,prev_panorama->height),   IPL_DEPTH_32F, 1 );
	/*allocateOnDemand( &num1, cvSize(prev_panorama->width,prev_panorama->height),   IPL_DEPTH_32F, 1 );
	allocateOnDemand( &num2, cvSize(prev_panorama->width,prev_panorama->height),   IPL_DEPTH_32F, 1 );
	allocateOnDemand( &den2, cvSize(prev_panorama->width,prev_panorama->height),   IPL_DEPTH_32F, 1 );
	allocateOnDemand( &iden2, cvSize(prev_panorama->width,prev_panorama->height),   IPL_DEPTH_32F, 1 );
	allocateOnDemand( &fact1, cvSize(prev_panorama->width,prev_panorama->height),   IPL_DEPTH_32F, 1 );
	allocateOnDemand( &fact2, cvSize(prev_panorama->width,prev_panorama->height),   IPL_DEPTH_32F, 1 );
*/

	//rkf = convert2gray32(rk);
	delta_Gradientf = convert2gray32(delta_Gradient);
	panorama5_Deltaf = convert2gray32(panorama5_Delta);
	prev_pseudoHessianMosaicf = convert2gray32(prev_pseudoHessianMosaic);

	if(it == 1) ContructAuI(prev_pseudoHessianMosaicf, 1.00);

    printf(" Computing the r(k) .... \n");
	if( it == 1)	rk = cvCloneImage(delta_Gradient);
	else  cvMul(prev_pseudoHessianMosaic, delta_Gradient, rk, 1);

	printf(" Working with iteration  ... %d...\n", it);

	

	for(i = 0; i < prev_panorama->height; i++){
		for(j = 0; j < prev_panorama->width; j++){
			pixel32_Qk = ((float *)(prev_pseudoHessianMosaicf->imageData + i*prev_pseudoHessianMosaicf->widthStep))[j];
			pixel32_Qkt = ((float *)(prev_pseudoHessianMosaicf->imageData + j*prev_pseudoHessianMosaicf->widthStep))[i];
			pixel32_delta_gk = ((float *)(delta_Gradientf->imageData + i*delta_Gradientf->widthStep))[j];
			pixel32_delta_gkt = ((float *)(delta_Gradientf->imageData + j*delta_Gradientf->widthStep))[i];
			pixel32_delta_xk = ((float *)(panorama5_Deltaf->imageData + i*panorama5_Deltaf->widthStep))[j];
			pixel32_delta_xkt = ((float *)(panorama5_Deltaf->imageData + j*panorama5_Deltaf->widthStep))[i];
            pixel32_rk = pixel32_Qk*pixel32_delta_gk;
			pixel32_rkt = pixel32_delta_gkt*pixel32_Qkt;

			factor1 = (pixel32_rk*pixel32_rkt)/(pixel32_rkt*pixel32_delta_gk);
			factor2 = (pixel32_delta_xk*pixel32_delta_xkt)/(pixel32_delta_xkt*pixel32_delta_gk);

			pixel32_Qk1 = pixel32_Qk - factor1 + factor2;
			((float *)(pseudoHessianMosaicf->imageData + j*pseudoHessianMosaicf->widthStep))[i] = pixel32_Qk1;
		}
	}

         
	pseudoHessianMosaic = convert_gray32_to_color_blending(pseudoHessianMosaicf);

	cvNamedWindow( "pseudoHessianMosaic", 1 );
	cvShowImage( "pseudoHessianMosaic", pseudoHessianMosaic );
	cvWaitKey(0);

	/***********************************************************************************
	Computing the error of the norm(SR)
	*****************************************************************************************/
	/*
	//printf(" Computing errorSR = %f \t *errorSRp = %f n", errorSR, *errorSRp);

	//if (it >= 2){

	errorSR = cvNorm(panoramaSR, prev_panorama, CV_RELATIVE_L2, NULL);

	errorSRp[it-1] = errorSR;
	printf(" errorSR = %f \n", errorSR);

  	*/

	cvReleaseMat( &kernelLLt );
	cvReleaseMat( &kernelL );
	cvReleaseMat( &kernelB );
	cvReleaseMat( &lambdaM );



	return;

}
/*
extern struct feature* get_match_inliers( struct feature* feat, int mtype )
{
	if( mtype == FEATURE_MDL_MATCH )
		return feat->mdl_match;
	if( mtype == FEATURE_BCK_MATCH )
		return feat->bck_match;
	if( mtype == FEATURE_FWD_MATCH )
		return feat->fwd_match;
	return NULL;
}

extern int get_matched_features_inliers( struct feature* features, int n, int mtype,
struct feature*** matched )
{
	struct feature** _matched;
	struct ransac_data* rdata;
	int i, m = 0;

	_matched = calloc( n, sizeof( struct feature* ) );
	for( i = 0; i < n; i++ )
		if( get_match_inliers( features + i, mtype ) )
		{
			rdata = malloc( sizeof( struct ransac_data ) );
			memset( rdata, 0, sizeof( struct ransac_data ) );
			rdata->orig_feat_data = features[i].feature_data;
			_matched[m] = features + i;
			_matched[m]->feature_data = rdata;
			m++;
		}
		*matched = _matched;
		return m;
}
// This function is a modification of ransac_xform to compute the inliers and also the correspondent features associated
CvMat* ransac_xform_inliers( struct feature* features, int n, int mtype,
					ransac_xform_fn xform_fn, int m, double p_badxform,
					ransac_err_fn err_fn, double err_tol,
struct feature*** inliers, int* n_in )
{
	struct feature** matched, ** sample, ** consensus, ** consensus_max = NULL;
	struct ransac_data* rdata;
	CvPoint2D64f* pts, * mpts;
	CvMat* M = NULL;
	gsl_rng* rng;
	double p, in_frac = RANSAC_INLIER_FRAC_EST;
	int i, nm, in, in_min, in_max = 0, k = 0;

	nm = get_matched_features( features, n, mtype, &matched );
	if( nm < m )
	{
		fprintf( stderr, "Warning: not enough matches to compute xform, %s" \
			" line %d\n", __FILE__, __LINE__ );
		goto end;
	}

	// initialize random number generator 
	rng = gsl_rng_alloc( gsl_rng_mt19937 );
	gsl_rng_set( rng, time(NULL) );

	in_min = calc_min_inliers( nm, m, RANSAC_PROB_BAD_SUPP, p_badxform );
	p = pow( 1.0 - pow( in_frac, m ), k );
	i = 0;
	while( p > p_badxform )
	{
		sample = draw_ransac_sample( matched, nm, m, rng );
		extract_corresp_pts( sample, m, mtype, &pts, &mpts );
		M = xform_fn( pts, mpts, m );
		if( ! M )
			goto iteration_end;
		in = find_consensus( matched, nm, mtype, M, err_fn, err_tol, &consensus);
		if( in > in_max )
		{
			if( consensus_max )
				free( consensus_max );
			consensus_max = consensus;
			in_max = in;
			in_frac = (double)in_max / nm;
		}
		else
			free( consensus );
		cvReleaseMat( &M );

iteration_end:
		release_mem_inliers( pts, mpts, sample );
		p = pow( 1.0 - pow( in_frac, m ), ++k );
	}

	///calculate final transform based on best consensus set 
	if( in_max >= in_min )
	{
		extract_corresp_pts( consensus_max, in_max, mtype, &pts, &mpts );
		M = xform_fn( pts, mpts, in_max );
		in = find_consensus_inliers( matched, nm, mtype, M, err_fn, err_tol, &consensus);
		cvReleaseMat( &M );
		release_mem_inliers( pts, mpts, consensus_max );
		extract_corresp_pts( consensus, in, mtype, &pts, &mpts );
		M = xform_fn( pts, mpts, in );
		if( inliers )
		{
			*inliers = consensus;
			consensus = NULL;
		}
		if( n_in )
			*n_in = in;
		release_mem_inliers( pts, mpts, consensus );
	}
	else if( consensus_max )
	{
		if( inliers )
			*inliers = NULL;
		if( n_in )
			*n_in = 0;
		free( consensus_max );
	}

	gsl_rng_free( rng );
end:
	for( i = 0; i < nm; i++ )
	{
		rdata = feat_ransac_data( matched[i] );
		matched[i]->feature_data = rdata->orig_feat_data;
		free( rdata );
	}
	free( matched );
	return M;
}

int find_consensus_inliers( struct feature** features, int n, int mtype,
				   CvMat* M, ransac_err_fn err_fn, double err_tol,
				   struct feature*** consensus )
{
	struct feature** _consensus;
	struct feature* match;
	CvPoint2D64f pt, mpt;
	double err;
	int i, in = 0;

	_consensus = calloc( n, sizeof( struct feature* ) );

	if( mtype == FEATURE_MDL_MATCH )
		for( i = 0; i < n; i++ )
		{
			match = get_match_inliers( features[i], mtype );
			if( ! match )
				fatal_error( "feature does not have match of type %d, %s line %d",
							mtype, __FILE__, __LINE__ );
			pt = features[i]->img_pt;
			mpt = match->mdl_pt;
			err = err_fn( pt, mpt, M );
			if( err <= err_tol )
				_consensus[in++] = features[i];
		}

	else
		for( i = 0; i < n; i++ )
		{
			match = get_match_inliers( features[i], mtype );
			if( ! match )
				fatal_error( "feature does not have match of type %d, %s line %d",
							mtype, __FILE__, __LINE__ );
			pt = features[i]->img_pt;
			mpt = match->img_pt;
			
			// printf(" pt.x = %f  pt.y = %f \t mpt.x = %f mpt.y = %f \n", pt.x, pt.y, mpt.x, mpt.y);
			// getchar();
			err = err_fn( pt, mpt, M );
			if( err <= err_tol ){
				_consensus[in++] = features[i];
				//pt =  inliers[i]->img_pt;
		 //mpt = inliers[i]->mdl_pt;
				//printf("pt_cons[%d].x = %f  pt_cons[%d].y = %f", in, _consensus[in]->img_pt.x, in, _consensus[in]->img_pt.x);
				//printf("pt_cons.x = %f  pt_cons.y = %f", _consensus[in++]->img_pt.x, _consensus[in++]->img_pt.x);
				//getchar();
				

			}
		}
	*consensus = _consensus;
	return in;
}

static __inline void release_mem_inliers( CvPoint2D64f* pts1, CvPoint2D64f* pts2,struct feature** features )
{
	free( pts1 );
	free( pts2 );
	if( features )
		free( features );
}*/

extern float SRCG_find_r(IplImage* img1, IplImage* img2, IplImage* img3, IplImage* img4, IplImage* img5, IplImage* prev_panorama, 
					struct image_border ROI_frame1, struct image_border ROI_frame2, struct image_border ROI_frame3, struct image_border ROI_frame4, struct image_border ROI_frame5,
					IplImage* SRCG_r)
					{

//IplImage* panoramaSR = NULL;
IplImage* img1_inv = NULL, *img2_inv = NULL, *img3_inv=NULL, *img4_inv=NULL, *img5_inv=NULL;
IplImage* img1_inv_temp1 = NULL, *img2_inv_temp1 = NULL, *img3_inv_temp1=NULL, *img4_inv_temp1=NULL, *img5_inv_temp1=NULL;
IplImage* img1_inv_temp2 = NULL, *img2_inv_temp2 = NULL, *img3_inv_temp2=NULL, *img4_inv_temp2=NULL, *img5_inv_temp2=NULL;

IplImage* dimg1=NULL, *dimg2=NULL, *dimg3=NULL, *dimg4=NULL, *dimg5=NULL;
IplImage* Ddimg1=NULL, *Ddimg2=NULL, *Ddimg3=NULL, *Ddimg4=NULL, *Ddimg5=NULL;
IplImage* ddst1 = NULL, *ddst2 = NULL, *ddst3 = NULL, *ddst4 = NULL, *ddst5=NULL;
IplImage* panorama5d = NULL;
IplImage* panorama5d2 = NULL; //after multiplication with alpha
IplImage* panorama5LLt = NULL;
IplImage* panorama5LLt2 = NULL; //after multiplication
IplImage* panorama5_Delta = NULL;
IplImage* Xregularized = NULL; // To use with the algorithm Fast and Robust Regularization
IplImage* Limg = NULL;
IplImage* SRCG_rf = NULL;

CvMat* kernelLLt = cvCreateMat(3,3,CV_32FC1);
CvMat* kernelB = cvCreateMat(3,3,CV_32FC1);
CvMat* kernelL = cvCreateMat(3,3,CV_32FC1);

CvMat* lambdaM = NULL;

double lambda_1, lambda_2, lambda_3, lambda_4, lambda_5;
double lambda_num, lambda_den;
double lambdad_1, lambdad_2, lambdad_3, lambdad_4, lambdad_5;
double lambda_it;
double alpha_it = 1.1127;// was 7, 1, 1.1127
double errorSR;

float value_img, alpha_num;

float alpha = 0.0;

CvPoint2D32f P; // P for the mosaic

int i, j, D;

struct RegionS  S1, S2, S3, S4, S5;

CvSize sizeD1, sizeD2, sizeD3, sizeD4, sizeD5;
CvSize size; //size of the frames

//FILE *pFile;




	/// First part:  Interpolation : resizing the image to 2 times the original size

  D = 2;

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

	size.height = img1->height;
	size.width = img1->width;

/**************************************************************************************/ 
   //   Starting a new iteration (it = 3)
   //   
	//  First Step: Getting the imgs from the mosaic image
   //
   /**************************************************************************************/
/*
	allocateOnDemand( &img1_inv, cvSize(img2->width,img2->height),  img1->depth, img1->nChannels );
    WarpImageUnderQuadInv(prev_panorama,img1_inv,ROI_frame1);

    allocateOnDemand( &img2_inv, cvSize(img2->width,img2->height),  img1->depth, img1->nChannels );
    WarpImageUnderQuadInv(prev_panorama,img2_inv,ROI_frame2);

	allocateOnDemand( &img3_inv, cvSize(img2->width,img2->height),  img1->depth, img1->nChannels );
	WarpImageUnderQuadInv(prev_panorama,img3_inv,ROI_frame3);

	allocateOnDemand( &img4_inv, cvSize(img2->width,img2->height),  img1->depth, img1->nChannels );
	WarpImageUnderQuadInv(prev_panorama,img4_inv,ROI_frame4);

	allocateOnDemand( &img5_inv, cvSize(img2->width,img2->height),  img1->depth, img1->nChannels );
	WarpImageUnderQuadInv(prev_panorama,img5_inv,ROI_frame5);
*/

	allocateOnDemand( &img1_inv_temp1, sizeD1,  img1->depth, img1->nChannels );
    WarpImageUnderQuadInv(prev_panorama,img1_inv_temp1,ROI_frame1);

    allocateOnDemand( &img2_inv_temp1, sizeD1,  img1->depth, img1->nChannels );
    WarpImageUnderQuadInv(prev_panorama,img2_inv_temp1,ROI_frame2);

	allocateOnDemand( &img3_inv_temp1, sizeD1,  img1->depth, img1->nChannels );
	WarpImageUnderQuadInv(prev_panorama,img3_inv_temp1,ROI_frame3);

	allocateOnDemand( &img4_inv_temp1, sizeD1,  img1->depth, img1->nChannels );
	WarpImageUnderQuadInv(prev_panorama,img4_inv_temp1,ROI_frame4);

	allocateOnDemand( &img5_inv_temp1, sizeD1,  img1->depth, img1->nChannels );
	WarpImageUnderQuadInv(prev_panorama,img5_inv_temp1,ROI_frame5);

	// Using a Gaussian Filter to perform an Antia-alias filtering
	
	allocateOnDemand( &img1_inv_temp2, sizeD1,  img1->depth, img1->nChannels );
    allocateOnDemand( &img2_inv_temp2, sizeD1,  img1->depth, img1->nChannels );
    allocateOnDemand( &img3_inv_temp2, sizeD1,  img1->depth, img1->nChannels );
	allocateOnDemand( &img4_inv_temp2, sizeD1,  img1->depth, img1->nChannels );
	allocateOnDemand( &img5_inv_temp2, sizeD1,  img1->depth, img1->nChannels );
/*
	cvSmooth(img1_inv_temp1,img1_inv_temp2, CV_GAUSSIAN, 3, 0, 0, 0);
	cvSmooth(img2_inv_temp1,img2_inv_temp2, CV_GAUSSIAN, 3, 0, 0, 0);
	cvSmooth(img3_inv_temp1,img3_inv_temp2, CV_GAUSSIAN, 3, 0, 0, 0);
	cvSmooth(img4_inv_temp1,img4_inv_temp2, CV_GAUSSIAN, 3, 0, 0, 0);
	cvSmooth(img5_inv_temp1,img5_inv_temp2, CV_GAUSSIAN, 3, 0, 0, 0);
*/
	// Using Filter 2D:
    //This kernel is the same as the used in the paper: " A Transform-Domain Approach to Super-resolution Mosaicing of Compressed Images"

	cvmSet(kernelB,0,0,0.1016);
	cvmSet(kernelB,0,1,0.1172);
	cvmSet(kernelB,0,2,0.1016);
	cvmSet(kernelB,1,0,0.1172);
	cvmSet(kernelB,1,1,0.1250);
	cvmSet(kernelB,1,2,0.1172);
	cvmSet(kernelB,2,0,0.1016);
	cvmSet(kernelB,2,1,0.1172);
	cvmSet(kernelB,2,2,0.1016);

	cvFilter2D(img1_inv_temp1,img1_inv_temp2,kernelB,cvPoint(-1,1));
	cvFilter2D(img2_inv_temp1,img2_inv_temp2,kernelB,cvPoint(-1,1));
	cvFilter2D(img3_inv_temp1,img3_inv_temp2,kernelB,cvPoint(-1,1));
	cvFilter2D(img4_inv_temp1,img4_inv_temp2,kernelB,cvPoint(-1,1));
	cvFilter2D(img5_inv_temp1,img5_inv_temp2,kernelB,cvPoint(-1,1));
	
  // Downsampling using cubic interpolation

	allocateOnDemand( &img1_inv, size,  img1->depth, img1->nChannels );
    allocateOnDemand( &img2_inv, size,  img1->depth, img1->nChannels );
    allocateOnDemand( &img3_inv, size,  img1->depth, img1->nChannels );
	allocateOnDemand( &img4_inv, size,  img1->depth, img1->nChannels );
	allocateOnDemand( &img5_inv, size,  img1->depth, img1->nChannels );
	
	cvResize(img1_inv_temp2, img1_inv, CV_INTER_CUBIC);
	cvResize(img2_inv_temp2, img2_inv, CV_INTER_CUBIC);
	cvResize(img3_inv_temp2, img3_inv, CV_INTER_CUBIC);
	cvResize(img4_inv_temp2, img4_inv, CV_INTER_CUBIC);
	cvResize(img5_inv_temp2, img5_inv, CV_INTER_CUBIC);

	/*
// Downsampling using my own code

	DownSampleImage(img1_inv_temp2, img1_inv, 2);
	DownSampleImage(img2_inv_temp2, img2_inv, 2);
	DownSampleImage(img3_inv_temp2, img3_inv, 2);
	DownSampleImage(img4_inv_temp2, img4_inv, 2);
	DownSampleImage(img5_inv_temp2, img5_inv, 2);
*/
/*
	cvSaveImage("DBWRx1.jpg", img1_inv);
	cvSaveImage("DBWRx2.jpg", img2_inv);
	cvSaveImage("DBWRx3.jpg", img3_inv);
	cvSaveImage("DBWRx4.jpg", img4_inv);
	cvSaveImage("DBWRx5.jpg", img5_inv);


	cvNamedWindow( "img1_inv", 1 );
	cvShowImage( "img1_inv", img1_inv);
	cvWaitKey(0);

	cvNamedWindow( "img2_inv", 1 );
	cvShowImage( "img2_inv", img2_inv);
	cvWaitKey(0);

	cvNamedWindow( "img3_inv", 1 );
	cvShowImage( "img3_inv", img3_inv );
	cvWaitKey(0);

	cvNamedWindow( "img4_inv", 1 );
	cvShowImage( "img4_inv", img4_inv );
	cvWaitKey(0);

	cvNamedWindow( "img5_inv", 1 );
	cvShowImage( "img5_inv", img5_inv );
	cvWaitKey(0);
*/	

   /**************************************************************************************/ 
   //    
   //   
   //  Second Step: Compute the frame difference between the original low frames and the LR reconstructed ones
   //
   /**************************************************************************************/

  /************************************************************************/
  //  Doing the frame substracion
  /****************************************************************************/
 //  printf("\n Doing the frame substracion ...\n" );


  allocateOnDemand( &dimg1, cvSize(img2->width,img2->height),  img1->depth, img1->nChannels );
  allocateOnDemand( &dimg2, cvSize(img2->width,img2->height),  img1->depth, img1->nChannels );
  allocateOnDemand( &dimg3, cvSize(img2->width,img2->height),  img1->depth, img1->nChannels );
  allocateOnDemand( &dimg4, cvSize(img2->width,img2->height),  img1->depth, img1->nChannels );
  allocateOnDemand( &dimg5, cvSize(img2->width,img2->height),  img1->depth, img1->nChannels );
/*
   printf("\n size_img1 = (%d , %d) \t size_img1_inv = (%d, %d)", img1->height, img1->width, img1_inv->height, img1_inv->width);
   printf("\n size_dimg1 = (%d , %d) ", dimg1->height, dimg1->width);

   getchar();
*/

  cvSub(img1,img1_inv,dimg1,NULL);
  cvSub(img2,img2_inv,dimg2,NULL);
  cvSub(img3,img3_inv,dimg3,NULL);
  cvSub(img4,img4_inv,dimg4,NULL);
  cvSub(img5,img5_inv,dimg5,NULL);

 
  //*********************************************************************************************//
  //  Doing the interpolation of the images differences											//
  //*********************************************************************************************//
    //printf("\n Doing the frame interpolation of the difference of frames ...\n" );
    allocateOnDemand( &Ddimg1, sizeD1,  img1->depth, img1->nChannels );  
	allocateOnDemand( &Ddimg2, sizeD2,  img1->depth, img1->nChannels );  
	allocateOnDemand( &Ddimg3, sizeD3,  img1->depth, img1->nChannels );  
	allocateOnDemand( &Ddimg4, sizeD4,  img1->depth, img1->nChannels );  
	allocateOnDemand( &Ddimg5, sizeD5,  img1->depth, img1->nChannels );  

	/*
  	cvResize(dimg1, Ddimg1,CV_INTER_CUBIC);
	cvResize(dimg2, Ddimg2,CV_INTER_CUBIC);
	cvResize(dimg3, Ddimg3,CV_INTER_CUBIC);
	cvResize(dimg4, Ddimg4,CV_INTER_CUBIC);
	cvResize(dimg5, Ddimg5,CV_INTER_CUBIC);
	*/

	UpSampleImageD(dimg1, Ddimg1);
	UpSampleImageD(dimg2, Ddimg2);
	UpSampleImageD(dimg3, Ddimg3);
	UpSampleImageD(dimg4, Ddimg4);
	UpSampleImageD(dimg5, Ddimg5);

/***********************************************************************************/
//			 Doing the Mosaicking of the Error Images								//
//																					//
/***********************************************************************************/
	//printf("\n Doing the Mosaicking of the Error Images ...\n" );

	allocateOnDemand( &ddst1,  cvSize(prev_panorama->width,prev_panorama->height),  img1->depth, img1->nChannels );  
	allocateOnDemand( &ddst2,  cvSize(prev_panorama->width,prev_panorama->height),  img1->depth, img1->nChannels );  
	allocateOnDemand( &ddst3,  cvSize(prev_panorama->width,prev_panorama->height),  img1->depth, img1->nChannels );  
	allocateOnDemand( &ddst4,  cvSize(prev_panorama->width,prev_panorama->height),  img1->depth, img1->nChannels );  
	allocateOnDemand( &ddst5,  cvSize(prev_panorama->width,prev_panorama->height),  img1->depth, img1->nChannels );  
	
	/*
	allocateOnDemand( &ddst1,  cvSize(1360, 640),  img1->depth, img1->nChannels );  
	allocateOnDemand( &ddst2,  cvSize(1360, 640),  img1->depth, img1->nChannels );  
	allocateOnDemand( &ddst3,  cvSize(1360, 640),  img1->depth, img1->nChannels );  
	allocateOnDemand( &ddst4,  cvSize(1360, 640),  img1->depth, img1->nChannels );  
	allocateOnDemand( &ddst5,  cvSize(1360, 640),  img1->depth, img1->nChannels );  
*/

    WarpImageUnderQuad(Ddimg1,ddst1,ROI_frame1);
	//cvNamedWindow( "Perspective_Warpdd1", 1 );
	//cvShowImage( "Perspective_Warpdd1", ddst1);
	//cvWaitKey(10);


  WarpImageUnderQuad(Ddimg2,ddst2,ROI_frame2);

 // cvNamedWindow( "Perspective_Warpdd2", 1 );
 // cvShowImage( "Perspective_Warpdd2", ddst2 );
  //cvWaitKey(10);


 WarpImageUnderQuad(Ddimg3,ddst3,ROI_frame3);
 //cvNamedWindow( "Perspective_Warpdd3", 1 );
 //cvShowImage( "Perspective_Warpdd3", ddst3 );
 //cvWaitKey(10);


 WarpImageUnderQuad(Ddimg4,ddst4,ROI_frame4);
 //cvNamedWindow( "Perspective_Warpdd4", 1 );
 //cvShowImage( "Perspective_Warpdd4", ddst4 );
 //cvWaitKey(10);

 
 WarpImageUnderQuad(Ddimg5,ddst5,ROI_frame5);
 //cvNamedWindow( "Perspective_Warpdd5", 1 );
 ///cvShowImage( "Perspective_Warpdd5", ddst5 );
 //cvWaitKey(10);

 allocateOnDemand( &panorama5d,  cvSize(prev_panorama->width, prev_panorama->height),  img1->depth, img1->nChannels );  

 S2 = FindSRegion(ROI_frame2);

	  for(i = 0; i < panorama5d->height; i ++){ //rows
		  for(j = 0; j < panorama5d->width; j++){ //cols
			  P.x = j;
			  P.y = i;
			  if(Belongs2S(P,S2))cvSet2D(panorama5d,i,j,cvGet2D(ddst2,i,j));
			  else cvSet2D(panorama5d,i,j,cvGet2D(ddst1,i,j));			  
		  }//end for
	  }// end for

	

////////////////////////////
// Finding the region S3 //
////////////////////////////

   S3 = FindSRegion(ROI_frame3);

	  for(i = 0; i < panorama5d->height; i ++){ //rows
		  for(j = 0; j < panorama5d->width; j++){ //cols
			  P.x = j;
			  P.y = i;
			  if(Belongs2S(P,S3))cvSet2D(panorama5d,i,j,cvGet2D(ddst3,i,j));
			  else cvSet2D(panorama5d,i,j,cvGet2D(panorama5d,i,j));			  
		  }//end for
	  }// end for



 ////////////////////////////
// Finding the region S4 //
////////////////////////////

   S4 = FindSRegion(ROI_frame4);

	  for(i = 0; i < panorama5d->height; i ++){ //rows
		  for(j = 0; j < panorama5d->width; j++){ //cols
			  P.x = j;
			  P.y = i;
			  if(Belongs2S(P,S4))cvSet2D(panorama5d,i,j,cvGet2D(ddst4,i,j));
			  else cvSet2D(panorama5d,i,j,cvGet2D(panorama5d,i,j));			  
		  }//end for
	  }// end for


	    ////////////////////////////
		// Finding the region S4 //
		////////////////////////////

   S5 = FindSRegion(ROI_frame5);


	  for(i = 0; i < panorama5d->height; i ++){ //rows
		  for(j = 0; j < panorama5d->width; j++){ //cols
			  P.x = j;
			  P.y = i;
			  if(Belongs2S(P,S5))cvSet2D(panorama5d,i,j,cvGet2D(ddst5,i,j));
			  else cvSet2D(panorama5d,i,j,cvGet2D(panorama5d,i,j));			  
		  }//end for
	  }// end for

	// cvNamedWindow( "panorama5d", 1 );
	//cvShowImage( "panorama5d", panorama5d );
	 //cvWaitKey(0);

    /****************************************
     Now updating r
	*****************************************/
   //cvAdd(SRCG_r,panorama5d,SRCG_r, 0);

   

   /**************************************************************************************/
   //
   // Computing the normalizing parameter lambda^(n) 
   //
   /**************************************************************************************/
    
	  lambda_1 = cvNorm(img1, img1_inv, CV_L2, NULL);
	 // printf("\n lambda_1 = %f", lambda_1);
	  lambda_2 = cvNorm(img2, img2_inv, CV_L2, NULL);
	 // printf("\n lambda_2 = %f", lambda_2);
	  lambda_3 = cvNorm(img3, img3_inv, CV_L2, NULL);
	 // printf("\n lambda_3 = %f", lambda_3);
	  lambda_4 = cvNorm(img4, img4_inv, CV_L2, NULL);
	 // printf("\n lambda_4 = %f", lambda_4);
	  lambda_5 = cvNorm(img5, img5_inv, CV_L2, NULL);
	 // printf("\n lambda_5 = %f", lambda_5);

   /**************************************************************************************/
   //
   // Computing the Laplace of the x^(n) image x(n) comes to be the panorama5 variables (mosaic of the previous state)
   //
   /**************************************************************************************/
   allocateOnDemand( &Limg, cvSize(prev_panorama->width,prev_panorama->height),  img1->depth, img1->nChannels );  
   //cvLaplace(prev_panorama, Limg,3);   
/*
// Uncomment this to use Laplace Prior Function

   // Using in stead cvFilter2D

   	cvmSet(kernelL,0,0,0.0000);
	cvmSet(kernelL,0,1,0.2500);
	cvmSet(kernelL,0,2,0.0000);
	cvmSet(kernelL,1,0,0.2500);
	cvmSet(kernelL,1,1,-1.000);
	cvmSet(kernelL,1,2,0.2500);
	cvmSet(kernelL,2,0,0.0000);
	cvmSet(kernelL,2,1,0.2500);
	cvmSet(kernelL,2,2,0.0000);

   cvFilter2D(prev_panorama, Limg, kernelL, cvPoint(-1,-1));
*/
 // Uncomment this to use Huber Prior Function
  // Using Huber Prior Function to find lambda

     alpha = 1.75;//was 1.75

     HubbertFunctionCliques(prev_panorama, Limg, alpha);

  // cvNamedWindow( "Limg", 1 );
  // cvShowImage( "Limg", Limg );
  // cvWaitKey(0);


   /**************************************************************************************/
   //
   // Computing the K norm(x^(n),2) , K = 1, 2 ...5
   //
   /**************************************************************************************/

  lambda_num = pow((lambda_1 + lambda_2 + lambda_3 + lambda_4 + lambda_5),2);
  lambdad_1 = cvNorm(Limg, NULL, CV_L2, NULL);
 

    /**************************************************************************************/
   //
   // Computing the lambda(n), K = 5, because I am just working with 5 frames
   //
   /**************************************************************************************/
 
  lambda_den = pow((5*lambdad_1),2);

    
  lambda_it = lambda_num/lambda_den;

  
  // printf("\n lambda_num = %f", lambda_num);
  // printf("\n lambda_den = %f", lambda_den);
  // printf("\n lambda_it = %f", lambda_it);

  // *lambda_itp =  lambda_it;

  // printf("\n lambda_it = %f \n", *lambda_itp);

  // pFile = fopen(name, "w");

   //fprintf(pFile, "%f\n", lambda_it);

  // fclose(pFile);
   
  // lambda_it = 1.1127;

  // printf("\n Using alpha from GCV = %f", alpha_it);

    /**************************************************************************************/
   //
   // Computing the lambda(n)*L'*L*x(n)
   //
   /**************************************************************************************/

   /*
    cvmSet(kernelLLt,0,0,1.000*lambda_it);
	cvmSet(kernelLLt,0,1,-4.000*lambda_it);
	cvmSet(kernelLLt,0,2,1.000*lambda_it);
	cvmSet(kernelLLt,1,0,-4.000*lambda_it);
	cvmSet(kernelLLt,1,1,18.000*lambda_it);
	cvmSet(kernelLLt,1,2,-4.000*lambda_it);
	cvmSet(kernelLLt,2,0,1.000*lambda_it);
	cvmSet(kernelLLt,2,1,-4.000*lambda_it);
	cvmSet(kernelLLt,2,2,1.000*lambda_it);
*/
	cvmSet(kernelLLt,0,0,1.000);
	cvmSet(kernelLLt,0,1,-4.000);
	cvmSet(kernelLLt,0,2,1.000);
	cvmSet(kernelLLt,1,0,-4.000);
	cvmSet(kernelLLt,1,1,18.000);
	cvmSet(kernelLLt,1,2,-4.000);
	cvmSet(kernelLLt,2,0,1.000);
	cvmSet(kernelLLt,2,1,-4.000);
	cvmSet(kernelLLt,2,2,1.000);

	allocateOnDemand( &panorama5LLt, cvSize(prev_panorama->width,prev_panorama->height),  img1->depth, img1->nChannels );  
	allocateOnDemand( &panorama5LLt2, cvSize(prev_panorama->width,prev_panorama->height),  img1->depth, img1->nChannels );  
	allocateOnDemand( &panorama5d2, cvSize(prev_panorama->width,prev_panorama->height),  img1->depth, img1->nChannels ); 

 //	printf("\n Limg->height = %d \t Limg->width = %d", Limg->height, Limg->width);
 //	printf("\n prev_panorama->height = %d \t prev_panorama->width = %d", prev_panorama->height, prev_panorama->width);

  //  Uncomment to use Huber Regularization

    // Using Huber Regularization here as well
	HubbertRegularization(Limg, panorama5LLt, alpha);
/*
  // Uncomment to use Laplace Regularization
    // Using Laplace prior
	cvFilter2D(Limg,panorama5LLt,kernelL,cvPoint(-1,1));
	cvFilter2D(panorama5LLt,panorama5LLt2, kernelL,cvPoint(-1,1)); // Because os L^t*L*x
*/	

/*
    //Using the regularization proposed in the paper: "Fast and Robust Super-Resolution"

	allocateOnDemand( &Xregularized, cvSize(prev_panorama->width,prev_panorama->height), IPL_DEPTH_32F, 1 );

	FastRobustRegularization(Limg, Xregularized, 2, alpha); // p = 2

	panorama5LLt = convert_gray32_to_color(Xregularized);

*/

	/*
	//
	// Converting to Matrix lambda doesn't work, because of the size of the mosaicking
	//

	lambdaM = cvCreateMat(panorama5LLt->height,panorama5LLt->width,CV_32FC1);
	cvSetIdentity(lambdaM,cvRealScalar(lambda_it));

	cvMatMul(panorama5LLt,lambdaM, panorama5LLt2);
	*/

	//panorama5LLt2 = MulImagewithDouble(panorama5LLt, lambda_it);// Use this for Huber Regularization
	
	//panorama5LLt2 = MulImagewithDouble(panorama5LLt2, lambda_it); // Use this for Laplace Regularization
	
	//panorama5d2 = MulImagewithDouble(panorama5d, alpha_it);

   /**************************************************************************************/ 
   //
   //	Now updating the iteration of x(n+1). First I have to do: panorama5d - panorama5LLt
   //   Then I have to do : panorma5_2 = panorama5_1 + (panorama5d - panorama5LLt)
   //
   /**************************************************************************************/
	
	//allocateOnDemand( &panoramaSR, cvSize(prev_panorama->width,prev_panorama->height),  img1->depth, img1->nChannels );  
	allocateOnDemand( &panorama5_Delta, cvSize(prev_panorama->width,prev_panorama->height),  img1->depth, img1->nChannels );  

	cvSub(panorama5d,panorama5LLt,panorama5_Delta,NULL);
	
	//cvCloneImage(panorama5_Delta);

	  /****************************************
     Now updating r
	*****************************************/
    cvAdd(SRCG_r,panorama5_Delta,SRCG_r, 0);

	// SRCG is the Jacobian, so to find alpha_num, we have to:

	allocateOnDemand( &SRCG_rf, cvSize(prev_panorama->width,prev_panorama->height),  IPL_DEPTH_32F, 1 );  

	SRCG_rf = convert2gray32(SRCG_r);

	alpha_num = 0.0;
	for(i = 0; i < SRCG_rf->height; i++){
		for( j = 0; j < SRCG_rf->width; j++){
			value_img = ((float *)(SRCG_rf->imageData + i*SRCG_rf->widthStep))[j];
			alpha_num += value_img*value_img;
		}
	}







   

	cvReleaseMat( &kernelLLt );
	cvReleaseMat( &kernelL );
	cvReleaseMat( &kernelB );
	cvReleaseMat( &lambdaM );

	return alpha_num;

}

extern void SRCG_find_w(IplImage* img1, IplImage* img2, IplImage* img3, IplImage* img4, IplImage* img5, IplImage* prev_panorama, 
					struct image_border ROI_frame1, struct image_border ROI_frame2, struct image_border ROI_frame3, struct image_border ROI_frame4, struct image_border ROI_frame5,
					IplImage* SRCG_w)
					{

//IplImage* panoramaSR = NULL;
IplImage* img1_inv = NULL, *img2_inv = NULL, *img3_inv=NULL, *img4_inv=NULL, *img5_inv=NULL;
IplImage* img1_inv_temp1 = NULL, *img2_inv_temp1 = NULL, *img3_inv_temp1=NULL, *img4_inv_temp1=NULL, *img5_inv_temp1=NULL;
IplImage* img1_inv_temp2 = NULL, *img2_inv_temp2 = NULL, *img3_inv_temp2=NULL, *img4_inv_temp2=NULL, *img5_inv_temp2=NULL;

IplImage* dimg1=NULL, *dimg2=NULL, *dimg3=NULL, *dimg4=NULL, *dimg5=NULL;
IplImage* Ddimg1=NULL, *Ddimg2=NULL, *Ddimg3=NULL, *Ddimg4=NULL, *Ddimg5=NULL;
IplImage* ddst1 = NULL, *ddst2 = NULL, *ddst3 = NULL, *ddst4 = NULL, *ddst5=NULL;
IplImage* panorama5d = NULL;
IplImage* panorama5d2 = NULL; //after multiplication with alpha
IplImage* panorama5LLt = NULL;
IplImage* panorama5LLt2 = NULL; //after multiplication
IplImage* panorama5_Delta = NULL;
IplImage* Xregularized = NULL; // To use with the algorithm Fast and Robust Regularization
IplImage* Limg = NULL;

CvMat* kernelLLt = cvCreateMat(3,3,CV_32FC1);
CvMat* kernelB = cvCreateMat(3,3,CV_32FC1);
CvMat* kernelL = cvCreateMat(3,3,CV_32FC1);

CvMat* lambdaM = NULL;

double lambda_1, lambda_2, lambda_3, lambda_4, lambda_5;
double lambda_num, lambda_den;
double lambdad_1, lambdad_2, lambdad_3, lambdad_4, lambdad_5;
double lambda_it;
double alpha_it = 1.1127;// was 7, 1, 1.1127
double errorSR;

float alpha = 0.0;

CvPoint2D32f P; // P for the mosaic

int i, j, D;

struct RegionS  S1, S2, S3, S4, S5;

CvSize sizeD1, sizeD2, sizeD3, sizeD4, sizeD5;
CvSize size; //size of the frames

//FILE *pFile;




	/// First part:  Interpolation : resizing the image to 2 times the original size

  D = 2;

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

	size.height = img1->height;
	size.width = img1->width;

/**************************************************************************************/ 
   //   Starting a new iteration (it = 3)
   //   
	//  First Step: Getting the imgs from the mosaic image
   //
   /**************************************************************************************/
/*
	allocateOnDemand( &img1_inv, cvSize(img2->width,img2->height),  img1->depth, img1->nChannels );
    WarpImageUnderQuadInv(prev_panorama,img1_inv,ROI_frame1);

    allocateOnDemand( &img2_inv, cvSize(img2->width,img2->height),  img1->depth, img1->nChannels );
    WarpImageUnderQuadInv(prev_panorama,img2_inv,ROI_frame2);

	allocateOnDemand( &img3_inv, cvSize(img2->width,img2->height),  img1->depth, img1->nChannels );
	WarpImageUnderQuadInv(prev_panorama,img3_inv,ROI_frame3);

	allocateOnDemand( &img4_inv, cvSize(img2->width,img2->height),  img1->depth, img1->nChannels );
	WarpImageUnderQuadInv(prev_panorama,img4_inv,ROI_frame4);

	allocateOnDemand( &img5_inv, cvSize(img2->width,img2->height),  img1->depth, img1->nChannels );
	WarpImageUnderQuadInv(prev_panorama,img5_inv,ROI_frame5);
*/

	allocateOnDemand( &img1_inv_temp1, sizeD1,  img1->depth, img1->nChannels );
    WarpImageUnderQuadInv(prev_panorama,img1_inv_temp1,ROI_frame1);

    allocateOnDemand( &img2_inv_temp1, sizeD1,  img1->depth, img1->nChannels );
    WarpImageUnderQuadInv(prev_panorama,img2_inv_temp1,ROI_frame2);

	allocateOnDemand( &img3_inv_temp1, sizeD1,  img1->depth, img1->nChannels );
	WarpImageUnderQuadInv(prev_panorama,img3_inv_temp1,ROI_frame3);

	allocateOnDemand( &img4_inv_temp1, sizeD1,  img1->depth, img1->nChannels );
	WarpImageUnderQuadInv(prev_panorama,img4_inv_temp1,ROI_frame4);

	allocateOnDemand( &img5_inv_temp1, sizeD1,  img1->depth, img1->nChannels );
	WarpImageUnderQuadInv(prev_panorama,img5_inv_temp1,ROI_frame5);

	// Using a Gaussian Filter to perform an Antia-alias filtering
	
	allocateOnDemand( &img1_inv_temp2, sizeD1,  img1->depth, img1->nChannels );
    allocateOnDemand( &img2_inv_temp2, sizeD1,  img1->depth, img1->nChannels );
    allocateOnDemand( &img3_inv_temp2, sizeD1,  img1->depth, img1->nChannels );
	allocateOnDemand( &img4_inv_temp2, sizeD1,  img1->depth, img1->nChannels );
	allocateOnDemand( &img5_inv_temp2, sizeD1,  img1->depth, img1->nChannels );
/*
	cvSmooth(img1_inv_temp1,img1_inv_temp2, CV_GAUSSIAN, 3, 0, 0, 0);
	cvSmooth(img2_inv_temp1,img2_inv_temp2, CV_GAUSSIAN, 3, 0, 0, 0);
	cvSmooth(img3_inv_temp1,img3_inv_temp2, CV_GAUSSIAN, 3, 0, 0, 0);
	cvSmooth(img4_inv_temp1,img4_inv_temp2, CV_GAUSSIAN, 3, 0, 0, 0);
	cvSmooth(img5_inv_temp1,img5_inv_temp2, CV_GAUSSIAN, 3, 0, 0, 0);
*/
	// Using Filter 2D:
    //This kernel is the same as the used in the paper: " A Transform-Domain Approach to Super-resolution Mosaicing of Compressed Images"

	cvmSet(kernelB,0,0,0.1016);
	cvmSet(kernelB,0,1,0.1172);
	cvmSet(kernelB,0,2,0.1016);
	cvmSet(kernelB,1,0,0.1172);
	cvmSet(kernelB,1,1,0.1250);
	cvmSet(kernelB,1,2,0.1172);
	cvmSet(kernelB,2,0,0.1016);
	cvmSet(kernelB,2,1,0.1172);
	cvmSet(kernelB,2,2,0.1016);

	cvFilter2D(img1_inv_temp1,img1_inv_temp2,kernelB,cvPoint(-1,1));
	cvFilter2D(img2_inv_temp1,img2_inv_temp2,kernelB,cvPoint(-1,1));
	cvFilter2D(img3_inv_temp1,img3_inv_temp2,kernelB,cvPoint(-1,1));
	cvFilter2D(img4_inv_temp1,img4_inv_temp2,kernelB,cvPoint(-1,1));
	cvFilter2D(img5_inv_temp1,img5_inv_temp2,kernelB,cvPoint(-1,1));
	
  // Downsampling using cubic interpolation

	allocateOnDemand( &img1_inv, size,  img1->depth, img1->nChannels );
    allocateOnDemand( &img2_inv, size,  img1->depth, img1->nChannels );
    allocateOnDemand( &img3_inv, size,  img1->depth, img1->nChannels );
	allocateOnDemand( &img4_inv, size,  img1->depth, img1->nChannels );
	allocateOnDemand( &img5_inv, size,  img1->depth, img1->nChannels );
	
	cvResize(img1_inv_temp2, img1_inv, CV_INTER_CUBIC);
	cvResize(img2_inv_temp2, img2_inv, CV_INTER_CUBIC);
	cvResize(img3_inv_temp2, img3_inv, CV_INTER_CUBIC);
	cvResize(img4_inv_temp2, img4_inv, CV_INTER_CUBIC);
	cvResize(img5_inv_temp2, img5_inv, CV_INTER_CUBIC);

	/*
// Downsampling using my own code

	DownSampleImage(img1_inv_temp2, img1_inv, 2);
	DownSampleImage(img2_inv_temp2, img2_inv, 2);
	DownSampleImage(img3_inv_temp2, img3_inv, 2);
	DownSampleImage(img4_inv_temp2, img4_inv, 2);
	DownSampleImage(img5_inv_temp2, img5_inv, 2);
*/
/*
	cvSaveImage("DBWRx1.jpg", img1_inv);
	cvSaveImage("DBWRx2.jpg", img2_inv);
	cvSaveImage("DBWRx3.jpg", img3_inv);
	cvSaveImage("DBWRx4.jpg", img4_inv);
	cvSaveImage("DBWRx5.jpg", img5_inv);


	cvNamedWindow( "img1_inv", 1 );
	cvShowImage( "img1_inv", img1_inv);
	cvWaitKey(0);

	cvNamedWindow( "img2_inv", 1 );
	cvShowImage( "img2_inv", img2_inv);
	cvWaitKey(0);

	cvNamedWindow( "img3_inv", 1 );
	cvShowImage( "img3_inv", img3_inv );
	cvWaitKey(0);

	cvNamedWindow( "img4_inv", 1 );
	cvShowImage( "img4_inv", img4_inv );
	cvWaitKey(0);

	cvNamedWindow( "img5_inv", 1 );
	cvShowImage( "img5_inv", img5_inv );
	cvWaitKey(0);
*/	

  

/***********************************************************************************/
//			 Doing the Mosaicking of the inv images							//
//																					//
/***********************************************************************************/
	//printf("\n Doing the Mosaicking of the Error Images ...\n" );

	allocateOnDemand( &ddst1,  cvSize(prev_panorama->width,prev_panorama->height),  img1->depth, img1->nChannels );  
	allocateOnDemand( &ddst2,  cvSize(prev_panorama->width,prev_panorama->height),  img1->depth, img1->nChannels );  
	allocateOnDemand( &ddst3,  cvSize(prev_panorama->width,prev_panorama->height),  img1->depth, img1->nChannels );  
	allocateOnDemand( &ddst4,  cvSize(prev_panorama->width,prev_panorama->height),  img1->depth, img1->nChannels );  
	allocateOnDemand( &ddst5,  cvSize(prev_panorama->width,prev_panorama->height),  img1->depth, img1->nChannels );  
	
	/*
	allocateOnDemand( &ddst1,  cvSize(1360, 640),  img1->depth, img1->nChannels );  
	allocateOnDemand( &ddst2,  cvSize(1360, 640),  img1->depth, img1->nChannels );  
	allocateOnDemand( &ddst3,  cvSize(1360, 640),  img1->depth, img1->nChannels );  
	allocateOnDemand( &ddst4,  cvSize(1360, 640),  img1->depth, img1->nChannels );  
	allocateOnDemand( &ddst5,  cvSize(1360, 640),  img1->depth, img1->nChannels );  
*/

    WarpImageUnderQuad(img1_inv,ddst1,ROI_frame1);
	//cvNamedWindow( "Perspective_Warpdd1", 1 );
	//cvShowImage( "Perspective_Warpdd1", ddst1);
	//cvWaitKey(10);


  WarpImageUnderQuad(img2_inv,ddst2,ROI_frame2);

 // cvNamedWindow( "Perspective_Warpdd2", 1 );
 // cvShowImage( "Perspective_Warpdd2", ddst2 );
  //cvWaitKey(10);


 WarpImageUnderQuad(img3_inv,ddst3,ROI_frame3);
 //cvNamedWindow( "Perspective_Warpdd3", 1 );
 //cvShowImage( "Perspective_Warpdd3", ddst3 );
 //cvWaitKey(10);


 WarpImageUnderQuad(img4_inv,ddst4,ROI_frame4);
 //cvNamedWindow( "Perspective_Warpdd4", 1 );
 //cvShowImage( "Perspective_Warpdd4", ddst4 );
 //cvWaitKey(10);

 
 WarpImageUnderQuad(img5_inv,ddst5,ROI_frame5);
 //cvNamedWindow( "Perspective_Warpdd5", 1 );
 ///cvShowImage( "Perspective_Warpdd5", ddst5 );
 //cvWaitKey(10);

 allocateOnDemand( &panorama5d,  cvSize(prev_panorama->width, prev_panorama->height),  img1->depth, img1->nChannels );  

 S2 = FindSRegion(ROI_frame2);

	  for(i = 0; i < panorama5d->height; i ++){ //rows
		  for(j = 0; j < panorama5d->width; j++){ //cols
			  P.x = j;
			  P.y = i;
			  if(Belongs2S(P,S2))cvSet2D(panorama5d,i,j,cvGet2D(ddst2,i,j));
			  else cvSet2D(panorama5d,i,j,cvGet2D(ddst1,i,j));			  
		  }//end for
	  }// end for

	

////////////////////////////
// Finding the region S3 //
////////////////////////////

   S3 = FindSRegion(ROI_frame3);

	  for(i = 0; i < panorama5d->height; i ++){ //rows
		  for(j = 0; j < panorama5d->width; j++){ //cols
			  P.x = j;
			  P.y = i;
			  if(Belongs2S(P,S3))cvSet2D(panorama5d,i,j,cvGet2D(ddst3,i,j));
			  else cvSet2D(panorama5d,i,j,cvGet2D(panorama5d,i,j));			  
		  }//end for
	  }// end for



 ////////////////////////////
// Finding the region S4 //
////////////////////////////

   S4 = FindSRegion(ROI_frame4);

	  for(i = 0; i < panorama5d->height; i ++){ //rows
		  for(j = 0; j < panorama5d->width; j++){ //cols
			  P.x = j;
			  P.y = i;
			  if(Belongs2S(P,S4))cvSet2D(panorama5d,i,j,cvGet2D(ddst4,i,j));
			  else cvSet2D(panorama5d,i,j,cvGet2D(panorama5d,i,j));			  
		  }//end for
	  }// end for


	////////////////////////////
	// Finding the region S4 //
	////////////////////////////

   S5 = FindSRegion(ROI_frame5);


	  for(i = 0; i < panorama5d->height; i ++){ //rows
		  for(j = 0; j < panorama5d->width; j++){ //cols
			  P.x = j;
			  P.y = i;
			  if(Belongs2S(P,S5))cvSet2D(panorama5d,i,j,cvGet2D(ddst5,i,j));
			  else cvSet2D(panorama5d,i,j,cvGet2D(panorama5d,i,j));			  
		  }//end for
	  }// end for

	// cvNamedWindow( "panorama5d", 1 );
	//cvShowImage( "panorama5d", panorama5d );
	 //cvWaitKey(0);

    /****************************************
     Now updating r
	*****************************************/
   cvAdd(SRCG_w,panorama5d,SRCG_w, 0);
   

	cvReleaseMat( &kernelLLt );
	cvReleaseMat( &kernelL );
	cvReleaseMat( &kernelB );
	cvReleaseMat( &lambdaM );

	return;

}

extern void SRMosaicking_Conjugate_Gradient(IplImage* img1, IplImage* img2, IplImage* img3, IplImage* img4, IplImage* img5, IplImage* prev_panorama, 
					struct image_border ROI_frame1, struct image_border ROI_frame2, struct image_border ROI_frame3, struct image_border ROI_frame4, struct image_border ROI_frame5,
					IplImage* panoramaSR, IplImage* prev_Pn, IplImage* Pn,  double* lambda_itp, double* errorSRp, double* alpha_nump, int it)
					{

//IplImage* panoramaSR = NULL;
IplImage* img1_inv = NULL, *img2_inv = NULL, *img3_inv=NULL, *img4_inv=NULL, *img5_inv=NULL;
IplImage* img1_inv_temp1 = NULL, *img2_inv_temp1 = NULL, *img3_inv_temp1=NULL, *img4_inv_temp1=NULL, *img5_inv_temp1=NULL;
IplImage* img1_inv_temp2 = NULL, *img2_inv_temp2 = NULL, *img3_inv_temp2=NULL, *img4_inv_temp2=NULL, *img5_inv_temp2=NULL;

IplImage* dimg1=NULL, *dimg2=NULL, *dimg3=NULL, *dimg4=NULL, *dimg5=NULL;
IplImage* Ddimg1=NULL, *Ddimg2=NULL, *Ddimg3=NULL, *Ddimg4=NULL, *Ddimg5=NULL;
IplImage* ddst1 = NULL, *ddst2 = NULL, *ddst3 = NULL, *ddst4 = NULL, *ddst5=NULL;
IplImage* panorama5d = NULL;
IplImage* panorama5d2 = NULL; //after multiplication with alpha
IplImage* panorama5LLt = NULL;
IplImage* panorama5LLt2 = NULL; //after multiplication
IplImage* panorama5_Delta = NULL, *panorama5_Deltaf = NULL;
IplImage* Xregularized = NULL; // To use with the algorithm Fast and Robust Regularization
IplImage* Limg = NULL;
IplImage* SRCG_rf = NULL;
IplImage* alpha_cg_Pn_1 = NULL, *alpha_Pn = NULL;

CvMat* kernelLLt = cvCreateMat(3,3,CV_32FC1);
CvMat* kernelB = cvCreateMat(3,3,CV_32FC1);
CvMat* kernelL = cvCreateMat(3,3,CV_32FC1);

CvMat* lambdaM = NULL;

double lambda_1, lambda_2, lambda_3, lambda_4, lambda_5;
double lambda_num, lambda_den;
double lambdad_1, lambdad_2, lambdad_3, lambdad_4, lambdad_5;
double lambda_it;
double alpha_it = 1.1127;// was 7, 1, 1.1127
double errorSR;

float value_img, alpha_num, alpha_cg;

float alpha = 0.0;

CvPoint2D32f P; // P for the mosaic

int i, j, D;

struct RegionS  S1, S2, S3, S4, S5;

CvSize sizeD1, sizeD2, sizeD3, sizeD4, sizeD5;
CvSize size; //size of the frames

//FILE *pFile;




	/// First part:  Interpolation : resizing the image to 2 times the original size

  D = 2;

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

	size.height = img1->height;
	size.width = img1->width;

/**************************************************************************************/ 
   //   Starting a new iteration (it = 3)
   //   
	//  First Step: Getting the imgs from the mosaic image
   //
   /**************************************************************************************/
/*
	allocateOnDemand( &img1_inv, cvSize(img2->width,img2->height),  img1->depth, img1->nChannels );
    WarpImageUnderQuadInv(prev_panorama,img1_inv,ROI_frame1);

    allocateOnDemand( &img2_inv, cvSize(img2->width,img2->height),  img1->depth, img1->nChannels );
    WarpImageUnderQuadInv(prev_panorama,img2_inv,ROI_frame2);

	allocateOnDemand( &img3_inv, cvSize(img2->width,img2->height),  img1->depth, img1->nChannels );
	WarpImageUnderQuadInv(prev_panorama,img3_inv,ROI_frame3);

	allocateOnDemand( &img4_inv, cvSize(img2->width,img2->height),  img1->depth, img1->nChannels );
	WarpImageUnderQuadInv(prev_panorama,img4_inv,ROI_frame4);

	allocateOnDemand( &img5_inv, cvSize(img2->width,img2->height),  img1->depth, img1->nChannels );
	WarpImageUnderQuadInv(prev_panorama,img5_inv,ROI_frame5);
*/

	allocateOnDemand( &img1_inv_temp1, sizeD1,  img1->depth, img1->nChannels );
    WarpImageUnderQuadInv(prev_panorama,img1_inv_temp1,ROI_frame1);

    allocateOnDemand( &img2_inv_temp1, sizeD1,  img1->depth, img1->nChannels );
    WarpImageUnderQuadInv(prev_panorama,img2_inv_temp1,ROI_frame2);

	allocateOnDemand( &img3_inv_temp1, sizeD1,  img1->depth, img1->nChannels );
	WarpImageUnderQuadInv(prev_panorama,img3_inv_temp1,ROI_frame3);

	allocateOnDemand( &img4_inv_temp1, sizeD1,  img1->depth, img1->nChannels );
	WarpImageUnderQuadInv(prev_panorama,img4_inv_temp1,ROI_frame4);

	allocateOnDemand( &img5_inv_temp1, sizeD1,  img1->depth, img1->nChannels );
	WarpImageUnderQuadInv(prev_panorama,img5_inv_temp1,ROI_frame5);

	// Using a Gaussian Filter to perform an Antia-alias filtering
	
	allocateOnDemand( &img1_inv_temp2, sizeD1,  img1->depth, img1->nChannels );
    allocateOnDemand( &img2_inv_temp2, sizeD1,  img1->depth, img1->nChannels );
    allocateOnDemand( &img3_inv_temp2, sizeD1,  img1->depth, img1->nChannels );
	allocateOnDemand( &img4_inv_temp2, sizeD1,  img1->depth, img1->nChannels );
	allocateOnDemand( &img5_inv_temp2, sizeD1,  img1->depth, img1->nChannels );
/*
	cvSmooth(img1_inv_temp1,img1_inv_temp2, CV_GAUSSIAN, 3, 0, 0, 0);
	cvSmooth(img2_inv_temp1,img2_inv_temp2, CV_GAUSSIAN, 3, 0, 0, 0);
	cvSmooth(img3_inv_temp1,img3_inv_temp2, CV_GAUSSIAN, 3, 0, 0, 0);
	cvSmooth(img4_inv_temp1,img4_inv_temp2, CV_GAUSSIAN, 3, 0, 0, 0);
	cvSmooth(img5_inv_temp1,img5_inv_temp2, CV_GAUSSIAN, 3, 0, 0, 0);
*/
	// Using Filter 2D:
    //This kernel is the same as the used in the paper: " A Transform-Domain Approach to Super-resolution Mosaicing of Compressed Images"

	cvmSet(kernelB,0,0,0.1016);
	cvmSet(kernelB,0,1,0.1172);
	cvmSet(kernelB,0,2,0.1016);
	cvmSet(kernelB,1,0,0.1172);
	cvmSet(kernelB,1,1,0.1250);
	cvmSet(kernelB,1,2,0.1172);
	cvmSet(kernelB,2,0,0.1016);
	cvmSet(kernelB,2,1,0.1172);
	cvmSet(kernelB,2,2,0.1016);

	cvFilter2D(img1_inv_temp1,img1_inv_temp2,kernelB,cvPoint(-1,1));
	cvFilter2D(img2_inv_temp1,img2_inv_temp2,kernelB,cvPoint(-1,1));
	cvFilter2D(img3_inv_temp1,img3_inv_temp2,kernelB,cvPoint(-1,1));
	cvFilter2D(img4_inv_temp1,img4_inv_temp2,kernelB,cvPoint(-1,1));
	cvFilter2D(img5_inv_temp1,img5_inv_temp2,kernelB,cvPoint(-1,1));
	
  // Downsampling using cubic interpolation

	allocateOnDemand( &img1_inv, size,  img1->depth, img1->nChannels );
    allocateOnDemand( &img2_inv, size,  img1->depth, img1->nChannels );
    allocateOnDemand( &img3_inv, size,  img1->depth, img1->nChannels );
	allocateOnDemand( &img4_inv, size,  img1->depth, img1->nChannels );
	allocateOnDemand( &img5_inv, size,  img1->depth, img1->nChannels );
	
	cvResize(img1_inv_temp2, img1_inv, CV_INTER_CUBIC);
	cvResize(img2_inv_temp2, img2_inv, CV_INTER_CUBIC);
	cvResize(img3_inv_temp2, img3_inv, CV_INTER_CUBIC);
	cvResize(img4_inv_temp2, img4_inv, CV_INTER_CUBIC);
	cvResize(img5_inv_temp2, img5_inv, CV_INTER_CUBIC);

	/*
// Downsampling using my own code

	DownSampleImage(img1_inv_temp2, img1_inv, 2);
	DownSampleImage(img2_inv_temp2, img2_inv, 2);
	DownSampleImage(img3_inv_temp2, img3_inv, 2);
	DownSampleImage(img4_inv_temp2, img4_inv, 2);
	DownSampleImage(img5_inv_temp2, img5_inv, 2);
*/
/*
	cvSaveImage("DBWRx1.jpg", img1_inv);
	cvSaveImage("DBWRx2.jpg", img2_inv);
	cvSaveImage("DBWRx3.jpg", img3_inv);
	cvSaveImage("DBWRx4.jpg", img4_inv);
	cvSaveImage("DBWRx5.jpg", img5_inv);


	cvNamedWindow( "img1_inv", 1 );
	cvShowImage( "img1_inv", img1_inv);
	cvWaitKey(0);

	cvNamedWindow( "img2_inv", 1 );
	cvShowImage( "img2_inv", img2_inv);
	cvWaitKey(0);

	cvNamedWindow( "img3_inv", 1 );
	cvShowImage( "img3_inv", img3_inv );
	cvWaitKey(0);

	cvNamedWindow( "img4_inv", 1 );
	cvShowImage( "img4_inv", img4_inv );
	cvWaitKey(0);

	cvNamedWindow( "img5_inv", 1 );
	cvShowImage( "img5_inv", img5_inv );
	cvWaitKey(0);
*/	

   /**************************************************************************************/ 
   //    
   //   
   //  Second Step: Compute the frame difference between the original low frames and the LR reconstructed ones
   //
   /**************************************************************************************/

  /************************************************************************/
  //  Doing the frame substracion
  /****************************************************************************/
 //  printf("\n Doing the frame substracion ...\n" );


  allocateOnDemand( &dimg1, cvSize(img2->width,img2->height),  img1->depth, img1->nChannels );
  allocateOnDemand( &dimg2, cvSize(img2->width,img2->height),  img1->depth, img1->nChannels );
  allocateOnDemand( &dimg3, cvSize(img2->width,img2->height),  img1->depth, img1->nChannels );
  allocateOnDemand( &dimg4, cvSize(img2->width,img2->height),  img1->depth, img1->nChannels );
  allocateOnDemand( &dimg5, cvSize(img2->width,img2->height),  img1->depth, img1->nChannels );
/*
   printf("\n size_img1 = (%d , %d) \t size_img1_inv = (%d, %d)", img1->height, img1->width, img1_inv->height, img1_inv->width);
   printf("\n size_dimg1 = (%d , %d) ", dimg1->height, dimg1->width);

   getchar();
*/

  cvSub(img1,img1_inv,dimg1,NULL);
  cvSub(img2,img2_inv,dimg2,NULL);
  cvSub(img3,img3_inv,dimg3,NULL);
  cvSub(img4,img4_inv,dimg4,NULL);
  cvSub(img5,img5_inv,dimg5,NULL);

 
  //*********************************************************************************************//
  //  Doing the interpolation of the images differences											//
  //*********************************************************************************************//
    //printf("\n Doing the frame interpolation of the difference of frames ...\n" );
    allocateOnDemand( &Ddimg1, sizeD1,  img1->depth, img1->nChannels );  
	allocateOnDemand( &Ddimg2, sizeD2,  img1->depth, img1->nChannels );  
	allocateOnDemand( &Ddimg3, sizeD3,  img1->depth, img1->nChannels );  
	allocateOnDemand( &Ddimg4, sizeD4,  img1->depth, img1->nChannels );  
	allocateOnDemand( &Ddimg5, sizeD5,  img1->depth, img1->nChannels );  

	/*
  	cvResize(dimg1, Ddimg1,CV_INTER_CUBIC);
	cvResize(dimg2, Ddimg2,CV_INTER_CUBIC);
	cvResize(dimg3, Ddimg3,CV_INTER_CUBIC);
	cvResize(dimg4, Ddimg4,CV_INTER_CUBIC);
	cvResize(dimg5, Ddimg5,CV_INTER_CUBIC);
	*/

	UpSampleImageD(dimg1, Ddimg1);
	UpSampleImageD(dimg2, Ddimg2);
	UpSampleImageD(dimg3, Ddimg3);
	UpSampleImageD(dimg4, Ddimg4);
	UpSampleImageD(dimg5, Ddimg5);

/***********************************************************************************/
//			 Doing the Mosaicking of the Error Images								//
//																					//
/***********************************************************************************/
	//printf("\n Doing the Mosaicking of the Error Images ...\n" );

	allocateOnDemand( &ddst1,  cvSize(prev_panorama->width,prev_panorama->height),  img1->depth, img1->nChannels );  
	allocateOnDemand( &ddst2,  cvSize(prev_panorama->width,prev_panorama->height),  img1->depth, img1->nChannels );  
	allocateOnDemand( &ddst3,  cvSize(prev_panorama->width,prev_panorama->height),  img1->depth, img1->nChannels );  
	allocateOnDemand( &ddst4,  cvSize(prev_panorama->width,prev_panorama->height),  img1->depth, img1->nChannels );  
	allocateOnDemand( &ddst5,  cvSize(prev_panorama->width,prev_panorama->height),  img1->depth, img1->nChannels );  
	
	/*
	allocateOnDemand( &ddst1,  cvSize(1360, 640),  img1->depth, img1->nChannels );  
	allocateOnDemand( &ddst2,  cvSize(1360, 640),  img1->depth, img1->nChannels );  
	allocateOnDemand( &ddst3,  cvSize(1360, 640),  img1->depth, img1->nChannels );  
	allocateOnDemand( &ddst4,  cvSize(1360, 640),  img1->depth, img1->nChannels );  
	allocateOnDemand( &ddst5,  cvSize(1360, 640),  img1->depth, img1->nChannels );  
*/

    WarpImageUnderQuad(Ddimg1,ddst1,ROI_frame1);
	//cvNamedWindow( "Perspective_Warpdd1", 1 );
	//cvShowImage( "Perspective_Warpdd1", ddst1);
	//cvWaitKey(10);


  WarpImageUnderQuad(Ddimg2,ddst2,ROI_frame2);

 // cvNamedWindow( "Perspective_Warpdd2", 1 );
 // cvShowImage( "Perspective_Warpdd2", ddst2 );
  //cvWaitKey(10);


 WarpImageUnderQuad(Ddimg3,ddst3,ROI_frame3);
 //cvNamedWindow( "Perspective_Warpdd3", 1 );
 //cvShowImage( "Perspective_Warpdd3", ddst3 );
 //cvWaitKey(10);


 WarpImageUnderQuad(Ddimg4,ddst4,ROI_frame4);
 //cvNamedWindow( "Perspective_Warpdd4", 1 );
 //cvShowImage( "Perspective_Warpdd4", ddst4 );
 //cvWaitKey(10);

 
 WarpImageUnderQuad(Ddimg5,ddst5,ROI_frame5);
 //cvNamedWindow( "Perspective_Warpdd5", 1 );
 ///cvShowImage( "Perspective_Warpdd5", ddst5 );
 //cvWaitKey(10);

 allocateOnDemand( &panorama5d,  cvSize(prev_panorama->width, prev_panorama->height),  img1->depth, img1->nChannels );  

 S2 = FindSRegion(ROI_frame2);

	  for(i = 0; i < panorama5d->height; i ++){ //rows
		  for(j = 0; j < panorama5d->width; j++){ //cols
			  P.x = j;
			  P.y = i;
			  if(Belongs2S(P,S2))cvSet2D(panorama5d,i,j,cvGet2D(ddst2,i,j));
			  else cvSet2D(panorama5d,i,j,cvGet2D(ddst1,i,j));			  
		  }//end for
	  }// end for

	

////////////////////////////
// Finding the region S3 //
////////////////////////////

   S3 = FindSRegion(ROI_frame3);

	  for(i = 0; i < panorama5d->height; i ++){ //rows
		  for(j = 0; j < panorama5d->width; j++){ //cols
			  P.x = j;
			  P.y = i;
			  if(Belongs2S(P,S3))cvSet2D(panorama5d,i,j,cvGet2D(ddst3,i,j));
			  else cvSet2D(panorama5d,i,j,cvGet2D(panorama5d,i,j));			  
		  }//end for
	  }// end for



 ////////////////////////////
// Finding the region S4 //
////////////////////////////

   S4 = FindSRegion(ROI_frame4);

	  for(i = 0; i < panorama5d->height; i ++){ //rows
		  for(j = 0; j < panorama5d->width; j++){ //cols
			  P.x = j;
			  P.y = i;
			  if(Belongs2S(P,S4))cvSet2D(panorama5d,i,j,cvGet2D(ddst4,i,j));
			  else cvSet2D(panorama5d,i,j,cvGet2D(panorama5d,i,j));			  
		  }//end for
	  }// end for


	    ////////////////////////////
		// Finding the region S4 //
		////////////////////////////

   S5 = FindSRegion(ROI_frame5);


	  for(i = 0; i < panorama5d->height; i ++){ //rows
		  for(j = 0; j < panorama5d->width; j++){ //cols
			  P.x = j;
			  P.y = i;
			  if(Belongs2S(P,S5))cvSet2D(panorama5d,i,j,cvGet2D(ddst5,i,j));
			  else cvSet2D(panorama5d,i,j,cvGet2D(panorama5d,i,j));			  
		  }//end for
	  }// end for

	// cvNamedWindow( "panorama5d", 1 );
	//cvShowImage( "panorama5d", panorama5d );
	 //cvWaitKey(0);

    /****************************************
     Now updating r
	*****************************************/
   //cvAdd(SRCG_r,panorama5d,SRCG_r, 0);

   

   /**************************************************************************************/
   //
   // Computing the normalizing parameter lambda^(n) 
   //
   /**************************************************************************************/
    
	  lambda_1 = cvNorm(img1, img1_inv, CV_L2, NULL);
	 // printf("\n lambda_1 = %f", lambda_1);
	  lambda_2 = cvNorm(img2, img2_inv, CV_L2, NULL);
	 // printf("\n lambda_2 = %f", lambda_2);
	  lambda_3 = cvNorm(img3, img3_inv, CV_L2, NULL);
	 // printf("\n lambda_3 = %f", lambda_3);
	  lambda_4 = cvNorm(img4, img4_inv, CV_L2, NULL);
	 // printf("\n lambda_4 = %f", lambda_4);
	  lambda_5 = cvNorm(img5, img5_inv, CV_L2, NULL);
	 // printf("\n lambda_5 = %f", lambda_5);

   /**************************************************************************************/
   //
   // Computing the Laplace of the x^(n) image x(n) comes to be the panorama5 variables (mosaic of the previous state)
   //
   /**************************************************************************************/
   allocateOnDemand( &Limg, cvSize(prev_panorama->width,prev_panorama->height),  img1->depth, img1->nChannels );  
   //cvLaplace(prev_panorama, Limg,3);   
/*
// Uncomment this to use Laplace Prior Function

   // Using in stead cvFilter2D

   	cvmSet(kernelL,0,0,0.0000);
	cvmSet(kernelL,0,1,0.2500);
	cvmSet(kernelL,0,2,0.0000);
	cvmSet(kernelL,1,0,0.2500);
	cvmSet(kernelL,1,1,-1.000);
	cvmSet(kernelL,1,2,0.2500);
	cvmSet(kernelL,2,0,0.0000);
	cvmSet(kernelL,2,1,0.2500);
	cvmSet(kernelL,2,2,0.0000);

   cvFilter2D(prev_panorama, Limg, kernelL, cvPoint(-1,-1));
*/
 // Uncomment this to use Huber Prior Function
  // Using Huber Prior Function to find lambda

     alpha = 1.75;//was 1.75

     HubbertFunctionCliques(prev_panorama, Limg, alpha);

  // cvNamedWindow( "Limg", 1 );
  // cvShowImage( "Limg", Limg );
  // cvWaitKey(0);


   /**************************************************************************************/
   //
   // Computing the K norm(x^(n),2) , K = 1, 2 ...5
   //
   /**************************************************************************************/

  lambda_num = pow((lambda_1 + lambda_2 + lambda_3 + lambda_4 + lambda_5),2);
  lambdad_1 = cvNorm(Limg, NULL, CV_L2, NULL);
 

    /**************************************************************************************/
   //
   // Computing the lambda(n), K = 5, because I am just working with 5 frames
   //
   /**************************************************************************************/
 
  lambda_den = pow((5*lambdad_1),2);

    
  lambda_it = lambda_num/lambda_den;

  
  // printf("\n lambda_num = %f", lambda_num);
  // printf("\n lambda_den = %f", lambda_den);
  // printf("\n lambda_it = %f", lambda_it);

  // *lambda_itp =  lambda_it;

  // printf("\n lambda_it = %f \n", *lambda_itp);

  // pFile = fopen(name, "w");

   //fprintf(pFile, "%f\n", lambda_it);

  // fclose(pFile);
   
  // lambda_it = 1.1127;

  // printf("\n Using alpha from GCV = %f", alpha_it);

    /**************************************************************************************/
   //
   // Computing the lambda(n)*L'*L*x(n)
   //
   /**************************************************************************************/

   /*
    cvmSet(kernelLLt,0,0,1.000*lambda_it);
	cvmSet(kernelLLt,0,1,-4.000*lambda_it);
	cvmSet(kernelLLt,0,2,1.000*lambda_it);
	cvmSet(kernelLLt,1,0,-4.000*lambda_it);
	cvmSet(kernelLLt,1,1,18.000*lambda_it);
	cvmSet(kernelLLt,1,2,-4.000*lambda_it);
	cvmSet(kernelLLt,2,0,1.000*lambda_it);
	cvmSet(kernelLLt,2,1,-4.000*lambda_it);
	cvmSet(kernelLLt,2,2,1.000*lambda_it);
*/
	cvmSet(kernelLLt,0,0,1.000);
	cvmSet(kernelLLt,0,1,-4.000);
	cvmSet(kernelLLt,0,2,1.000);
	cvmSet(kernelLLt,1,0,-4.000);
	cvmSet(kernelLLt,1,1,18.000);
	cvmSet(kernelLLt,1,2,-4.000);
	cvmSet(kernelLLt,2,0,1.000);
	cvmSet(kernelLLt,2,1,-4.000);
	cvmSet(kernelLLt,2,2,1.000);

	allocateOnDemand( &panorama5LLt, cvSize(prev_panorama->width,prev_panorama->height),  img1->depth, img1->nChannels );  
	allocateOnDemand( &panorama5LLt2, cvSize(prev_panorama->width,prev_panorama->height),  img1->depth, img1->nChannels );  
	allocateOnDemand( &panorama5d2, cvSize(prev_panorama->width,prev_panorama->height),  img1->depth, img1->nChannels ); 

 //	printf("\n Limg->height = %d \t Limg->width = %d", Limg->height, Limg->width);
 //	printf("\n prev_panorama->height = %d \t prev_panorama->width = %d", prev_panorama->height, prev_panorama->width);

  //  Uncomment to use Huber Regularization

    // Using Huber Regularization here as well
	HubbertRegularization(Limg, panorama5LLt, alpha);
/*
  // Uncomment to use Laplace Regularization
    // Using Laplace prior
	cvFilter2D(Limg,panorama5LLt,kernelL,cvPoint(-1,1));
	cvFilter2D(panorama5LLt,panorama5LLt2, kernelL,cvPoint(-1,1)); // Because os L^t*L*x
*/	

/*
    //Using the regularization proposed in the paper: "Fast and Robust Super-Resolution"

	allocateOnDemand( &Xregularized, cvSize(prev_panorama->width,prev_panorama->height), IPL_DEPTH_32F, 1 );

	FastRobustRegularization(Limg, Xregularized, 2, alpha); // p = 2

	panorama5LLt = convert_gray32_to_color(Xregularized);

*/

	/*
	//
	// Converting to Matrix lambda doesn't work, because of the size of the mosaicking
	//

	lambdaM = cvCreateMat(panorama5LLt->height,panorama5LLt->width,CV_32FC1);
	cvSetIdentity(lambdaM,cvRealScalar(lambda_it));

	cvMatMul(panorama5LLt,lambdaM, panorama5LLt2);
	*/

	//panorama5LLt2 = MulImagewithDouble(panorama5LLt, lambda_it);// Use this for Huber Regularization
	
	//panorama5LLt2 = MulImagewithDouble(panorama5LLt2, lambda_it); // Use this for Laplace Regularization
	
	//panorama5d2 = MulImagewithDouble(panorama5d, alpha_it);

   /**************************************************************************************/ 
   //
   //	Now updating the iteration of x(n+1). First I have to do: panorama5d - panorama5LLt
   //   Then I have to do : panorma5_2 = panorama5_1 + (panorama5d - panorama5LLt)
   //
   /**************************************************************************************/
	
	//allocateOnDemand( &panoramaSR, cvSize(prev_panorama->width,prev_panorama->height),  img1->depth, img1->nChannels );  
	allocateOnDemand( &panorama5_Delta, cvSize(prev_panorama->width,prev_panorama->height),  img1->depth, img1->nChannels );  

	cvSub(panorama5d,panorama5LLt,panorama5_Delta,NULL);
	
	//cvCloneImage(panorama5_Delta);

	

	// SRCG is the Jacobian, so to find alpha_num, we have to:

	allocateOnDemand( &panorama5_Deltaf, cvSize(prev_panorama->width,prev_panorama->height),  IPL_DEPTH_32F, 1 );  

	panorama5_Deltaf = convert2gray32(panorama5_Delta);

	alpha_num = 0.0;
	for(i = 0; i < panorama5_Deltaf->height; i++){
		for( j = 0; j < panorama5_Deltaf->width; j++){
			value_img = ((float *)(panorama5_Deltaf->imageData + i*panorama5_Deltaf->widthStep))[j];
			alpha_num += value_img*value_img;
		}
	}

     alpha_nump[it] = alpha_num;

     alpha_cg = alpha_nump[it]/alpha_nump[it-1];

	 alpha_cg_Pn_1 = MulImagewithDouble(prev_Pn, alpha_cg);

     cvAdd(panorama5_Delta, alpha_cg_Pn_1, Pn, NULL);

	 alpha_Pn = MulImagewithDouble(Pn, alpha_it);
   
	 cvAdd(alpha_Pn,prev_panorama,panoramaSR,NULL);

	//cvNamedWindow( "panoramaSR", 1 );
	//cvShowImage( "panoramaSR", panoramaSR );
	//cvWaitKey(0);


	/***********************************************************************************
	Computing the error of the SR
	*****************************************************************************************/

	errorSR = cvNorm(panoramaSR, prev_panorama, CV_RELATIVE_L2, NULL);
	//errorSR = cvNorm(panoramaSR, prev_panorama, CV_L2, NULL);

    printf(" \n errorSR = %f\n", errorSR);


	cvReleaseMat( &kernelLLt );
	cvReleaseMat( &kernelL );
	cvReleaseMat( &kernelB );
	cvReleaseMat( &lambdaM );

	return;

}

/*
IplImage* ConstrucMosaic4NImgs(IplImage** Imgs, int NImgs, IpImage* ImgBase ) 
{
 int i,j;

 for( i = 0; i < NImgs; i++ ){
		Imgs[i] = cvCreateImage( cvGetSize(ImgBase), ImgBase->depth, ImgBase->nChannels);
	}
 
	sizeD1.height = D*(Imgs[0]->height);
	sizeD1.width = D*(Imgs[0]->width);

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
	UpSampleImageD(img2, Dimg2);
	UpSampleImageD(img3, Dimg3);
	UpSampleImageD(img4, Dimg4);
	UpSampleImageD(img5, Dimg5);


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
 
 ROI_frame4.P1.x = ROI_frame4.P1.x + x_offset;
 ROI_frame4.P1.y = ROI_frame4.P1.y + y_offset;
 ROI_frame4.P2.x = ROI_frame4.P2.x + x_offset;
 ROI_frame4.P2.y = ROI_frame4.P2.y + y_offset;
 ROI_frame4.P3.x = ROI_frame4.P3.x + x_offset;
 ROI_frame4.P3.y = ROI_frame4.P3.y + y_offset;
 ROI_frame4.P4.x = ROI_frame4.P4.x + x_offset;
 ROI_frame4.P4.y = ROI_frame4.P4.y + y_offset;

 WarpImageUnderQuad(Dimg4,dst4,ROI_frame4);

 ROI_frame5.P1.x = ROI_frame5.P1.x + x_offset;
 ROI_frame5.P1.y = ROI_frame5.P1.y + y_offset;
 ROI_frame5.P2.x = ROI_frame5.P2.x + x_offset;
 ROI_frame5.P2.y = ROI_frame5.P2.y + y_offset;
 ROI_frame5.P3.x = ROI_frame5.P3.x + x_offset;
 ROI_frame5.P3.y = ROI_frame5.P3.y + y_offset;
 ROI_frame5.P4.x = ROI_frame5.P4.x + x_offset;
 ROI_frame5.P4.y = ROI_frame5.P4.y + y_offset;

 WarpImageUnderQuad(Dimg5,dst5,ROI_frame5);


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
}

*/
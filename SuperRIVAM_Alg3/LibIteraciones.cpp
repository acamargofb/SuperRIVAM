#include "LibIteraciones.h"


//includes de VTK
#include <vtkSmartPointer.h>
#include <vtkImageViewer2.h>
#include <vtkDICOMImageReader.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkImageData.h>
#include <vtkImageCast.h>
#include <vtkUnsignedCharArray.h>
#include <vtkFloatArray.h>
#include <vtkJPEGWriter.h>
#include <vtkWriter.h>
#include <vtkPNGWriter.h>

//includes GDCM
#include <vtkGDCMImageReader.h>
#include <vtkGDCMImageWriter.h>
#include <vtkImageData.h>
#include <vtkImageMagnify.h>
#include <vtkImageCast.h>
#include <gdcmTesting.h>
#include <gdcmSystem.h>


void IterativeBackProjecting(IplImage *ref[],IplImage *delta_est, IplImage *phi_est, double factor, const int N){

	/* Estableciendo los parametros necesarios para la estimacion inicial*/
	const int max_iter = 50;
	int  iter;
	double lambda,delta;
	IplImage *X, *X1;
	IplImage *m, *E;
	SYSTEMTIME t1;
	SYSTEMTIME t2;

	/* Estableciendo las propiedades de la imagen de referencia */
	int width  	 = ref[0]->width;
	int height   = ref[0]->height;
	double dx, dy;
	int x,y,i,j,k;

	double cc;

	/* Iniciacion */
	lambda = 0.01;

	/*Contador del numero de iteraciones*/
	iter = 1;


	/* Comenzando con un estimado, la imagen de referncia ampliada con el metodo interpolacion */
	/*X es la imagen estimada de superresolucion, X es la primera estimacion*/
	X = cvCreateImage(cvSize(width*(int)factor,height*(int)factor), IPL_DEPTH_32F, 1);
	X1 = cvCreateImage(cvGetSize(X),IPL_DEPTH_32F,1);

	int w = X->width;
	int h = X->height;

	cvResize(ref[0], X, CV_INTER_CUBIC);

	cv::gpu::GpuMat g_X;
	g_X.upload(X);
	g_X.convertTo(g_X, CV_32FC1, 1, 0);

	/* g_X_prev es la imagen en superesolution en la iteracion anterior */
	cv::gpu::GpuMat g_X_prev( X->height, X->width, CV_32FC1 );
	g_X.copyTo(g_X_prev);	

	cv::gpu::GpuMat g_X1(  X->height, X->width, CV_32FC1 );
	cv::gpu::GpuMat g_temp(  X->height, X->width, CV_32FC1 );
	 
	/* Definicion de temp1 */
	dx = (double)g_temp.cols;
	dy = (double)g_temp.rows;

	if( g_temp.cols%(int)factor ==0 ){
		x  = (int)(dx/(int)factor);
	} else{	x  = (int)(dx/(int)factor)+1;	}

	if( g_temp.rows%(int)factor ==0 ){
		y  = (int)(dy/(int)factor); 
	} else{	y  = (int)(dy/(int)factor)+1;	}


	/* Creacion de matrices requeridas */
	m = cvCreateImage(cvSize(2,1),X->depth,1);
	E = cvCreateImage(cvSize(max_iter*2,1),X->depth,1);

	//cv::gpu::GpuMat g_temp1(y, x, CV_32FC1);
	cv::gpu::GpuMat g_G(h, w, CV_32FC1);

	//cv::Mat temp1(ref[0]->height, ref[0]->width, CV_32FC1);
	//cv::Mat temp(g_temp.rows, g_temp.cols, CV_32FC1);

	int elements = g_temp.cols*g_temp.rows;
	size_t datasize = elements*sizeof(float);

	//copiando las imagenes a float*
	std::vector<float *>cl_ref(N);
	int w11 = ref[0]->width;
	int w22 = ref[0]->height;
	for(k=0; k<N; k++){
		cl_ref[k] = (float *)malloc(w11*w22*sizeof(float));
		for(i=0; i<w22; i++){
			float *p32 =  (float *)(ref[k]->imageData + i*ref[k]->widthStep);
			memcpy( cl_ref[k] + i*w11 , p32,w11*sizeof(float) ); 

		}
		cvReleaseImage(&ref[k]);
	}
	
	/* Realizando las iteraciones */
	while(iter < max_iter){

		g_G.setTo(cv::Scalar(0));

		for(i=0; i<N; i++){

			dy=(double)factor*cvGetReal2D(delta_est,i,1);
			dx=(double)factor*cvGetReal2D(delta_est,i,0);
			double *rot1 = (double *)(phi_est->imageData + i*phi_est->widthStep);

			g_desplazamiento2dim( g_X, g_temp, -dy, -dx );
			//gc_integral(g_temp, cl_ref[i],g_temp.cols, g_temp.rows, out, factor, rot1[0] );
			//gc_rotarImagen(g_temp, out , rot1[0] );
			//g_rotarImagen(g_temp, g_temp , rot1[0] );
			//g_filtro_blur(g_temp, g_temp, 1);

				gc_iterar(g_temp, cl_ref[i], g_temp.cols, g_temp.rows, x, y, factor, rot1[0], g_temp);

			//gc_reducirmatriz(out, cl_ref[i] ,g_temp.cols, g_temp.rows, temp1, factor);
			//g_reducirmatriz(g_temp, g_temp1, factor);
			//cv::gpu::subtract(g_temp1, g_ref[i], g_temp1);
			//g_temp1.download(temp1);
			//gc_resize(temp1 , temp);
			//cv::resize(temp1,temp,cv::Size(g_temp.cols,g_temp.rows),0,0, CV_INTER_CUBIC);

			//g_temp.upload(temp);
			//g_filtro_sharpen(g_temp, g_temp, 2);
			//g_rotarImagen(g_temp, g_temp , -rot1[0] );
			//gc_integral2(out, x, y, g_temp, -rot1[0]);
			g_desplazamiento2dim( g_temp, g_temp, dy, dx );
			
			cv::gpu::add(g_G, g_temp, g_G);

		}

		cv::gpu::multiply(g_G, cv::Scalar(lambda), g_G);
		cv::gpu::subtract(g_X, g_G, g_X);	
		cv::gpu::subtract(g_X, g_X_prev, g_X1);

		ArrtoIplImage(g_X, X);
		ArrtoIplImage(g_X1, X1);
		SVD_DC(X, m, 0);
		SVD_DC(X1, m, 1);

		float *ptre = (float *)( E->imageData );
		float *ptrm = (float *)( m->imageData );

		delta = ptrm[1]/ptrm[0];
		ptre[ 2*(iter-1) ] = iter;
		ptre[ 2*(iter-1) + 1 ] = delta;
		
		fprintf(stdout,"\nDelta: %f",delta);
		if (iter>3){
			cc = fabs( ptre[ 2*(iter-4)+1 ] ) - delta ;
		if ( cc < 0.00001 || iter== 15 ){
				break;
			}
		}
		g_X.copyTo( g_X_prev );
		iter++;

	}



  char *out = "Img_SuperResolution";
  IplImageDICOM(out, X);
  //MostrarDICOM(out); 
  //MostrarDICOM("31425514"); 

  //Estos comandos solo se efecutan para grabar la imagen en jpeg
  F64to8f(X,X);
  cv::Mat Y = X;
  std::vector<int> a(1);
  a[0] = 9;
  cv::imwrite("Img_En_SuperResolution.png",Y, a);
	
  g_G.~GpuMat();
  g_temp.~GpuMat();
  //g_temp1.~GpuMat();
  g_X.~GpuMat();


}



void F8to64f(IplImage *ref,IplImage *dst){
	int i,j;

	for(i=0; i<ref->height ;i++){
		uchar* ptr = (uchar*)(ref->imageData + i*ref->widthStep);
		float *ptr2 = (float *)(dst->imageData + i*dst->widthStep);

		for(j=0; j<ref->width ; j++){
			ptr2[j] = (float)ptr[j]/255.0f;
		}
	}

}



void F64to8f(IplImage *ref,IplImage *dst){
	int i,j;

	for(i=0; i<ref->height ;i++){
		float * ptr = (float *)(ref->imageData + i*ref->widthStep);
		float *ptr2 = (float *)(dst->imageData + i*dst->widthStep);

		for(j=0; j<ref->width ; j++){
			ptr2[j] = ptr[j];
		}
	}
}



void F16to64f(IplImage *ref,IplImage *dst){
	int i,j;
	double temp;

	for(i=0; i<ref->height ;i++){
		for(j=0; j<ref->width ; j++){
		temp = cvGetReal2D(ref,i,j)/65535;
		cvSetReal2D(dst,i,j, temp );
		}
	}
}





void g_desplazamiento2dim(cv::gpu::GpuMat &src,cv::gpu::GpuMat &dst ,double y, double x){
	
	int dx, dy;

	//Creacion de Variables
	cv::gpu::GpuMat g_q1;
	cv::gpu::GpuMat g_q2;
	cv::gpu::GpuMat g_d1;
	cv::gpu::GpuMat g_d2;

	int width, height;
	
	height	= src.rows;
	width = src.cols;
		
	cv::gpu::GpuMat g_t(src.rows, src.cols, CV_32FC1, cv::Scalar(0) );  

	/* Asignando las variable x e y */
	dx = (int)floor(x)%src.cols;
	dy = (int)floor(y)%src.rows;

		if ( dy > 0 ){
			
			g_q1 = src.rowRange(0, height-dy);
			g_d1 = g_t.rowRange(dy , height);
			g_q2 = src.rowRange(height -dy , height);
			g_d2 = g_t.rowRange(0 , dy);

			g_q1.copyTo(g_d1);
			g_q2.copyTo(g_d2);

		} else if ( dy < 0  ){

			g_q1 = src.rowRange(0 , -dy);
			g_d1 = g_t.rowRange(height + dy , height);
			g_q2 = src.rowRange(-dy , height);
			g_d2 = g_t.rowRange(0 , height + dy);

			g_q1.copyTo(g_d1);
			g_q2.copyTo(g_d2);

		} else {

			g_q1 = src.rowRange(0, height-dy);
			g_d1 = g_t.rowRange(dy , height);
			g_q1.copyTo(g_d1);
			
		}
		
		if ( dx > 0 ){

			g_q1 = g_t.colRange(0 , width-dx);
			g_d1 = dst.colRange(dx , width);
			g_q2 = g_t.colRange(width-dx , width);
			g_d2 = dst.colRange(0 , dx );

			g_q1.copyTo(g_d1);
			g_q2.copyTo(g_d2);

		} else if ( dx < 0  ){

			g_q1 = g_t.colRange(0 , -dx);
			g_d1 = dst.colRange(width + dx , width);
			g_q2 = g_t.colRange( -dx , width);
			g_d2 = dst.colRange(0 , width + dx );

			g_q1.copyTo(g_d1);
			g_q2.copyTo(g_d2);

		} else {

			g_q1 = g_t.colRange(0  , width-dx);
			g_d1 = dst.colRange(dx , width);
			g_q1.copyTo(g_d1);
		}

}



void rotarImagen(IplImage *src, IplImage *dst ,double angle){

	CvMat* rot_mat = cvCreateMat(2,3,CV_64FC1);
	CvPoint2D32f center = cvPoint2D32f(	src->width/2, src->height/2 );
	IplImage *rgt1;

	rgt1 = cvCreateImage(cvSize(src->width,src->height),src->depth,1);
	rot_mat	= cvCreateMat(2,3,CV_64FC1);

	cv2DRotationMatrix( center, angle, 1, rot_mat );
	cvWarpAffine( src, rgt1, rot_mat, CV_WARP_FILL_OUTLIERS, cvScalarAll(0));

	cvCopy(rgt1,dst,0);

	cvReleaseImage( &rgt1 );
}



void g_rotarImagen(cv::gpu::GpuMat &src, cv::gpu::GpuMat &dst ,double angle){

	int width, height;

	width  = src.cols;
	height = src.rows;

	CvMat* rot_mat;
	CvPoint2D32f center = cvPoint2D32f(	width/2, height/2 );

	cv::gpu::GpuMat g_rgt1(height , width, CV_32FC1, cv::Scalar(0) );

	rot_mat	= cvCreateMat( 2,3,CV_32FC1 );
	cv2DRotationMatrix( center, angle, 1, rot_mat );

	cv::gpu::warpAffine( src, g_rgt1, rot_mat, cv::Size(width, height), 2  );
	g_rgt1.copyTo( dst );
	
}

void gc_rotarImagen(cv::gpu::GpuMat &src, float *out ,double angle){

	
	static const float pi = (float)3.14159265359;
	int i,j;
	float theta = (float)angle/180*pi; 
	float filter[9];

	//Matriz
	filter[0*3+0] = 0;
	filter[0*3+1] = 0.1766666666667;
	filter[0*3+2] = 0;
	filter[1*3+0] = 0.1766666666667;
	filter[1*3+1] = 0.2533333333334;
	filter[1*3+2] = 0.1766666666667;
	filter[2*3+0] = 0;
	filter[2*3+1] = 0.1766666666667;
	filter[2*3+2] = 0;
	//copiando de gpu al host
	cv::Mat p1 = src;

	//Definiendo la cantidad de datos
	int width  = src.cols;
	int height = src.rows;
	int elements = width*height;
	
	size_t datasize = elements*sizeof(float);

	float *d = (float *)malloc(datasize);//entrada

	for( i=0; i<height; i++){
		float *p_src = p1.ptr<float>(i);
		for( j=0; j<width; j++){
			d[j+i*width] = p_src[j];
		}
	}

	//OpenClRotar(d,out,width,height,theta,elements);
	OpenClRotarBlurr(d, out, width, height, filter, theta, elements);

	/*
	cv::Mat p2(p1.rows, p1.cols, CV_32FC1);
	for( i=0; i<height; i++){
		float *pc = p2.ptr<float>(i);
		for( j=0; j<width; j++){
			pc[j] = out[i*width+j];
		}
	}
	cv::namedWindow("rotacion",CV_WINDOW_AUTOSIZE);
	cv::imshow("rotacion", p2);
	cv::waitKey(0);
	p2.~Mat();
	*/

	p1.~Mat();

}



void gc_rotarImagen1(float *d, cv::gpu::GpuMat &dst ,double angle){

	int width  = dst.cols;
	int height = dst.rows;
	int i,j;
	static const float pi = (float)3.14159265359;
	float theta = (float)angle/180*pi; 


	//Definiendo la cantidad de datos
	int elements = width*height;
	size_t datasize = elements*sizeof(float);

	float *out = (float *)malloc(datasize);//entrada


	OpenClRotar(d,out,width,height,theta,elements);


	cv::Mat p2(height, width, CV_32FC1);
	for( i=0; i<height; i++){
		float *pc = p2.ptr<float>(i);
		for( j=0; j<width; j++){
			pc[j] = out[i*width+j];
		}
	}

	/*
	cv::namedWindow("rotacion",CV_WINDOW_AUTOSIZE);
	cv::imshow("rotacion", p2);
	cv::waitKey(0);	*/

	dst.upload(p2);
	p2.~Mat();
	free(out);

}




void g_filtro_blur( cv::gpu::GpuMat &src, cv::gpu::GpuMat &dst , int n){

	cv::Mat nucleo(3, 3 , CV_32FC1, cv::Scalar(0));

		nucleo.at<float>(0,1) = (float)0.1766666666667;
		nucleo.at<float>(1,0) = (float)0.1766666666667;
		nucleo.at<float>(1,1) = (float)0.2533333333334;
		nucleo.at<float>(1,2) = (float)0.1766666666667;
		nucleo.at<float>(2,1) = (float)0.1766666666667;

	cv::Mat ss = src;
	cv::Mat dd(dst.rows, dst.cols, CV_32FC1);
	cv::filter2D(ss, dd, ss.depth(), nucleo, cv::Point(-1,-1) , 0 , 4);
	dst = dd;
	
}


void gc_filtro_blur( float *d, int &width, int &height, float *out , int n){



	//Definiendo la cantidad de datos
	int elements = width*height;
	size_t datasize = elements*sizeof(float);

	float filter[9];


	//Matriz
	filter[0*3+0] = 0;
	filter[0*3+1] = 0.1766666666667;
	filter[0*3+2] = 0;
	filter[1*3+0] = 0.1766666666667;
	filter[1*3+1] = 0.2533333333334;
	filter[1*3+2] = 0.1766666666667;
	filter[2*3+0] = 0;
	filter[2*3+1] = 0.1766666666667;
	filter[2*3+2] = 0;


	OpenClBlurr(d, out,width, height, filter ,elements);

	/*
	cv::namedWindow("filtro",CV_WINDOW_AUTOSIZE);
	cv::imshow("filtro", p2);
	cv::waitKey(0);*/


}


void g_filtro_sharpen( cv::gpu::GpuMat &src, cv::gpu::GpuMat &dst , int n){

	cv::Mat nucleo(3, 3 , CV_32FC1);

		nucleo.at<float>(0,0) = (float)0;
		nucleo.at<float>(0,1) = (float)-0.27;
		nucleo.at<float>(0,2) = (float)0;
		nucleo.at<float>(1,0) = (float)-0.27;
		nucleo.at<float>(1,1) = (float)4.000;
		nucleo.at<float>(1,2) = (float)-0.27;
		nucleo.at<float>(2,0) = (float)0;
		nucleo.at<float>(2,1) = (float)-0.27;
		nucleo.at<float>(2,2) = (float)0;

	cv::Mat ss = src;
	cv::Mat dd(dst.rows, dst.cols, CV_32FC1);
	cv::filter2D(ss, dd, ss.depth(), nucleo, cv::Point(-1,-1) , 0 , 4);
	dst = dd;
	
}



void gc_filtro_sharpen( cv::Mat temp, int &width, int &height, float *out , int n){

	int i,j;

	//Definiendo la cantidad de datos
	int elements = width*height;
	size_t datasize = elements*sizeof(float);

	//entrada
	float *d	= (float *)malloc(datasize);
	float filter[9];

	for( i=0; i<height; i++){
		float *pc =temp.ptr<float>(i);
		for( j=0; j<width; j++){
			d[i*width+j] = pc[j] ;
		}
	}

	//Matriz
	filter[0*3+0] = 0;
	filter[0*3+1] = -0.27;
	filter[0*3+2] = 0;
	filter[1*3+0] = 0.27;
	filter[1*3+1] = 4;
	filter[1*3+2] = -0.27;
	filter[2*3+0] = 0;
	filter[2*3+1] = 0.27;
	filter[2*3+2] = 0;

	OpenClBlurr(d, out,width, height, filter ,elements);

	/*
	cv::namedWindow("filtro",CV_WINDOW_AUTOSIZE);
	cv::imshow("filtro", p2);
	cv::waitKey(0);*/

}



void desp_x(IplImage *src,IplImage *dst, double x){

	cvConvertScale(src,dst,1,-x);	

}


void M_scalar(IplImage *src,IplImage *dst, double K){
	IplImage *temporal;
	
	temporal = cvCreateImage( cvGetSize(src), src->depth, src->nChannels);
	cvSet( temporal, cvScalar(K,0,0,0));
	cvMul( src, temporal, dst, 1);
	cvReleaseImage( &temporal );
}


void SVD_DC(IplImage *X, IplImage *m, int j){

	IplImage *temp1, *W;
	int w,h;
	float K;

	w = X->width;
	h = X->height;
	K=1;
	while(  w/K > 80 ){
		K++;
	}

	temp1 =cvCreateImage(cvSize((int)floor(w/K),(int)floor(h/K)),X->depth,1);
	cvResize(X,temp1,CV_INTER_LINEAR);

	W = cvCreateImage(cvGetSize(temp1),temp1->depth,temp1->nChannels);

	/* Creando variable temporal */       

	cvSVD(temp1,W,0,0,CV_SVD_V_T);

	float *ptr = (float *)(m->imageData);
	ptr[j] = (float)cvNorm(W,0,CV_C,0)*K;

	cvReleaseImage(&temp1);
	
	cvReleaseImage(&W);

}




void g_SVD_DC(cv::Mat &X, IplImage *m, int j){

	cv::Mat temp1, W;
	int w,h;
	double K;

	w = X.cols;
	h = X.rows;
	K=1;
	while(  w/K > 80 ){
		K++;
	}
	temp1.create( (int)floor(h/K),(int)floor(w/K),CV_32FC1 );
	cv::resize(X, temp1, cv::Size(temp1.cols, temp1.rows), 0, 0, CV_INTER_LINEAR);

	W.create(temp1.rows, temp1.cols, CV_32FC1 );

	/* Creando variable temporal */
	cv::SVD();
	cv::SVD::compute(temp1, W, CV_SVD_V_T);

	double *ptr = (double *)(m->imageData);
	ptr[j] = cv::norm(W,CV_C)*K;


}




void g_iterim(double factor, IplImage *delta_est, IplImage *phi_est, std::vector<cv::gpu::GpuMat> &g_ref, cv::gpu::GpuMat &g_G, cv::gpu::GpuMat &g_X, const int N, double lambda, int i, int w, int h){

	double dy, dx, dy1, dx1;
	int x,y;
	cv::Mat temp1;
	cv::Mat temp;

	cv::gpu::GpuMat g_temp(  h, w, CV_32FC1 );
	 
	/* Definicion de temp1 */
	dx = (double)g_temp.cols;
	dy = (double)g_temp.rows;

	if( g_temp.cols%(int)factor ==0 ){
		x  = (int)(dx/(int)factor);
	} else{	x  = (int)(dx/(int)factor)+1;	}

	if( g_temp.rows%(int)factor ==0 ){
		y  = (int)(dy/(int)factor); 
	} else{	y  = (int)(dy/(int)factor)+1;	}

	cv::gpu::GpuMat g_temp1(y, x, CV_32FC1);

	dy= factor*cvGetReal2D(delta_est, i, 1);
	dy1= -dy;
	dx= factor*cvGetReal2D(delta_est, i, 0);
	dx1= -dx;

	g_desplazamiento2dim( g_X, g_temp, dy1, dx1 );
    g_rotarImagen(g_temp, g_temp , cvGetReal2D(phi_est, i, 0) );
	g_filtro_blur(g_temp, g_temp, 1);
	g_reducirmatriz(g_temp, g_temp1, factor);

	cv::gpu::subtract(g_temp1, g_ref[i], g_temp1);
	g_temp1.download(temp1);
	g_temp.download(temp);
	
	cv::resize(temp1,temp,cv::Size(temp.cols,temp.rows),0,0, CV_INTER_CUBIC);
	g_temp.upload(temp);
	g_filtro_sharpen(g_temp, g_temp, 2);

	g_rotarImagen(g_temp, g_temp , -cvGetReal2D(phi_est,i,0) );
	g_desplazamiento2dim( g_temp, g_temp,dy,dx );
		
	cv::gpu::add(g_G, g_temp, g_G);



}


void g_reducirmatriz(cv::gpu::GpuMat &A, cv::gpu::GpuMat &t, double &factor){

	float dx, dy;
	int x,y,f;
	int i;
	
	cv::gpu::GpuMat g_q1, g_d1;
	/* Iniciando la funcion */  
	dx = (float)A.cols;
	dy = (float)A.rows;
	f = (int)factor;

	if( A.cols%f ==0 ){
		x  = (int)(dx/f);
	} else{
		x  = (int)(dx/f)+1;
	}

	if( A.rows%f ==0 ){
		y  = (int)(dy/f); 
	} else{
		y  = (int)(dy/f)+1;
	}

	/* Seleccion de las filas*/
	cv::gpu::GpuMat g_t1(y, (int)dx, CV_32FC1);
	i = 0;

	while(i < y){
		g_q1 = A.row( f*i );
		g_d1 = g_t1.row( i );
		g_q1.copyTo(g_d1);
		i++;
	}

	i=0;

	while(i < x){
		g_q1 = g_t1.col( f*i );
		g_d1 = t.col( i );
		g_q1.copyTo(g_d1);
		i++;
	}


}




void gc_reducirmatriz(float *in, float * B,int &W, int &H, cv::Mat &ou, double &factor){

	float dx, dy;
	int x,y,f;
	int i,j;


	/* Iniciando la funcion */  
	dx = (float)W;
	dy = (float)H;
	f = (int)factor;

	if( W%f ==0 ){
		x  = (int)(dx/f);
	} else{
		x  = (int)(dx/f)+1;
	}

	if( H%f ==0 ){
		y  = (int)(dy/f); 
	} else{
		y  = (int)(dy/f)+1;
	}


	/* Seleccion de las filas*/
	float *temp = (float *)malloc(y*W*sizeof(float));

	int elementos	 = x*y;
	size_t datasize  = elementos*sizeof(float);
	float *out		 = (float *)malloc(datasize);
	float *salida    = (float *)malloc(datasize);

	i = 0;
	while(i < y){
		for(j=0; j<W; j++){
			temp[i*W+j]=in[f*i*W+j];
		}	
		i++;
	}

	j = 0;
	while(j < x){
		for(i=0; i<y; i++){
			out[i*x+j]=temp[i*W+j*f];
		}
		j++;
	}

	OpenClSuma(out, B, salida, elementos);

	//convirtiendo de float a cv::Mat
	for( i = 0; i<y ; i++){ 
		float *po = ou.ptr<float>(i);
		for( j = 0; j<x ; j++){
			po[j] = salida[i*x+j];
		}
	}
	free(salida);
	free(in);
	free(temp);
}


void gc_resize(cv::Mat &temp1 , cv::Mat &temp){
	
	int i,j;
	int wL = temp1.cols;
	int hL = temp1.rows;
	int wH = temp.cols;
	int hH = temp.rows;
	int elemH = wH*hH;
	int elemL = wL*hL;
	size_t datasizeL = elemL*sizeof(float);
	size_t datasizeH = elemH*sizeof(float);

	float *d   = (float *)malloc(datasizeL);//entrada
	float *out11 = (float *)malloc(datasizeH);//entrada

	for( i=0; i<hL; i++){
		float *p_src = temp1.ptr<float>(i);
		for( j=0; j<wL; j++){
			d[j+i*wL] = p_src[j];
		}
	}

	OpenCLResize(d, out11, wL, hL, wH, hH, elemH, elemL);

	for( i = 0; i<hH ; i++){ 
		float *po = temp.ptr<float>(i);
		for( j = 0; j<wH ; j++){
			po[j] = out11[i*wH+j];
		}
	}

	free(out11);

	/*
	cv::namedWindow("ampliado",CV_WINDOW_AUTOSIZE);
	cv::imshow("ampliado", temp);
	cv::waitKey(0);
	*/
	
}


void gc_integral(cv::gpu::GpuMat &src, float * B,int &W, int &H, float *out, double &factor,double angle){

	//variables para rotacion y filtro
	static const float pi = (float)3.14159265359;
	int i,j;
	float theta = (float)angle/180*pi; 
	float filter[9];

	//variables para reducir matriz
	float dx, dy;
	int x,y,f;

	//Matriz
	filter[0*3+0] = 0;
	filter[0*3+1] = 0.1766666666667;
	filter[0*3+2] = 0;
	filter[1*3+0] = 0.1766666666667;
	filter[1*3+1] = 0.2533333333334;
	filter[1*3+2] = 0.1766666666667;
	filter[2*3+0] = 0;
	filter[2*3+1] = 0.1766666666667;
	filter[2*3+2] = 0;
	//copiando de gpu al host
	cv::Mat p1 = src;

	//Definiendo la cantidad de datos
	int width  = src.cols;
	int height = src.rows;
	int elements1 = width*height;
	
	size_t datasize = elements1*sizeof(float);

	float *d = (float *)malloc(datasize);//entrada, se borra al ejecutar openclrotarblurr
	//float *out1 = (float *)malloc(datasize);//intermedio - salida de openclROtarBlurr

	for( i=0; i<height; i++){
		float *p_src = p1.ptr<float>(i);
		for( j=0; j<width; j++){
			d[j+i*width] = p_src[j];
		}
	}

	//OpenClRotarBlurr(d, out1, width, height, filter, theta, elements1);
	p1.~Mat();

	//Sentecias para reducir matriz
	dx = (float)W;
	dy = (float)H;
	f = (int)factor;

	if( W%f ==0 ){	x  = (int)(dx/f);
	} else{	x  = (int)(dx/f)+1;
	}

	if( H%f ==0 ){	y  = (int)(dy/f); 
	} else{	y  = (int)(dy/f)+1;
	}
	
	/* Seleccion de las filas*/
	int elementos2	 = x*y;

	OpenCLRoBlReSu( d, B, out, width, height, filter, theta, x, y, f);

}

void gc_integral2(float *in, int &wL, int &hL ,cv::gpu::GpuMat g_temp, double angle){

	int i,j;
	//variables para ejecutar un resize
	int wH = g_temp.cols;
	int hH = g_temp.rows;
	int elemH = wH*hH;
	int elemL = wL*hL;
	size_t datasizeL = elemL*sizeof(float);
	size_t datasizeH = elemH*sizeof(float);

	//variables para ejecutar openclblurr
	float filter[9];
	//Matriz
	filter[0*3+0] = 0;
	filter[0*3+1] = -0.27;
	filter[0*3+2] = 0;
	filter[1*3+0] = 0.27;
	filter[1*3+1] = 4;
	filter[1*3+2] = -0.27;
	filter[2*3+0] = 0;
	filter[2*3+1] = 0.27;
	filter[2*3+2] = 0;

	//variables para ejecutar openclRotar
	static const float pi = (float)3.14159265359;
	float theta = (float)angle/180*pi; 
	float *out = (float *)malloc(datasizeH);//salida de openclRotar se borra al final
	
	OpenCLReBlRo(in, out, theta, filter, wL, hL, wH, hH, elemH, elemL );

	cv::Mat p2(hH, wH, CV_32FC1);
	for( i=0; i<hH; i++){
		float *pc = p2.ptr<float>(i);
		for( j=0; j<wH; j++){
			pc[j] = out[i*wH+j];
		}
	}

	/*
	cv::namedWindow("rotacion",CV_WINDOW_AUTOSIZE);
	cv::imshow("rotacion", p2);
	cv::waitKey(0);	*/

	g_temp.upload(p2);
	p2.~Mat();
	free(out);


}





void gc_iterar(cv::gpu::GpuMat &src, float * B,int &wH, int &hH,int &wL, int &hL, double &factor, double &angle, cv::gpu::GpuMat &dst){
	//variables para rotacion y filtro
	static const float pi = (float)3.14159265359;
	int i,j;
	float theta1 = (float)angle/180*pi; 
	float filter1[9];

	//variables para reducir matriz
	int f;

	//Matriz
	filter1[0*3+0] = 0;
	filter1[0*3+1] = 0.1666666666667;
	filter1[0*3+2] = 0;
	filter1[1*3+0] = 0.1666666666667;
	filter1[1*3+1] = 0.3333333333334;
	filter1[1*3+2] = 0.1666666666667;
	filter1[2*3+0] = 0;
	filter1[2*3+1] = 0.1666666666667;
	filter1[2*3+2] = 0;
	//copiando de gpu al host
	cv::Mat p1 = src;

	//Definiendo la cantidad de datos
	int width  = src.cols;
	int height = src.rows;
	int elements1 = width*height;
	
	size_t datasize = elements1*sizeof(float);

	float *d = (float *)malloc(datasize);//entrada, se borra al ejecutar openclrotarblurr

	for( i=0; i<height; i++){
		float *p_src = p1.ptr<float>(i);
		for( j=0; j<width; j++){
			d[j+i*width] = p_src[j];
		}
	}
	p1.~Mat();

	//Sentecias para reducir matriz
	f = (int)factor;

	/* Seleccion de las filas*/
	int elementos2	 = wL*hL;
	//float *out1 = (float *)malloc(wL*hL*sizeof(float)); //salida intermedia, se elimina

	//OpenCLRoBlReSu( d, B, out1, width, height, filter1, theta1, wL, hL, f);

	//variables para ejecutar un resize
	int elemH = wH*hH;
	int elemL = wL*hL;
	size_t datasizeL = elemL*sizeof(float);
	size_t datasizeH = elemH*sizeof(float);

	//variables para ejecutar filtro sharp
	float filter[9];
	//Matriz
	filter[0*3+0] = 0;
	filter[0*3+1] = -0.37;
	filter[0*3+2] = 0;
	filter[1*3+0] = -0.37;
	filter[1*3+1] = 2.00;
	filter[1*3+2] = -0.37;
	filter[2*3+0] = 0;
	filter[2*3+1] = -0.37;
	filter[2*3+2] = 0;

	//variables para ejecutar openclRotar
	float theta2 = -theta1; 
	float *out = (float *)malloc(datasizeH);//salida de openclRotar se borra al final

	Cl_iteraciones( d, B, out, wL, hL, wH, hH, filter1, filter, theta1, theta2, f);

	//OpenCLReBlRo(out1, out, theta2, filter, wL, hL, wH, hH, elemH, elemL );

	cv::Mat p2(hH, wH, CV_32FC1);
	for( i=0; i<hH; i++){
		float *pc = p2.ptr<float>(i);
		for( j=0; j<wH; j++){
			pc[j] = out[i*wH+j];
		}
	}

	/*
	cv::namedWindow("rotacion",CV_WINDOW_AUTOSIZE);
	cv::imshow("rotacion", p2);
	cv::waitKey(0);	*/

	dst.upload(p2);
	p2.~Mat();
	free(out);


}
void psnr(IplImage *src1, IplImage *src2){

	int m,n;
	double mse;
	CvScalar sum;
	IplImage *p;
	double psnr, maxi;

	m = src1->width;
	n = src1->height;
	p = cvCreateImage(cvGetSize(src1), src1->depth, src1->nChannels);

	cvSub( src1, src2, p, 0);
	cvPow( p, p, 2);
	sum = cvSum(p);
	mse = sum.val[0]/(m*n); 
	maxi = 255;
	psnr = 10*log10(maxi*maxi/mse);
	
	fprintf(stdout,"\nEl PSNR es: %f",psnr);

}


void tukeywin(IplImage *dst, double alfa){

	int i,j,Nh,Nv;
	IplImage *h,*v;
	int a,b,c;
	static const double pi = 3.141592653589793 ;

	h = cvCreateImage(cvSize(dst->width, 1) , dst->depth, dst->nChannels);
	v = cvCreateImage(cvSize(dst->height, 1), dst->depth, dst->nChannels);

	Nh = dst->width;
	Nv = dst->height;

	a = alfa/2*(Nh-1) ;
	b = (1 - (alfa/2) )*(Nh-1) ;
	c = Nh-1;
	
	double *p_h = (double *)( h->imageData );
	for(i=0 ; i <= a ; i++){
		p_h[i] =  0.5+ 0.5*cos( pi*(double)i/(double)a -pi ) ;
	}
	for(i=a+1 ; i <= b ; i++){
		p_h[i] =  1 ;
	}
	for(i=b+1 ; i <= c ; i++){
		p_h[i] =  0.5 + 0.5*cos( pi*(double)i/(double)a -2*pi/alfa + pi ) ;
	}

	a = (alfa/2)*(Nv-1) ;
	b = (1 - (alfa/2) )*(Nv-1) ;
	c = Nv-1;
	double *p_v = (double *)( v->imageData );
	for(i=0 ; i <= a ; i++){
		p_v[i] = 0.5 + 0.5*cos( pi*(double)i/(double)a -pi );
	}
	for(i=a+1 ; i <= b ; i++){
		p_v[i] = 1;
	}
	for(i=b+1 ; i <= c ; i++){
		p_v[i] =  0.5 + 0.5*cos( pi*(double)i/(double)a -2*pi/alfa + pi ) ;
	}


	for(i = 0 ; i < Nv ; i++){
		double *p_v = (double *)( v->imageData);
		double *p_h = (double *)( h->imageData);
		double *p_dst = (double *)( dst->imageData + i*dst->widthStep);
		for(j = 0 ; j < Nh ; j++){
			p_dst[j] = p_v[i]*p_h[j];
		}
	}

	cvReleaseImage(&h);
	cvReleaseImage(&v);

}

void ArrtoIplImage(cv::Mat src, IplImage *dst){
	IplImage dst1 = src;
	cvScale(&dst1,dst,1,0);
}

void delta_tiempo(SYSTEMTIME &t1, SYSTEMTIME &t2){
	int min,seg,mseg;

	if(t2.wMilliseconds >= t1.wMilliseconds ){
		mseg = t2.wMilliseconds - t1.wMilliseconds;
	} else{
		if(t2.wSecond >= t1.wSecond ){

		t2.wSecond =  t2.wSecond-1;
		} else{

			if(t2.wMinute >= t1.wMinute ){
				t2.wMinute = t2.wMinute - 1;
				t2.wSecond = t2.wSecond-1+60;
			} 
		}	

		mseg = t2.wMilliseconds - t1.wMilliseconds + 1000;
	} 		

	if(t2.wSecond >= t1.wSecond ){
		seg = t2.wSecond - t1.wSecond;
	} else{
		if(t2.wMinute >= t1.wMinute ){
			t2.wMinute = t2.wMinute - 1;
		} 
		seg = t2.wSecond - t1.wSecond + 60;
	}	

	if(t2.wMinute >= t1.wMinute ){
		min = t2.wMinute - t1.wMinute;
	} 

	printf("\nMinutos: %i min, %i seg, %i mseg\n", min, seg, mseg);
	
	
}
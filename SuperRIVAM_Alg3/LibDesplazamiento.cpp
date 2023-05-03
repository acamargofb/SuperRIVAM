#include "LibDesplazamiento.h"


void estimacion_desplazamiento( IplImage *reg[], int n , CvMat *delta_est, const int N)
{
	int 	i, mx, my,m;
	static const float pi = (float)3.141592653589793;

	
	/* Estableciendo las propiedades de la imagen */
	int width  	 = reg[0]->width;
	int height   = reg[0]->height;
	int step     = reg[0]->widthStep;
	int fft_size = width * height;

	/* Definiendo las variables necesarios para aplicar el filtro */
	int	beginy;
	int	endy; 
	int	beginx; 
	int endx; 


	/* Definiendo la matriz para calcular la diferencia de fases */
	CvMat* Ang = cvCreateMat(height,width,CV_32FC1);

	/* Definiendo las variables para la transformada de Fourier de la referencia*/
    IplImage * image_Re1 = cvCreateImage( cvSize(width, height), IPL_DEPTH_32F, 1); 
    IplImage * image_Im1 = cvCreateImage( cvSize(width, height), IPL_DEPTH_32F, 1); 

	/* Definiendo las variables para la transformada de Fourier de la imagen del registro*/
    IplImage * image_Re2 = cvCreateImage( cvSize(width, height), IPL_DEPTH_32F, 1); 
    IplImage * image_Im2 = cvCreateImage( cvSize(width, height), IPL_DEPTH_32F, 1); 
 
 	/* Transformada de Fourier de la imagen de referencia */
	DFT(reg[0],image_Re1,image_Im1);

	/*Creando la ventana para filtrar las bajas frecuencias y eliminar el aliasing */
    beginy = (int)floor((float)(height/2))-n+1;
	endy   = (int)floor((float)height/2)+n+1;
	beginx = (int)floor((float)width/2)-n+1;
	endx   = (int)floor((float)width/2)+n+1;

	my = (endy - beginy + 1);
	mx = (endx - beginx + 1);
	m = mx*my;

	/* Iniciando los valores estimados */
	cvSetReal2D(delta_est,0,0,0);
	cvSetReal2D(delta_est,0,1,0);

	CvMat *B;
	B = cvCreateMat( m, 1, CV_32FC1); 

	/*Desarrollando las coordenadas x e y de los pixeles */
	float  *x, *y;
	x = (float*) malloc( m *sizeof(int) );	
	y = (float*) malloc( m *sizeof(int) );	
	ventanaxy(x, y, mx, my, beginx, beginy);

	for(i=1 ; i<N; i++){

		DFT(reg[i],image_Re2,image_Im2);

		g_angulo(image_Im1, image_Re1, image_Im2, image_Re2, Ang);

		/*Definiendo la matriz v que son los elementos de angle en las coordenadas establecidas por x e y */
		matriz_v(beginx, beginy, Ang, mx, my, B);

		/* Resolviendo el sistema lineal de ecuaciones*/
		prueba(x, y, B ,beginx, endx, beginy, endy, width, height, delta_est, i);

	}

	cvReleaseImage(&image_Re1);
	cvReleaseImage(&image_Re2);
	cvReleaseImage(&image_Im1);
	cvReleaseImage(&image_Im2);

}


void DFT(IplImage *src, IplImage *dst, IplImage *dstIm){
	int w, h;

	w = src->width;
	h = src->height;

	std::vector<cv::gpu::GpuMat>entrada(2);
	cv::gpu::GpuMat g_complexIn1(h,w,CV_32FC2);
	cv::gpu::GpuMat g_complex(h,w,CV_32FC2);

	entrada[0].upload(src);
	entrada[0].convertTo(entrada[0],CV_32FC1,1,0);
	entrada[1].upload( cv::Mat::zeros(h,w,CV_32FC1) );

	//caculo de la transformada discreta de fourier		
	cv::gpu::merge(entrada,g_complexIn1);
	cv::gpu::dft(g_complexIn1 , g_complex , cv::Size(w,h) , CV_DXT_FORWARD);
	cv::gpu::split(g_complex,entrada);

	//Desplazando la frecuencia fundamental al centro
	g_cvShiftDFT(entrada[0],entrada[0]);
	g_cvShiftDFT(entrada[1],entrada[1]);

	//Copiando al formato IplImage
	ArrtoIplImageD(entrada[0], dst);
	ArrtoIplImageD(entrada[1], dstIm);

	entrada.~vector();
	g_complex.~GpuMat();
	g_complexIn1.~GpuMat();
}


void prueba(float *x, float *y, CvMat *v ,int beginx, int endx, int beginy, int endy, int width, int height, CvMat *delta_est, int index){
	int 	i,l;
	CvMat* A  ;
	CvMat* Xr ;

	l  = (endy - beginy +1)*(endx - beginx + 1);
	A  = cvCreateMat(l,3,CV_32FC1);
	Xr = cvCreateMat(3,1,CV_32FC1);

	/* Ingresando los Valores de la Matriz A*/
	for(i=0;i< l ;i++)
	{
	 CV_MAT_ELEM(*A,float,i,0) = y[i];
	 CV_MAT_ELEM(*A,float,i,1) = x[i];
	 CV_MAT_ELEM(*A,float,i,2) = 1;
	}

	cvSolve(A, v, Xr, CV_SVD);
	solucion(Xr,width,height,delta_est, index);

	cvReleaseMat(&A);
	cvReleaseMat(&Xr);
}

void solucion(CvMat *Xr,int width,int height,CvMat *delta_est, int index){
	float x1,x2,x3;  
	static const float pi = (float)3.1415926535;

	x1	 = CV_MAT_ELEM(*Xr, float, 0, 0);
	x2	 = CV_MAT_ELEM(*Xr, float, 1, 0);
	x3   = CV_MAT_ELEM(*Xr, float, 2, 0);

	CV_MAT_ELEM(*delta_est, double, index, 0) = -x1*width/( 2*(float)pi );
	CV_MAT_ELEM(*delta_est, double, index, 1) = -x2*height/( 2*(float)pi );


}


void coreccionang(CvMat *Ang, int i, int j){
	static const double pi = 3.14159265358979323846;
	double k;
	
	k = cvGetReal2D(Ang, i, j);
	if( k < -pi){
	cvSetReal2D(Ang, i, j, k + (double)2*pi );
	}

	if( k > pi){
	cvSetReal2D(Ang, i, j, k - (double)2*pi );
	}
}


void ventanaxy(float *x, float *y, int &mx, int &my, int &beginx, int &beginy){
	int i,j;

	for( i=0 ; i< my ; i++){
		for( j=0; j< mx ; j++ ) {
			x[ (j + i * mx) ] = (float)beginx + (float)j;
		}
    }


	for( i=0 ; i< my ; i++){
		for( j=0; j<  mx ; j++ ) {
			y[ (j + i*mx) ] = (float)i + (float)beginy;
		}
    }

}



void error_ref(){
	IplImage *err = 0;
	err = cvLoadImage( "ERROR_REF.jpg", CV_LOAD_IMAGE_COLOR );
	cvNamedWindow("ERROR",CV_WINDOW_AUTOSIZE);
	cvShowImage("ERROR",err);
	cvWaitKey(5000);
}

void error_reg(){
	IplImage *err = 0;
	err = cvLoadImage( "ERROR_REG.jpg", CV_LOAD_IMAGE_COLOR );
	cvNamedWindow("ERROR",CV_WINDOW_AUTOSIZE);
	cvShowImage("ERROR",err);
	cvWaitKey(5000);
}

void mostrar_matriz(int filas, int columnas, float *matriz){
	int i,j;
	for( i=0 ; i< filas ; i++){
		for( j=0; j< (columnas-1)  ; j++ ) {
			fprintf(stdout, "%f ", matriz[j + i*columnas ]);
		}
		fprintf(stdout, "%f\n", matriz[j + i*columnas ]);
    }
	fprintf(stdout, "\n\n");
}

void cvShiftDFT(CvArr * src_arr, CvArr * dst_arr ) 
{ 
    CvMat * tmp; 
    CvMat q1stub, q2stub; 
    CvMat q3stub, q4stub; 
    CvMat d1stub, d2stub; 
    CvMat d3stub, d4stub; 
    CvMat * q1, * q2, * q3, * q4; 
    CvMat * d1, * d2, * d3, * d4; 

    CvSize size = cvGetSize(src_arr); 
    CvSize dst_size = cvGetSize(dst_arr); 
    int cx, cy; 

    if(dst_size.width != size.width || dst_size.height != size.height){ 
        cvError( CV_StsUnmatchedSizes, "cvShiftDFT", "Source and Destination arrays must have equal sizes",__FILE__, __LINE__ );   
    } 

    if(src_arr==dst_arr){ 
        tmp = cvCreateMat(size.height/2, size.width/2, cvGetElemType(src_arr)); 
    } 
    
    cx = size.width/2; 
    cy = size.height/2; // image center 

    q1 = cvGetSubRect( src_arr, &q1stub, cvRect(0,0,cx, cy) ); 
    q2 = cvGetSubRect( src_arr, &q2stub, cvRect(cx,0,cx,cy) ); 
    q3 = cvGetSubRect( src_arr, &q3stub, cvRect(cx,cy,cx,cy) ); 
    q4 = cvGetSubRect( src_arr, &q4stub, cvRect(0,cy,cx,cy) ); 
    d1 = cvGetSubRect( src_arr, &d1stub, cvRect(0,0,cx,cy) ); 
    d2 = cvGetSubRect( src_arr, &d2stub, cvRect(cx,0,cx,cy) ); 
    d3 = cvGetSubRect( src_arr, &d3stub, cvRect(cx,cy,cx,cy) ); 
    d4 = cvGetSubRect( src_arr, &d4stub, cvRect(0,cy,cx,cy) ); 

    if(src_arr!=dst_arr){ 
        if( !CV_ARE_TYPES_EQ( q1, d1 )){ 
            cvError( CV_StsUnmatchedFormats, "cvShiftDFT", "Source and Destination arrays must have the same format", __FILE__, __LINE__ ); 
        } 
        cvCopy(q3, d1, 0); 
        cvCopy(q4, d2, 0); 
        cvCopy(q1, d3, 0); 
        cvCopy(q2, d4, 0); 
    } 
    else{ 
        cvCopy(q3, tmp, 0); 
        cvCopy(q1, q3, 0); 
        cvCopy(tmp, q1, 0); 
        cvCopy(q4, tmp, 0); 
        cvCopy(q2, q4, 0); 
        cvCopy(tmp, q2, 0); 
    } 
} 


void g_cvShiftDFT(cv::gpu::GpuMat &src_arr,cv::gpu::GpuMat &dst_arr ){

    int cx, cy; 

	cx = src_arr.cols;
	cy = src_arr.rows;

	cv::gpu::GpuMat q1 = src_arr.operator()( cv::Rect(0,0,cx/2,cy/2) );
	cv::gpu::GpuMat q2 = src_arr.operator()( cv::Rect(cx/2,0,cx/2,cy/2) );
	cv::gpu::GpuMat q3 = src_arr.operator()( cv::Rect(cx/2,cy/2,cx/2,cy/2) );
	cv::gpu::GpuMat q4 = src_arr.operator()( cv::Rect(0,cy/2,cx/2,cy/2) );

	cv::gpu::GpuMat tmp(cy,cx,src_arr.type());

	q3.copyTo(tmp);
	q1.copyTo(q3);
	tmp.copyTo(q1);
	q4.copyTo(tmp);
	q2.copyTo(q4);
	tmp.copyTo(q2);

}





void espectro_magnitud(IplImage *image_Re2, IplImage *image_Im2 ){
   double m2, M2;
		cvPow( image_Re2, image_Re2, 2.0); 
		cvPow( image_Im2, image_Im2, 2.0); 
		cvAdd( image_Re2, image_Im2, image_Re2, NULL); 
		cvPow( image_Re2, image_Re2, 0.5 ); 
		cvAddS( image_Re2, cvScalarAll(1.0), image_Re2, NULL ); // 1 + Mag 
		cvLog( image_Re2, image_Re2 ); // log(1 + Mag) 
		cvNamedWindow("Espectro de Magnitud de la Imagen del Registro",CV_WINDOW_AUTOSIZE);
		cvShiftDFT( image_Re2, image_Re2 ); 
		cvMinMaxLoc(image_Re2, &m2, &M2, NULL, NULL, NULL); 
		cvScale(image_Re2, image_Re2, 1.0/(M2-m2), 1.0*(-m2)/(M2-m2)); 
		cvShowImage("Espectro de Magnitud de la Imagen del Registro", image_Re2); 

}

void angulo(IplImage *image_Im1, IplImage *image_Re1, IplImage *image_Im2, IplImage *image_Re2, CvMat *Ang){

	int i,j;
	int height, width;


	height = image_Re1->height;
	width  = image_Re1->width;

	
		/*inciando la matriz angle cuyos elementos son resultados de la diferencia de fase en cada elemento de la	
		imagen*/
		for( i = 0; i < height ; i++ ) {
			for( j = 0 ; j < width ; j++) {
				cvSetReal2D(Ang, i, j, atan2( cvGetReal2D(image_Im1,i,j),cvGetReal2D(image_Re1,i,j) ) - atan2( cvGetReal2D(image_Im2,i,j),cvGetReal2D(image_Re2,i,j) ) );
			}
		}
		/*Factor de correccion */
		for( i = 0; i < height ; i++ ) {
			for( j = 0 ; j < width ; j++) {
				coreccionang(Ang,i,j);
			}
		}

}

void g_angulo(IplImage *image_Im1, IplImage *image_Re1, IplImage *image_Im2, IplImage *image_Re2, CvMat *Ang){

	int i,j;
	int height, width;

	float *Re1 ; // Input array
	float *Im1 ; // Input array
	float *Re2 ; // Input array
	float *Im2 ; // Input array
	float *R ; // Output array

	height = image_Re1->height;
	width  = image_Re1->width;

	// numero de elementos en cada arreglo
	const int elements = height*width;
	// tamaño de los datos
	size_t datasize = sizeof(float)*elements;
	Re1 = (float*)malloc(datasize);
	Im1 = (float*)malloc(datasize);
	Re2 = (float*)malloc(datasize);
	Im2 = (float*)malloc(datasize);
	R   = (float*)malloc(datasize);

		for( i = 0; i < height ; i++ ) {
			float* ptr1 = (float*)(image_Re1->imageData + i*image_Re1->widthStep);
			float* ptr2 = (float*)(image_Im1->imageData + i*image_Im1->widthStep);
			float* ptr3 = (float*)(image_Re2->imageData + i*image_Re2->widthStep);
			float* ptr4 = (float*)(image_Im2->imageData + i*image_Im2->widthStep);
			for( j = 0 ; j < width ; j++) {
				Re1[j+i*width]= ptr1[j];
				Im1[j+i*width]= ptr2[j];
				Re2[j+i*width]= ptr3[j];
				Im2[j+i*width]= ptr4[j];
			}
		}	
	g_tang(Re1, Im1,Re2,Im2, R, elements);

		/*inciando la matriz angle cuyos elementos son resultados de la diferencia de fase en cada elemento de la imagen*/
		for( i = 0; i < height ; i++ ) {
			for( j = 0 ; j < width ; j++) {

			CV_MAT_ELEM(*Ang, float,  i, j ) = R[j+i*width] ;

			}
		}

}



void matriz_v(int beginx, int beginy, CvMat *Ang, int mx, int my, CvMat *B){
	int i,j;

	for( i=0 ; i< my ; i++){
		for( j=0; j< mx ; j++ ) {
	
			CV_MAT_ELEM(*B, float,  i + j*mx, 0 ) = CV_MAT_ELEM(*Ang, float,  i+beginy-1, j+beginx-1 ) ;
		}
	} 

}



void ArrtoIplImageD(cv::Mat src, IplImage *dst){
	IplImage dst1 = src;
	cvScale(&dst1,dst,1,0);
}


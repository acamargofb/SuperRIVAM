#include "LibRotacion.h"


void est_rotacion( IplImage *reg[], double *d_front, float precision,CvMat *angle_est, const int N, IplImage *h_A[], IplImage *H_AI[], IplImage *H_AR[] )
{

	/* Variables de Entrada */
	int 	i, j, k ;
	double  d ;
	IplImage *m ;
	IplImage *n ;
	static const float pi = (float)3.141592653589793 ;

	/* Estableciendo las propiedades de la imagen de referencia */
	int width  	 = reg[0]->width ;
	int height   = reg[0]->height;

	/* Definiendo las variables CvArray para la conversion de coordenadas */
	cv::Mat xx ;
	cv::Mat yy ;
	CvMat* th;
	CvMat* th1;
	CvMat* ra;
	CvMat* ix;
	CvMat* T ;

	//GPU
	cv::gpu::GpuMat g_x;
	cv::gpu::GpuMat g_y;	
	cv::gpu::GpuMat g_raT( height, width, CV_32FC1);
	cv::gpu::GpuMat g_thT( height, width, CV_32FC1);
	cv::gpu::GpuMat g_ra ( width, height, CV_32FC1);
	cv::gpu::GpuMat g_th ( width, height, CV_32FC1);

	/* Definiendo las variables para la transformada de Fourier de la primera Imagen */
    IplImage * image_Re1;			//Salida, parte real
    IplImage * image_Im1;			//Salida, parte imaginaria

	/* Definiendo las variables adicionales para calcular el valor promedio de la transformada de fourier de las imagenes */


	//indica la cantidad de elementos que se obtieron debido a un promedio
	float nep;

	/* Estableciendo el centro de la imagen y la matriz que sera el dominio de las frecuencias(coordenadas)*/
	d = 1*pi/180;

	/* Creacion de las coordenadas x e y */
	xx.create(height,width,CV_32FC1);
	yy.create(height,width,CV_32FC1);

	cv::Mat qy(height,1,CV_32FC1);
	cv::Mat qx(1,width,CV_32FC1);

	//Ingresando las componentes x de la matriz
	for(i=0 ; i < width; i++)
	{
		qx.at<float>(0,i) = (float)-1+(float)i*(float)2/(float)width;
	}

	cv::Mat tmp;
	for(i=0 ; i < height; i++)
	{
		tmp  = xx.row(i) ;
		qx.copyTo(tmp);
	}	
	
	//Ingresando las componentes y de la matriz
	for(i=0 ; i < height; i++)
	{
		qy.at<float>(i,0) = (float)-1+(float)i*(float)2/(float)height;
	}

	for(i=0 ; i < width; i++)
	{
		tmp  = yy.col(i) ;
		qy.copyTo(tmp);
	}

	g_x.upload(xx);
	g_y.upload(yy); 

	//Se procedera a hacer un cambio en el sistema de coordenadas de cartesiano a polar

	cv::gpu::cartToPolar(g_x, g_y, g_raT, g_thT);
	g_x.~GpuMat();
	g_y.~GpuMat();

	cv::gpu::transpose(g_thT,g_th);
	cv::gpu::transpose(g_raT,g_ra);

	th = cvCreateMat(width,height,CV_32FC1);
	ra = cvCreateMat(width,height,CV_32FC1);

	ArrtoIplImageRM(g_th, th);
	ArrtoIplImageRM(g_ra, ra);

	g_th.~GpuMat();
	g_ra.~GpuMat();

	//Pasando a OpenCL
	float *cl_th;  //Entrada
	float *cl_ra;  //Entrada
	float *cl_tt;  //Salida
	float *cl_thR; //Entrada
	float *dd;


	// Numero de elementos en cada arreglo
	const int elements = height*width;
	size_t datasize = sizeof(float)*elements;
	cl_th  = (float*)malloc(datasize);
	cl_ra  = (float*)malloc(datasize);
	cl_tt  = (float*)malloc(datasize);
	cl_thR = (float*)malloc(datasize);
	dd	   = (float*)malloc(sizeof(float)*2);

	for(i=0 ; i < width;i++)
	{

		for(j=0; j< height ;j++){

			cl_th[j+i*height] = (float)CV_MAT_ELEM(*th,float,i,j );
			cl_ra[j+i*height] = (float)CV_MAT_ELEM(*ra,float,i,j );
		}
	}

	cvReleaseMat(&ra);
	/*El rango th calculado es de [0;2*pi], se realiza una traslacion para que rango sea 
	  desde [ -pi ; pi ]*/

	g_factor1(cl_th, cl_thR, elements);


	/*Seleccionando los valores de "ra" que estan dentro del intervalo [ d_front(0), d_front(1) ]
	  Cambiando algunos elementos de la tt segun DB*/
	dd[0] = (float)d_front[0];
	dd[1] = (float)d_front[1];

	g_selec(cl_ra, cl_tt , cl_thR, dd, elements);

	/* Antes de ordenar los angulos, se les reubicara en una sola fila */

	th1  =	 cvCreateMat(1,height*width,CV_32FC1);
	cvReleaseMat(&th);

	/* Aplicando un factor de correcion con respecto a los resultados que da Matlab, debido al valor de -pi */
	float kk = (float)-pi+(float)0.000001;
	for(i=0 ; i < width*height;i++)
	{
			if ( cl_tt[i] < kk ){
				CV_MAT_ELEM(*th1,float,0,i) = cl_tt[i] +2*(float)pi;

			} else {
				CV_MAT_ELEM(*th1,float,0,i) = cl_tt[i];

			}
	}

	/* Ordenando los elementos de la tt en forma ascendente */
	ix	 =	 cvCreateMat(1,height*width,CV_32SC1);
	T    =	 cvCreateMat(1,height*width,CV_32FC1);
	cvSort(th1,T,ix,CV_SORT_ASCENDING);

	cvReleaseMat(&th1);

	/* Calculando el valor promedio de la transformada de Fourier para cada elemento: m,	n no son cualquier valor */
	nep = (360/precision + 1);
	m = cvCreateImage(cvSize((int)nep,1), IPL_DEPTH_32F, 1);
	n = cvCreateImage(cvSize((int)nep,1), IPL_DEPTH_32F, 1);

	float *ptt = (float *)(m->imageData);
	float *ptn = (float *)(n->imageData);
	for(i=0; i < (int)nep; i++ ){
		ptt[i] = -pi+i*pi*(float)precision/180;
		ptn[i] = (float)i;
	}

	image_Re1   =   cvCreateImage( cvSize(height, width), IPL_DEPTH_32F, 1); 
	image_Im1   =   cvCreateImage( cvSize(width, height), IPL_DEPTH_32F, 1); 

	for(i=0; i < N; i++ ){
		
			/* Transformada de Fourier de la imagen */
			g_fftshift_abs_DFT(reg[i], image_Re1, image_Im1);
;
			/*Se obtiene la funcion h_A*/
			h_A[i] = cvCreateImage(cvSize((int)nep,1), IPL_DEPTH_32F, 1);
			promedio_dft(image_Re1, height, width, nep, h_A[i], m, T, ix, d, n);

	}

	cvReleaseMat(&T);
	cvReleaseMat(&ix);
	cvReleaseImage(&image_Re1);
	cvReleaseImage(&image_Im1);
	cvReleaseImage(&m);
	cvReleaseImage(&n);

	/* Especificando las variables para el calculo de la dft */
	H_AR[0] = cvCreateImage(cvSize((int)nep,1), IPL_DEPTH_32F, 1);
	H_AI[0] = cvCreateImage(cvSize((int)nep,1), IPL_DEPTH_32F, 1);

	/* Obteniendo la fft de h(f)=h_A1 para el calculo de la correlacion */
    g_DFT1(h_A[0], H_AR[0], H_AI[0]);

	/* Especificando el valor estiamdo de la primera imagen respecto de su propia imagen */
	cvSetReal2D(angle_est,0,0,0);

	for(i=1; i < N; i++ ){

			/*Obteniendo la fft de h(f)=h_A2 para el calculo de la correlacion*/
			H_AR[i] = cvCreateImage(cvSize((int)nep,1), IPL_DEPTH_32F, 1);
			H_AI[i] = cvCreateImage(cvSize((int)nep,1), IPL_DEPTH_32F, 1);
			g_DFT1Rev(h_A[i], H_AR[i], H_AI[i]);

			/*Calculo de la correlacion*/
			correlacion(H_AR[0], H_AI[0], H_AR[i], H_AI[i], precision, nep, angle_est,i);
	}


		
}


void promedio_dft(IplImage *TDF_image_Re2,int &width, int &height, float &len, IplImage *h_A, IplImage *m, CvMat *T, CvMat *ix, double &d, IplImage *n){

	int ihigh, ilow, ik, i, j;
	float prom;
	float k1, k2;

	float *cl_img;		   //Input Array

	//Numero de elementos en cada arreglo
	int elements = height*width;

	//Tamaño de los datos
	size_t datasize = sizeof(float)*elements;
	cl_img	    = (float*)malloc(datasize);

	/* Reordenando los elementos de la imagen en una sola fila  */
	for(i=0 ; i < height ;i++)
	{
		float* ptr = (float*)(TDF_image_Re2->imageData + i*TDF_image_Re2->widthStep);

		for(j=0; j< width ;j++){
			cl_img[j+i*width] = ptr[j] ;
		}
	}

	/*Calculando el valor de h(alfa) en funcion de la frecuencia, esto es necesario para hallar la correlacion */
	ilow=0;
	ihigh=0;
	ik=0;

	for(i=0 ; i < (int)len ; i++){
		ik=ilow;
		prom=0;
		k1 = ( (float)cvGetReal2D(m,0,i) - (float)d );
		k2 = ( (float)cvGetReal2D(m,0,i) + (float)d );

		while( k1 > CV_MAT_ELEM(*T,float,0,ik) ){
			ik++;
		}

		ilow = ik;

		if ( ik<ihigh ){
			ik = ihigh;
		}

		while( k2 > CV_MAT_ELEM(*T,float,0,ik) ){
			ik++;
			if ((ik > width*height) ||( CV_MAT_ELEM(*T,float,0,ik) >= 1000 )){
				break;
			}
		}	
		ihigh = ik;

		if (ihigh -1 >ilow){

			g_mean(ihigh, ilow, cl_img, elements, ix, h_A, i, n);
		}
		else {
			cvSetReal2D(h_A,0,i,0);
		}

	}


}


void mean(int ihigh, int ilow, IplImage *img, CvMat *ix, IplImage *h, int k, IplImage *n){

	/* Variables */
	int w;
	float prom;

	/* Valores iniciales */
	prom = 0;
	w = ilow;
	
	while( ihigh > w ){
		
			prom = prom + (float)(img->imageData)[ CV_MAT_ELEM(*ix,int,0,w)  ];	
			w++;
	}
	prom = prom/( (float)ihigh - (float)ilow );

	float* ptr = (float*)(h->imageData );

	ptr[ (int)cvGetReal2D(n,0,k) ] = prom;

}

void g_mean(int ihigh, int ilow, float *cl_img, int &elements, CvMat *ix, IplImage *h, int &k, IplImage *n){

	/* Variables */
	int w;
	float prom;

	/* Valores iniciales */
	prom = 0;
	w = ilow;
		
	/* Datasize */
	size_t datasize = sizeof(float)*elements;

	while( ihigh > w ){
			prom = prom + cl_img[ CV_MAT_ELEM(*ix,int,0,w)]; ;	
			w++;
	}
	prom = prom/( (float)ihigh - (float)ilow );
	
	float * ptr = (float*)(h->imageData );
	ptr[ (int)cvGetReal2D(n,0,k) ] = prom;


}


void correlacion(IplImage  *H_AR, IplImage  *H_AI, IplImage  *H_2AR, IplImage  *H_2AI, float precision, float size, CvMat *angle_est, int index){
	
	/* Variables */
	IplImage *H_CR, *H_CI;
	int 	i, len, width;
  	static const double pi = 3.14159265358979323846;

	/* Variables para la transformada inversa */
	IplImage * image_Re;
	IplImage * image_Im;

	/* Variables para el calculo via OpenCL */
	float *Re1; //Entrada
	float *Im1; //Entrada
	float *Re2; //Entrada
	float *Im2; //Entrada
	float *p11; //salida
	float *p12; //salida

	//Numero de elementos en cada arreglo
	const int elements = H_AR->width;
	width = H_AR->width;
	//Tamaño de los datos
	size_t datasize = sizeof(float)*elements;

	//Asignando las direcciones de memoria en el host
	Re1 = (float*)malloc(datasize);
	Im1 = (float*)malloc(datasize);
	Re2 = (float*)malloc(datasize);
	Im2 = (float*)malloc(datasize);
	p11 = (float*)malloc(datasize);
	p12 = (float*)malloc(datasize);

	//ingresando los valores
	for ( i=0; i<width ; i++){
		Re1[i] = (float)cvGetReal2D(H_AR,0,i);
		Im1[i] = (float)cvGetReal2D(H_AI,0,i);
		Re2[i] = (float)cvGetReal2D(H_2AR,0,i);
		Im2[i] = (float)cvGetReal2D(H_2AI,0,i);
	}

	g_calc(Re1, Im1, Re2, Im2, p11, p12, elements);

	H_CR	 = cvCreateImage( cvSize( (int)size, 1), IPL_DEPTH_32F, 1);
	H_CI	 = cvCreateImage( cvSize( (int)size, 1), IPL_DEPTH_32F, 1);
	
	for ( i=0; i<elements; i++){
		cvSetReal2D(H_CR,0,i,p11[i]);
		cvSetReal2D(H_CI,0,i,p12[i]);
	}
	free(p11);
	free(p12);
	/* Especificando las variables */
	len	= (int)(60/precision+1);

	/* Definiendo las variables para calculaar la IDFT */
	image_Re = cvCreateImage( cvSize((int)size,1), IPL_DEPTH_32F, 1); 
	image_Im = cvCreateImage( cvSize((int)size,1), IPL_DEPTH_32F, 1); 


	/* Calculo de la transformada inversa de H_C*/
	g_DFT1Inv(H_CR, H_CI, image_Re, image_Im);

	
	/* Seleccionando un intervalo en funcion de la precision */	
	MinMax(angle_est, index, precision, image_Re, len);

	/* Eliminando Variables */
	cvReleaseImage(&H_CR);
	cvReleaseImage(&H_CI);
	cvReleaseImage(&image_Re);
	cvReleaseImage(&image_Im);

}


void MinMax(CvMat *angle_est, int &index, float &precision, IplImage *image_Re,int &len){
	
	int i;

	CvPoint  minloc, maxloc;
	double   minval, maxval;
	IplImage * pf;

	pf		 = cvCreateImage( cvSize( len, 1), IPL_DEPTH_64F, 1);
	/* Seleccionando un intervalo en funcion de la precision */	
	for(i=0; i <len; i++){
		cvSetReal2D(  pf, 0, i, cvGetReal2D( image_Re,0,i+(int)(150/precision) )  );
	}

	/* Localizando el valor maximo y el angulo */	
	cvMinMaxLoc( pf, &minval, &maxval, &minloc, &maxloc, 0 );
	cvSetReal2D(angle_est ,0 ,index , (maxloc.x-30/precision)*precision ); 

	cvReleaseImage(&pf);
}



void g_fftshift_abs_DFT(IplImage *src, IplImage *dst, IplImage *dstIm){

	std::vector<cv::gpu::GpuMat>entrada(2);
	cv::gpu::GpuMat g_complexIn1(src->height,src->width,CV_32FC2);
	cv::gpu::GpuMat g_complex(src->height,src->width,CV_32FC2);

	entrada[0].upload(src);
	entrada[0].convertTo(entrada[0],CV_32FC1,1,0);

	entrada[1].create(src->height,src->width,CV_32FC1);
	entrada[1].setTo(cv::Scalar(0));


	//caculo de la transformada discreta de fourier		
	cv::gpu::merge(entrada,g_complexIn1);
	cv::gpu::dft(g_complexIn1 , g_complex , cv::Size(src->width,src->height) , CV_DXT_FORWARD);
	cv::gpu::split(g_complex,entrada);

	// Calculando la magnitud del espectro Mag = sqrt(Re^2 + Im^2) 
	cv::gpu::pow(entrada[0],2.0,entrada[0]);
	cv::gpu::pow(entrada[1],2.0,entrada[1]);
	cv::gpu::add(entrada[0],entrada[1],entrada[0]);
	cv::gpu::pow(entrada[0],0.5,entrada[0]);

	//Desplazando la frecuencia fundamental al centro
	gr_cvShiftDFT(entrada[0],entrada[0]);
	cv::gpu::GpuMat dstt(src->width,src->height,CV_32FC1);
	cv::gpu::transpose(entrada[0],dstt);


	//Copiando al formato IplImage

	ArrtoIplImageR(dstt, dst);
	ArrtoIplImageR(entrada[1], dstIm);


}

void gr_cvShiftDFT(cv::gpu::GpuMat &src_arr,cv::gpu::GpuMat &dst_arr ){

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


void factor1(CvMat *A, int i, int j){

	/* Constante pi*/
	const double pi =  3.141592653589793;

	if (  cvGetReal2D(A,i,j) < -pi + 0.0000001  ){
	cvSetReal2D( A, i, j,-cvGetReal2D(A,i,j) );
	}

	if (  cvGetReal2D(A,i,j) >  pi + 0.0000001  ){
	cvSetReal2D( A, i, j, cvGetReal2D(A,i,j)-(double)2*pi);
	}

	if (  cvGetReal2D(A,i,j) < -pi - 0.0000001 ) {
	cvSetReal2D( A, i, j, cvGetReal2D(A,i,j)+(double)2*pi);
	}

}


void g_DFT1(IplImage *src, IplImage *dst, IplImage *dstIm){

	std::vector<cv::gpu::GpuMat>entrada(2);
	cv::gpu::GpuMat g_complexIn1(src->height,src->width,CV_32FC2);
	cv::gpu::GpuMat g_complex(src->height,src->width,CV_32FC2);

	entrada[0].upload(src);
	entrada[0].convertTo(entrada[0],CV_32FC1,1,0);
	entrada[1].upload(cv::Mat::zeros(src->height,src->width,CV_32FC1));

	//caculo de la transformada discreta de fourier		
	cv::gpu::merge(entrada,g_complexIn1);
	cv::gpu::dft(g_complexIn1 , g_complex , cv::Size(src->width,src->height) , CV_DXT_FORWARD);
	cv::gpu::split(g_complex,entrada);

	//Copiando al formato IplImage
	ArrtoIplImageR(entrada[0], dst);
	ArrtoIplImageR(entrada[1], dstIm);
	
}


void g_DFT1Rev(IplImage *src, IplImage *dst, IplImage *dstIm){

	/* Variables */
	int i,w;
	IplImage * realIn1; 

	std::vector<cv::gpu::GpuMat>entrada(2);
	cv::gpu::GpuMat g_complexIn1(src->height,src->width,CV_32FC2);
	cv::gpu::GpuMat g_complex(src->height,src->width,CV_32FC2);

	/* especificando las variables */
	realIn1 = cvCreateImage( cvGetSize(src), IPL_DEPTH_32F, 1); 
	w = src->width;

	/* Invirtiendo el orden de los elementos de la matriz src */
	float* ptr = (float*)(realIn1->imageData );
	for(i=0; i<w ;i++){
			ptr[w-i-1] = (float)cvGetReal2D(src,0,i);
	}

	entrada[0].upload(realIn1);
	entrada[0].convertTo(entrada[0],CV_32FC1,1,0);

	/* Poniendo a cero la parte imaginaria de la entrada */
	entrada[1].upload(cv::Mat::zeros(src->height,src->width,CV_32FC1));

	/* Ingresando los valores de complexIn1 */
	cv::gpu::merge(entrada,g_complexIn1);

	/* Calculando la DFT de compplexIn1 */
	cv::gpu::dft(g_complexIn1 , g_complex , cv::Size(src->width,src->height) , CV_DXT_FORWARD);

    // Dividiendo la DFT en su parte real e imaginaria
	cv::gpu::split(g_complex,entrada);

	//Copiando al formato IplImage
	ArrtoIplImageR(entrada[0], dst);
	ArrtoIplImageR(entrada[1], dstIm);


	cvReleaseImage(&realIn1);
}


void g_DFT1Inv(IplImage *src, IplImage *srcIm, IplImage *dst, IplImage *dstIm){

	/* Variables */
	std::vector<cv::gpu::GpuMat>entrada(2);
	cv::gpu::GpuMat g_complexIn1(src->height,src->width,CV_32FC2);
	cv::gpu::GpuMat g_complex(src->height,src->width,CV_32FC2);

	/* Especificando las variables  */
	entrada[0].upload(src);

	entrada[1].upload(srcIm);

	/* ingresando los valores de complexIn1  */
	cv::gpu::merge(entrada,g_complexIn1);

	/* Calculando la transformada inversa de Fourier */
	cv::gpu::dft(g_complexIn1 , g_complex , cv::Size(src->width,src->height) , CV_DXT_INVERSE);

	// Dividiendo la IDFT en su parte real e imaginaria 
	cv::gpu::split(g_complex,entrada);

	//Copiando al formato IplImage
	ArrtoIplImageR(entrada[0], dst);
	ArrtoIplImageR(entrada[1], dstIm);

}

void ArrtoIplImageR(cv::Mat src, IplImage *dst){
	IplImage dst1 = src;
	cvScale(&dst1,dst,1,0);
}

void ArrtoIplImageRM(cv::Mat src, CvMat *dst){
	IplImage dst1 = src;
	cvScale(&dst1,dst,1,0);
	
}

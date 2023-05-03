#ifndef _LIBROTACION_H__
#define _LIBROTACION_H__ 
	
/* Parametros  */
#include <stdio.h>
#include <math.h>
#include "cv.h"
#include "highgui.h"	

#include <opencv2\highgui\highgui.hpp>
#include <opencv2\core\core.hpp>
#include <opencv2\gpu\gpu.hpp>

#include "LibRotacionOpenCL.h"

	void est_rotacion( IplImage *reg[], double *d_front, float precision,CvMat *angle_est, const int N, IplImage *h_A[], IplImage *H_AI[], IplImage *H_AR[] );
	

	/* Rutinas Axiliares para la est_rotacion */

	//*******************************************************************************************//
	// La funcion cvShiftDFT desplaza la frecuencia fundamental al centro de la imagen TDF       //

	void gr_cvShiftDFT(cv::gpu::GpuMat &src_arr,cv::gpu::GpuMat &dst_arr );
	//*******************************************************************************************//


	//*******************************************************************************************//
	// promedio_dft calcula el promedio de un intervalo de valores de la TDF de la imagen        //
	// TDF_imageRe2 este intervalo es denotado por el angulo que corresponde a la coordenada en  //
	// polares de un punto de la imagen															 //

	void promedio_dft(IplImage *TDF_image_Re2,int &width, int &height , float &len, 
			IplImage *h_A, IplImage *m, CvMat *T, CvMat *ix, double &d, IplImage *n);

	//*******************************************************************************************//


	//*******************************************************************************************//
	// mean sirve para calcular el promedio  de los valores ingresados en img y los almacena en h//

	void g_mean(int ihigh, int ilow, float *cl_img, int &elements, CvMat *ix, IplImage *h, int &k, IplImage *n);

	//*******************************************************************************************//



	//*******************************************************************************************//
	// correlacion, primero calcula la caorrelacion enter dos variables IplImage H_A y H_2A,     //
	// como estas funciones son de elementos complejos se ingresa a la funcion la parte real     //
	//  e imaginaria de H_A denotados por H_AR e H_AI respectivamente, analogamente para H_2A su //
	// parte real e imaginaria son H_2AR e H_2AI respectivamente.								 //
	// luego del calculo de la correlacion, se calcula el angulo estimado esnter dos imagenes    //  
	
	void correlacion(IplImage  *H_AR, IplImage  *H_AI, IplImage  *H_2AR, IplImage  *H_2AI, 
		 float precision, float size, CvMat *angle_est, int index);

	//*******************************************************************************************//



	//*******************************************************************************************//
	
	void MinMax(CvMat *angle_est, int &index, float &precision, IplImage *image_Re, int &len);

	//*******************************************************************************************//




	//*******************************************************************************************//
	// Calcula la magnitud del TDF de una imagen src desplazando la frecuencia fundamental al    //
	// con cvShiftDFT este resultado es almacenado en dst , su parte real                        //

	void g_fftshift_abs_DFT(IplImage *src, IplImage *dst, IplImage *dstIm);

	//*******************************************************************************************//

	


	//*******************************************************************************************//
	// factor1 es un factor de correccion														 //

	void factor1(CvMat *A, int i, int j);

	//*******************************************************************************************//


	//*******************************************************************************************//
	// Calcula la transformada discreta de fourier de la imagen src y el resultado es alamacenado//
	// en su parte real e imaginaria en dst e dstIm												 //

	void g_DFT1(IplImage *src, IplImage *dst, IplImage *dstIm);

	//*******************************************************************************************//




	//*******************************************************************************************//
	// Calcula la transformada discreta de fourier de la imagen src alterando el orden de sus    //
	// elementos del ukltimo elemento al primer elemento del penultimo elemento al segundo		 //
	// elemento... y el resultado es alamacenado en su parte real e imaginaria en dst e dstIm    //	

	void g_DFT1Rev(IplImage *src, IplImage *dst, IplImage *dstIm);

	//*******************************************************************************************//




	//*******************************************************************************************//
	// Calcula la transformada inversa de fourier de la imagen src alamcenando la parte real e   //
	// imaginaria en dst e dstIm

	void g_DFT1Inv(IplImage *src, IplImage *srcIm, IplImage *dst, IplImage *dstIm);

	//*******************************************************************************************//



	//*******************************************************************************************//

	void ArrtoIplImageR(cv::Mat src, IplImage *dst);

	//*******************************************************************************************//


	void ArrtoIplImageRM(cv::Mat src, CvMat *dst);


#endif
#ifndef _LIBDESPLAZAMIENTO_H_
#define _LIBDESPLAZAMIENTO_H_

#include <stdio.h>
#include <math.h>
#include "cv.h"
#include "highgui.h"	

#include <opencv2\highgui\highgui.hpp>
#include <opencv2\core\core.hpp>
#include <opencv2\gpu\gpu.hpp>

#include "LibDesplazamientoOpenCL.h"

	void estimacion_desplazamiento( IplImage *reg[], int n, CvMat *delta_est , const int N);
	

	//*******************************************************************************************//
	// Calcula la transformada de fourier de la imagen src alamcenando la parte real e           //
	// imaginaria en dst e dstIm																 //

	void DFT(IplImage *src, IplImage *dst, IplImage *dstIm);
	//*******************************************************************************************//
	
	void g_DFT(IplImage *src, cv::gpu::GpuMat &dst, cv::gpu::GpuMat &dstIm);

	//*******************************************************************************************//
	// prueba calcula la solucion a un sistema lineal de ecuacion Ax=B mediante el metodo de     //
	// minimos cudrados																			 //

	void prueba(float *x, float *y, CvMat *v ,int beginx, int endx, int beginy, int endy, 
		int width, int height, CvMat *delta_est, int index);
	//*******************************************************************************************//



	//*******************************************************************************************//
	// En base a la solucion de Ax=B se calcula el desplazamiento estimado x e y				 //

	void solucion(CvMat *Xr,int width,int height, CvMat *delta_est, int index);
	//*******************************************************************************************//



	//*******************************************************************************************//
	// Se crea un array de elementos que se almacenan en x e y en base a mx, my, begin x, beginy //
	// x sirve para especificar la posicion en el eje x de cada pixel en una imagen              //
	// y sirve para especificar la posicion en el eje y de cada pixel en una imagen				 //

	void ventanaxy(float *x, float *y, int &mx, int &my, int &beginx, int &beginy);
	//*******************************************************************************************//



	//*******************************************************************************************//
	// cvShiftDFT desplaza el valor de la frecuencia fundamental en el centro de la imagen DFT   //

	void g_cvShiftDFT(cv::gpu::GpuMat &src_arr,cv::gpu::GpuMat &dst_arr );
	//*******************************************************************************************//


	//*******************************************************************************************//
	// Permite calcular el angulo de desfase entre dos matricez cuyos elementos son complejos    //

	void g_angulo(IplImage *image_Im1, IplImage *image_Re1, IplImage *image_Im2, 
		IplImage *image_Re2, CvMat *Ang);
	//*******************************************************************************************//


	//*******************************************************************************************//
	// Selecciona una submatriz de imagen Ang en base a beginx, beginy, mx, my que espcifican las//
	// dimensiones de esta submatriz															 //

	void matriz_v(int beginx, int beginy, CvMat *Ang, int mx, int my, CvMat *B);
	//*******************************************************************************************//


	void ArrtoIplImageD(cv::Mat src, IplImage *dst);

	

#endif
#ifndef _LIBITERACIONES_H__
#define _LIBITERACIONES_H__ 
	
/* Parametros  */
#include <stdio.h>
#include <math.h>
#include "cv.h"
#include "highgui.h"	

#include <opencv2\highgui\highgui.hpp>
#include <opencv2\core\core.hpp>
#include <opencv2\gpu\gpu.hpp>

#include "LibIteracionesOpenCL.h"

#include "DICOM_IO.h"


	//*******************************************************************************************//
	// Convierte una imagen ref a otro array dst cuyos elementos estan en formato double         //

	void F8to64f(IplImage *ref,IplImage *dst);
	//*******************************************************************************************//


	//*******************************************************************************************//
	// Convierte una imagen ref a otro array dst cuyos elementos estan en formato double         //

	void F64to8f(IplImage *ref,IplImage *dst);
	//*******************************************************************************************//




	//*******************************************************************************************//
	// Convierte una imagen ref a otro array dst cuyos elementos estan en formato double         //

	void F16to64f(IplImage *ref,IplImage *dst);
	//*******************************************************************************************//




	//*******************************************************************************************//
	// Desarrolla una imagen en superesolucion a partir de un registro de imagen ref[] y los     //
	// parametros de estimacion ya calculados delta_est y phi_est							     //
	// El tamaño de la imagen en superesolucion esta definido por la variable factor			 //

	void IterativeBackProjecting(IplImage *ref[],IplImage *delta_est, IplImage *phi_est, double factor, const int N);
	//*******************************************************************************************//



	//*******************************************************************************************//
	// Se calcula el desplazamiento de una imagen src en y pixeles eje vertical y luego x pixels //
	// en el eje horizontal este resultado se almacena en la variable dst						 //

	void g_desplazamiento2dim(cv::gpu::GpuMat &src,cv::gpu::GpuMat &dst ,double y, double x);
	//*******************************************************************************************//



	//*******************************************************************************************//
	// Rota la imagen src en un angulo definido por angle, si es positivo el sentido de giro es  //
	// horario si es negativo el sentido de giro es antihorario									 //

	void rotarImagen(IplImage *src, IplImage *dst ,double angle);
	//*******************************************************************************************//

	void g_rotarImagen(cv::gpu::GpuMat &src, cv::gpu::GpuMat &dst ,double angle);

	void gc_rotarImagen(cv::gpu::GpuMat &src, float *out ,double angle);
	void gc_rotarImagen1(float *d, cv::gpu::GpuMat &dst ,double angle);

	//*******************************************************************************************//
	// Se filtra la imagen en el dominio de las frecuencias mediante una matriz kernel           //
	// si n=1 el filtro es un blur  															 //
	// si n=2 el filtro es un sharpen															 //

	void g_filtro_blur(cv::gpu::GpuMat &src, cv::gpu::GpuMat &dst, int n);

	
	void gc_filtro_blur( float *d, int &width, int &height, float *out , int n);


	void g_filtro_sharpen( cv::gpu::GpuMat &src, cv::gpu::GpuMat &dst , int n);

	void gc_filtro_sharpen( cv::Mat temp, int &width, int &height, float *out , int n);
	//*******************************************************************************************//


	//*******************************************************************************************//
	// Restamos todos los elementos de una imagen src en un valor dado por x					 //

	void desp_x(IplImage *src,IplImage *dst, double x);
	//*******************************************************************************************//



	//*******************************************************************************************//
	// Multiplicamos los elementos de una imagen src en un factor K y se almacena en dst		 //

	void M_scalar(IplImage *src,IplImage *dst, double K);
	//*******************************************************************************************//



	//*******************************************************************************************//
	// Calcula el valor maximo de los valores singulares de la imagen X, este valor maximo es    //
	// almacenado  en m																			 //

	void SVD_DC(IplImage *X, IplImage *m, int j);
	//*******************************************************************************************//

	void g_SVD_DC(cv::Mat &X, IplImage *m, int j);

	//*******************************************************************************************//
	// Simula el proceso de los diversos fenomenos que hacen que la imagen baje de calidad y se  //
	// obtenga una imagen en poca resolucion													 //

	void g_iterim(double factor, IplImage *delta_est, IplImage *phi_est, std::vector<cv::gpu::GpuMat> &g_ref, cv::gpu::GpuMat &g_G, cv::gpu::GpuMat &g_X, const int N, double lambda, int i, int w, int h);	//*******************************************************************************************//



	//*******************************************************************************************//
	// disminuye el tamaño de una imagen A en un factor este resultado se alamacena en t		 //

	void g_reducirmatriz(cv::gpu::GpuMat &A, cv::gpu::GpuMat &t, double &factor);
	//*******************************************************************************************//
	void gc_reducirmatriz(float *in, float * B,int &W, int &H, cv::Mat &ou, double &factor);

	void gc_resize(cv::Mat &temp1 , cv::Mat &temp);

	void gc_integral(cv::gpu::GpuMat &src, float * B,int &W, int &H, float *out, double &factor,double angle);

	void gc_integral2(float *in, int &wL, int &hL ,cv::gpu::GpuMat g_temp, double angle);
	//*******************************************************************************************//
	// Muestra el valor del PSNR entre dos imagenes dadas										 //

	void gc_iterar(cv::gpu::GpuMat &src, float * B,int &wH, int &hH,int &wL, int &hL, double &factor, double &angle, cv::gpu::GpuMat &dst);


	void psnr(IplImage *src1, IplImage *src2);
	//*******************************************************************************************//


	//*******************************************************************************************//
	// genera un filtro pasabajo de 2 dimensiones que se emplea para eliminar el aliasing		 //
	// en las imagenes del registro																 //

	void tukeywin(IplImage *dst, double alfa);
	//*******************************************************************************************//


	void ArrtoIplImage(cv::Mat src, IplImage *dst);



	void delta_tiempo(SYSTEMTIME &t1, SYSTEMTIME &t2);


#endif
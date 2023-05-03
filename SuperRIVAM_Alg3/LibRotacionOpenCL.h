#ifndef _LIBROTACIONOPENCL_H__
#define _LIBROTACIONOPENCL_H__ 
#include <CL\cl.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <opencv\cv.h>


void g_factor1(float* cl_th, float *cl_thR, int elements);
	
void g_selec(float* cl_ra, float *cl_tt , float *cl_th, float *d_front, int elements);

void g_reordenacion(float *cl_image_Re, float *cl_img, int elements);

void g_calc(float *Re1, float *Im1, float *Re2,  float *Im2, float *p11, float *p12, int elements);


#endif
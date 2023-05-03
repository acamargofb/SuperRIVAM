#ifndef _LIBDESPLAZAMIENTOOPENCL_H_
#define _LIBDESPLAZAMIENTOOPENCL_H_

#include <CL\cl.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

void g_tang(float *cl_Re1, float *cl_Im1, float *cl_Re2, float *cl_Im2, float *cl_R, int elements);

#endif
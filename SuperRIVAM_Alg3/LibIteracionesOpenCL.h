#ifndef _LIBITERACIONESOPENCL_H__
#define _LIBITERACIONESOPENCL_H__ 
#include <CL\cl.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <opencv\cv.h>
#include <opencv\highgui.h>


void OpenClRotar(float *cl_input, float *cl_output, int &W, int &H, float &theta,int &elements);

int roundUp(int value, int multiple);

void OpenClBlurr(float *cl_input, float *cl_output, int &W, int &H, float cl_filter[9],int &elements);

void OpenClSuma(float *cl_input, float *cl_input2, float *cl_output, int &elements);

void OpenClRotarBlurr(float *cl_input, float *cl_output, int &W, int &H, float cl_filter[9], float &theta,int &elements);

void OpenCLResize(float *cl_inputlow, float * cl_outputhigh, int &Wl,int &Hl, int &Wh, int &Hh, int &elemH, int &elemL);

void OpenCLReducir(float *cl_inputhigh, float * cl_outputlow, int &x,int &y, int &Wh, int &Hh, int &elemH, int &elemL, int &f);

void OpenCLReBlRo(float *cl_in, float *cl_out, float &theta, float cl_filter[9],  int &Wl,int &Hl, int &Wh, int &Hh, int &elemH, int &elemL );

void OpenCLRoBlReSu( float *cl_in, float *cl_in2,float *cl_out, int &wH, int &hH, float cl_filter[9], float &theta, int &x, int &y,  int &f);

void Cl_iteraciones( float *cl_in, float *cl_in2,float *cl_out,  int &wL,int &hL, int &wH, int &hH, float cl_filter1[9], float cl_filter[9], float &theta1, float &theta2, int &f);


#endif
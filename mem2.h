#ifndef __MEM2_H__
#define __MEM2_H__

#include <SR2.h>

float **AllocFPArray(int width,
				  int height)
{
	int i;
	float **its;

	its = (float **) malloc(height*sizeof(float *));
	if (its==(float **)0) {
		fprintf(stderr, "Out of memory allocating an array.\n");
	}
	its[0] = (float *)malloc(width*height*sizeof(float));
	if (its[0]==(float *)0) {
		fprintf(stderr, "Out of memory allocating an array.\n");
		exit(0);
	}
	for (i=1; i<height; i++)
		its[i] = &(its[0][i*width]);
	return its;
}

void FreeFPArray(float  **its,
				 int height)
{
	free(its[0]);
	free(its);
}

PlaneFP *AllocPlaneFP(int width,int height)
{
	PlaneFP *its;
	its = (PlaneFP *)malloc(sizeof(PlaneFP));
	if (its==(PlaneFP *)0)
		fprintf(stderr, "Out of memory allocating a gray array.\n");

	its->p = AllocFPArray(width, height);

	its->width  = width;
	its->height = height;

	return its;
} 

void FreePlaneFP(PlaneFP *its)
{
	FreeFPArray(its->p, its->height);
	free(its);
}
#endif

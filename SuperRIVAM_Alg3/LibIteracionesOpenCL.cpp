#include "LibIteracionesOpenCL.h"



const char* rot =
"__kernel \n"
"void rotar(__global float *p_in, \n"
"  int p_W,   \n "
"  int p_H,   \n "
"  float cosTheta,   \n "
"  float sinTheta,   \n "
" __global float *p_out)   \n "
"{ \n"
" \n"
" // Definiendo los indices \n"
" int idx = get_global_id(0); \n"
" int idy = get_global_id(1); \n"
" float cx = (float)(p_W/2); \n"
" float cy = (float)(p_H/2); \n"
" // Lugar de los datos a moverse \n"
" float xpos = ((float)idx - cx )*cosTheta + ((float)idy - cy)*sinTheta + cx; \n"
" float ypos = -1.0*((float)idx - cx )*sinTheta + ((float)idy - cy)*cosTheta + cy; \n"
" // Revisando en la frontera \n"
" if( ((int)xpos>=0)&&((int)xpos<p_W)&&((int)ypos>=0)&&((int)ypos<p_H)){ \n"
" p_out[((int)ypos)*p_W+(int)xpos]=p_in[idy*p_W +idx]; } \n"
"} \n"
;

const char* c_blurr =
"__kernel								\n"
"void blurring(__global float *p_in,	\n "
"  int cols,							\n "
"  int rows,							\n "
"  int filterWidth,	   					\n "
" __constant float *filter,				\n "
" __global float *p_out)				\n "
"{										\n"
"										\n"
" // Definiendo los indices				\n"
" int column = get_global_id(0);		\n"
" int row = get_global_id(1);			\n"
" int halfWidth = (int)(filterWidth/2);	\n"
" float sum = 0.0f;						\n"
" // Cada work-item itera alrededor		\n"
" int cx;								\n"
" int cy;								\n"
" if( (row>0&&row<rows)&&(column>0&&column<cols) ){		\n"
" int Id = 0;											\n"
" for(int i = -halfWidth; i<=halfWidth; i++){			\n"
"	cy=row+i;											\n"
"	for(int j = -halfWidth; j<=halfWidth; j++){			\n"
"		cx=column+j;									\n"
"		sum =sum+ p_in[cy*cols+cx]*filter[Id++];		\n"
"	}													\n"
" }														\n"
" // Copiando los datos de salida						\n"
"														\n"
" p_out[row*cols+column]=sum;							\n"
" }else{												\n"
"	p_out[row*cols+column]=p_in[row*cols+column];		\n"
"		}												\n"
"}														\n"
;


const char* ch_sumar =
"__kernel \n"
"void sumar(__global float *p_A, \n"
" __global float *p_B,			 \n "
" __global float *p_out)		 \n "
"{								 \n"
" int idx = get_global_id(0);	 \n"
" p_out[idx] = p_A[idx] - p_B[idx]; \n"
"}								 \n"
;

const char* ch_sumar1 =
"__kernel \n"
"void sumar1(__global float *p_A, \n"
" __global float *p_B,			 \n "
"  int p_W,   \n "
" __global float *p_out)		 \n "
"{								 \n"
" int idx = get_global_id(0);	 \n"
" int idy = get_global_id(1);	 \n"
" p_out[idy*p_W + idx] = p_A[idy*p_W + idx] - p_B[idy*p_W + idx]; \n"
"}								 \n"
;

const char* ch_interpolar = 
"__kernel \n"
"void res(__global float *p_in, \n"
"  int p_Wl,   \n"
"  int p_Hl,   \n"
"  int p_Wh,   \n"
"  int p_Hh,   \n"
" __global float *p_out)      \n"
"{			\n"
" // Definiendo los indices   \n"
" int idx = get_global_id(0); \n"
" int idy = get_global_id(1); \n"
" int j = idx*p_Wl/p_Wh; \n"
" int i = idy*p_Hl/p_Hh; \n"
" p_out[idy*p_Wh + idx] = p_in[i*p_Wl + j];	\n"
" \n"
" }	\n"
;


const char* ch_intercubic = 
"__kernel 	\n"
"void rescubic(__global float *p_in, 	\n"
"  int p_Wl,   	\n"
"  int p_Hl,   	\n"
"  int p_Wh,   	\n"
"  int p_Hh,   	\n"
" __global float *p_out)      	\n"
"{				\n"
" // Definiendo los indices  	\n"
" int idx = get_global_id(0);	\n"
" int idy = get_global_id(1); 	\n"
" int j = (int)(idx*p_Wl/p_Wh);	 	\n"
" int i = (int)(idy*p_Hl/p_Hh); 		\n"
" float Sx = (float)idx*(float)p_Wl/(float)p_Wh - (float)(j);			\n"
" float Sy = (float)idy*(float)p_Hl/(float)p_Hh - (float)(i);			\n"
" //Introduciendo caracteriticas de asimetria local	\n"
" 	\n"
" if(   (i+1<= p_Hl) && (i>=0) && (j+1<= p_Wl) && (j>=0)   ){\n"
" //Pesos eje y o i'			\n"
" float w00 = (1-Sx)*(1-Sy);	\n"
" float w01 = (1-Sx)*Sy;	\n"
" float w10 = Sx*(1-Sy); 	\n"
" float w11 = (Sx*Sy);		\n"
" float ww = w00 + w01 + w10 + w11;	\n"
" w00 = w00 / ww ;	\n"
" w01 = w01 / ww;		\n"
" w10 = w10 / ww; 	\n"
" w11 = w11 / ww;		\n"
" 	\n"
" 	\n"
" 	\n"
" float s; 	\n"
" float s1;	\n"
" float s2; \n"
" float s3;	\n"
"	\n"
" s = w00*p_in[(i)*p_Wl+(j)];	\n"
"	\n"
" s1 =w01*p_in[(i+1)*p_Wl+(j)]; 	\n"
"	\n"
" s2 =w10*p_in[(i)*p_Wl+(j+1)]; 	\n"
"	\n"
" s3 =w11*p_in[(i+1)*p_Wl+(j+1)]; 	\n"
"	\n"
"    p_out[idy*p_Wh + idx] = s +	\n"
"	    s1 + s2 + s3;	\n"
"	\n"
" 	} else { \n"
"	p_out[idy*p_Wh + idx]= p_in[i*p_Wl + j];   \n"
"	} // fin de else\n"
" }	\n"
;	


const char* ch_reducir = 
"__kernel 			\n"
"void reducir(__global float *p_in,		\n"
"  int p_x,   		\n"
"  int p_wH,   		\n"
"  int f,  			\n"
" __global float *p_out)      	\n"
"{								\n"
" // Definiendo los indices   	\n"
" int idx = get_global_id(0); 	\n"
" int idy = get_global_id(1); 	\n"
" p_out[idy*p_x + idx] = p_in[idy*p_wH*f + idx*f];	\n"
"}		\n"	
;


void OpenClRotar(float *cl_input, float *cl_output, int &W, int &H, float &theta,int &elements){

float cos_theta = cos(theta);
float sin_theta = sin(theta);

// tamaño de datos
size_t datasize = sizeof(float)*elements;

//Se crea la variable para revisar la salida de cada 
cl_int status;



//------------------------------------------------
//Paso1: Encontrar e inicializar las plataformas
//------------------------------------------------
cl_uint numPlatforms = 0;
cl_platform_id *platforms = NULL;
clGetPlatformIDs(0,NULL,&numPlatforms);
platforms = (cl_platform_id*)malloc(numPlatforms*sizeof(cl_platform_id));
status = clGetPlatformIDs(numPlatforms,platforms,NULL);



//------------------------------------------------
//Paso2: Encontrar e inicializar los dispositivos
//------------------------------------------------
cl_uint numDevices = 0;
cl_device_id *devices = NULL;
status = clGetDeviceIDs(platforms[0],CL_DEVICE_TYPE_ALL,0,NULL,&numDevices);
devices = (cl_device_id*)malloc(numDevices*sizeof(cl_device_id));
status = clGetDeviceIDs(platforms[0],CL_DEVICE_TYPE_ALL, numDevices, devices, NULL);



//------------------------------------------------
//Paso3: Creando un contexto
//------------------------------------------------
cl_context context = NULL;
context = clCreateContext(NULL,numDevices,devices,NULL,NULL,&status);



//------------------------------------------------
//Paso4: Creando un comandqueue
//------------------------------------------------
cl_command_queue cmdQueue;
cmdQueue = clCreateCommandQueue(context,devices[0],0,&status);



//------------------------------------------------
//Paso5: Creando un device buffers
//------------------------------------------------
cl_mem bufferin = clCreateBuffer(context,CL_MEM_READ_ONLY,datasize,NULL,&status);
cl_mem bufferout = clCreateBuffer(context,CL_MEM_WRITE_ONLY,datasize,NULL,&status);



//---------------------------------------------------
//Paso6: Escribiendo datos del host al device buffers
//---------------------------------------------------
// Use clEnqueueWriteBuffer() to write input array Re1 to
// the device buffer bufferB
status = clEnqueueWriteBuffer(cmdQueue,	bufferin,CL_TRUE,0,datasize,(void *)cl_input,0,NULL,NULL);



//-----------------------------------------
//Paso7: Creando y conmpilando programas
//-----------------------------------------
cl_program program = clCreateProgramWithSource(context,1,(const char**)&rot,NULL,&status);
// Build (compile) the program for the devices with
// clBuildProgram()
status = clBuildProgram(program,numDevices,devices,NULL,NULL,NULL);



//-----------------------------------------
//Paso8: Creando un kernel
//-----------------------------------------
cl_kernel kernel =  clCreateKernel(program, "rotar", &status);



//------------------------------------------------
//Paso9: Estableciendo los argumentos del kernell
//------------------------------------------------
status   = clSetKernelArg(kernel, 0, sizeof(cl_mem)   , (void *)&bufferin);
status  |= clSetKernelArg(kernel, 1, sizeof(cl_int)   , (void *)&W);
status  |= clSetKernelArg(kernel, 2, sizeof(cl_int)   , (void *)&H);
status  |= clSetKernelArg(kernel, 3, sizeof(cl_float) , (void *)&cos_theta);
status  |= clSetKernelArg(kernel, 4, sizeof(cl_float) , (void *)&sin_theta);
status  |= clSetKernelArg(kernel, 5, sizeof(cl_mem)   , (void *)&bufferout);



//------------------------------------------------
//Paso10: Configurando el wirk-item structure
//------------------------------------------------
size_t globalWorkSize[2] = {W,H};
size_t localws[2] = {10,10};


//———————————————————————————————————————————————————
// STEP 11: Enqueue the kernel for execution
//———————————————————————————————————————————————————
status = clEnqueueNDRangeKernel(cmdQueue,kernel,2,NULL,globalWorkSize,0,0,NULL,NULL);


//------------------------------------------------
// Paso 12: Leyendo el buffer de salida de regreso al host
//------------------------------------------------
clEnqueueReadBuffer(cmdQueue,bufferout,CL_TRUE,0,datasize,(void *)cl_output,0,NULL,NULL);
// Verify the output

//———————————————————————————————————————————————————
// STEP 13: Release OpenCL resources
//———————————————————————————————————————————————————
// Free OpenCL resources
clReleaseKernel(kernel);
clReleaseProgram(program);
clReleaseCommandQueue(cmdQueue);
clReleaseMemObject(bufferin);
clReleaseMemObject(bufferout);
clReleaseContext(context);
// Free host resources
free(cl_input);
free(platforms);
free(devices);


}








int roundUp(int value, int multiple){

	int remainder = value%multiple;

	if(remainder != 0 ){
		value += (multiple - remainder);
	}

	return value;
}




void OpenClBlurr(float *cl_input, float *cl_output, int &W, int &H, float cl_filter[9],int &elements){
 

// tamaño de datos
size_t datasize = sizeof(float)*elements;
size_t dataf = sizeof(float)*9;
cl_int filterW = 3;
cl_int paddingPixels = (int)(3/2)*2;

//Se crea la variable para revisar la salida de cada 
cl_int status;


//------------------------------------------------
//Paso1: Encontrar e inicializar las plataformas
//------------------------------------------------
cl_uint numPlatforms = 0;
cl_platform_id *platforms = NULL;
clGetPlatformIDs(0,NULL,&numPlatforms);
platforms = (cl_platform_id*)malloc(numPlatforms*sizeof(cl_platform_id));
status = clGetPlatformIDs(numPlatforms,platforms,NULL);



//------------------------------------------------
//Paso2: Encontrar e inicializar los dispositivos
//------------------------------------------------
cl_uint numDevices = 0;
cl_device_id *devices = NULL;
status = clGetDeviceIDs(platforms[0],CL_DEVICE_TYPE_ALL,0,NULL,&numDevices);
devices = (cl_device_id*)malloc(numDevices*sizeof(cl_device_id));
status = clGetDeviceIDs(platforms[0],CL_DEVICE_TYPE_ALL, numDevices, devices, NULL);



//------------------------------------------------
//Paso3: Creando un contexto
//------------------------------------------------
cl_context context = NULL;
context = clCreateContext(NULL,numDevices,devices,NULL,NULL,&status);



//------------------------------------------------
//Paso4: Creando un comandqueue
//------------------------------------------------
cl_command_queue cmdQueue;
cmdQueue = clCreateCommandQueue(context,devices[0],0,&status);



//------------------------------------------------
//Paso5: Creando un device buffers
//------------------------------------------------
cl_mem bufferFilter = clCreateBuffer(context,CL_MEM_READ_ONLY,dataf,NULL,NULL);
cl_mem bufferin		= clCreateBuffer(context,CL_MEM_READ_ONLY,datasize,NULL,NULL);
cl_mem bufferout	= clCreateBuffer(context,CL_MEM_READ_WRITE,datasize,NULL,NULL);



//---------------------------------------------------
//Paso6: Escribiendo datos del host al device buffers
//---------------------------------------------------
// Use clEnqueueWriteBuffer() to write input array Re1 to
// the device buffer bufferB
status = clEnqueueWriteBuffer(cmdQueue,	bufferin,CL_FALSE,0,datasize,cl_input,0,NULL,NULL);
status = clEnqueueWriteBuffer(cmdQueue,	bufferFilter,CL_FALSE,0,dataf,cl_filter,0,NULL,NULL);



//-----------------------------------------
//Paso7: Creando y conmpilando programas
//-----------------------------------------
cl_program program = clCreateProgramWithSource(context,1,(const char**)&c_blurr,NULL,&status);
// Compilando el programa con el dispositivo con clBuildProgram()
status = clBuildProgram(program,numDevices,devices,NULL,NULL,NULL);



//-----------------------------------------
//Paso8: Creando un kernel
//-----------------------------------------
cl_kernel kernel =  clCreateKernel(program, "blurring", &status);



//------------------------------------------------
//Paso9: Estableciendo los argumentos del kernell
//------------------------------------------------
status   = clSetKernelArg(kernel, 0, sizeof(cl_mem)	  , (void *)&bufferin);
status  |= clSetKernelArg(kernel, 1, sizeof(cl_int)   , (void *)&W);
status  |= clSetKernelArg(kernel, 2, sizeof(cl_int)   , (void *)&H);
status  |= clSetKernelArg(kernel, 3, sizeof(cl_int)   , (void *)&filterW);
status  |= clSetKernelArg(kernel, 4, sizeof(cl_mem)   , (void *)&bufferFilter);
status  |= clSetKernelArg(kernel, 5, sizeof(cl_mem)	  , (void *)&bufferout);



//------------------------------------------------
//Paso10: Configurando el wirk-item structure
//------------------------------------------------
size_t globalWorkSize[2] = {W,H};
size_t localws[2] = {10,10};


//———————————————————————————————————————————————————
// STEP 11: Enqueue the kernel for execution
//———————————————————————————————————————————————————
status = clEnqueueNDRangeKernel(cmdQueue,kernel,2,NULL,globalWorkSize,0,0,NULL,NULL);


//------------------------------------------------
// Paso 12: Leyendo el buffer de salida de regreso al host
//------------------------------------------------
clEnqueueReadBuffer(cmdQueue,bufferout,CL_TRUE,0,datasize,(void *)cl_output,0,NULL,NULL);
// Verify the output

//———————————————————————————————————————————————————
// STEP 13: Release OpenCL resources
//———————————————————————————————————————————————————
// Free OpenCL resources
clReleaseKernel(kernel);
clReleaseProgram(program);
clReleaseCommandQueue(cmdQueue);
clReleaseMemObject(bufferin);
clReleaseMemObject(bufferFilter);
clReleaseMemObject(bufferout);
clReleaseContext(context);
// Free host resources
free(cl_input);


free(platforms);
free(devices);


}





void OpenClSuma(float *cl_input, float *cl_input2, float *cl_output, int &elements){



// tamaño de datos
size_t datasize = sizeof(float)*elements;

//Se crea la variable para revisar la salida de cada 
cl_int status;



//------------------------------------------------
//Paso1: Encontrar e inicializar las plataformas
//------------------------------------------------
cl_uint numPlatforms = 0;
cl_platform_id *platforms = NULL;
clGetPlatformIDs(0,NULL,&numPlatforms);
platforms = (cl_platform_id*)malloc(numPlatforms*sizeof(cl_platform_id));
status = clGetPlatformIDs(numPlatforms,platforms,NULL);



//------------------------------------------------
//Paso2: Encontrar e inicializar los dispositivos
//------------------------------------------------
cl_uint numDevices = 0;
cl_device_id *devices = NULL;
status = clGetDeviceIDs(platforms[0],CL_DEVICE_TYPE_ALL,0,NULL,&numDevices);
devices = (cl_device_id*)malloc(numDevices*sizeof(cl_device_id));
status = clGetDeviceIDs(platforms[0],CL_DEVICE_TYPE_ALL, numDevices, devices, NULL);



//------------------------------------------------
//Paso3: Creando un contexto
//------------------------------------------------
cl_context context = NULL;
context = clCreateContext(NULL,numDevices,devices,NULL,NULL,&status);


//------------------------------------------------
//Paso4: Creando un comandqueue
//------------------------------------------------
cl_command_queue cmdQueue;
cmdQueue = clCreateCommandQueue(context,devices[0],0,&status);



//------------------------------------------------
//Paso5: Creando un device buffers
//------------------------------------------------
cl_mem bufferin  = clCreateBuffer(context,CL_MEM_READ_ONLY,datasize,NULL,&status);
cl_mem bufferin2 = clCreateBuffer(context,CL_MEM_READ_ONLY,datasize,NULL,&status);
cl_mem bufferout = clCreateBuffer(context,CL_MEM_WRITE_ONLY,datasize,NULL,&status);



//---------------------------------------------------
//Paso6: Escribiendo datos del host al device buffers
//---------------------------------------------------
// Use clEnqueueWriteBuffer() to write input array Re1 to
// the device buffer bufferB
status = clEnqueueWriteBuffer(cmdQueue,	bufferin  ,CL_FALSE ,0 ,datasize ,cl_input  ,0 ,NULL,NULL);
status = clEnqueueWriteBuffer(cmdQueue,	bufferin2 ,CL_FALSE ,0 ,datasize ,cl_input2 ,0 ,NULL,NULL);



//-----------------------------------------
//Paso7: Creando y conmpilando programas
//-----------------------------------------
cl_program program = clCreateProgramWithSource(context,1,(const char**)&ch_sumar,NULL,&status);
// Build (compile) the program for the devices with
// clBuildProgram()
status = clBuildProgram(program,numDevices,devices,NULL,NULL,NULL);



//-----------------------------------------
//Paso8: Creando un kernel
//-----------------------------------------
cl_kernel kernel =  clCreateKernel(program, "sumar", &status);



//------------------------------------------------
//Paso9: Estableciendo los argumentos del kernell
//------------------------------------------------
status   = clSetKernelArg(kernel, 0, sizeof(cl_mem)   , &bufferin);
status  |= clSetKernelArg(kernel, 1, sizeof(cl_mem)   , &bufferin2);
status  |= clSetKernelArg(kernel, 2, sizeof(cl_mem)   , &bufferout);




//------------------------------------------------
//Paso10: Configurando el wirk-item structure
//------------------------------------------------
size_t globalWorkSize[1];
globalWorkSize[0] = elements;


//———————————————————————————————————————————————————
// STEP 11: Enqueue the kernel for execution
//———————————————————————————————————————————————————
status = clEnqueueNDRangeKernel(cmdQueue,kernel,1,NULL,globalWorkSize,0,0,NULL,NULL);


//------------------------------------------------
// Paso 12: Leyendo el buffer de salida de regreso al host
//------------------------------------------------
clEnqueueReadBuffer(cmdQueue,bufferout,CL_TRUE,0,datasize,cl_output,0,NULL,NULL);
// Verify the output

//———————————————————————————————————————————————————
// STEP 13: Release OpenCL resources
//———————————————————————————————————————————————————
// Free OpenCL resources
clReleaseKernel(kernel);
clReleaseProgram(program);
clReleaseCommandQueue(cmdQueue);
clReleaseMemObject(bufferin);
clReleaseMemObject(bufferin2);
clReleaseMemObject(bufferout);
clReleaseContext(context);

// Free host resources
free(cl_input);

free(platforms);
free(devices);

}





void OpenClRotarBlurr(float *cl_input, float *cl_output, int &W, int &H, float cl_filter[9], float &theta,int &elements){

float cos_theta = cos(theta);
float sin_theta = sin(theta);


size_t dataf = sizeof(float)*9;
cl_int filterW = 3;
cl_int paddingPixels = (int)(3/2)*2;

// tamaño de datos
size_t datasize = sizeof(float)*elements;

//Se crea la variable para revisar la salida de cada 
cl_int status;



//------------------------------------------------
//Paso1: Encontrar e inicializar las plataformas
//------------------------------------------------
cl_uint numPlatforms = 0;
cl_platform_id *platforms = NULL;
clGetPlatformIDs(0,NULL,&numPlatforms);
platforms = (cl_platform_id*)malloc(numPlatforms*sizeof(cl_platform_id));
status = clGetPlatformIDs(numPlatforms,platforms,NULL);



//------------------------------------------------
//Paso2: Encontrar e inicializar los dispositivos
//------------------------------------------------
cl_uint numDevices = 0;
cl_device_id *devices = NULL;
status = clGetDeviceIDs(platforms[0],CL_DEVICE_TYPE_ALL,0,NULL,&numDevices);
devices = (cl_device_id*)malloc(numDevices*sizeof(cl_device_id));
status = clGetDeviceIDs(platforms[0],CL_DEVICE_TYPE_ALL, numDevices, devices, NULL);



//------------------------------------------------
//Paso3: Creando un contexto
//------------------------------------------------
cl_context context = NULL;
context = clCreateContext(NULL,numDevices,devices,NULL,NULL,&status);



//------------------------------------------------
//Paso4: Creando un comandqueue
//------------------------------------------------
cl_command_queue cmdQueue;
cmdQueue = clCreateCommandQueue(context,devices[0],0,&status);



//------------------------------------------------
//Paso5: Creando un device buffers
//------------------------------------------------
cl_mem bufferFilter = clCreateBuffer(context,CL_MEM_READ_ONLY,dataf,NULL,NULL);
cl_mem bufferin     = clCreateBuffer(context,CL_MEM_READ_ONLY,datasize,NULL,&status);
cl_mem buffertemp   = clCreateBuffer(context,CL_MEM_READ_WRITE,datasize,NULL,&status);
cl_mem bufferout    = clCreateBuffer(context,CL_MEM_WRITE_ONLY,datasize,NULL,&status);



//---------------------------------------------------
//Paso6: Escribiendo datos del host al device buffers
//---------------------------------------------------
// Use clEnqueueWriteBuffer() to write input array Re1 to
// the device buffer bufferB
status = clEnqueueWriteBuffer(cmdQueue,	bufferin,CL_TRUE,0,datasize,(void *)cl_input,0,NULL,NULL);
status = clEnqueueWriteBuffer(cmdQueue,	bufferFilter,CL_FALSE,0,dataf,cl_filter,0,NULL,NULL);
status = clEnqueueWriteBuffer(cmdQueue,	buffertemp,CL_FALSE,0,datasize,0,0,NULL,NULL);



//-----------------------------------------
//Paso7: Creando y conmpilando programas
//-----------------------------------------
cl_program program  = clCreateProgramWithSource(context,1,(const char**)&rot ,NULL,&status);

// Build (compile) the program for the devices with
// clBuildProgram()
status = clBuildProgram(program,numDevices,devices,NULL,NULL,NULL);



//-----------------------------------------
//Paso8: Creando un kernel
//-----------------------------------------
cl_kernel kernel  =  clCreateKernel(program, "rotar"   , &status);

status   = clSetKernelArg(kernel, 0, sizeof(cl_mem)   , (void *)&bufferin);
status  |= clSetKernelArg(kernel, 1, sizeof(cl_int)   , (void *)&W);
status  |= clSetKernelArg(kernel, 2, sizeof(cl_int)   , (void *)&H);
status  |= clSetKernelArg(kernel, 3, sizeof(cl_float) , (void *)&cos_theta);
status  |= clSetKernelArg(kernel, 4, sizeof(cl_float) , (void *)&sin_theta);
status  |= clSetKernelArg(kernel, 5, sizeof(cl_mem)   , (void *)&buffertemp);

size_t globalWorkSize[2] = {W,H};
size_t localws[2] = {10,10};
status	= clEnqueueNDRangeKernel(cmdQueue,kernel,2,NULL,globalWorkSize,0,0,NULL,NULL);
//———————————————————————————————————————————————————
// STEP 11: Enqueue the kernel for execution
//———————————————————————————————————————————————————


cl_program program1 = clCreateProgramWithSource(context,1,(const char**)&c_blurr,NULL,&status);
status			    = clBuildProgram(program1,numDevices,devices,NULL,NULL,NULL);
cl_kernel kernel1   = clCreateKernel(program1, "blurring", &status);

status   = clSetKernelArg(kernel1, 0, sizeof(cl_mem)   , (void *)&buffertemp);
status  |= clSetKernelArg(kernel1, 1, sizeof(cl_int)   , (void *)&W);
status  |= clSetKernelArg(kernel1, 2, sizeof(cl_int)   , (void *)&H);
status  |= clSetKernelArg(kernel1, 3, sizeof(cl_int)   , (void *)&filterW);
status  |= clSetKernelArg(kernel1, 4, sizeof(cl_mem)   , (void *)&bufferFilter);
status  |= clSetKernelArg(kernel1, 5, sizeof(cl_mem)   , (void *)&bufferout);


status  = clEnqueueNDRangeKernel(cmdQueue,kernel1,2,NULL,globalWorkSize,0,0,NULL,NULL);



//--------------------------------------------------------
// Paso 12: Leyendo el buffer de salida de regreso al host
//--------------------------------------------------------
clEnqueueReadBuffer(cmdQueue,buffertemp,CL_TRUE,0,datasize,(void *)cl_output,0,NULL,NULL);
// Verify the output



//———————————————————————————————————————————————————
// STEP 13: Release OpenCL resources
//———————————————————————————————————————————————————
// Free OpenCL resources
clReleaseKernel(kernel);
clReleaseKernel(kernel1);
clReleaseProgram(program);
clReleaseProgram(program1);
clReleaseCommandQueue(cmdQueue);

clReleaseMemObject(bufferin);
clReleaseMemObject(bufferout);
clReleaseMemObject(buffertemp);
clReleaseMemObject(bufferFilter);

clReleaseContext(context);
// Free host resources
free(cl_input);

free(platforms);
free(devices);


}





void OpenCLResize(float *cl_inputlow, float * cl_outputhigh, int &Wl,int &Hl, int &Wh, int &Hh, int &elemH, int &elemL){



// tamaño de datos
size_t dataf	= elemL*sizeof(float);
size_t datasize = elemH*sizeof(float);

//Se crea la variable para revisar la salida de cada 
cl_int status;

//------------------------------------------------
//Paso1: Encontrar e inicializar las plataformas
//------------------------------------------------
cl_uint numPlatforms = 0;
cl_platform_id *platforms = NULL;
clGetPlatformIDs(0,NULL,&numPlatforms);
platforms = (cl_platform_id*)malloc(numPlatforms*sizeof(cl_platform_id));
status = clGetPlatformIDs(numPlatforms,platforms,NULL);



//------------------------------------------------
//Paso2: Encontrar e inicializar los dispositivos
//------------------------------------------------
cl_uint numDevices = 0;
cl_device_id *devices = NULL;
status = clGetDeviceIDs(platforms[0],CL_DEVICE_TYPE_ALL,0,NULL,&numDevices);
devices = (cl_device_id*)malloc(numDevices*sizeof(cl_device_id));
status = clGetDeviceIDs(platforms[0],CL_DEVICE_TYPE_ALL, numDevices, devices, NULL);



//------------------------------------------------
//Paso3: Creando un contexto
//------------------------------------------------
cl_context context = NULL;
context = clCreateContext(NULL,numDevices,devices,NULL,NULL,&status);



//------------------------------------------------
//Paso4: Creando un comandqueue
//------------------------------------------------
cl_command_queue cmdQueue;
cmdQueue = clCreateCommandQueue(context,devices[0],0,&status);



//------------------------------------------------
//Paso5: Creando un device buffers
//------------------------------------------------
cl_mem bufferin = clCreateBuffer(context,CL_MEM_READ_ONLY,dataf,NULL,&status);
cl_mem bufferout = clCreateBuffer(context,CL_MEM_WRITE_ONLY,datasize,NULL,&status);



//---------------------------------------------------
//Paso6: Escribiendo datos del host al device buffers
//---------------------------------------------------
// Use clEnqueueWriteBuffer() to write input array Re1 to
// the device buffer bufferB
status = clEnqueueWriteBuffer(cmdQueue,	bufferin,CL_TRUE,0,dataf,(void *)cl_inputlow,0,NULL,NULL);



//-----------------------------------------
//Paso7: Creando y conmpilando programas
//-----------------------------------------
cl_program program = clCreateProgramWithSource(context,1,(const char**)&ch_interpolar,NULL,&status);
// Build (compile) the program for the devices with
// clBuildProgram()
status = clBuildProgram(program,numDevices,devices,NULL,NULL,NULL);



//-----------------------------------------
//Paso8: Creando un kernel
//-----------------------------------------
cl_kernel kernel =  clCreateKernel(program, "res", &status);



//------------------------------------------------
//Paso9: Estableciendo los argumentos del kernell
//------------------------------------------------
status   = clSetKernelArg(kernel, 0, sizeof(cl_mem)   , (void *)&bufferin);
status  |= clSetKernelArg(kernel, 1, sizeof(cl_int)   , (void *)&Wl);
status  |= clSetKernelArg(kernel, 2, sizeof(cl_int)   , (void *)&Hl);
status  |= clSetKernelArg(kernel, 3, sizeof(cl_int)   , (void *)&Wh);
status  |= clSetKernelArg(kernel, 4, sizeof(cl_int)   , (void *)&Hh);
status  |= clSetKernelArg(kernel, 5, sizeof(cl_mem)   , (void *)&bufferout);


//------------------------------------------------
//Paso10: Configurando el wirk-item structure
//------------------------------------------------
size_t globalWorkSize[2] = {Wh,Hh};
size_t localws[2] = {10,10};


//———————————————————————————————————————————————————
// STEP 11: Enqueue the kernel for execution
//———————————————————————————————————————————————————
status = clEnqueueNDRangeKernel(cmdQueue,kernel,2,NULL,globalWorkSize,0,0,NULL,NULL);


//------------------------------------------------
// Paso 12: Leyendo el buffer de salida de regreso al host
//------------------------------------------------
clEnqueueReadBuffer(cmdQueue,bufferout,CL_TRUE,0,datasize,(void *)cl_outputhigh,0,NULL,NULL);
// Verify the output

//———————————————————————————————————————————————————
// STEP 13: Release OpenCL resources
//———————————————————————————————————————————————————
// Free OpenCL resources
clReleaseKernel(kernel);
clReleaseProgram(program);
clReleaseCommandQueue(cmdQueue);
clReleaseMemObject(bufferin);
clReleaseMemObject(bufferout);
clReleaseContext(context);

// Free host resources
free(cl_inputlow);
free(platforms);
free(devices);


}


void OpenCLReducir(float *cl_inputhigh, float * cl_outputlow, int &x,int &y, int &Wh, int &Hh, int &elemH, int &elemL, int &f){

// tamaño de datos
size_t dataf	= elemL*sizeof(float);
size_t datasize = elemH*sizeof(float);

//Se crea la variable para revisar la salida de cada 
cl_int status;

//------------------------------------------------
//Paso1: Encontrar e inicializar las plataformas
//------------------------------------------------
cl_uint numPlatforms = 0;
cl_platform_id *platforms = NULL;
clGetPlatformIDs(0,NULL,&numPlatforms);
platforms = (cl_platform_id*)malloc(numPlatforms*sizeof(cl_platform_id));
status = clGetPlatformIDs(numPlatforms,platforms,NULL);



//------------------------------------------------
//Paso2: Encontrar e inicializar los dispositivos
//------------------------------------------------
cl_uint numDevices = 0;
cl_device_id *devices = NULL;
status = clGetDeviceIDs(platforms[0],CL_DEVICE_TYPE_ALL,0,NULL,&numDevices);
devices = (cl_device_id*)malloc(numDevices*sizeof(cl_device_id));
status = clGetDeviceIDs(platforms[0],CL_DEVICE_TYPE_ALL, numDevices, devices, NULL);



//------------------------------------------------
//Paso3: Creando un contexto
//------------------------------------------------
cl_context context = NULL;
context = clCreateContext(NULL,numDevices,devices,NULL,NULL,&status);



//------------------------------------------------
//Paso4: Creando un comandqueue
//------------------------------------------------
cl_command_queue cmdQueue;
cmdQueue = clCreateCommandQueue(context,devices[0],0,&status);



//------------------------------------------------
//Paso5: Creando un device buffers
//------------------------------------------------
cl_mem bufferin = clCreateBuffer(context,CL_MEM_READ_ONLY,datasize,NULL,&status);
cl_mem bufferout = clCreateBuffer(context,CL_MEM_WRITE_ONLY,dataf,NULL,&status);



//---------------------------------------------------
//Paso6: Escribiendo datos del host al device buffers
//---------------------------------------------------
// Use clEnqueueWriteBuffer() to write input array Re1 to
// the device buffer bufferB
status = clEnqueueWriteBuffer(cmdQueue,	bufferin,CL_TRUE,0,datasize,(void *)cl_inputhigh,0,NULL,NULL);



//-----------------------------------------
//Paso7: Creando y conmpilando programas
//-----------------------------------------
cl_program program = clCreateProgramWithSource(context,1,(const char**)&ch_reducir,NULL,&status);
// Build (compile) the program for the devices with
// clBuildProgram()
status = clBuildProgram(program,numDevices,devices,NULL,NULL,NULL);



//-----------------------------------------
//Paso8: Creando un kernel
//-----------------------------------------
cl_kernel kernel =  clCreateKernel(program, "reducir", &status);



//------------------------------------------------
//Paso9: Estableciendo los argumentos del kernell
//------------------------------------------------
status   = clSetKernelArg(kernel, 0, sizeof(cl_mem)   , (void *)&bufferin);
status  |= clSetKernelArg(kernel, 1, sizeof(cl_int)   , (void *)&x);
status  |= clSetKernelArg(kernel, 2, sizeof(cl_int)   , (void *)&Wh);
status  |= clSetKernelArg(kernel, 3, sizeof(cl_int)   , (void *)&f);
status  |= clSetKernelArg(kernel, 4, sizeof(cl_mem)   , (void *)&bufferout);


//------------------------------------------------
//Paso10: Configurando el wirk-item structure
//------------------------------------------------
size_t globalWorkSize[2] = {x,y};
size_t localws[2] = {10,10};


//———————————————————————————————————————————————————
// STEP 11: Enqueue the kernel for execution
//———————————————————————————————————————————————————
status = clEnqueueNDRangeKernel(cmdQueue,kernel,2,NULL,globalWorkSize,0,0,NULL,NULL);


//------------------------------------------------
// Paso 12: Leyendo el buffer de salida de regreso al host
//------------------------------------------------
clEnqueueReadBuffer(cmdQueue,bufferout,CL_TRUE,0,dataf,(void *)cl_outputlow,0,NULL,NULL);
// Verify the output

//———————————————————————————————————————————————————
// STEP 13: Release OpenCL resources
//———————————————————————————————————————————————————
// Free OpenCL resources
clReleaseKernel(kernel);
clReleaseProgram(program);
clReleaseCommandQueue(cmdQueue);
clReleaseMemObject(bufferin);
clReleaseMemObject(bufferout);
clReleaseContext(context);

// Free host resources
free(cl_inputhigh);
free(platforms);
free(devices);


}
void OpenCLReBlRo(float *cl_in, float *cl_out, float &theta, float cl_filter[9],  int &Wl,int &Hl, int &Wh, int &Hh, int &elemH, int &elemL ){


// tamaño de datos
size_t dataf	= elemL*sizeof(float);
size_t datasize = elemH*sizeof(float);

//Se crea la variable para revisar la salida de cada 
cl_int status;

size_t datafilter = sizeof(float)*9;
cl_int filterW = 3;
cl_int paddingPixels = (int)(3/2)*2;

float cos_theta = cos(theta);
float sin_theta = sin(theta);

//------------------------------------------------
cl_uint numPlatforms	  = 0;
cl_platform_id *platforms = NULL;
clGetPlatformIDs(0,NULL,&numPlatforms);
platforms = (cl_platform_id*)malloc(numPlatforms*sizeof(cl_platform_id));
status	  = clGetPlatformIDs(numPlatforms,platforms,NULL);

//------------------------------------------------
cl_uint numDevices	  = 0;
cl_device_id *devices = NULL;
status  = clGetDeviceIDs(platforms[0],CL_DEVICE_TYPE_ALL,0,NULL,&numDevices);
devices = (cl_device_id*)malloc(numDevices*sizeof(cl_device_id));
status  = clGetDeviceIDs(platforms[0],CL_DEVICE_TYPE_ALL, numDevices, devices, NULL);

//------------------------------------------------
cl_context context = NULL;
context = clCreateContext(NULL,numDevices,devices,NULL,NULL,&status);

//------------------------------------------------
cl_command_queue cmdQueue;
cmdQueue = clCreateCommandQueue(context,devices[0],0,&status);

//------------------------------------------------
//Paso5: Creando un device buffers
//------------------------------------------------
cl_mem bufferin	   = clCreateBuffer(context,CL_MEM_READ_ONLY ,dataf   ,NULL,&status);
cl_mem bufferFilter= clCreateBuffer(context,CL_MEM_READ_ONLY ,datafilter,NULL,NULL);
cl_mem buffertemp  = clCreateBuffer(context,CL_MEM_READ_WRITE,datasize,NULL,&status);
cl_mem buffertemp1 = clCreateBuffer(context,CL_MEM_READ_WRITE,datasize,NULL,&status);
cl_mem bufferout   = clCreateBuffer(context,CL_MEM_WRITE_ONLY,datasize,NULL,&status);

//---------------------------------------------------
//Paso6: Escribiendo datos del host al device buffers
//---------------------------------------------------
status = clEnqueueWriteBuffer(cmdQueue,	bufferin,CL_TRUE,0,dataf,cl_in,0,NULL,NULL);
status = clEnqueueWriteBuffer(cmdQueue,	bufferFilter,CL_FALSE,0,datafilter,cl_filter,0,NULL,NULL);

//-----------------------------------------
// Creando y conmpilando programa 1
//-----------------------------------------
cl_program program = clCreateProgramWithSource(context,1,(const char**)&ch_interpolar,NULL,&status);

status = clBuildProgram(program,numDevices,devices,NULL,NULL,NULL);

cl_kernel kernel =  clCreateKernel(program, "res", &status);

status   = clSetKernelArg(kernel, 0, sizeof(cl_mem)   , (void *)&bufferin);
status  |= clSetKernelArg(kernel, 1, sizeof(cl_int)   , (void *)&Wl);
status  |= clSetKernelArg(kernel, 2, sizeof(cl_int)   , (void *)&Hl);
status  |= clSetKernelArg(kernel, 3, sizeof(cl_int)   , (void *)&Wh);
status  |= clSetKernelArg(kernel, 4, sizeof(cl_int)   , (void *)&Hh);
status  |= clSetKernelArg(kernel, 5, sizeof(cl_mem)   , (void *)&buffertemp);

size_t globalWorkSize[2] = {Wh,Hh};
size_t localws[2] = {10,10};

status = clEnqueueNDRangeKernel(cmdQueue,kernel,2,NULL,globalWorkSize,0,0,NULL,NULL);


//-----------------------------------------
// Creando y conmpilando programa 2
//-----------------------------------------
cl_program program1 = clCreateProgramWithSource(context,1,(const char**)&c_blurr,NULL,&status);

status = clBuildProgram(program1,numDevices,devices,NULL,NULL,NULL);

cl_kernel kernel1 =  clCreateKernel(program1, "blurring", &status);

status   = clSetKernelArg(kernel1, 0, sizeof(cl_mem)   , (void *)&buffertemp);
status  |= clSetKernelArg(kernel1, 1, sizeof(cl_int)   , (void *)&Wh);
status  |= clSetKernelArg(kernel1, 2, sizeof(cl_int)   , (void *)&Hh);
status  |= clSetKernelArg(kernel1, 3, sizeof(cl_int)   , (void *)&filterW);
status  |= clSetKernelArg(kernel1, 4, sizeof(cl_mem)   , (void *)&bufferFilter);
status  |= clSetKernelArg(kernel1, 5, sizeof(cl_mem)   , (void *)&buffertemp1);

status	= clEnqueueNDRangeKernel(cmdQueue,kernel1,2,NULL,globalWorkSize,0,0,NULL,NULL);


//-----------------------------------------
// Creando y conmpilando programa 3
//-----------------------------------------
cl_program program2 = clCreateProgramWithSource(context,1,(const char**)&rot,NULL,&status);

status = clBuildProgram(program2,numDevices,devices,NULL,NULL,NULL);

cl_kernel kernel2 =  clCreateKernel(program2, "rotar", &status);

status   = clSetKernelArg(kernel2, 0, sizeof(cl_mem)   , (void *)&buffertemp1);
status  |= clSetKernelArg(kernel2, 1, sizeof(cl_int)   , (void *)&Wh);
status  |= clSetKernelArg(kernel2, 2, sizeof(cl_int)   , (void *)&Hh);
status  |= clSetKernelArg(kernel2, 3, sizeof(cl_float) , (void *)&cos_theta);
status  |= clSetKernelArg(kernel2, 4, sizeof(cl_float) , (void *)&sin_theta);
status  |= clSetKernelArg(kernel2, 5, sizeof(cl_mem)   , (void *)&bufferout);

status	= clEnqueueNDRangeKernel(cmdQueue,kernel2,2,NULL,globalWorkSize,0,0,NULL,NULL);

//------------------------------------------------
// Paso 12: Leyendo el buffer de salida de regreso al host
//------------------------------------------------
clEnqueueReadBuffer(cmdQueue,bufferout,CL_TRUE,0,datasize,(void *)cl_out,0,NULL,NULL);
// Verify the output

//———————————————————————————————————————————————————
// STEP 13: Release OpenCL resources
//———————————————————————————————————————————————————
// Free OpenCL resources
clReleaseKernel(kernel);
clReleaseKernel(kernel1);
clReleaseKernel(kernel2);
clReleaseProgram(program);
clReleaseProgram(program1);
clReleaseProgram(program2);
clReleaseCommandQueue(cmdQueue);
clReleaseMemObject(bufferin);
clReleaseMemObject(buffertemp);
clReleaseMemObject(buffertemp1);
clReleaseMemObject(bufferFilter);
clReleaseMemObject(bufferout);
clReleaseContext(context);

// Free host resources
free(cl_in);
free(platforms);
free(devices);

}
void OpenCLRoBlReSu( float *cl_in, float *cl_in2,float *cl_out, int &wH, int &hH, float cl_filter[9], float &theta, int &x, int &y,int &f){

float cos_theta = cos(theta);
float sin_theta = sin(theta);

int elemL = x*y;
int elemH = wH*hH;

// tamaño de datos
size_t dataH	= elemH*sizeof(float);
size_t dataL	= elemL*sizeof(float);
size_t dataf = sizeof(float)*9;

size_t datafilter = sizeof(float)*9;
cl_int filterW = 3;
cl_int paddingPixels = (int)(3/2)*2;

cl_int status;


//------------------------------------------------
cl_uint numPlatforms = 0;
cl_platform_id *platforms = NULL;
clGetPlatformIDs(0,NULL,&numPlatforms);
platforms = (cl_platform_id*)malloc(numPlatforms*sizeof(cl_platform_id));
status = clGetPlatformIDs(numPlatforms,platforms,NULL);


//------------------------------------------------
cl_uint numDevices = 0;
cl_device_id *devices = NULL;
status = clGetDeviceIDs(platforms[0],CL_DEVICE_TYPE_ALL,0,NULL,&numDevices);
devices = (cl_device_id*)malloc(numDevices*sizeof(cl_device_id));
status = clGetDeviceIDs(platforms[0],CL_DEVICE_TYPE_ALL, numDevices, devices, NULL);


//------------------------------------------------
cl_context context = NULL;
context = clCreateContext(NULL,numDevices,devices,NULL,NULL,&status);


//------------------------------------------------
cl_command_queue cmdQueue;
cmdQueue = clCreateCommandQueue(context,devices[0],0,&status);



//------------------------------------------------
//Paso5: Creando un device buffers
//------------------------------------------------
cl_mem bufferin     = clCreateBuffer(context,CL_MEM_READ_ONLY ,dataH,NULL,&status);
cl_mem buffertemp   = clCreateBuffer(context,CL_MEM_READ_WRITE,dataH,NULL,&status);
cl_mem bufferout    = clCreateBuffer(context,CL_MEM_WRITE_ONLY,dataL,NULL,&status);

//---------------------------------------------------
//Paso6: Escribiendo datos del host al device buffers
//---------------------------------------------------
status = clEnqueueWriteBuffer(cmdQueue,	bufferin, CL_FALSE,0,dataH,cl_in,0,NULL,NULL);


//-----------------------------------------
// Creando y conmpilando programa 1
//-----------------------------------------
cl_program program  = clCreateProgramWithSource(context,1,(const char**)&rot ,NULL,&status);

status = clBuildProgram(program,numDevices,devices,NULL,NULL,NULL);

cl_kernel kernel  =  clCreateKernel(program, "rotar"   , &status);

status   = clSetKernelArg(kernel, 0, sizeof(cl_mem)   , (void *)&bufferin);
status  |= clSetKernelArg(kernel, 1, sizeof(cl_int)   , (void *)&wH);
status  |= clSetKernelArg(kernel, 2, sizeof(cl_int)   , (void *)&hH);
status  |= clSetKernelArg(kernel, 3, sizeof(cl_float) , (void *)&cos_theta);
status  |= clSetKernelArg(kernel, 4, sizeof(cl_float) , (void *)&sin_theta);
status  |= clSetKernelArg(kernel, 5, sizeof(cl_mem)   , (void *)&buffertemp);

size_t globalWorkSize[2] = {wH,hH};
size_t localws[2] = {10,10};
status	= clEnqueueNDRangeKernel(cmdQueue,kernel,2,NULL,globalWorkSize,0,0,NULL,NULL);

free(cl_in);
clReleaseMemObject(bufferin);

//-----------------------------------------
// Creando y conmpilando programa 2
//-----------------------------------------
cl_mem buffertemp1  = clCreateBuffer(context,CL_MEM_READ_WRITE,dataH,NULL,&status);
cl_mem bufferFilter = clCreateBuffer(context,CL_MEM_READ_ONLY ,dataf,NULL,NULL);
status = clEnqueueWriteBuffer(cmdQueue,	bufferFilter, CL_FALSE,0,dataf,cl_filter,0,NULL,NULL);

cl_program program1 = clCreateProgramWithSource(context,1,(const char**)&c_blurr,NULL,&status);

status			    = clBuildProgram(program1,numDevices,devices,NULL,NULL,NULL);
cl_kernel kernel1   = clCreateKernel(program1, "blurring", &status);

status   = clSetKernelArg(kernel1, 0, sizeof(cl_mem)   , (void *)&buffertemp);
status  |= clSetKernelArg(kernel1, 1, sizeof(cl_int)   , (void *)&wH);
status  |= clSetKernelArg(kernel1, 2, sizeof(cl_int)   , (void *)&hH);
status  |= clSetKernelArg(kernel1, 3, sizeof(cl_int)   , (void *)&filterW);
status  |= clSetKernelArg(kernel1, 4, sizeof(cl_mem)   , (void *)&bufferFilter);
status  |= clSetKernelArg(kernel1, 5, sizeof(cl_mem)   , (void *)&buffertemp1);

status  = clEnqueueNDRangeKernel(cmdQueue,kernel1,2,NULL,globalWorkSize,0,0,NULL,NULL);

clReleaseMemObject(buffertemp);


//-----------------------------------------
// Creando y conmpilando programa 3
//-----------------------------------------
cl_mem buffertemp2  = clCreateBuffer(context,CL_MEM_READ_WRITE,dataL,NULL,&status);

cl_program program2 = clCreateProgramWithSource(context,1,(const char**)&ch_reducir,NULL,&status);

status			    = clBuildProgram(program2,numDevices,devices,NULL,NULL,NULL);
cl_kernel kernel2   = clCreateKernel(program2, "reducir", &status);

status   = clSetKernelArg(kernel2, 0, sizeof(cl_mem)   , (void *)&buffertemp1);
status  |= clSetKernelArg(kernel2, 1, sizeof(cl_int)   , (void *)&x);
status  |= clSetKernelArg(kernel2, 2, sizeof(cl_int)   , (void *)&wH);
status  |= clSetKernelArg(kernel2, 3, sizeof(cl_int)   , (void *)&f);
status  |= clSetKernelArg(kernel2, 4, sizeof(cl_mem)   , (void *)&buffertemp2);

globalWorkSize[0] = x;
globalWorkSize[1] = y;

status  = clEnqueueNDRangeKernel(cmdQueue,kernel2,2,NULL,globalWorkSize,0,0,NULL,NULL);

clReleaseMemObject(buffertemp1);


//-----------------------------------------
// Creando y conmpilando programa 4
//-----------------------------------------
cl_mem bufferin2    = clCreateBuffer(context,CL_MEM_READ_ONLY ,dataL,NULL,&status);


status = clEnqueueWriteBuffer(cmdQueue,	bufferin2, CL_FALSE, 0, dataL, cl_in2, 0, NULL, NULL);

cl_program program3 = clCreateProgramWithSource(context,1,(const char**)&ch_sumar1,NULL,&status);

status		  	    = clBuildProgram(program3,numDevices,devices,NULL,NULL,NULL);

cl_kernel kernel3   = clCreateKernel(program3, "sumar1", &status);

status   = clSetKernelArg(kernel3, 0, sizeof(cl_mem)   , &buffertemp2);
status  |= clSetKernelArg(kernel3, 1, sizeof(cl_mem)   , &bufferin2);
status  |= clSetKernelArg(kernel3, 2, sizeof(cl_int)   , (void *)&x);
status  |= clSetKernelArg(kernel3, 3, sizeof(cl_mem)   , &bufferout);

size_t globalWork[1] = {x*y};

status  = clEnqueueNDRangeKernel(cmdQueue,kernel3,2,NULL,globalWorkSize,0,0,NULL,NULL);


//--------------------------------------------------------
// Paso 12: Leyendo el buffer de salida de regreso al host
//--------------------------------------------------------
clEnqueueReadBuffer(cmdQueue,bufferout,CL_TRUE,0,dataL,(void *)cl_out,0,NULL,NULL);
// Verify the output



//———————————————————————————————————————————————————
// STEP 13: Release OpenCL resources
//———————————————————————————————————————————————————
// Free OpenCL resources
clReleaseKernel(kernel);
clReleaseKernel(kernel1);
clReleaseKernel(kernel2);
clReleaseKernel(kernel3);
clReleaseProgram(program);
clReleaseProgram(program1);
clReleaseProgram(program2);
clReleaseProgram(program3);
clReleaseCommandQueue(cmdQueue);

clReleaseMemObject(bufferin2);
clReleaseMemObject(bufferout);

clReleaseMemObject(buffertemp2);
clReleaseMemObject(bufferFilter);

clReleaseContext(context);
// Free host resources

free(platforms);
free(devices);





}
void Cl_iteraciones( float *cl_in, float *cl_in2,float *cl_out,  int &wL,int &hL, int &wH, int &hH, float cl_filter1[9], float cl_filter[9], float &theta1, float &theta2, int &f){

float cos_theta1 = cos(theta1);
float sin_theta1 = sin(theta1);

float cos_theta2 = cos(theta2);
float sin_theta2 = sin(theta2);

int elemL = wL*hL;
int elemH = wH*hH;
// tamaño de datos
size_t dataH	= elemH*sizeof(float);
size_t dataL	= elemL*sizeof(float);
size_t dataf = sizeof(float)*9;

cl_int filterW = 3;
cl_int paddingPixels = (int)(3/2)*2;

//Se crea la variable para revisar la salida de cada 
cl_int status;

//------------------------------------------------
cl_uint numPlatforms	  = 0;
cl_platform_id *platforms = NULL;
clGetPlatformIDs(0,NULL,&numPlatforms);
platforms = (cl_platform_id*)malloc(numPlatforms*sizeof(cl_platform_id));
status	  = clGetPlatformIDs(numPlatforms,platforms,NULL);

//------------------------------------------------
cl_uint numDevices	  = 0;
cl_device_id *devices = NULL;
status  = clGetDeviceIDs(platforms[0],CL_DEVICE_TYPE_ALL,0,NULL,&numDevices);
devices = (cl_device_id*)malloc(numDevices*sizeof(cl_device_id));
status  = clGetDeviceIDs(platforms[0],CL_DEVICE_TYPE_ALL, numDevices, devices, NULL);

//------------------------------------------------
cl_context context = NULL;
context = clCreateContext(NULL,numDevices,devices,NULL,NULL,&status);

//------------------------------------------------
cl_command_queue cmdQueue;
cmdQueue = clCreateCommandQueue(context,devices[0],0,&status);


//------------------------------------------------
cl_mem bufferin     = clCreateBuffer(context,CL_MEM_READ_ONLY ,dataH,NULL,&status);
cl_mem buffertemp   = clCreateBuffer(context,CL_MEM_READ_WRITE,dataH,NULL,&status);

//---------------------------------------------------
status = clEnqueueWriteBuffer(cmdQueue,	bufferin, CL_TRUE,0,dataH,cl_in,0,NULL,NULL);

//-----------------------------------------
// Creando y conmpilando programa 1
//-----------------------------------------
cl_program program  = clCreateProgramWithSource(context,1,(const char**)&rot ,NULL,&status);

status = clBuildProgram(program,numDevices,devices,NULL,NULL,NULL);

cl_kernel kernel  =  clCreateKernel(program, "rotar"   , &status);

status   = clSetKernelArg(kernel, 0, sizeof(cl_mem)   , (void *)&bufferin);
status  |= clSetKernelArg(kernel, 1, sizeof(cl_int)   , (void *)&wH);
status  |= clSetKernelArg(kernel, 2, sizeof(cl_int)   , (void *)&hH);
status  |= clSetKernelArg(kernel, 3, sizeof(cl_float) , (void *)&cos_theta1);
status  |= clSetKernelArg(kernel, 4, sizeof(cl_float) , (void *)&sin_theta1);
status  |= clSetKernelArg(kernel, 5, sizeof(cl_mem)   , (void *)&buffertemp);

size_t globalWorkSize[2] = {wH,hH};
size_t localws[2] = {10,10};
status	= clEnqueueNDRangeKernel(cmdQueue,kernel,2,NULL,globalWorkSize,0,0,NULL,NULL);

clReleaseMemObject(bufferin);

clEnqueueReadBuffer(cmdQueue,buffertemp,CL_TRUE,0,dataH,(void *)cl_out,0,NULL,NULL);


//-----------------------------------------
// Creando y conmpilando programa 2
//-----------------------------------------
cl_mem buffertemp1  = clCreateBuffer(context,CL_MEM_READ_WRITE,dataH,NULL,&status);
cl_mem bufferFilter = clCreateBuffer(context,CL_MEM_READ_ONLY ,dataf,NULL,NULL);
status = clEnqueueWriteBuffer(cmdQueue,	bufferFilter, CL_FALSE,0,dataf,cl_filter1,0,NULL,NULL);

cl_program program1 = clCreateProgramWithSource(context,1,(const char**)&c_blurr,NULL,&status);

status			    = clBuildProgram(program1,numDevices,devices,NULL,NULL,NULL);
cl_kernel kernel1   = clCreateKernel(program1, "blurring", &status);

status   = clSetKernelArg(kernel1, 0, sizeof(cl_mem)   , (void *)&buffertemp);
status  |= clSetKernelArg(kernel1, 1, sizeof(cl_int)   , (void *)&wH);
status  |= clSetKernelArg(kernel1, 2, sizeof(cl_int)   , (void *)&hH);
status  |= clSetKernelArg(kernel1, 3, sizeof(cl_int)   , (void *)&filterW);
status  |= clSetKernelArg(kernel1, 4, sizeof(cl_mem)   , (void *)&bufferFilter);
status  |= clSetKernelArg(kernel1, 5, sizeof(cl_mem)   , (void *)&buffertemp1);

status  = clEnqueueNDRangeKernel(cmdQueue,kernel1,2,NULL,globalWorkSize,0,0,NULL,NULL);

clReleaseMemObject(buffertemp);
clReleaseMemObject(bufferFilter);

//-----------------------------------------
// Creando y conmpilando programa 3
//-----------------------------------------
cl_mem buffertemp2  = clCreateBuffer(context,CL_MEM_READ_WRITE,dataL,NULL,&status);

cl_program program2 = clCreateProgramWithSource(context,1,(const char**)&ch_reducir,NULL,&status);

status			    = clBuildProgram(program2,numDevices,devices,NULL,NULL,NULL);
cl_kernel kernel2   = clCreateKernel(program2, "reducir", &status);

status   = clSetKernelArg(kernel2, 0, sizeof(cl_mem)   , (void *)&buffertemp1);
status  |= clSetKernelArg(kernel2, 1, sizeof(cl_int)   , (void *)&wL);
status  |= clSetKernelArg(kernel2, 2, sizeof(cl_int)   , (void *)&wH);
status  |= clSetKernelArg(kernel2, 3, sizeof(cl_int)   , (void *)&f);
status  |= clSetKernelArg(kernel2, 4, sizeof(cl_mem)   , (void *)&buffertemp2);

globalWorkSize[0] = wL;
globalWorkSize[1] = hL;

status  = clEnqueueNDRangeKernel(cmdQueue,kernel2,2,NULL,globalWorkSize,0,0,NULL,NULL);

clReleaseMemObject(buffertemp1);


//-----------------------------------------
// Creando y conmpilando programa 4
//-----------------------------------------
cl_mem bufferin2    = clCreateBuffer(context,CL_MEM_READ_ONLY ,dataL,NULL,&status);
cl_mem buffertemp3  = clCreateBuffer(context,CL_MEM_READ_WRITE ,dataL,NULL,&status);

status = clEnqueueWriteBuffer(cmdQueue,	bufferin2, CL_FALSE, 0, dataL, cl_in2, 0, NULL, NULL);

cl_program program3 = clCreateProgramWithSource(context,1,(const char**)&ch_sumar1,NULL,&status);
status		  	    = clBuildProgram(program3,numDevices,devices,NULL,NULL,NULL);
cl_kernel kernel3   = clCreateKernel(program3, "sumar1", &status);

status   = clSetKernelArg(kernel3, 0, sizeof(cl_mem)  , &buffertemp2);
status  |= clSetKernelArg(kernel3, 1, sizeof(cl_mem)  , &bufferin2  );
status  |= clSetKernelArg(kernel3, 2, sizeof(cl_int)  , (void *)&wL );
status  |= clSetKernelArg(kernel3, 3, sizeof(cl_mem)  , &buffertemp3);


status  = clEnqueueNDRangeKernel(cmdQueue,kernel3,2,NULL,globalWorkSize,0,0,NULL,NULL);

clReleaseMemObject(buffertemp2);
clReleaseMemObject(bufferin2);



//-----------------------------------------
// Creando y conmpilando programa 5
//-----------------------------------------
cl_program program4 = clCreateProgramWithSource(context,1,(const char**)&ch_intercubic,NULL,&status);
cl_mem buffertemp4    = clCreateBuffer(context,CL_MEM_READ_WRITE ,dataH,NULL,&status);

status = clBuildProgram(program4,numDevices,devices,NULL,NULL,NULL);

cl_kernel kernel4 =  clCreateKernel(program4, "rescubic", &status);

status   = clSetKernelArg(kernel4, 0, sizeof(cl_mem)   , (void *)&buffertemp3);
status  |= clSetKernelArg(kernel4, 1, sizeof(cl_int)   , (void *)&wL);
status  |= clSetKernelArg(kernel4, 2, sizeof(cl_int)   , (void *)&hL);
status  |= clSetKernelArg(kernel4, 3, sizeof(cl_int)   , (void *)&wH);
status  |= clSetKernelArg(kernel4, 4, sizeof(cl_int)   , (void *)&hH);
status  |= clSetKernelArg(kernel4, 5, sizeof(cl_mem)   , (void *)&buffertemp4);



globalWorkSize[0] = wH;
globalWorkSize[1] = hH;

status = clEnqueueNDRangeKernel(cmdQueue,kernel4,2,NULL,globalWorkSize,0,0,NULL,NULL);
clReleaseMemObject(buffertemp3);
/*
clEnqueueReadBuffer(cmdQueue,buffertemp4,CL_TRUE,0,dataH,(void *)cl_out,0,NULL,NULL);

	cv::Mat p2(hH, wH, CV_32FC1);
	for( i=0; i<hH; i++){
		float *pc = p2.ptr<float>(i);
		for( j=0; j<wH; j++){
			pc[j] = cl_out[i*wH+j];
		}
	}
	cv::namedWindow("rotacion",CV_WINDOW_AUTOSIZE);
	cv::imshow("rotacion", p2);
	cv::waitKey(0);*/

//-----------------------------------------
// Creando y conmpilando programa 6
//-----------------------------------------
cl_mem bufferFilter2 = clCreateBuffer(context,CL_MEM_READ_ONLY ,dataf,NULL,NULL);
cl_mem buffertemp5   = clCreateBuffer(context,CL_MEM_READ_WRITE,dataH,NULL,&status);

status = clEnqueueWriteBuffer(cmdQueue,	bufferFilter2,CL_FALSE,0,dataf,cl_filter,0,NULL,NULL);

cl_program program5 = clCreateProgramWithSource(context,1,(const char**)&c_blurr,NULL,&status);
status = clBuildProgram(program5,numDevices,devices,NULL,NULL,NULL);

cl_kernel kernel5 =  clCreateKernel(program5, "blurring", &status);

status   = clSetKernelArg(kernel5, 0, sizeof(cl_mem)   , (void *)&buffertemp4);
status  |= clSetKernelArg(kernel5, 1, sizeof(cl_int)   , (void *)&wH);
status  |= clSetKernelArg(kernel5, 2, sizeof(cl_int)   , (void *)&hH);
status  |= clSetKernelArg(kernel5, 3, sizeof(cl_int)   , (void *)&filterW);
status  |= clSetKernelArg(kernel5, 4, sizeof(cl_mem)   , (void *)&bufferFilter2);
status  |= clSetKernelArg(kernel5, 5, sizeof(cl_mem)   , (void *)&buffertemp5);

status	= clEnqueueNDRangeKernel(cmdQueue,kernel5,2,NULL,globalWorkSize,0,0,NULL,NULL);

clReleaseMemObject(buffertemp4);
clReleaseMemObject(bufferFilter2);


//-----------------------------------------
// Creando y conmpilando programa 7
//-----------------------------------------
cl_mem bufferout    = clCreateBuffer(context,CL_MEM_WRITE_ONLY,dataH,NULL,&status);

cl_program program6 = clCreateProgramWithSource(context,1,(const char**)&rot,NULL,&status);

status = clBuildProgram(program6,numDevices,devices,NULL,NULL,NULL);

cl_kernel kernel6 =  clCreateKernel(program6, "rotar", &status);

status   = clSetKernelArg(kernel6, 0, sizeof(cl_mem)   , (void *)&buffertemp5);
status  |= clSetKernelArg(kernel6, 1, sizeof(cl_int)   , (void *)&wH);
status  |= clSetKernelArg(kernel6, 2, sizeof(cl_int)   , (void *)&hH);
status  |= clSetKernelArg(kernel6, 3, sizeof(cl_float) , (void *)&cos_theta2);
status  |= clSetKernelArg(kernel6, 4, sizeof(cl_float) , (void *)&sin_theta2);
status  |= clSetKernelArg(kernel6, 5, sizeof(cl_mem)   , (void *)&bufferout);

status	= clEnqueueNDRangeKernel(cmdQueue,kernel6,2,NULL,globalWorkSize,0,0,NULL,NULL);

clReleaseMemObject(buffertemp5);


//------------------------------------------------
// Paso 12: Leyendo el buffer de salida de regreso al host
//------------------------------------------------
clEnqueueReadBuffer(cmdQueue,bufferout,CL_TRUE,0,dataH,(void *)cl_out,0,NULL,NULL);
// Verify the output

//———————————————————————————————————————————————————
// STEP 13: Release OpenCL resources
//———————————————————————————————————————————————————
// Free OpenCL resources
clReleaseKernel(kernel);
clReleaseKernel(kernel1);
clReleaseKernel(kernel2);
clReleaseKernel(kernel3);
clReleaseKernel(kernel4);
clReleaseKernel(kernel5);
clReleaseKernel(kernel6);

clReleaseProgram(program);
clReleaseProgram(program1);
clReleaseProgram(program2);
clReleaseProgram(program3);
clReleaseProgram(program4);
clReleaseProgram(program5);
clReleaseProgram(program6);

clReleaseCommandQueue(cmdQueue);
clReleaseMemObject(bufferout);
clReleaseContext(context);
free(cl_in);
// Free host resources
free(platforms);
free(devices);

}
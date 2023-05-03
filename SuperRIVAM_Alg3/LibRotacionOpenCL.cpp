#include "LibRotacionOpenCL.h"


// OpenCL kernel to perform an element-wise
// add of two arrays
const char* programSourceR =
"__kernel \n"
"void genr(__global float *Re1, \n"
" __global float *Im1,   \n "
" __global float *Re2,   \n "
" __global float *Im2,   \n "
" __global float *p1,   \n "
" __global float *p2)   \n "
"{ \n"
" \n"
" // Get the work-item’s unique ID \n"
" int idx = get_global_id(0); \n"
" \n"
" // Add the corresponding locations of \n"
" p1[idx] = Re1[idx]*Re2[idx]-Im1[idx]*Im2[idx]; \n"
" p2[idx] = Re2[idx]*Im1[idx]+Im2[idx]*Re1[idx]; \n"
"} \n"
;


const char* Sourcefactor1 =
"__kernel \n"
"void factor(__global float *p_th,				  \n"
" __global float *p_R)							  \n "
"{												  \n"
" 	const float pi =  3.141592653589793;		  \n"
" // Get the work-item’s unique ID				  \n"
" int idx = get_global_id(0);					  \n"
"												  \n"
" // Estableciendo los datos de la matriz         \n"
" p_R[idx] = p_th[idx]; \n"
" if (  p_th[idx] <= -pi + 0.0000001  ){  \n"
" p_R[idx] = -p_th[idx];     \n"
" } \n "
" if (  p_th[idx] >  pi + 0.0000001  ){  \n"
" p_R[idx] = p_th[idx] -(float)2*pi;   \n"
" } \n "
" if (  p_th[idx] < -pi - 0.0000001 ){  \n"
" p_R[idx] = p_th[idx] +(float)2*pi;   \n"
" } \n "
"} \n"
;


const char* Sourceselec =
"__kernel \n"
"void selec(__global float *p_ra,				  \n"
" __global float *p_front,						  \n "
" __global float *p_th,						      \n "
" __global float *p_tt)							  \n "
"{												  \n"
" 	const float pi =  3.141592653589793;		  \n"
" // Get the work-item’s unique ID				  \n"
" int idx = get_global_id(0);					  \n"
"												  \n"
" // Estableciendo los datos de la matriz         \n"
" if( ( p_ra[idx] > p_front[0] ) && ( p_ra[idx] < p_front[1] ) ){  \n"
" p_tt[idx] = p_th[idx]; \n "
" } else { \n"
" p_tt[idx] = 1000000;     \n"
" } \n "
"} \n"
;

const char* reordenacion =
"__kernel \n"
"void sort(__global float *p_image_Re,				  \n"
" __global float *p_img)							  \n "
"{												  \n"
" // Get the work-item’s unique ID				  \n"
" int idx = get_global_id(0);					  \n"
"												  \n"
" // Estableciendo los datos de la matriz         \n"
" if( ( p_ra[idx] > p_front[0] ) && ( p_ra[idx] < p_front[1] ) ){  \n"
" } else { \n"
" p_th[idx] = 1000000;     \n"
" } \n "
"} \n"
;


void g_factor1(float* cl_th, float *cl_thR, int elements){


// tamaño de los datos
size_t datasize = sizeof(float)*elements;

//Se crea la variable para revisar la salida de cada 
//llamado
cl_int status;

//------------------------------------------------
//Paso1: Encontrar e inicializar las plataformas
//------------------------------------------------

//Primer llamado para identificar la plataforma
cl_uint numPlatforms = 0;
cl_platform_id *platforms = NULL;

clGetPlatformIDs(0,NULL,&numPlatforms);

//platforms es un arreglo de cl_platform_id con esto puede llamase
//a la plataforma que se desee
platforms = (cl_platform_id*)malloc(numPlatforms*sizeof(cl_platform_id));

//Segundo llamado obteniendo el espacio disponible 
status = clGetPlatformIDs(numPlatforms,platforms,NULL);

//------------------------------------------------
//Paso2: Encontrar e inicializar los dispositivos
//------------------------------------------------
cl_uint numDevices = 0;
cl_device_id *devices = NULL;

status = clGetDeviceIDs(platforms[0],CL_DEVICE_TYPE_ALL,0,NULL,&numDevices);

//platforms es un arreglo de cl_device_id con esto puede llamase
//al device que se desee
devices = (cl_device_id*)malloc(numDevices*sizeof(cl_device_id));

status = clGetDeviceIDs(platforms[0],CL_DEVICE_TYPE_ALL, numDevices, devices, NULL);

//------------------------------------------------
//Paso3: Creando un contexto
//------------------------------------------------
cl_context context = NULL;
// Create a context using clCreateContext() and
// associate it with the devices
context = clCreateContext(NULL,numDevices,devices,NULL,NULL,&status);


//------------------------------------------------
//Paso4: Creando un comandqueue
//------------------------------------------------
cl_command_queue cmdQueue;
// Create a command queue using clCreateCommandQueue(),
// and associate it with the device you want to execute
// on
cmdQueue = clCreateCommandQueue(context,devices[0],0,&status);

//------------------------------------------------
//Paso5: Creando un device buffers
//------------------------------------------------
cl_mem buffer_th; // Input array on the device
cl_mem buffer_thR;   // Output array on the device


// Use clCreateBuffer() to create a buffer object (d_A)
// that will contain the data from the host array A

buffer_th  = clCreateBuffer(context,CL_MEM_READ_ONLY,datasize,NULL,&status);

buffer_thR = clCreateBuffer(context,CL_MEM_WRITE_ONLY,datasize,NULL,&status);

//---------------------------------------------------
//Paso6: Escribiendo datos del host al device buffers
//---------------------------------------------------
// Use clEnqueueWriteBuffer() to write input array Re1 to
// the device buffer bufferB
status = clEnqueueWriteBuffer(cmdQueue,	buffer_th,CL_FALSE,0,datasize,cl_th,0,NULL,NULL);


//-----------------------------------------
//Paso7: Creando y conmpilando programas
//-----------------------------------------
cl_program program = clCreateProgramWithSource(context,1,(const char**)&Sourcefactor1,NULL,&status);
// Build (compile) the program for the devices with
// clBuildProgram()
status = clBuildProgram(program,numDevices,devices,NULL,NULL,NULL);



//-----------------------------------------
//Paso8: Creando un kernel
//-----------------------------------------
cl_kernel kernel = NULL;
// Use clCreateKernel() to create a kernel from the
// vector addition function (named "vecadd")
kernel = clCreateKernel(program, "factor", &status);



//------------------------------------------------
//Paso9: Estableciendo los argumentos del kernell
//------------------------------------------------
status  = clSetKernelArg(kernel, 0, sizeof(cl_mem), &buffer_th);
status |= clSetKernelArg(kernel, 1, sizeof(cl_mem), &buffer_thR);


//------------------------------------------------
//Paso10: Configurando el wirk-item structure
//------------------------------------------------
size_t globalWorkSize[2];

// There are ’elements’ work-items
globalWorkSize[0] = elements;

//———————————————————————————————————————————————————
// STEP 11: Enqueue the kernel for execution
//———————————————————————————————————————————————————
// Execute the kernel by using
// clEnqueueNDRangeKernel().
// ’globalWorkSize’ is the 1D dimension of the
// work-items
status = clEnqueueNDRangeKernel(cmdQueue,kernel,1,NULL,globalWorkSize,NULL,0,NULL,NULL);


//------------------------------------------------
// Paso 12: Leyendo el buffer de salida de regreso al host
//------------------------------------------------
clEnqueueReadBuffer(cmdQueue,buffer_thR,CL_TRUE,0,datasize,cl_thR,0,NULL,NULL);
// Verify the output

//———————————————————————————————————————————————————
// STEP 13: Release OpenCL resources
//———————————————————————————————————————————————————
// Free OpenCL resources
clReleaseKernel(kernel);
clReleaseProgram(program);
clReleaseCommandQueue(cmdQueue);
clReleaseMemObject(buffer_th);
clReleaseMemObject(buffer_thR);
clReleaseContext(context);
// Free host resources
free(cl_th);
free(platforms);
free(devices);

}



void g_selec(float* cl_ra, float *cl_tt , float *cl_th, float *d_front, int elements){


// tamaño de los datos
size_t datasize = sizeof(float)*elements;
size_t frontsize = sizeof(float)*2;

//Se crea la variable para revisar la salida de cada 
//llamado
cl_int status;

//------------------------------------------------
//Paso1: Encontrar e inicializar las plataformas
//------------------------------------------------

//Primer llamado para identificar la plataforma
cl_uint numPlatforms = 0;
cl_platform_id *platforms = NULL;

clGetPlatformIDs(0,NULL,&numPlatforms);

//platforms es un arreglo de cl_platform_id con esto puede llamase
//a la plataforma que se desee
platforms = (cl_platform_id*)malloc(numPlatforms*sizeof(cl_platform_id));

//Segundo llamado obteniendo el espacio disponible 
status = clGetPlatformIDs(numPlatforms,platforms,NULL);

//------------------------------------------------
//Paso2: Encontrar e inicializar los dispositivos
//------------------------------------------------
cl_uint numDevices = 0;
cl_device_id *devices = NULL;

status = clGetDeviceIDs(platforms[0],CL_DEVICE_TYPE_ALL,0,NULL,&numDevices);

//platforms es un arreglo de cl_device_id con esto puede llamase
//al device que se desee
devices = (cl_device_id*)malloc(numDevices*sizeof(cl_device_id));

status = clGetDeviceIDs(platforms[0],CL_DEVICE_TYPE_ALL, numDevices, devices, NULL);

//------------------------------------------------
//Paso3: Creando un contexto
//------------------------------------------------
cl_context context = NULL;
// Create a context using clCreateContext() and
// associate it with the devices
context = clCreateContext(NULL,numDevices,devices,NULL,NULL,&status);


//------------------------------------------------
//Paso4: Creando un comandqueue
//------------------------------------------------
cl_command_queue cmdQueue;
// Create a command queue using clCreateCommandQueue(),
// and associate it with the device you want to execute
// on
cmdQueue = clCreateCommandQueue(context,devices[0],0,&status);

//------------------------------------------------
//Paso5: Creando un device buffers
//------------------------------------------------
cl_mem buffer_ra;	   // Input array on the device
cl_mem buffer_front;   // Input array on the device
cl_mem buffer_th;      // Input array on the device
cl_mem buffer_tt;      // Output array on the device


// Use clCreateBuffer() to create a buffer object (d_A)
// that will contain the data from the host array A

buffer_ra     = clCreateBuffer(context,CL_MEM_READ_ONLY,datasize,NULL,&status);
buffer_front  = clCreateBuffer(context,CL_MEM_READ_ONLY,frontsize,NULL,&status);
buffer_th     = clCreateBuffer(context,CL_MEM_READ_ONLY,datasize,NULL,&status);
buffer_tt     = clCreateBuffer(context,CL_MEM_WRITE_ONLY,datasize,NULL,&status);

//---------------------------------------------------
//Paso6: Escribiendo datos del host al device buffers
//---------------------------------------------------
// Use clEnqueueWriteBuffer() to write input array Re1 to
// the device buffer bufferB
status = clEnqueueWriteBuffer(cmdQueue,	buffer_ra,CL_FALSE,0,datasize,cl_ra,0,NULL,NULL);
status = clEnqueueWriteBuffer(cmdQueue,	buffer_front,CL_FALSE,0,frontsize,d_front,0,NULL,NULL);
status = clEnqueueWriteBuffer(cmdQueue,	buffer_th,CL_FALSE,0,datasize,cl_th,0,NULL,NULL);


//-----------------------------------------
//Paso7: Creando y conmpilando programas
//-----------------------------------------
cl_program program = clCreateProgramWithSource(context,1,(const char**)&Sourceselec,NULL,&status);
// Build (compile) the program for the devices with
// clBuildProgram()
status = clBuildProgram(program,numDevices,devices,NULL,NULL,NULL);



//-----------------------------------------
//Paso8: Creando un kernel
//-----------------------------------------
cl_kernel kernel = NULL;
// Use clCreateKernel() to create a kernel from the
// vector addition function (named "vecadd")
kernel = clCreateKernel(program, "selec", &status);



//------------------------------------------------
//Paso9: Estableciendo los argumentos del kernell
//------------------------------------------------
status  = clSetKernelArg(kernel, 0, sizeof(cl_mem), &buffer_ra);
status |= clSetKernelArg(kernel, 1, sizeof(cl_mem), &buffer_front);
status |= clSetKernelArg(kernel, 2, sizeof(cl_mem), &buffer_th);
status |= clSetKernelArg(kernel, 3, sizeof(cl_mem), &buffer_tt);

//------------------------------------------------
//Paso10: Configurando el wirk-item structure
//------------------------------------------------
size_t globalWorkSize[1];

// There are ’elements’ work-items
globalWorkSize[0] = elements;

//———————————————————————————————————————————————————
// STEP 11: Enqueue the kernel for execution
//———————————————————————————————————————————————————
// Execute the kernel by using
// clEnqueueNDRangeKernel().
// ’globalWorkSize’ is the 1D dimension of the
// work-items
status = clEnqueueNDRangeKernel(cmdQueue,kernel,1,NULL,globalWorkSize,NULL,0,NULL,NULL);


//------------------------------------------------
// Paso 12: Leyendo el buffer de salida de regreso al host
//------------------------------------------------
clEnqueueReadBuffer(cmdQueue,buffer_tt,CL_TRUE,0,datasize,cl_tt,0,NULL,NULL);
// Verify the output

//———————————————————————————————————————————————————
// STEP 13: Release OpenCL resources
//———————————————————————————————————————————————————
// Free OpenCL resources
clReleaseKernel(kernel);
clReleaseProgram(program);
clReleaseCommandQueue(cmdQueue);

clReleaseMemObject(buffer_ra);
clReleaseMemObject(buffer_front);
clReleaseMemObject(buffer_th);
clReleaseMemObject(buffer_tt);

clReleaseContext(context);
// Free host resources
free(platforms);
free(devices);
free(cl_ra);
free(cl_th);
}



void g_reordenacion(float *cl_image_Re, float *cl_img, int elements){

size_t datasize = sizeof(float)*elements;

cl_int status;

//------------------------------------------------
//Paso1: Encontrar e inicializar las plataformas
//------------------------------------------------

//Primer llamado para identificar la plataforma
cl_uint numPlatforms = 0;
cl_platform_id *platforms = NULL;

clGetPlatformIDs(0,NULL,&numPlatforms);

//platforms es un arreglo de cl_platform_id con esto puede llamase
//a la plataforma que se desee
platforms = (cl_platform_id*)malloc(numPlatforms*sizeof(cl_platform_id));

//Segundo llamado obteniendo el espacio disponible 
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
cl_mem bufferimage_Re; // Input array on the device

cl_mem bufferimg;   // Output array on the device

// Use clCreateBuffer() to create a buffer object (d_A)
// that will contain the data from the host array A

bufferimage_Re = clCreateBuffer(context,CL_MEM_READ_ONLY,datasize,NULL,&status);

bufferimg      = clCreateBuffer(context,CL_MEM_WRITE_ONLY,datasize,NULL,&status);


//---------------------------------------------------
//Paso6: Escribiendo datos del host al device buffers
//---------------------------------------------------
// Use clEnqueueWriteBuffer() to write input array Re1 to
// the device buffer bufferB
status = clEnqueueWriteBuffer(cmdQueue,	bufferimage_Re,CL_FALSE,0,datasize,cl_image_Re,0,NULL,NULL);


//-----------------------------------------
//Paso7: Creando y conmpilando programas
//-----------------------------------------
cl_program program = clCreateProgramWithSource(context,1,(const char**)&reordenacion,NULL,&status);
// Build (compile) the program for the devices with
// clBuildProgram()
status = clBuildProgram(program,numDevices,devices,NULL,NULL,NULL);



//-----------------------------------------
//Paso8: Creando un kernel
//-----------------------------------------
cl_kernel kernel = NULL;
// Use clCreateKernel() to create a kernel from the
// vector addition function (named "vecadd")
kernel = clCreateKernel(program, "sort", &status);



//------------------------------------------------
//Paso9: Estableciendo los argumentos del kernell
//------------------------------------------------
status  = clSetKernelArg(kernel, 0, sizeof(cl_mem), &bufferimage_Re);

status |= clSetKernelArg(kernel, 1, sizeof(cl_mem), &bufferimg);


//------------------------------------------------
//Paso10: Configurando el wirk-item structure
//------------------------------------------------
size_t globalWorkSize[1];

// There are ’elements’ work-items
globalWorkSize[0] = elements;


//———————————————————————————————————————————————————
// STEP 11: Enqueue the kernel for execution
//———————————————————————————————————————————————————
// Execute the kernel by using
// clEnqueueNDRangeKernel().
// ’globalWorkSize’ is the 1D dimension of the
// work-items
status = clEnqueueNDRangeKernel(cmdQueue,kernel,1,NULL,globalWorkSize,NULL,0,NULL,NULL);


//------------------------------------------------
// Paso 12: Leyendo el buffer de salida de regreso al host
//------------------------------------------------
clEnqueueReadBuffer(cmdQueue,bufferimg,CL_TRUE,0,datasize,cl_img,0,NULL,NULL);
// Verify the output


//———————————————————————————————————————————————————
// STEP 13: Release OpenCL resources
//———————————————————————————————————————————————————
// Free OpenCL resources
clReleaseKernel(kernel);
clReleaseProgram(program);
clReleaseCommandQueue(cmdQueue);
clReleaseMemObject(bufferimage_Re);
clReleaseMemObject(bufferimg);
clReleaseContext(context);
// Free host resources
free(cl_image_Re);
free(platforms);
free(devices);


}



void g_calc(float *Re1, float *Im1, float *Re2, float *Im2, float *p11, float *p12, int elements){


// tamaño de los datos
size_t datasize = sizeof(float)*elements;


//Se crea la variable para revisar la salida de cada 
//llamado
cl_int status;

//------------------------------------------------
//Paso1: Encontrar e inicializar las plataformas
//------------------------------------------------

//Primer llamado para identificar la plataforma
cl_uint numPlatforms = 0;
cl_platform_id *platforms = NULL;

clGetPlatformIDs(0,NULL,&numPlatforms);

//platforms es un arreglo de cl_platform_id con esto puede llamase
//a la plataforma que se desee
platforms = (cl_platform_id*)malloc(numPlatforms*sizeof(cl_platform_id));

//Segundo llamado obteniendo el espacio disponible 
status = clGetPlatformIDs(numPlatforms,platforms,NULL);

//------------------------------------------------
//Paso2: Encontrar e inicializar los dispositivos
//------------------------------------------------
cl_uint numDevices = 0;
cl_device_id *devices = NULL;

status = clGetDeviceIDs(platforms[0],CL_DEVICE_TYPE_ALL,0,NULL,&numDevices);

//platforms es un arreglo de cl_device_id con esto puede llamase
//al device que se desee
devices = (cl_device_id*)malloc(numDevices*sizeof(cl_device_id));

status = clGetDeviceIDs(platforms[0],CL_DEVICE_TYPE_ALL, numDevices, devices, NULL);

//------------------------------------------------
//Paso3: Creando un contexto
//------------------------------------------------
cl_context context = NULL;
// Create a context using clCreateContext() and
// associate it with the devices
context = clCreateContext(NULL,numDevices,devices,NULL,NULL,&status);


//------------------------------------------------
//Paso4: Creando un comandqueue
//------------------------------------------------
cl_command_queue cmdQueue;
// Create a command queue using clCreateCommandQueue(),
// and associate it with the device you want to execute
// on
cmdQueue = clCreateCommandQueue(context,devices[0],0,&status);

//------------------------------------------------
//Paso5: Creando un device buffers
//------------------------------------------------
cl_mem bufferRe1; // Input array on the device
cl_mem bufferIm1; // Input array on the device
cl_mem bufferRe2; // Input array on the device
cl_mem bufferIm2; // Input array on the device
cl_mem bufferp1;   // Output array on the device
cl_mem bufferp2;   // Output array on the device

// Use clCreateBuffer() to create a buffer object (d_A)
// that will contain the data from the host array A

bufferRe1 = clCreateBuffer(context,CL_MEM_READ_ONLY,datasize,NULL,&status);
bufferIm1 = clCreateBuffer(context,CL_MEM_READ_ONLY,datasize,NULL,&status);
bufferRe2 = clCreateBuffer(context,CL_MEM_READ_ONLY,datasize,NULL,&status);
bufferIm2 = clCreateBuffer(context,CL_MEM_READ_ONLY,datasize,NULL,&status);

bufferp1 = clCreateBuffer(context,CL_MEM_WRITE_ONLY,datasize,NULL,&status);
bufferp2 = clCreateBuffer(context,CL_MEM_WRITE_ONLY,datasize,NULL,&status);


//---------------------------------------------------
//Paso6: Escribiendo datos del host al device buffers
//---------------------------------------------------
// Use clEnqueueWriteBuffer() to write input array Re1 to
// the device buffer bufferB
status = clEnqueueWriteBuffer(cmdQueue,	bufferRe1,CL_FALSE,0,datasize,Re1,0,NULL,NULL);
status = clEnqueueWriteBuffer(cmdQueue,	bufferIm1,CL_FALSE,0,datasize,Im1,0,NULL,NULL);
status = clEnqueueWriteBuffer(cmdQueue,	bufferRe2,CL_FALSE,0,datasize,Re2,0,NULL,NULL);
status = clEnqueueWriteBuffer(cmdQueue,	bufferIm2,CL_FALSE,0,datasize,Im2,0,NULL,NULL);


//-----------------------------------------
//Paso7: Creando y conmpilando programas
//-----------------------------------------
cl_program program = clCreateProgramWithSource(context,1,(const char**)&programSourceR,NULL,&status);
// Build (compile) the program for the devices with
// clBuildProgram()
status = clBuildProgram(program,numDevices,devices,NULL,NULL,NULL);



//-----------------------------------------
//Paso8: Creando un kernel
//-----------------------------------------
cl_kernel kernel = NULL;
// Use clCreateKernel() to create a kernel from the
// vector addition function (named "vecadd")
kernel = clCreateKernel(program, "genr", &status);



//------------------------------------------------
//Paso9: Estableciendo los argumentos del kernell
//------------------------------------------------
status  = clSetKernelArg(kernel, 0, sizeof(cl_mem), &bufferRe1);
status |= clSetKernelArg(kernel, 1, sizeof(cl_mem), &bufferIm1);
status |= clSetKernelArg(kernel, 2, sizeof(cl_mem), &bufferRe2);
status |= clSetKernelArg(kernel, 3, sizeof(cl_mem), &bufferIm2);

status |= clSetKernelArg(kernel, 4, sizeof(cl_mem), &bufferp1);
status |= clSetKernelArg(kernel, 5, sizeof(cl_mem), &bufferp2);


//------------------------------------------------
//Paso10: Configurando el wirk-item structure
//------------------------------------------------
size_t globalWorkSize[1];

// There are ’elements’ work-items
globalWorkSize[0] = elements;

//———————————————————————————————————————————————————
// STEP 11: Enqueue the kernel for execution
//———————————————————————————————————————————————————
// Execute the kernel by using
// clEnqueueNDRangeKernel().
// ’globalWorkSize’ is the 1D dimension of the
// work-items
status = clEnqueueNDRangeKernel(cmdQueue,kernel,1,NULL,globalWorkSize,NULL,0,NULL,NULL);


//------------------------------------------------
// Paso 12: Leyendo el buffer de salida de regreso al host
//------------------------------------------------
clEnqueueReadBuffer(cmdQueue,bufferp1,CL_TRUE,0,datasize,p11,0,NULL,NULL);
clEnqueueReadBuffer(cmdQueue,bufferp2,CL_TRUE,0,datasize,p12,0,NULL,NULL);
// Verify the output

//———————————————————————————————————————————————————
// STEP 13: Release OpenCL resources
//———————————————————————————————————————————————————
// Free OpenCL resources
clReleaseKernel(kernel);
clReleaseProgram(program);
clReleaseCommandQueue(cmdQueue);
clReleaseMemObject(bufferRe1);
clReleaseMemObject(bufferIm1);
clReleaseMemObject(bufferRe2);
clReleaseMemObject(bufferIm2);
clReleaseMemObject(bufferp1);
clReleaseMemObject(bufferp2);
clReleaseContext(context);
// Free host resources
free(Re1);
free(Im1);
free(Re2);
free(Im2);
free(platforms);
free(devices);


}
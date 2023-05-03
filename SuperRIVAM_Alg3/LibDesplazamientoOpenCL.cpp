#include "LibDesplazamientoOpenCL.h"



// OpenCL kernel to perform an element-wise
// add of two arrays
const char* programSource =
"__kernel \n"
"void vect(__global float *p_Re1, \n"
" __global float *p_Im1, \n"
" __global float *p_Re2, \n"
" __global float *p_Im2, \n"
" __global float *p_R)   \n "
"{ \n"
" \n"
" // Get the work-item’s unique ID \n"
" int idx = get_global_id(0); \n"
" \n"
" // Add the corresponding locations of \n"
" // Calculando la tangente entre Im1 y Re1 como Im2 y Re2 \n"
" // Restando estos resultados. \n"
" p_R[idx] = atan2(p_Im1[idx],p_Re1[idx]) - atan2(p_Im2[idx],p_Re2[idx]); \n"
"	if( p_R[idx] < -3.1415926535){\n"
"	p_R[idx] = p_R[idx] + (float)2*3.1415926535 ;\n"
"	}\n"
"	if( p_R[idx] > 3.14159265){ \n"
"	p_R[idx] = p_R[idx] - (float)2*3.1415926535 ;\n"
"	}"

"} \n"
;


void g_tang(float *cl_Re1, float *cl_Im1, float *cl_Re2, float *cl_Im2, float *cl_R, int elements){


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
cl_mem bufferR;   // Output array on the device
// Use clCreateBuffer() to create a buffer object (d_A)
// that will contain the data from the host array A
bufferRe1 = clCreateBuffer(context,CL_MEM_READ_ONLY,datasize,NULL,&status);
bufferIm1 = clCreateBuffer(context,CL_MEM_READ_ONLY,datasize,NULL,&status);
bufferIm2 = clCreateBuffer(context,CL_MEM_READ_ONLY,datasize,NULL,&status);
bufferRe2 = clCreateBuffer(context,CL_MEM_READ_ONLY,datasize,NULL,&status);

bufferR = clCreateBuffer(context,CL_MEM_WRITE_ONLY,datasize,NULL,&status);


//---------------------------------------------------
//Paso6: Escribiendo datos del host al device buffers
//---------------------------------------------------
// Use clEnqueueWriteBuffer() to write input array Re1 to
// the device buffer bufferB
status = clEnqueueWriteBuffer(cmdQueue,	bufferRe1,CL_FALSE,0,datasize,cl_Re1,0,NULL,NULL);
// Use clEnqueueWriteBuffer() to write input array Im1 to
// the device buffer bufferA
status = clEnqueueWriteBuffer(cmdQueue,bufferIm1,CL_FALSE,0,datasize,cl_Im1,0,NULL,NULL);
// Use clEnqueueWriteBuffer() to write input array Re2 to
// the device buffer bufferB
status = clEnqueueWriteBuffer(cmdQueue,	bufferRe2,CL_FALSE,0,datasize,cl_Re2,0,NULL,NULL);
// Use clEnqueueWriteBuffer() to write input array Im2 to
// the device buffer bufferB
status = clEnqueueWriteBuffer(cmdQueue,	bufferIm2,CL_FALSE,0,datasize,cl_Im2,0,NULL,NULL);



//-----------------------------------------
//Paso7: Creando y conmpilando programas
//-----------------------------------------
cl_program program = clCreateProgramWithSource(context,1,(const char**)&programSource,NULL,&status);
// Build (compile) the program for the devices with
// clBuildProgram()
status = clBuildProgram(program,numDevices,devices,NULL,NULL,NULL);



//-----------------------------------------
//Paso8: Creando un kernel
//-----------------------------------------
cl_kernel kernel = NULL;
// Use clCreateKernel() to create a kernel from the
// vector addition function (named "vecadd")
kernel = clCreateKernel(program, "vect", &status);



//------------------------------------------------
//Paso9: Estableciendo los argumentos del kernell
//------------------------------------------------
status = clSetKernelArg(kernel,0,sizeof(cl_mem),&bufferRe1);
status |= clSetKernelArg(kernel,1,sizeof(cl_mem),&bufferIm1);
status |= clSetKernelArg(kernel,2,sizeof(cl_mem),&bufferRe2);
status |= clSetKernelArg(kernel,3,sizeof(cl_mem),&bufferIm2);
status |= clSetKernelArg(kernel,4,sizeof(cl_mem),&bufferR);


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
//Paso12: Leyendo el buffer de salida de regreso al host
//------------------------------------------------
clEnqueueReadBuffer(cmdQueue,bufferR,CL_TRUE,0,datasize,cl_R,0,NULL,NULL);
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
clReleaseMemObject(bufferR);
clReleaseContext(context);
// Free host resources
free(cl_Re1);
free(cl_Im1);
free(cl_Re2);
free(cl_Im2);
free(platforms);
free(devices);

}
#ifndef _DICOM_IO_H_
#define _DICOM_IO_H_

#include <cv.h>
#include <cxcore.h>
#include <highgui.h>


#include <opencv2\highgui\highgui.hpp>
#include <opencv2\core\core.hpp>
#include <opencv2\gpu\gpu.hpp>


//includes de VTK
#include <vtkSmartPointer.h>
#include <vtkImageViewer2.h>
#include <vtkDICOMImageReader.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkImageData.h>
#include <vtkImageCast.h>
#include <vtkUnsignedCharArray.h>
#include <vtkFloatArray.h>
#include <vtkJPEGWriter.h>
#include <vtkWriter.h>
#include <vtkPNGWriter.h>

//includes GDCM
#include <vtkGDCMImageReader.h>
#include <vtkGDCMImageWriter.h>
#include <vtkImageData.h>
#include <vtkImageMagnify.h>
#include <vtkImageCast.h>
#include <gdcmTesting.h>
#include <gdcmSystem.h>


// Lee una Imagen DICOM y lo convierte a la estructura IplImage
IplImage *LeerDicom_IplImage( char *nombre);



// Lee una Imagen DICOM y lo convierte a la estructura cv::Mat
cv::Mat LeerDicom_cvMat( char *nombre);



// Amplifica una imagen DICOM en un factor dado
// Para grabar imagenes DICOM es necesario usar esta funcion u guardarla  
// asi se crea una estrctura DICOM un factor de veces mas grande que el 
// original
void Amp_Factor(char *nombre, int factor, char *archivo_guardar_nombre );



//Sirve para mostrar la imagen DICOM
void MostrarDICOM(char *nombre);



// Graba una imagen dicom, para ello, en el codigo de esta funcion
// es necesario escribir el nombre "archivo_guardar_nombre" de la
// funcion Amp_Factor
void IplImageDICOM(char *guardar, IplImage *X);


#endif

/*
  This header has functions and class to read and work with DICOM Images and OpenCV
*/
#ifndef _COMMON_DICOM_H_
#define _COMMON_DICOM_H_

#include <vtkSmartPointer.h>
#include <vtkImageViewer2.h>
#include <vtkDICOMImageReader.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkImageData.h>
#include <opencv\cv.h>
#include <opencv\highgui.h>
#include <vtkImageCast.h>
#include <vtkUnsignedCharArray.h>
#include <vtkFloatArray.h>
#include <vtkJPEGWriter.h>
#include <vtkWriter.h>
#include <vtkPNGWriter.h>


//includes GDCM
/*
#include <vtkGDCMImageReader.h>
#include <vtkGDCMImageWriter.h>
#include <vtkImageData.h>
#include <vtkImageMagnify.h>
#include <vtkImageCast.h>
#include <gdcmTesting.h>
#include <gdcmSystem.h>
*/

#define IPV6STRICT


IplImage* ConvertDicom2OpenCV(vtkImageData *VTKImage, int Image_Width, int Image_Height);

// Graba una imagen dicom, para ello, en el codigo de esta funcion
// es necesario escribir el nombre "archivo_guardar_nombre" de la
// funcion Amp_Factor
//void IplImageDICOM(char *guardar, IplImage *X);

void F64to8f(IplImage *ref,IplImage *dst);

void F16to64f(IplImage *ref,IplImage *dst);


#endif
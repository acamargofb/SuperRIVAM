#ifndef _DICOM_IO_H_
#define _DICOM_IO_H_

#include "cv.h"
#include "highgui.h"	
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

IplImage *LeerDicom_IplImage( char *nombre);

cv::Mat LeerDicom_cvMat( char *nombre);

void Amp_Factor(char *nombre, int factor, char *archivo_guardar_nombre );

void MostrarDICOM(char *nombre);

void IplImageDICOM(char *guardar, IplImage *X);

#endif

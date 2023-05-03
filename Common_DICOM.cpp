#include "Common_DICOM.h"

IplImage* ConvertDicom2OpenCV(vtkImageData *VTKImage, int Image_Width, int Image_Height){

IplImage* temp = NULL;
int i, j;

temp = cvCreateImage( cvSize(Image_Width, Image_Height ),IPL_DEPTH_32F,1 );

  for(i=0; i< Image_Height; i++){
	  for(j=0; j< Image_Width; j++){
		 // if (VTKImage->GetScalarComponentAsDouble(i,j,0,0)/512 != 0.0){
		  //   printf("Img[%d][%d] = %f", i,j, VTKImage->GetScalarComponentAsDouble(i,j,0,0)/512);
	      // }
		  cvSetReal2D(temp,i,j,VTKImage->GetScalarComponentAsDouble(i,j,0,0)/512);  
		//  getchar();
	  }
  }
  return temp;
}

void F64to8f(IplImage *ref,IplImage *dst){
	int i,j;
	double temp;

	for(i=0; i<ref->height ;i++){
		for(j=0; j<ref->width ; j++){
		temp = cvGetReal2D(ref,i,j)*255;
		cvSetReal2D(dst,i,j, temp );
		}
	}
}

void F16to64f(IplImage *ref,IplImage *dst){
	int i,j;
	double temp;

	for(i=0; i<ref->height ;i++){
		for(j=0; j<ref->width ; j++){
		temp = cvGetReal2D(ref,i,j)/65535;
		cvSetReal2D(dst,i,j, temp );
		}
	}
}

/*

void IplImageDICOM(char *guardar, IplImage *X){

  int i,j;
  //pasando la estructura a vtk para poder cargar la imagen y luego grabarlo
  // Creando un objeto VTK, Reader
  vtkSmartPointer<vtkDICOMImageReader> readerf = vtkSmartPointer<vtkDICOMImageReader>::New();
  vtkSmartPointer<vtkImageData> f3 = vtkSmartPointer<vtkImageData>::New();

  //ingresando la direccion de un archivo dicom
  readerf->SetFileName("salida");
  readerf->SetDataScalarTypeToFloat();

  //actualizando
  readerf->Update();
  f3 = readerf->GetOutput();

  //Transformado IPLIMAGE A DICOM
   for(i=0; i<readerf->GetHeight(); i++){

	  float* ptr = (float*)(X->imageData + i*X->widthStep);
	  for(j=0; j<readerf->GetWidth(); j++){
			
		  f3->SetScalarComponentFromDouble(i,j,0,0, ptr[j]);
	  }
  }
  readerf->SetOutput(f3);
  readerf->Update();

  //grabando la imagen
  vtkGDCMImageWriter *writer = vtkGDCMImageWriter::New();
  writer->SetFileName( guardar );
  writer->SetInput( readerf->GetOutput() );
  writer->Update();
  writer->Write();

}

*/

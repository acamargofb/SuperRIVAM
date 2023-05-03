#include "DICOM_IO.h"
#include "LibIteraciones.h"

IplImage *LeerDicom_IplImage( char *nombre){

int i,j;

//Creando el puntero a la imagen DICOM
vtkSmartPointer<vtkDICOMImageReader> reader  =	  vtkSmartPointer<vtkDICOMImageReader>::New();
vtkImageData *f;

//ingresando la direccion de un archivo dicom
reader->SetFileName(nombre);
reader->SetDataScalarTypeToFloat();
reader->Update();   //actualizando
;
//Asignando los datos en reader a f para que se pueda tener acceso a los pixeles
f = reader->GetOutput();

//Copiando los datos a OpenCV
IplImage *img;
img = cvCreateImage( cvSize(reader->GetWidth(), reader->GetHeight() ),IPL_DEPTH_32F,1 );

for(i=0; i<reader->GetHeight(); i++){
  float* ptr = (float*)(img->imageData + i*img->widthStep);
  for(j=0; j<reader->GetWidth(); j++){
	ptr[j] = f->GetScalarComponentAsFloat(i,j,0,0);  
  }
}
	
return img;

}


cv::Mat LeerDicom_cvMat( char *nombre){

int i,j;

//Creando el puntero a la imagen DICOM
vtkSmartPointer<vtkDICOMImageReader> reader  =	  vtkSmartPointer<vtkDICOMImageReader>::New();
vtkImageData *f;

//ingresando la direccion de un archivo dicom
reader->SetFileName(nombre);
reader->SetDataScalarTypeToFloat();
reader->Update();   //actualizando

//Asignando los datos en reader a f1 para que se pueda tener acceso a los pixeles
f = reader->GetOutput();

//Copiando los datos a OpenCV
cv::Mat img;
img.create(reader->GetHeight(), reader->GetWidth(), CV_32FC1 );

for(i=0; i<reader->GetHeight(); i++){
  float * pt = img.ptr<float>(i)	;
  for(j=0; j<reader->GetWidth(); j++){
	  pt[j] =  f->GetScalarComponentAsFloat(i,j,0,0);
  }
}


return img;
}


void Amp_Factor(char *nombre, int factor, char *archivo_guardar_nombre ){

  //Creando el puntero a la imagen DICOM
  vtkSmartPointer<vtkDICOMImageReader> reader  =  vtkSmartPointer<vtkDICOMImageReader>::New();

  //ingresando la direccion de un archivo dicom 
  reader->SetFileName(nombre);
  reader->SetDataScalarTypeToFloat();
  reader->Update();   //actualizando

 //Creando una estructura mas grande en un facctor dado
  vtkImageMagnify *magnify = vtkImageMagnify::New();
  magnify->SetInput( reader->GetOutput() );
  magnify->SetInterpolate( 1 );
  magnify->SetInterpolate( 0 );
  magnify->SetMagnificationFactors (factor, factor, 1);

  if ( archivo_guardar_nombre != ""){
  //grabando la imagen
  vtkGDCMImageWriter *writer = vtkGDCMImageWriter::New();
  writer->SetFileName( archivo_guardar_nombre );
  writer->SetInput( magnify->GetOutput() );
  writer->Write();
  writer->Delete();
  }

  magnify->Delete();
  

}


void MostrarDICOM(char *nombre){
  
  //Creando el puntero a la imagen DICOM
  vtkSmartPointer<vtkDICOMImageReader> reader  =	  vtkSmartPointer<vtkDICOMImageReader>::New();
 
  //ingresando la direccion de un archivo dicom
  reader->SetFileName(nombre);
  reader->SetDataScalarTypeToFloat();
  reader->Update();   //actualizando
  
  // Funciones para visualizar la imagen DICOM	
  vtkSmartPointer<vtkImageViewer2> imageViewer =  vtkSmartPointer<vtkImageViewer2>::New();
  imageViewer->SetInput( reader->GetOutput());
  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
  imageViewer->SetupInteractor(renderWindowInteractor);
  imageViewer->Render();
  imageViewer->GetRenderer()->ResetCamera();
  imageViewer->Render();
  renderWindowInteractor->Start();


}


void IplImageDICOM(char *guardar, IplImage *X){

  int i,j;
  //pasando la estructura a vtk para poder cargar la imagen y luego grabarlo
  // Creando un objeto VTK, Reader
  vtkSmartPointer<vtkDICOMImageReader> readerf = vtkSmartPointer<vtkDICOMImageReader>::New();
  vtkSmartPointer<vtkImageData> f3 = vtkSmartPointer<vtkImageData>::New();

  //ingresando la direccion de un archivo dicom
  readerf->SetFileName("salida");

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
  vtkGDCMImageWriter *writerf = vtkGDCMImageWriter::New();
  writerf->SetFileName( guardar );
  writerf->SetInput( readerf->GetOutput() );
  writerf->Update();
  writerf->Write();
  writerf->Delete();
	
}



void CvMatDICOM(char *guardar, cv::Mat X){

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

	  for(j=0; j<readerf->GetWidth(); j++){
			
		  f3->SetScalarComponentFromDouble(i,j,0,0, X.at<float>(i,j));
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
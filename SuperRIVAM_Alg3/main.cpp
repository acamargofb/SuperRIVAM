#include "LibRotacion.h"
#include "LibDesplazamiento.h"
#include "LibIteraciones.h"

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


#include "DICOM_IO.h"

int main(){

SYSTEMTIME t1;
SYSTEMTIME t2;
GetSystemTime(&t1);

//*************************************************************************//
// Especificacion el numero de imagenes a ingresar, variable a enlazar con 
// la interfaz grafica 
const int N = 2;
//
//*************************************************************************//
int i;

/*Definiendo la variable donde se alamcenara los desplazamientos estimados*/
CvMat* delta_est; 

/*Definiendo la variables necesarias para ejecutar cada uno  de los 3 algoritmos*/
IplImage *reg[N];



/************************************************************************************/
/************************************************************************************/
// Ingresando las imagenes del registro									   //
// Rutas a asginar mediante la interfaz grafica

  char *nombre = "31425514";
  reg[0] = LeerDicom_IplImage( nombre );

  char *out = "salida";
  int fact = 5;
  Amp_Factor( nombre, fact, out );
  /*
  cv::Mat Y = LeerDicom_IplImage( out );
  std::vector<int> a(1);
  a[0] = 9;
  cv::imwrite("Img_Referencia.png",Y, a);
  */
  //Mostrando la Imagen DICOM
  //MostrarDICOM(out);

  nombre = "31425514";
  reg[1] = LeerDicom_IplImage( nombre ); 
 


//*************************************************************************//
//********************************zz*****************************************//



/************************************************************************************/
/************************************************************************************/
/*Definiendo la variables para calcular las rotaciones entre cada par de imagenes*/
IplImage *h_A[N];
IplImage *H_AI[N];
IplImage *H_AR[N]; 
double *d;
float exc;
int n = 25; //parametro para estimacion de desplazamiento
double factor;

/*Definiendo la variable donde se almacenara la rotacion estimada*/
	CvMat *angle_est;
	IplImage *delta, *phi;

// Caculo de las estimaciones de rotacion											//
	int wd, he;
	IplImage *pblur[N],*w;

/* Creando la imagen w, que se empleara para filtrar cada imagen del registro y */
/* eliminar el fenomeno de aliasing*/
	wd = reg[0]->width;
	he = reg[0]->height;

	w = cvCreateImage( cvSize(wd,he), IPL_DEPTH_64F, 1);
	tukeywin( w, 1);

	for(i=0; i<N; i++ ){
		pblur[i] = cvCreateImage( cvSize(wd,he), IPL_DEPTH_32F, 1);
	}
	
	for(i=0; i<N; i++ ){
		cvMul( reg[i], w, pblur[i], 1 );/*
		cv::Mat temp = pblur[i];
		cv::blur(temp,temp,cvSize(wd,he),cvPoint(-1,-1),0);
		ArrtoIplImageR(temp , pblur[i]);*/

	}

	//Eliminando w
	cvReleaseImage(&w);
	angle_est = cvCreateMat( 1, N, CV_64FC1 );

	/* Cargando los Parametros para iniciar el Algoritmo de Rotacion */
	d = (double*) malloc( 1*2*sizeof(int) );
		d[0]=0.515;
		d[1]=0.585;
	exc = (float)0.13;

	est_rotacion( pblur , d, exc, angle_est, N, h_A, H_AI, H_AR );

	fprintf(stdout,"El angulo determinado entre cada imagen y la imagen de referencia es: \n ");

	for(i=0; i<N; i++ ){
			fprintf(stdout,"Angulo[%i] :  %.15f \n ",i ,cvGetReal2D(angle_est,0,i));

	}

	cvReleaseImage(h_A);
	cvReleaseImage(H_AR);
	cvReleaseImage(H_AI);
/************************************************************************************/
/************************************************************************************/





/************************************************************************************/
/************************************************************************************/
// Caculo de las estimaciones de desplazamiento										//
	IplImage *tmp[N];

	for(i=0; i<N; i++ ){
		tmp[i]=cvCreateImage(cvGetSize(reg[0]),IPL_DEPTH_32F,1);
		rotarImagen( pblur[i] , tmp[i] , -cvGetReal2D(angle_est,0,i) );
	}

	//Eliminando Pblur 
	cvReleaseImage(pblur);

	delta_est = cvCreateMat(N,2,CV_64FC1);
	/* Obteniendo la estimacion por desplazamientos */

	estimacion_desplazamiento( tmp, n, delta_est, N);

	//Eliminando tmp
	cvReleaseImage(tmp);

	fprintf(stdout, "\nEl desplazamiento x e y entre cada imagen y la imagen de referencia es:\n");
	for( i=0; i<N; i++){
		fprintf(stdout, " x: %.15f  , y: %.15f\n",cvGetReal2D(delta_est,i,0) ,
		cvGetReal2D(delta_est,i,1) );	
	}


/************************************************************************************/
/************************************************************************************/





/************************************************************************************/
/************************************************************************************/
// Algoritmo iterative Back Projecting para generar la imagen en super resolucion	//

	fprintf( stdout, "\nUsage: Iterative Back Projecting <ref> <rgt>\n" );

// Especificando el factor de incremento de la imagen en superresolucion respecto de//
// la imagen en baja resolucion												 	    //
	factor = (double)fact;

	delta = cvCreateImage(cvSize(2,N),IPL_DEPTH_64F,1);
	phi   = cvCreateImage(cvSize(1,N),IPL_DEPTH_64F,1);

	for(i=0; i<delta->height ; i++ ){
		cvSetReal2D( delta, i, 0, cvGetReal2D(delta_est,i,0) ); 
		cvSetReal2D( delta, i, 1, cvGetReal2D(delta_est,i,1) ); 
	}

	for(i=0; i<phi->height ; i++ ){
		cvSetReal2D( phi, i, 0, cvGetReal2D(angle_est,0,i) );  
	}

	IterativeBackProjecting(reg , delta , phi , factor , N);

  GetSystemTime(&t2);

  delta_tiempo(t1, t2);

/************************************************************************************/
/************************************************************************************/


	cvReleaseImage(reg);


}
/* Applied Video Analysis of Sequences (AVSA)
 *
 *	LAB2: Blob detection & classification
 *	Lab2.0: Sample Opencv project
 * 
 *
 * Authors: José M. Martínez (josem.martinez@uam.es), Paula Moral (paula.moral@uam.es), Juan C. San Miguel (juancarlos.sanmiguel@uam.es)
 */

//system libraries C/C++
#include <stdio.h>
#include <iostream>
#include <sstream>
#include <string>
#include <algorithm>

//opencv libraries
#include <opencv2/opencv.hpp>
#include <opencv2/video/background_segm.hpp>

//Header ShowManyImages
#include "headers/ShowManyImages.hpp"
#include "headers/AllMethods.hpp"
#include "headers/blobs.hpp"
#include "headers/KalmanFilter.hpp"

//namespaces
using namespace cv;
using namespace std;


//main function
int main(int argc, char ** argv) 
{

  // 1. Read Video
  string path = read_video(argc, argv);
  if(path == "-1") return -1;

  // 2. OPEN VIDEO
  VideoCapture cap(path) ;
  if(!cap.isOpened()) return -1;

  Mat frame,fgmask;
  vector<cvBlob> bloblist; // list for blobs
  vector<cvBlob> filteredBloblist; // list for blobs

  // kalman Tracking
  kalman k(1);
//  cout<<"A"<<k.getA()<<endl;
//  cout<<"P"<<k.getP()<<endl;
//  cout<<"Q"<<k.getQ()<<endl;
//  cout<<"H"<<k.getH()<<endl;
//  cout<<"R"<<k.getR()<<endl;


  while(true){

	  // current frame
      cap.read(frame);
	  if(!frame.data) break;

       // 3. BACKGROUND SUBTRACTION
	  fgmask =  background_subtraction(frame,LEARNING_RATE,HISTORY,varTHERSHOLD);

	  // 4. MORPHOLOGCAL OPENING ..
      fgmask = morphological_operation(fgmask ,MOR_SIZE, MOR_TYPE);

      // 5. BLOB EXTRACTION
      extractBlobs(fgmask, bloblist, CONNECTIVITY);

      // 6. FILTER SMALL BLOBS
	  removeSmallBlobs(bloblist, filteredBloblist, MIN_WIDTH, MIN_HEIGHT);

	  // 7. only Max blob
	  Point center = getCentereOfMaxBlob(filteredBloblist);

	  //8. Kalman Tracking
      Point new_center = k.make_prediction(center);

      putText(frame,"+", new_center, FONT_HERSHEY_SIMPLEX, 2, Scalar(0,0,255));
      putText(frame,"-", center, FONT_HERSHEY_SIMPLEX, 2, Scalar(0,0,255));

     // Man i stoped here doing the trajecktory
      // Good job and good night.


     cout<<"New Cetner = "<<new_center<<endl;
      ShowManyImages("Show me",2, frame,fgmask);

	  // NOW CALMAN FILTER
	  if(waitKey(35) == 27) break;


  }

    cout<<"Done"<<endl;
  	cap.release();
  	destroyAllWindows();
  	waitKey(0);


return 0;
}





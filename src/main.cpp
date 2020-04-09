/* Applied Video Analysis of Sequences (AVSA)
 *
 *	LAB3: Object Tracking in Videos
 *
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

//Header Files
#include "headers/SetUp.hpp"
#include "headers/ShowManyImages.hpp"
#include "headers/blobs.hpp"
#include "headers/KalmanFilter.hpp"

//namespaces
using namespace cv;
using namespace std;


//main function
int main(int argc, char ** argv) 
{

  Mat frame,fgmask;
  vector<cvBlob> bloblist; // list for blobs
  vector<cvBlob> filteredBloblist; // list for blobs


  // 1. Read Video
  string path = read_video(argc, argv);
  if(path == "-1") return -1;

  // 2. OPEN VIDEO
  VideoCapture cap(path) ;
  if(!cap.isOpened()) return -1;

  // kalman Tracking
   kalman k(2);



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

      //9. Draw Trajectory
      for(int i = 0 ;i<k.getPredictedPoints().size();i++){

    	  putText(frame,"*",k.getPredictedPoints()[i],FONT_HERSHEY_COMPLEX,0.5,Scalar(0,0,255),1);
    	  putText(frame,"+",k.getCorrectedPoints()[i],FONT_HERSHEY_COMPLEX,0.5,Scalar(0,255,0),1);
      }

      // putText(frame,k.getLabel(),new_center,FONT_HERSHEY_COMPLEX,0.5,Scalar(0,0,255),1);

      ShowManyImages("Frame | Fgmask | ", 4, frame, fgmask,
      					paintBlobImage(frame, filteredBloblist, false),paintBlobImage(frame, filteredBloblist, true));

	  if(waitKey(35) == 27) break;


  }

  	cap.release();
  	destroyAllWindows();
  	waitKey(0);


return 0;
}





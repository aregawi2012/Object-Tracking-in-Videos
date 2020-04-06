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
  while(true){

	  // current frame
      cap.read(frame);
	  if(!frame.data) return -1;

       // 3. BACKGROUND SUBTRACTION
	  background_subtraction(frame,fgmask,LEARNING_RATE,HISTORY,varTHERSHOLD);

  }

//    if(argc<1){
//    	cout<<"NO input Video Provided";
//    	return -1;
//    }
//
//    VideoCapture cap ;
//
//    cap.open(argv[1]);
//
//    if(!cap.isOpened()){
//       cout<<"Unable to open Video";
//       return -1;
//    }
//
//    //CONTINUE WITH Background subtraction
//	Ptr<BackgroundSubtractor> pMOG2 = cv::createBackgroundSubtractorMOG2();
//	double learningrate=-1;
//	Mat frame , fgmask;
//
//   while(true){
//
//	    cap.read(frame);
//	    if (!frame.data)break;
//
//		pMOG2->apply(frame, fgmask, learningrate);
//
//
//
//		// apply  morphological opeations
//		int morph_size = 1;
//		Mat element = getStructuringElement( MORPH_RECT, Size( 2*morph_size + 1, 2*morph_size+1 ), Point( morph_size, morph_size ) );
//		morphologyEx(fgmask, fgmask, MORPH_OPEN, element );
//		morphologyEx(fgmask, fgmask, MORPH_CLOSE, element );
//
//        threshold(fgmask,fgmask,127,255,THRESH_BINARY);
//		ShowManyImages("Faame | fmask", 2, frame, fgmask);
//
//
//
//	   if(waitKey(35) == 27) break;
//   }
//
//	cap.release();
//	destroyAllWindows();
//	waitKey(0); // (should stop till any key is pressed .. doesn't!!!!!)
return 0;
}





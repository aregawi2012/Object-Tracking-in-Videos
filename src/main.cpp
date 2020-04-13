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


  // 1. Read Video
  string path = read_video(argc, argv);
  if(path == "-1") return -1;

  // 2. OPEN VIDEO
  VideoCapture cap(path) ;
  if(!cap.isOpened()) return -1;

  // kalman Tracking
  // 1- constant velocity
  // 2- constant Acceleration

   kalman k(2);
   Mat frame,fgmask,frame_taj , frame_blobs;      // for frame , fg , trajectories
   vector<cvBlob> bloblist;         // list for blobs
   vector<cvBlob> filteredBloblist; // list for large blobs only

//  cout<<"A = "<< k.getA()<<endl<<endl;
//  cout<<"P = "<< k.getP()<<endl<<endl;
//  cout<<"Q = "<< k.getQ()<<endl<<endl;
//  cout<<"H = "<< k.getH()<<endl<<endl;
//  cout<<"R = "<< k.getR()<<endl<<endl;

   int i = 0 ;
  while(true){

	  // current frame
      cap.read(frame);
	  if(!frame.data) break;
      frame.copyTo(frame_taj);
      frame.copyTo(frame_blobs);

	  /*
	   * ===============================
	   *    START  MEASURMENT EXTRACTION
	   * ===============================
	   *
	   */

       // 3. BACKGROUND SUBTRACTION
	  fgmask =  background_subtraction(frame,LEARNING_RATE,HISTORY,varTHERSHOLD);

	  // 4. MORPHOLOGCAL OPENING ..
      fgmask = morphological_operation(fgmask ,MOR_SIZE, MOR_TYPE);

      // 5. BLOB EXTRACTION
      extractBlobs(fgmask, bloblist, CONNECTIVITY);

      // 6. FILTER SMALL BLOBS

      // if(bloblist.size())
	  removeSmallBlobs(bloblist, filteredBloblist, MIN_WIDTH, MIN_HEIGHT);

	  // 7. only Max blob
      //  if(filteredBloblist.size())
	  Point center = getCentereOfMaxBlob(filteredBloblist);


	      /*
	 	   * ===============================
	 	   *    END OF MEASURMENT EXTRACTION
	 	   * ===============================
	 	   *
	 	   */

	  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

		  /*
		   * ===============================
		   *     START TRACKING
		   * ===============================
		   *
		   */

	  //8. Kalman Tracking

      k.make_prediction(center);

      //9. Draw Trajectory
      k.draw_trajectory(frame_taj);

      // putText(frame,k.getLabel(),new_center,FONT_HERSHEY_COMPLEX,0.5,Scalar(0,0,255),1);


      ShowManyImages("Frame | Fgmask | Blobs || Measurment", 5,
    		                                                frame,
															fgmask,
      					                                    paintBlobImage(frame, filteredBloblist, false),
															frame_taj,
															paintBlobImage(frame, filteredBloblist, true));

      // store results

      // create directory to store results
		string project_root_path = "/home/aron/AVSA2020results/LAB3/"; //SET THIS DIRECTORY according to your project
		string project_name = "Lab3.0AVSA2020"; //SET THIS DIRECTORY according to your project
		string results_path = project_root_path+"/"+project_name+"/results2";

		// create directory to store results
		string makedir_cmd = "mkdir -p "+project_root_path+"/"+project_name;
		system(makedir_cmd.c_str());
		makedir_cmd = "mkdir -p "+results_path;
		system(makedir_cmd.c_str());


		string outFile=results_path + "/" +"out"+ std::to_string(i) +".png";

		// if (_CONSOLE_DEBUG){cout << outFile << endl;}
		bool write_result=false;

				write_result=imwrite(outFile, frame_taj);
		if (!write_result) printf("ERROR: Can't save fg mask.\n");


		i++;
	  if(waitKey(30) == 27) break;


  }

  	cap.release();
  	destroyAllWindows();
  	waitKey(0);


return 0;
}





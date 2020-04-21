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

// include header files
#include "Utility.hpp"
#include "ForegroundSeg.hpp"
#include "ShowManyImages.hpp"
#include "KalmanFilter.hpp"



//namespaces
using namespace cv;
using namespace std;


//main function
int main(int argc, char ** argv) 
{





	    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	    // DECLARE VARIABLES
	    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	    Mat frame,fgmask,frame_traj,frame_blobs,frame_measurment,frame_all; // frame , foreground , trajectory , blobs , measurent and tracking
	    int it = 0 ; // iterator;
	    string results_folder ; // to save results
        int model_type  = 2; // 1 - VELOCITY , 2 - ACCELERATION.

	    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	    // CREATE OBJECTS FOR HELPER CLASSES
	    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	     ForegroundSeg fgseg; // -> Measurement extraction routines decleared here
	     Kalman kalman(model_type);    // -> kalma filter .. << Areguements Values> 1 - velocity , 2- acceleration
	     VideoCapture cap;    // ->  VideoCapture object


	     //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	     // HELPER METHODS FOR DIFFERENT OPERATION
	     // ARE CALLED SEQUENTIALLY IN STEPS.
	     //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



	    //1. CHECK ARUGMENTS AND READ VIDEOS
	    if(argc <=1){cout<<"Unable to open video , Video not found"; return -1;}
	    std::vector<std::string> all_videos;
	    all_videos.assign(argv+1 , argv+argc);


	     //2. ITERATE THROUGH ALL VIDEOS
	    for(int i = 0; i< all_videos.size(); i++){


	    	if(i!=0)break;
	    	  // go into signle videos
	    	 cap.open(all_videos[i]);
             if(!cap.isOpened()) { cout<<"Unable to open video"; return -1;}

             while(true){

				  cap.read(frame);
				  if(!frame.data){cout<<"End of video";break;};

				  // copy exact image for
				  frame.copyTo(frame_traj);
				  frame.copyTo(frame_blobs);
				  frame.copyTo(frame_measurment);
				  frame.copyTo(frame_all);




				 //3 . foreground segmentation
				  fgmask     =     fgseg.background_subtraction(frame,LEARNING_RATE,HISTORY,varTHERSHOLD);

                 //4. morphological operation
				  fgmask     =     fgseg.morphological_operation(fgmask,MOR_SIZE,MOR_TYPE);

				 //5. extract blob changes in the fgseg object
				  fgseg.max_blob_extraction(fgmask);

				 //6. kalman tracking
				  kalman.make_prediction(fgseg.getBlobCenters(),fgseg.isBlobFound());


				 // 7 . ANNOTATE TRACKING STATISTICS TO FRAMES
              	  kalman.draw_trajectory(frame_all,frame_blobs,frame_measurment,frame_traj,std::to_string((int)cap.get(CV_CAP_PROP_POS_FRAMES)),fgseg.getBlobCenters());


              	 //SHOW VIDOES IN 3x3 window
              	 ShowManyImages("Frame | Fgmask | Blobs || Measurment | ALL | TRAJECTORY", 6,
           						                            frame ,
															fgmask ,
															fgseg.paintBlobImage(frame_blobs, fgseg.getBloblist()),
															fgseg.paintBlobImage(frame_measurment, fgseg.getBloblist()),
															fgseg.paintBlobImage(frame_all, fgseg.getBloblist()),
															fgseg.paintBlobImage(frame_traj, fgseg.getBloblist())


              	 );

              	 // 9. SAVE RESULTS
                  results_folder = "Video_"+std::to_string(i+1);
				  fgseg.save_results(fgseg.paintBlobImage(frame_all, fgseg.getBloblist()),results_folder,it ,model_type);

				  //10 . END FRAME
				  if(waitKey(30) == 27) break;
				  it++;
             }

	        // release resources
	        cap.release();
	     	destroyAllWindows();
	     	waitKey(0);

	    }


 return 0 ;
}





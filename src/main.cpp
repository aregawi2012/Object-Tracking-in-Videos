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
#include "ForegroundSeg.hpp"
#include "ShowManyImages.hpp"
#include "KalmanFilter.hpp"


//namespaces
using namespace cv;
using namespace std;


//main function
int main(int argc, char ** argv) 
{


	    // 1. CHECK ARUGMENTS AND READ VIDEOS
	    if(argc <=1){cout<<"Unable to open video , Video not found"; return -1;}

	    std::vector<std::string> all_videos;
	    all_videos.assign(argv+1 , argv+argc);



	    // craete foreground segmentation object
	     ForegroundSeg fgseg;
	     Mat frame,fgmask,frame_taj,frame_blobs,mes,mes_track;

	     int it = 0 ;

         //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
         // Kalman Filter
	     // argument :  1 - for constant velocity
	     //             2 - for constant acceleration
         //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
         Kalman kalman(1);
	     //2. ITERATE THROUGH ALL VIDEOS
	     VideoCapture cap;

	    for(int i = 0; i< all_videos.size(); i++){

                    if(i!=0)continue;

	    	  // go into signle videos
	    	 cap.open(all_videos[i]);
             if(!cap.isOpened()) { cout<<"Unable to open video"; return -1;}

             while(true){

				  cap.read(frame);
				  if(!frame.data){cout<<"End of video";break;};

				  // copy exact image for trajectory and blobs
				  frame.copyTo(frame_taj);
				  frame.copyTo(frame_blobs);
				  frame.copyTo(mes_track);
				  frame.copyTo(mes);


				 //3 . foreground segmentation
				  fgmask     =     fgseg.background_subtraction(frame,LEARNING_RATE,HISTORY,varTHERSHOLD);

                 //4. morphological operation
				  fgmask     =     fgseg.morphological_operation(fgmask,MOR_SIZE,MOR_TYPE);

				 //5. extract blob changes in the fgseg object
				  fgseg.max_blob_extraction(fgmask);

				 // cout<<"it = "<<it<<" value = "<<fgseg.getBlobCenters()<<endl;


				 //6. kalman filter
				  kalman.make_prediction(fgseg.getBlobCenters(),fgseg.isBlobFound());


				 // 7 . PLOT statistics
              	 kalman.draw_trajectory(frame_taj,mes,mes_track,std::to_string((int)cap.get(CV_CAP_PROP_POS_FRAMES)),fgseg.getBlobCenters());

              	 // 8. Visualize Results
				  ShowManyImages("Frame | Fgmask | Blobs || Measurment | Tracking", 6,
						                            frame,
													fgmask,
													fgseg.paintBlobImage(frame_blobs, fgseg.getBloblist()),
													mes,
													frame_taj,
													mes_track);

				  //9/ Save results if interested
				  fgseg.save_results(fgseg.paintBlobImage(frame_taj, fgseg.getBloblist()),it);


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





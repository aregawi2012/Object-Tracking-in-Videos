/*
 * AllMethods.cpp
 *
 *  Created on: Apr 6, 2020
 *      Author: root
 */

#include "../headers/SetUp.hpp"

 /* Name : read_video
 * @input :
 * @output :
 */
Ptr<BackgroundSubtractor> pMOG2 = cv::createBackgroundSubtractorMOG2(HISTORY , varTHERSHOLD , false);

string read_video(int count, char ** value){

	if(count <1){
		cout<<"NO Video Provided as an input , Provide absolute path of your video in the command line";
		return "-1";
	}
	else{

       return value[1];
	}
}

/*
 * Name : background_subtraction
 * @input : frame , fgmask , learning_rate, history , varthreshold
 * @output : fgmask
 */
Mat background_subtraction(Mat frame,double learning_rate, int his, double VarThresh){

	Mat fgmask =Mat::zeros(frame.size(),frame.type());
	pMOG2->apply(frame , fgmask, LEARNING_RATE);

    // remove shadow
    threshold(fgmask,fgmask,127,255,THRESH_BINARY);

	return fgmask;
}

/*
* Name : morphological_operation
* @input :
* @output :
*/
Mat morphological_operation(Mat fgmask , int mor_sizes , int type){

	Mat element = getStructuringElement( type, Size(mor_sizes, mor_sizes ),Point(1,1));
	morphologyEx(fgmask, fgmask, MORPH_OPEN, element );
	//morphologyEx(fgmask, fgmask, MORPH_CLOSE, element );

	return fgmask;

}







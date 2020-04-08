/*
 * AllMethods.cpp
 *
 *  Created on: Apr 6, 2020
 *      Author: root
 */

#include "../headers/AllMethods.hpp"

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

	return fgmask;
}

/*
* Name : morphological_operation
* @input :
* @output :
*/
Mat morphological_operation(Mat fgmask , int mor_sizes , int type){

	Mat element = getStructuringElement( type, Size(mor_sizes, mor_sizes ));
	morphologyEx(fgmask, fgmask, MORPH_OPEN, element );
	//morphologyEx(fgmask, fgmask, MORPH_CLOSE, element );

	return fgmask;

}


/*
* Name : morphological_operation
* @input :
* @output :
*/
void small_blobs_removal(){

}





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
 * @input :
 * @output :
 */
void background_subtraction(Mat frame,Mat fgmask, double learning_rage, double history, double varThreshold){
 cout<<"Background";
}

/*
* Name : morphological_operation
* @input :
* @output :
*/
void morphological_operation(){

}

/*
* Name : morphological_operation
* @input :
* @output :
*/
void blob_extraction(){

}

/*
* Name : morphological_operation
* @input :
* @output :
*/
void small_blobs_removal(){

}





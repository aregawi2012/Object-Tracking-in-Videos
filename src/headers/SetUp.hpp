/*
 * AllMethods.hpp
 *
 *  Created on: Apr 6, 2020
 *      Author: root
 */
#include "opencv2/opencv.hpp"
#include <cstring>
#include <iostream>

using namespace cv;
using namespace std;


#ifndef SRC_HEADERS_ALLMETHODS_HPP_
#define SRC_HEADERS_ALLMETHODS_HPP_



// Parameters for MOG
#define LEARNING_RATE 0.001
#define HISTORY 50
#define varTHERSHOLD 16

// Params for blob extraction
#define MIN_WIDTH 10
#define MIN_HEIGHT 10

// Morphological opening

#define MOR_SIZE 3
#define MOR_TYPE MORPH_RECT


 /*
 * Name : read_video
 * @input :
 * @output :
 */

std::string read_video(int count, char ** value);

/*
 * Name : background_subtraction
 * @input :
 * @output :
 */
Mat background_subtraction(Mat frame, double learning_rage, int history, double varThreshold);

/*
* Name : morphological_operation
* @input :
* @output :
*/

Mat morphological_operation(Mat fgmask , int mor_size , int type);

/*
* Name : morphological_operation
* @input :
* @output :
*/
//void extractBlobs(cv::Mat fgmask, std::vector<cvBlob> &bloblist, int connectivity);

/*
* Name : morphological_operation
* @input :
* @output :
*/
void small_blobs_removal();


#endif /* SRC_HEADERS_ALLMETHODS_HPP_ */

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
#define MIN_WIDTH 20
#define MIN_HEIGHT 30

// Morphological opening

#define SIZE 3;
#define TYPE =MORPH_RECT;


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
void background_subtraction(Mat frame,Mat fgmask, double learning_rage, double history, double varThreshold);

/*
* Name : morphological_operation
* @input :
* @output :
*/
void morphological_operation();

/*
* Name : morphological_operation
* @input :
* @output :
*/
void blob_extraction();

/*
* Name : morphological_operation
* @input :
* @output :
*/
void small_blobs_removal();


#endif /* SRC_HEADERS_ALLMETHODS_HPP_ */

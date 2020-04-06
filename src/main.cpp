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
#include <algorithm>
//opencv libraries
#include <opencv2/opencv.hpp>
#include <opencv2/video/background_segm.hpp>

//Header ShowManyImages
#include "headers/ShowManyImages.hpp"


//namespaces
using namespace cv; //avoid using 'cv' to declare OpenCV functions and variables (cv::Mat or Mat)
using namespace std;

#define MIN_WIDTH 20
#define MIN_HEIGHT 30

//main function
int main(int argc, char ** argv) 
{


	// READ DOCUMENTS
    if(argc<1){
    	cout<<"NO input Video Provided";
    	return -1;
    }

    VideoCapture cap ;

    cap.open(argv[1]);

    if(!cap.isOpened()){
       cout<<"Unable to open Video";
       return -1;
    }
    cout<<"Arrived Here";


	// DO BACKGROUND SUBTRACTION

	// DO TRACKING NOW


return 0;
}





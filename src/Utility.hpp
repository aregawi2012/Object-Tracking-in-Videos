/*
 * Utility.hpp
 *
 *  Created on: Apr 21, 2020
 *      Author: root
 */

#ifndef SRC_UTILITY_HPP_
#define SRC_UTILITY_HPP_

#include "opencv2/opencv.hpp"
#include <cstring>
#include <iostream>

using namespace cv;
using namespace std;


class Utility{

private:
       std::string path;



public:
       Utility();
       void read_video_input();
       void draw_statistics_and_trajectory();
       void save_results();

};



#endif /* SRC_UTILITY_HPP_ */

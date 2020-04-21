/*
 * ForegroundSeg.hpp
 *
 *  Created on: Apr 14, 2020
 *      Author: root
 */

#ifndef SRC_FOREGROUNDSEG_HPP_
#define SRC_FOREGROUNDSEG_HPP_

#include "opencv2/opencv.hpp"
#include <cstring>
#include <iostream>

using namespace cv;
using namespace std;

// Parameters for MOG
#define LEARNING_RATE -1
#define HISTORY 50
#define varTHERSHOLD 16

// Params for blob extraction
#define MIN_WIDTH 10
#define MIN_HEIGHT 10

// Morphological opening

#define MOR_SIZE 3
#define MOR_TYPE MORPH_RECT

// blob analysis
const int CONNECTIVITY = 8;


typedef enum {
	UNKNOWN=0,
	PERSON=1,
	GROUP=2,
	CAR=3,
	OBJECT=4
} CLASS;

struct cvBlob {
	int     ID;  /* blob ID        */
	int   x, y;  /* blob position  */
	int   w, h;  /* blob sizes     */
	int cx,cy;
	int area ;
	CLASS label; /* type of blob   */
	char format[1024];

};

inline cvBlob initBlob(int id, int x, int y, int w, int h, int cx , int cy,int area)
{
	cvBlob B = { id,x,y,w,h,cx,cy,area,UNKNOWN};
	return B;
}


class ForegroundSeg{

    private:

       std::vector<Point> blob_centers;
       std::vector<Point> blob_dimensions;
       std::vector<cvBlob> bloblist;
       int found_count;
       bool blob_found = false;



	public:
		Mat background_subtraction(Mat frame, double learning_rage, int history, double varThreshold);
		Mat morphological_operation(Mat fgmask , int mor_size , int type);
        void max_blob_extraction(Mat fgmask);
        Mat paintBlobImage(Mat frame, std::vector<cvBlob> bloblist);


	const std::vector<Point>& getBlobCenters() const {
		return blob_centers;
	}
	const std::vector<cvBlob>& getBloblist() const {

			return bloblist;
		}
	bool isBlobFound() const {
		return blob_found;
	}

	void setBlobFound(bool blobFound = false) {
		blob_found = blobFound;
	}

	void save_results(Mat input , int it){

		 // create directory to store results
		string project_root_path = "/home/aron/AVSA2020results/LAB3/"; //SET THIS DIRECTORY according to your project
		string project_name = "Lab3.0AVSA2020"; //SET THIS DIRECTORY according to your project
		string results_path = project_root_path+"/"+project_name+"/results2";

		// create directory to store results
		string makedir_cmd = "mkdir -p "+project_root_path+"/"+project_name;
		system(makedir_cmd.c_str());
		makedir_cmd = "mkdir -p "+results_path;
		system(makedir_cmd.c_str());


		string outFile=results_path + "/" +"out"+ std::to_string(it) +".png";

		// if (_CONSOLE_DEBUG){cout << outFile << endl;}
		bool write_result=false;

				write_result=imwrite(outFile, input);
		if (!write_result) printf("ERROR: Can't save fg mask.\n");

	}

};



#endif /* SRC_FOREGROUNDSEG_HPP_ */

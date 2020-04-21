/*
 * ForegroundSeg.cpp
 *
 *  Created on: Apr 14, 2020
 *      Author: root
 */

#include "ForegroundSeg.hpp"

Ptr<BackgroundSubtractor> pMOG2 = cv::createBackgroundSubtractorMOG2(HISTORY , varTHERSHOLD , false);


// method for background subtraction
Mat ForegroundSeg::background_subtraction(Mat frame,double learning_rate, int his, double VarThresh){

	// apply background subtraction
	Mat fgmask =Mat::zeros(frame.size(),frame.type());
	pMOG2->apply(frame , fgmask, LEARNING_RATE);

    // remove shadow -- 127
    threshold(fgmask,fgmask,128,255,THRESH_BINARY);

	return fgmask;
}


//method for morphological opertion

Mat ForegroundSeg::morphological_operation(Mat fgmask , int mor_sizes , int type){

	Mat element = getStructuringElement( type, Size(mor_sizes, mor_sizes ),Point(1,1));
	morphologyEx(fgmask, fgmask, MORPH_OPEN, element );
	//morphologyEx(fgmask, fgmask, MORPH_CLOSE, element );

	return fgmask;

}


void ForegroundSeg::max_blob_extraction(Mat fgmask){

	 // clear blob list variable
     bool found = false;
     int max_area = 0 ;


 	  Mat aux; // image to be updated each time a blob is detected (blob cleared)
     fgmask.convertTo(aux, CV_32SC1);

      bloblist.clear();

	  Mat box , center , labels ;
	  int x, y, w, h, area, index ;      // x , y ,width , height - detected blobs

	  int max_x = 0 , max_y = 0  , max_w = 0 , max_h = 0 ;    //  for max blob;
	  double cx = 0,cy= 0 , max_cx = 0 , max_cy = 0;

	 // GET CONECTED COMPONET BLOBS
	 int detected_blobs =  connectedComponentsWithStats(fgmask, aux, box, center, CONNECTIVITY, CV_32SC1);

	 for (int i = 0 ; i < detected_blobs ; i++){


		 // get height and width
		  w = box.at<int>(i, cv::CC_STAT_WIDTH);
		  h = box.at<int>(i, cv::CC_STAT_HEIGHT);


		 // only conisder blobs greater than predifined height and width
		 if(h < MIN_HEIGHT || w < MIN_WIDTH){continue;}

		 // sometimes the whole image is detected as blob in the beginning
		 if(h == fgmask.rows && w == fgmask.cols){continue;}


		 // if arrived here consider it for blob.

		 // get the details of the blob
		  x = box.at<int>(i, cv::CC_STAT_LEFT);
		  y = box.at<int>(i, cv::CC_STAT_TOP);
		  area = box.at<int>(i, cv::CC_STAT_AREA);

		 cx = center.at<double>(i,0);
	     cy = center.at<double>(i,1);

		 // only maximum blob
		  if(max_area <= area){

			  max_area = area;
              max_cx = cx;
              max_cy = cy;
              max_h = h;
              max_w = w;
              index = i;
              max_x = x;
              max_y = y;
		  }

         }

	 if(max_area != 0){

	   cvBlob blob = initBlob(index, max_x, max_y, max_w, max_h,max_cx,max_cy,max_area);
	   bloblist.push_back(blob);
       blob_centers.push_back(Point(max_cx,max_cy));
       found = true;
	 }
	 this->blob_found = found;
}


Mat ForegroundSeg::paintBlobImage(cv::Mat frame, std::vector<cvBlob> bloblist)
{
	cv::Mat blobImage;
	//check input conditions and return original if any is not satisfied
	//...
	frame.copyTo(blobImage);
	//required variables to paint
	//...

	if(bloblist.size()){

		cvBlob blob = bloblist[bloblist.size()-1]; //get ith blob

		Point p1 = Point(blob.x, blob.y);
		Point p2 = Point(blob.x+blob.w, blob.y+blob.h);

		rectangle(blobImage, p1, p2, Scalar(51, 255, 204), 1, 1, 0);
	}


	//return the image to show
	return blobImage;
}





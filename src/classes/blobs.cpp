/* Applied Video Analysis of Sequences (AVSA)
 *
 *	LAB2: Blob detection & classification
 *	Lab2.0: Sample Opencv project
 *
 *
 * Authors: José M. Martínez (josem.martinez@uam.es), Paula Moral (paula.moral@uam.es), Juan C. San Miguel (juancarlos.sanmiguel@uam.es)
 */

#include "../headers/blobs.hpp"
using namespace std;


/**
 *	Draws blobs with different rectangles on the image 'frame'. All the input arguments must be
 *  initialized when using this function.
 *
 * \param frame Input image
 * \param pBlobList List to store the blobs found
 * \param labelled - true write label and color bb, false does not wirite label nor color bb
 *
 * \return Image containing the draw blobs. If no blobs have to be painted
 *  or arguments are wrong, the function returns a copy of the original "frame".
 *
 */
 Mat paintBlobImage(cv::Mat frame, std::vector<cvBlob> bloblist, bool labelled)
{
	cv::Mat blobImage;
	//check input conditions and return original if any is not satisfied
	//...
	frame.copyTo(blobImage);

	//required variables to paint
	//...

	//paint each blob of the list
	for(int i = 0; i < bloblist.size(); i++)
	{
		cvBlob blob = bloblist[i]; //get ith blob
		//...
		Scalar color;
		std::string label="";
		switch(blob.label){
		case 1:
			color = Scalar(0,255,0);
			label="PERSON";
			break;
		case 2:
			color = Scalar(30,255,255);
			label="GROUP";
			break;
		case 3:
			color = Scalar(0,255,0);
			label="CAR";
			break;
		case 4:
			color = Scalar(0,0,255);
			label="OBJECT";
			break;

		default:
			color = Scalar(255, 255, 255);
			label="UNKNOWN";
		}

		Point p1 = Point(blob.x, blob.y);
		Point p2 = Point(blob.x+blob.w, blob.y+blob.h);

		rectangle(blobImage, p1, p2, color, 2, 8, 0);
		if (labelled)
			{
			rectangle(blobImage, p1, p2, color, 2, 8, 0);
			putText(blobImage, label, p1, FONT_HERSHEY_DUPLEX, 0.75, color);
			}
			else
				rectangle(blobImage, p1, p2, Scalar(255, 255, 255), 2, 8, 0);
	}

	//destroy all resources (if required)
	//...

	//return the image to show
	return blobImage;
}

/**
 *	Blob extraction from 1-channel image (binary). The extraction is performed based
 *	on the analysis of the connected components. All the input arguments must be 
 *  initialized when using this function.
 *
 * \param fgmask Foreground/Background segmentation mask (1-channel binary image) 
 * \param bloblist List with found blobs
 *
 * \return Operation code (negative if not succesfull operation) 
 */
int extractBlobs(cv::Mat fgmask, std::vector<cvBlob> &bloblist, int connectivity)
{

    if(fgmask.empty()){
    	return -1;
    }
    if(connectivity != 4 && connectivity !=8) connectivity = 8 ;

     // CONNECTED COMPONENT ANALYSIS STARTS HERE
	 Mat aux; // image to be updated each time a blob is detected (blob cleared)
	 fgmask.convertTo(aux,fgmask.type());

	//clear blob list (to fill with this function)
	 bloblist.clear();

	 // DONE : OPTION1 .. connectedComponentsWithStats for blob extration.
     Mat box , center ;
     int x , y , w , h ,cx,cy ,area ; // x , y ,width , height
     // GET CONECTED COMPONET BLOBS
     connectedComponentsWithStats(fgmask,aux,box,center,connectivity,CV_32SC1);

     // ADD BLOBS TO BLOB LIST , start from one to skip the first :( freaking blob.
     for (int i = 1 ; i < box.rows ; i++){


    	 // get the details of the blob
         x = box.at<int>(Point(0,i));
		 y = box.at<int>(Point(1,i));
		 w = box.at<int>(Point(2,i));
		 h = box.at<int>(Point(3,i));
         cx = center.at<double>(i,0);
         cy = center.at<double>(i,1);
         area = box.at<int>(i, cv::CC_STAT_AREA);


         // sometimes the whole image is detected as blob in the beginning
                if(h == fgmask.rows && w == fgmask.cols){
                 cout<<"arrived here";
                 continue;

                }

         // create blob and add it to blob list
         cvBlob blob = initBlob(i, x, y, w, h,cx,cy,area);
         bloblist.push_back(blob);

     }

 //    DONE : OPTION 2 FOR floodFill ... // UNCOMENT THE FOLLOWING BLOCK FOR USING floodFill.
//     Rect rect ;
//     Scalar newVal = Scalar(0);
//     int blob_id = 0;
//     for (int i = 0; i < fgmask.cols; i++) {
//     		for (int j = 0; j < fgmask.rows; j++) {
//     			if (aux.at<uchar>(Point(i, j)) == 255) {
//
//     				floodFill(aux, cv::Point(i,j), newVal, &rect,Scalar(0),Scalar(0),connectivity);
//                    cvBlob blob = initBlob(blob_id, rect.x, rect.y, rect.width,rect.height);
//
//     				bloblist.push_back(blob);
//     				blob_id += 1;
//
//     			}
//       }
//     }

     // what do you want me to do
	//return OK code
	return 1;
}

/**
 *	Blob extraction from 1-channel image (binary). The extraction is performed based
 *	on the analysis of the connected components. All the input arguments must be
 *  initialized when using this function.
 *
 * \param fgmask Foreground/Background segmentation mask (1-channel binary image)
 * \param bloblist List with found blobs
 *
 * \return Operation code (negative if not succesfull operation)
 */

int removeSmallBlobs(std::vector<cvBlob> bloblist_in, std::vector<cvBlob> &bloblist_out, int min_width, int min_height)
{
    //check input conditions and return -1 if any is not satisfied
    if(bloblist_in.size()<1){
    	return - 1;
    }

	//clear blob list (to fill with this function)
	bloblist_out.clear();

	for(int i = 0; i < bloblist_in.size(); i++)
	{
		//get ith blob
		cvBlob blob_in = bloblist_in[i];

		if(blob_in.h >=min_height && blob_in.w >=min_width){

			// add to filtered blob.
			bloblist_out.push_back(blob_in);
		}
	}

	// Retrun STATUS = OK.
	return 1;
}


/** ========================================================================
 * Fuction name :  getCentereOfMaxBlob
 * Gets the maximum blob with larger area.
 *
 * @param:  bloblist.
 * @return:  cvBlob
 * ========================================================================
 */
Point getCentereOfMaxBlob(std::vector<cvBlob> blobs){

	   int x =0, y=0 ; // x and y parts of the point
	   int area = 0;

	   for(int i = 0 ; i< blobs.size();i++){

		   if(area <=blobs[i].area){
			   x = blobs[i].cx;
			   y=  blobs[i].cy;
			   area = blobs[i].area;
		   }

	   }
	//cout<<"Points ("<<x<<","<<y<<")"<<endl;
    return Point(x,y);
}





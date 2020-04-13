/* Applied Video Analysis of Sequences (AVSA)
 *
 *	LAB2: Blob detection & classification
 *	Lab2.0: Sample Opencv project
 *
 *
 * Authors: José M. Martínez (josem.martinez@uam.es), Paula Moral (paula.moral@uam.es), Juan C. San Miguel (juancarlos.sanmiguel@uam.es)
 */

 //class description
/**
 * \class BasicBlob
 * \brief Class to describe a basic blob and associated functions
 *
 * 
 */

#ifndef BLOBS_H_INCLUDE
#define BLOBS_H_INCLUDE

#include "opencv2/opencv.hpp"
using namespace cv; //avoid using 'cv' to declare OpenCV functions and variables (cv::Mat or Mat)


// Maximun number of char in the blob's format
const int MAX_FORMAT = 1024;
const int CONNECTIVITY = 8;


/// Type of labels for blobs
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
	char format[MAX_FORMAT];

};


/** ===================================================================
 * fuction name :  initBlob
 * Creates blobs
 *
 * @param id ,x , y , w,h .
 * @return cvBlob.
 * ====================================================================
 */

inline cvBlob initBlob(int id, int x, int y, int w, int h, int cx , int cy,int area)
{
	cvBlob B = { id,x,y,w,h,cx,cy,area,UNKNOWN};
	return B;
}


/** ===================================================================
 * fuction name :  paintBlobImage
 * Paints Bounding Box  a round blobs
 *
 * @param: bloblist , labelled .
 * @return: frame , if label == true , labeled box around the blobs ,
 * 					  false , box only.
 * ====================================================================
 */

Mat paintBlobImage(Mat frame, std::vector<cvBlob> bloblist, bool labelled);


/** ========================================================================
 * Fuction name :  extractBlobs
 * Finds connected blobs using opencv connectedComponentsWithStatus method
 *
 * @param: fgmask , bloblist(empty) , connectivity .
 * @return: bloblist ( with blobs)
 * ========================================================================
 */
int extractBlobs(Mat fgmask, std::vector<cvBlob> &bloblist, int connectivity);



/** ========================================================================
 * Fuction name :  extractBlobs
 * Finds connected blobs using opencv connectedComponentsWithStatus method
 *
 * @param: fgmask , bloblist(empty) , connectivity .
 * @return: bloblist ( with blobs)
 * ========================================================================
 */
int removeSmallBlobs(std::vector<cvBlob> bloblist_in, std::vector<cvBlob> &bloblist_out, int min_width, int min_height);


/** ========================================================================
 * Fuction name :  getCentereOfMaxBlob
 * Gets the maximum blob with larger area.
 *
 * @param:  bloblist.
 * @return:  cvBlob
 * ========================================================================
 */
Point getCentereOfMaxBlob(std::vector<cvBlob> blobs);




#endif


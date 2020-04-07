/*
 * KalmanFilter.hpp
 *
 *  Created on: Apr 7, 2020
 *      Author: root
 */

#ifndef SRC_HEADERS_KALMANFILTER_HPP_
#define SRC_HEADERS_KALMANFILTER_HPP_

#include "opencv2/opencv.hpp"
using namespace cv;
using namespace std;


namespace kalman{

  class kalman{

	     /**
	      * ===============================================
	      *    DEFINITION OF KALIMAN FILTER PARAMETERS
	      * ===============================================
          *
          * A = state transition matix
          * H = observation matrix
          * P = state covariance error
          * Q = process covariance noise

          *  X       -> state
          *  X_size  -> dimension of head state [ EX. X = [ x x' y y'] - dimension -- 4 .
          *  Z       -> measurement variable
          *  Z_size  ->  measurement dimension
          *
          * ================================================
	    */
	    Mat A, H , P, Q, R;
	    Mat X , Z;
	    int X_size , Z_size;
	    int Kalman_type = 0 ; // This 1. for constant velocity , 2.. for constant acceleration . 0 default

	    // public setter methods
  public :

	    // CONSTRUCTOR .. INTIALIZES EVERYTHING..
	    // type = 1 will set up constatn velocity.
	    // type = 2 will set up constant acceleration.
	    kalman(int type);

	    /**
		  * ===============================================
		  *   PUBLIC SETTER METHODS
		  * ===============================================
		  *
		  */

	    // set state transition matrix
	    void set_transition_matrix_A();

	    //set measerarment matrix
	    void set_observation_matrix_H();

	    // set Process noise covariance
	    void set_process_noise_cov_Q();

	    // set prediction error covariance
	    void set_prediction_error_cov_P();

	    // set measurment error covariance
	    void set_measuement_error_cov_R();

	    /**
		  * ===============================================
		  *   PUBLIC GETTER METHODS
		  * ===============================================
		  *
		  */

		const Mat& getA() const {
			return A;
		}

		const Mat& getH() const {
			return H;
		}

		const Mat& getP() const {
			return P;
		}

		const Mat& getQ() const {
			return Q;
		}

		const Mat& getR() const {
			return R;
		}

		const Mat& getX() const {
			return X;
		}

		int getSize() const {
			return X_size;
		}

		const Mat& getZ() const {
			return Z;
		}

		int getSizeZ() const {
				return Z_size;
			}


  };

}



#endif /* SRC_HEADERS_KALMANFILTER_HPP_ */

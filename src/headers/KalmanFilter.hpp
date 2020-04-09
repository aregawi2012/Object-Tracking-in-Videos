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

          *  X       ->  state
          *  X_size  ->  dimension of head state [ EX. X = [ x x' y y'] - dimension -- 4 .
          *  Z       ->  measurement variable
          *  Z_size  ->  measurement dimension
          *
          *
          * ================================================
	    */
	    Mat A, H , P, Q, R;
	    Mat X , Z;
	    KalmanFilter KF;
	    int X_size , Z_size;
	    int Kalman_type = 0 ;   // This 1. for constant velocity , 2.. for constant acceleration . 0 default
	    bool started = false ;    // to start Kalman
        Point new_center;

        std::vector<cv::Point> predicted_points;
        std::vector<cv::Point> corrected_points;
        string Label;

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

	    // set all kalman filter paramenters in one
	    void set_all_kalman_parms();

	    // make prediction
	    Point make_prediction(Point center);

	    // For constatn velocity
	    Point constantVelocity(Point center);

	    // For constant Acceleration
	    Point constantAcceleration(Point center);

	    // history for the trajoctory
	    void predicted_trajectory(Point p);
	    void corrected_trajectory(Point p);

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

		std::vector<cv::Point> getCorrectedPoints() const {
			return corrected_points;
		}

		std::vector<cv::Point> getPredictedPoints() const {
			return predicted_points;
		}

		string getLabel() const {
			return Label;
		}

		void setLabel(string label) {
			Label = label;
		}

};

#endif /* SRC_HEADERS_KALMANFILTER_HPP_ */

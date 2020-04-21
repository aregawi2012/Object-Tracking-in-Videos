/*
 * KalmanFilter.hpp
 *
 *  Created on: Apr 14, 2020
 *      Author: root
 */

#ifndef SRC_KALMANFILTER_HPP_
#define SRC_KALMANFILTER_HPP_

#include "opencv2/opencv.hpp"
using namespace cv;
using namespace std;

class Kalman{


	 /**
	  * ===============================================
	  *    DEFINITION OF KALIMAN FILTER PARAMETERS
	  * ===============================================
	  *
	  * A = state transition matix
	  * H = observation matrix
	  * P = state covariance error
	  * Q = process covariance noise
      *
	  *  X       ->  state
	  *  X_size  ->  dimension of head state [ EX. X = [ x x' y y'] - dimension -- 4 .
	  *
	  *  Z       ->  measurement variable
	  *  Z_size  ->  measurement dimension
	  *
	  *
	  * ================================================
	 */


	 Mat A, H , P, Q, R;
	 Mat X , Z;
	 int X_size , Z_size;

	 KalmanFilter KF;
	 int Kalman_type = 0 ;   // This 1. for constant velocity , 2.. for constant acceleration . 0 default

 	bool started = false ;    // to start Kalman
 	Point new_center;
 	int found_count = 0 ;

    // to store predicted points for trajectory
 	// to store label .. like intialized , corrected and predicted.

	std::vector<cv::Point> predicted_points;
	std::vector<std::string> label;

 public :


    // CONSTRUCTOR .. INTIALIZES EVERYTHING..
	// type = 1 will set up constatn velocity.
	// type = 2 will set up constant acceleration.
	Kalman(int type);

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
	void make_prediction(std::vector<Point> centers , bool blob_found);

	// For constatn velocity
	void constantVelocity(std::vector<Point> centers , bool blob_found);

	// For constant Acceleration
	void constantAcceleration(std::vector<Point> centers , bool blob_found);

	// history for the trajoctory
	void predicted_trajectory(Point p);

	//  draw trajectory
	void draw_trajectory(Mat &frame_all , Mat &frame_blobs , Mat &frame_measurment, Mat &frame_traj, string num,std::vector<Point> blob_centers);

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
		return KF.errorCovPre;
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

	const Point getNewCenter() const {
		return new_center;
	}


	void setLabel(string input) {

		label.push_back(input);
	}

	const std::vector<std::string> getLabel() const {
			return label;
		}

	const std::vector<cv::Point> getPredictedPoints() const {
		return predicted_points;
	}

	int getFoundCount() const {
		return found_count;
	}
};


#endif /* SRC_KALMANFILTER_HPP_ */

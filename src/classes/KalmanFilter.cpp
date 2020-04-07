/*
 * KalmanFilter.cpp
 *
 *  Created on: Apr 7, 2020
 *      Author: root
 */

#include "../headers/KalmanFilter.hpp"


using namespace kalman;

kalman::kalman(int type){

	Z_size = 2;
	X_size = 4;
	if(type == 1)
		X_size = 6;

}

// set state matrix
void kalman::set_transition_matrix_A(){

	// check if TYPE SPECIFIED
	if(Kalman_type){
       Kalman_type = 1;
	}

	A = Mat::zeros(Size(X_size, X_size), CV_32F);

	switch(Kalman_type){

	case 1:

        setIdentity(A);
        A.at<float>(0,1) = A.at<float>(2,3) = 1;
		break;

	case 2:
		 setIdentity(A);
		 A.at<float>(0,2) = A.at<float>(3,5) = 0.5;
	     A.at<float>(0,1) = A.at<float>(1,2)=A.at<float>(3,4) =A.at<float>(4,5)= 1;
	  break;

	default :

	}

}

// observation Matrix
void kalman::set_observation_matrix_H(){

	// check if TYPE SPECIFIED
			if(Kalman_type){

		       Kalman_type = 1; // velocity or acceleration..
			}

			H = Mat::zeros(Size(Z_size, X_size), CV_32F);

			switch(Kalman_type){

			case 1:
		        H.at<float>(0,0) = H.at<float>(1,2) = 1;
				break;

			case 2:
				 H.at<float>(0,0) = H.at<float>(1,3) = 1;
			  break;

			default :

	        }

}

//set processos covariance
void kalman::set_process_noise_cov_Q(){

	// check if TYPE SPECIFIED
		if(Kalman_type){
	       Kalman_type = 1;
		}

		Q = Mat::eye(Size(X_size, X_size), CV_32F);

		switch(Kalman_type){

		case 1:
	        Q.at<float>(0,0) = Q.at<float>(2,2) = 25;
	        Q.at<float>(1,1) = Q.at<float>(3,3) = 10;
			break;

		case 2:
			 Q.at<float>(0,0) = Q.at<float>(3,3) = 25;
			 Q.at<float>(1,1) = Q.at<float>(4,4) = 10;
			 Q.at<float>(2,2) = Q.at<float>(5,5) = 1;
		  break;
		default :
  }

}

// prediction error covariance
void kalman::set_prediction_error_cov_P(){

		P = Mat::eye(Size(X_size, X_size), CV_32F);
        setIdentity(P,Scalar(10e5));

}

// prediction error covariance
void kalman::set_measuement_error_cov_R(){

		R = Mat::eye(Size(2, 2), CV_32F);
        setIdentity(R,Scalar(25));

}




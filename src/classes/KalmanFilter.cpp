/*
 * KalmanFilter.cpp
 *
 *  Created on: Apr 7, 2020
 *      Author: root
 */

#include "../headers/KalmanFilter.hpp"


kalman::kalman(int type){

	Z_size = 2;
	X_size = 4;

	if(type == 2)X_size = 6;

	Kalman_type = type;
	new_center = Point(0 ,0);
	started =  false;

	// Intialize Parameters
	set_transition_matrix_A();
	set_prediction_error_cov_P();
	set_process_noise_cov_Q();
	set_observation_matrix_H();
	set_measuement_error_cov_R();

	// intialize kalman
	 KF.init(X_size,Z_size,0,CV_32F);
	 X = Mat::zeros(X_size,1,CV_32F);
	 Z = Mat::zeros(Z_size,1,CV_32F);

}

// set state matrix
void kalman::set_transition_matrix_A(){

	// check if TYPE SPECIFIED
	if(!Kalman_type){

       Kalman_type = 1; // 1 - velocity , 2 - acceleration
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
		 break;

	}

}

// observation Matrix
void kalman::set_observation_matrix_H(){

	// check if TYPE SPECIFIED
			if(!Kalman_type){

		       Kalman_type = 1; // velocity or acceleration..
			}

			H = Mat::zeros(Size(X_size, Z_size), CV_32F);

			switch(Kalman_type){

			case 1:
		        H.at<float>(0,0) = H.at<float>(1,2) = 1;
				break;

			case 2:
				 H.at<float>(0,0) = H.at<float>(1,3) = 1;
			  break;

			default :
				break;

	        }

}

//set processos covariance
void kalman::set_process_noise_cov_Q(){

	// check if TYPE SPECIFIED
		if(!Kalman_type){
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
			break;
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

// Make prediction
Point kalman::make_prediction(Point center){

  if(Kalman_type ==1)
	  kalman::constantVelocity(center);
  else
	  kalman::constantAcceleration(center);

 return new_center;

}

void kalman::constantVelocity(Point center){


	   // First time
	   if(!started){

		   //
		   if(center.x && center.y){

			     // Initialize Measurment
			     Z.at<float>(0) = center.x;
				 Z.at<float>(1) = center.y;

				 // Initialize State
				 X.at<float>(0) = Z.at<float>(0);
				 X.at<float>(1) = X.at<float>(3) = 0;
	             X.at<float>(2) =  Z.at<float>(1);

	             // set state.
	             KF.statePost = X;

	            // Initicalize
				new_center.x = X.at<float>(0);
				new_center.y = X.at<float>(2);

		   }
	   }
	   else{

		  // so get the state
		  // 1. step 1 ( Prediction)
		  X = KF.predict();

		  new_center.x = X.at<float>(0); // x part
		  new_center.y = X.at<float>(0); // y part

		 // If Measurement ... if there is blob
		  if(center.x && center.y){

			 Z.at<float>(0) = center.x;
			 Z.at<float>(1) = center.y;

		   // 2. correction step
			 KF.correct(Z);
		  }

	   }


}

void kalman::constantAcceleration(Point center){

}


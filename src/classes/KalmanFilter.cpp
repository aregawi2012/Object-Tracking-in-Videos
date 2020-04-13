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


	// initialize state and measusurment
	 X = Mat::zeros(X_size,1,CV_32F);
     Z = Mat::zeros(Z_size,1,CV_32F);


	// intialize kalman
	 KF.init(X_size,Z_size,0,CV_32F);
	 set_all_kalman_parms();

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
        A.at<float>(0,1) = A.at<float>(2,3) = 1.0f;
		break;

	case 2:
		 setIdentity(A);
		 A.at<float>(0,2) = A.at<float>(3,5) = 0.5f;
	     A.at<float>(0,1) = A.at<float>(1,2)=A.at<float>(3,4) =A.at<float>(4,5)= 1.0f;
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
		        H.at<float>(0,0) = H.at<float>(1,2) = 1.0f;
				break;

			case 2:
				 H.at<float>(0,0) = H.at<float>(1,3) = 1.0f;
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
	        Q.at<float>(0,0) = Q.at<float>(2,2) = 25.0f;
	        Q.at<float>(1,1) = Q.at<float>(3,3) = 10.0f;
			break;

		case 2:
			 Q.at<float>(0,0) = Q.at<float>(3,3) = 25.0f;
			 Q.at<float>(1,1) = Q.at<float>(4,4) = 10.0f;
			 Q.at<float>(2,2) = Q.at<float>(5,5) = 1.0f;
		  break;
		default :
			break;
  }

}

// prediction error covariance
void kalman::set_prediction_error_cov_P(){

		P = Mat::eye(Size(X_size, X_size), CV_32F);
        setIdentity(P,Scalar(10e5f));

}

// prediction error covariance
void kalman::set_measuement_error_cov_R(){

		R = Mat::eye(Size(2, 2), CV_32F);
        setIdentity(R,Scalar(25.0f));

}

void kalman::set_all_kalman_parms(){

	   //State transition matrix
		set_transition_matrix_A();
		A.copyTo(KF.transitionMatrix);

		//Process noise covariance
		set_process_noise_cov_Q();
		Q.copyTo(KF.processNoiseCov);

		//Prediction error covariance
	    set_prediction_error_cov_P();
		P.copyTo(KF.errorCovPre);

		//Measuremt matrix
		set_observation_matrix_H();
		H.copyTo(KF.measurementMatrix);

		//Measurement Noise covariance
		set_measuement_error_cov_R();
		R.copyTo(KF.measurementNoiseCov);
}

// Make prediction
Point kalman::make_prediction(Point center){

  if(Kalman_type ==1)
	  return kalman::constantVelocity(center);

 return kalman::constantAcceleration(center);

}

// constant Velocity
Point kalman::constantVelocity(Point center){


	// START PROCESS .. BY INTIALIZING
	if (!started) {


		// if there is measurement.
		if (center.x || center.y) {

			// Intitalize Measurement
			Z.at<float>(0) = center.x;
			Z.at<float>(1) = center.y;

			//Initialize state to Measument.
			X.at<float>(0) = Z.at<float>(0);
			X.at<float>(2) = Z.at<float>(1);
			X.at<float>(1) =X.at<float>(3) =  0;

			// UPLDATE KALMAN STATE
			KF.statePost = X;

			new_center.x = X.at<float>(0);
			new_center.y = X.at<float>(2);

            started = true;
            kalman::setLabel("Initlalized");
        	kalman::predicted_trajectory(new_center);


		}


	}
	else {


		//STEP 1. PREDCTION
		X = KF.predict();


		// Corrected State value for next
		new_center.x = X.at<float>(0);
		new_center.y = X.at<float>(2);

		// MEAUSERMENT OBSERVED.
		if (center.x || center.y) {

			Z.at<float>(0) = center.x;
			Z.at<float>(1) = center.y;
			kalman::setLabel("Corrected");

		    //STEP 2 . CORRECTION
			KF.correct(Z);

		}
		// IF MEASURMENT IS NOT OBSERVED .. ONLY PREDICTION
		else{

			 kalman::setLabel("Predicted");
			 Z.at<float>(0) = X.at<float>(0);
			 Z.at<float>(1) = X.at<float>(2);
		}

		kalman::predicted_trajectory(new_center);


	}



	// Return new value
	return new_center;

}

//  constant Acceleration
Point kalman::constantAcceleration(Point center){


	// START PROCESS .. BY INTIALIZING
	if (!started) {

		// Measurment observed .. else wait
		if (center.x != 0 || center.y != 0) {

			// Intitalize Measurement
			Z.at<float>(0) = center.x;
			Z.at<float>(1) = center.y;

			//Initialize state to Measument.
			X.at<float>(0) = Z.at<float>(0);
			X.at<float>(3) = Z.at<float>(1);
			X.at<float>(1) =X.at<float>(2) = X.at<float>(4) = X.at<float>(5)=0;

			// UPLDATE KALMAN STATE
			KF.statePost = X;


			new_center.x = X.at<float>(0);
			new_center.y = X.at<float>(3);

            started = true;
            kalman::setLabel("Initlalized");
            // add points to the trajectory
            kalman::predicted_trajectory(new_center);


		}

	}
	else {

		// 1. STEP 1. PREDCTION
		X = KF.predict();


		// Corrected State value for next
		new_center.x = X.at<float>(0);
		new_center.y = X.at<float>(3);

		// MEAUSERMENT OBSERVED.
		if (center.x || center.y) {

			Z.at<float>(0) = center.x;
			Z.at<float>(1) = center.y;
			kalman::setLabel("Corrected");


		// 2. STEP 2 . CORRECTION
			KF.correct(Z);
		}

		else{

			 kalman::setLabel("Predicted");
			 Z.at<float>(0) = X.at<float>(0);
			 Z.at<float>(1) = X.at<float>(3);

		   }

		// add points to the trajectory
	     kalman::predicted_trajectory(new_center);


	}


	// Return new value
	return new_center;

}

// Predicted Trajecktory
void kalman::predicted_trajectory(Point p){
	predicted_points.push_back(p);
}


// draw trajectory
void kalman::draw_trajectory(Mat &input){

	// Color Selection

	 Scalar color;

	 for(int i = 0 ;i<getPredictedPoints().size();i++){

		 cout<<getPredictedPoints().size()<<endl;

		   if(kalman::getLabel()[i] == "Corrected")
		 		color = Scalar(0,0 ,255 );
		 	else if(kalman::getLabel()[i] == "Predicted")
		 		color = Scalar(0,255 , 0 );

		 	else
		 		color = Scalar(0,255 , 255 );

		      cv::circle(input, getPredictedPoints()[i], 6, color, -1);

	    	  if(i == getPredictedPoints().size()-1){

	        	  putText(input,getLabel()[i],getPredictedPoints()[i],FONT_HERSHEY_COMPLEX,0.5,color,1);

	    	  }
	      }

}

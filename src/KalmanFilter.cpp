/*
 * KalmanFilter.cpp
 *
 *  Created on: Apr 14, 2020
 *      Author: root
 */

#include "KalmanFilter.hpp"

Kalman::Kalman(int type){

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
void Kalman::set_transition_matrix_A(){

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
void Kalman::set_observation_matrix_H(){

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
void Kalman::set_process_noise_cov_Q(){

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
void Kalman::set_prediction_error_cov_P(){

		P = Mat::eye(Size(X_size, X_size), CV_32F);
        setIdentity(P,Scalar(10e4f));

}

// prediction error covariance
void Kalman::set_measuement_error_cov_R(){

		R = Mat::eye(Size(2, 2), CV_32F);
        setIdentity(R,Scalar(25.0f));

}

void Kalman::set_all_kalman_parms(){

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
void Kalman::make_prediction(std::vector<Point> mes , bool blob_found){

  if(Kalman::Kalman_type ==1)
	   Kalman::constantVelocity(mes,blob_found);

  else
	  Kalman::constantAcceleration(mes,blob_found);

}

void Kalman::constantVelocity(std::vector<Point> mes , bool blob_found){

    if(blob_found){

    	 // First time .. Intialize Tracking
    	 if(!found_count){

    		 // Intitalize Measurement
			 Z.at<float>(0) = mes[mes.size()-1].x;
			 Z.at<float>(1) = mes[mes.size()-1].y;

		    //Initialize state to Measument.
			X.at<float>(0) = Z.at<float>(0);
			X.at<float>(2) = Z.at<float>(1);
			X.at<float>(1) =X.at<float>(3)=0;

           // UPDATE KALMAN to current state
			KF.statePost = X;

		   // intialize and set variables
    	   Kalman::setLabel("Intialized");

    	 }
    	 // not first time
    	 else
    	 {

    		 //STEP 1. PREDCTION
			X = KF.predict();

			// observation arrived
			Z.at<float>(0) = mes[mes.size()-1].x;
			Z.at<float>(1) = mes[mes.size()-1].y;
			KF.correct(Z);
			Kalman::setLabel("Corrected");
    	 }

    	 found_count++;

    }
    else{

    	 // if measurement lost in the middle .. predict only
    	 if(found_count > 0){

          X = KF.predict();

		  Z.at<float>(0) = X.at<float>(0);
		  Z.at<float>(1) = X.at<float>(2);
		  Kalman::setLabel("Predicted");

    	 }

    }

    // save data
    // If started .. there will be always prediction
    if(found_count>0){
    	 //Predicted State
		new_center.x = X.at<float>(0);
		new_center.y = X.at<float>(2);
		predicted_points.push_back(new_center);
    }

    // except the first round .. correct this
    if(found_count>1){
    	KF.correct(Z);
    }

}

void Kalman::constantAcceleration(std::vector<Point> mes , bool blob_found){


    if(blob_found){

    	 // First time .. Intialize Tracking
    	 if(!found_count){

    		 // Intitalize Measurement
			 Z.at<float>(0) = mes[mes.size()-1].x;
			 Z.at<float>(1) = mes[mes.size()-1].y;

			 cout<<"Observation :"<<Z<<endl;

		    //Initialize state to Measument.
			X.at<float>(0) = Z.at<float>(0);
			X.at<float>(3) = Z.at<float>(1);
			X.at<float>(1)=X.at<float>(2)=X.at<float>(4)=X.at<float>(5)=0;

           // UPDATE KALMAN to current state
			KF.statePost = X;


			cout<<"State = "<<X<<endl<<endl;
            cout<<"Kalman Gain"<<KF.gain<<endl;
            cout<<"Error covariance"<<KF.errorCovPre<<endl;


		   // intialize and set variables
    	   Kalman::setLabel("Intialized");

    	 }
    	 // not first time
    	 else
    	 {

    		 //STEP 1. PREDCTION
			X = KF.predict();

			// observation arrived
			Z.at<float>(0) = mes[mes.size()-1].x;
			Z.at<float>(1) = mes[mes.size()-1].y;
			KF.correct(Z);
			Kalman::setLabel("Corrected");
    	 }

    	 found_count++;

    }
    else{

    	 // if measurement lost in the middle .. predict only
    	 if(found_count > 0){

          X = KF.predict();

		  Z.at<float>(0) = X.at<float>(0);
		  Z.at<float>(1) = X.at<float>(3);
		  Kalman::setLabel("Predicted");

    	 }

    }

    // save data
    // If started .. there will be always prediction
    if(found_count>0){
    	 //Predicted State
		new_center.x = X.at<float>(0);
		new_center.y = X.at<float>(3);
		predicted_points.push_back(new_center);
    }

    // except the first round .. correct this
    if(found_count>1){
    	KF.correct(Z);
    }
}


// draw trajectory
void Kalman::draw_trajectory(Mat &frame_all , Mat &frame_blobs , Mat &frame_measurment, Mat &frame_traj, string num, std::vector<Point> blob_centers){

	// Color Selection
	 Scalar color_measurment = Scalar(255,0,0);
	 Scalar color_estimated = Scalar (0,0,255);
	 Scalar color_predicted = Scalar (0,255,0);
     Scalar color_label;


     // Put Text to the Measurement , Predicted/corrected imagte
	 putText(frame_all,"Measurment_Z",Point(20,20),FONT_HERSHEY_COMPLEX,0.5,color_measurment,1);
     putText(frame_all,"Estimate_Xk"  ,Point(20,40),FONT_HERSHEY_COMPLEX,0.5,color_estimated,1);
	 putText(frame_all,"Predicted_Xk" ,Point(20,60),FONT_HERSHEY_COMPLEX,0.5,color_predicted,1);


//	 putText(mes,"Measurment",Point(20,25),FONT_HERSHEY_COMPLEX,0.5,color_measurment,1);
//
//
//	 // add frame number to all the images
	 putText(frame_all,"Frame = "+ num,Point(frame_all.cols -120,20),FONT_HERSHEY_COMPLEX,0.4,Scalar(255,255,255),1);
	 putText(frame_blobs,"Frame = "+ num,Point(frame_blobs.cols - 120,20),FONT_HERSHEY_COMPLEX,0.4,Scalar(255,255,255),1);
	 putText(frame_measurment,"Frame = "+ num,Point(frame_measurment.cols - 120,20),FONT_HERSHEY_COMPLEX,0.4,Scalar(255,255,255),1);
	// putText(frame_measurment,"Measurment Z",Point(20,60),FONT_HERSHEY_COMPLEX,0.4,color_measurment,1);



	 // Put the measurement in to the measurement image

	 for(int i = 0 ;i<blob_centers.size();i++){

	      circle(frame_measurment,blob_centers[i], 4, color_measurment, -1);
	      circle(frame_all,blob_centers[i], 4, color_measurment, -1);

	 	 }



     // PLOT ESTIAMATES
	 for(int i = 0 ;i<getPredictedPoints().size();i++){

		 if(Kalman::getLabel()[i] == "Corrected")
					 color_label = color_estimated;
				else if(Kalman::getLabel()[i] == "Predicted")
					color_label = color_predicted;
				else
					color_label = color_measurment;
		 circle(frame_all,getPredictedPoints()[i], 2, color_label, -1);


 	   if(i!=0)
   	     line(frame_traj ,getPredictedPoints()[i-1] , getPredictedPoints()[i],color_estimated,1);

	 }





//
//
//
////		if(i == Kalman::getPredictedPoints().size()-1){
////
////	      putText(mes_track,Kalman::getLabel()[i],getPredictedPoints()[i],FONT_HERSHEY_COMPLEX,0.5,color_label,1);
////	    }
////
////	   if(i!=0)
////	   line(frame_traj ,getPredictedPoints()[i-1] , getPredictedPoints()[i],color_estimated,1);
//
//    }




}




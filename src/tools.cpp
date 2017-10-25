#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /*
	Calculation of RMSE
  */
	//Initialization
	VectorXd rmse(4);
	rmse << 0, 0, 0, 0;
	
	//Check inputs
	if ((estimations.size() != 0) && (estimations.size() == ground_truth.size())) {
		//Get sums of squared error
		for (int i=0; i< estimations.size(); ++i) {
			//Get Error
			VectorXd err = estimations[i] - ground_truth[i];
			//Square
			err = err.array()*err.array();
			//Sum
			rmse += err;
		}
		//Mean
		rmse = rmse / estimations.size();
		
		//Root
		rmse = rmse.array().sqrt();
	} else {
		//Bogus data, future TODO - throw invalid data exception
	}
	
	//Return data
	return rmse;
}
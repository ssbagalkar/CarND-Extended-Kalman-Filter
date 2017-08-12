#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
	VectorXd temp(4);
	temp << 0,0,0,0;

	if (estimations.size() != ground_truth.size()
		|| estimations.size() == 0)
	{
		cout << "invalid estimation or ground truth" << endl;
	}

	//accumulate squared residuals
	for (unsigned int i = 0; i < estimations.size(); ++i)
	{
		VectorXd residual = estimations[i] - ground_truth[i];

		//coefficient-wise multipilcation
		residual = residual.array() * residual.array();
		temp += residual;
	}

	//calculate mean
	temp = temp / estimations.size();

	//calculate squared root
	temp = temp.array().sqrt();
	return temp;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */

	MatrixXd temp(3, 4);
	//recover state parameters
	double px = x_state(0);
	double py = x_state(1);
	double vx = x_state(2);
	double vy = x_state(3);

	// pre-compute set of terms to avoid repeated calculation
	double c1 = (px * px) + (py * py);
	double c2 = sqrt(c1);
	double c3 = (c1*c2);

	// check division by zero
	if (fabs(c1) < 0.0001)
	{
		cout << "CalculateJacobian () - error - Division by Zero" << endl;
		return temp;
	}

	temp << (px / c2), (py / c2), 0, 0,
		-(py / c1), (px / c1), 0, 0,
		py*(vx*py - vy*px) / c3, px*(px*vy - py*vx) / c3, px / c2, py / c2;
	return temp;

}

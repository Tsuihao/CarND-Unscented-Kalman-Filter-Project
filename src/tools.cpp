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

  VectorXd rmse(4);
  double inf = std::numeric_limits<double>::infinity();
  rmse<< inf, inf, inf, inf;
  if( estimations.size() == 0 || ground_truth.size() == 0)
  {
    std::cout<<"[tools]: invlaid input"<<std::endl;
    return rmse;
  }
  if( estimations.size() != ground_truth.size())
  {
    std::cout<<"[tools]: estimation and ground truth has different size"<<std::endl;
    return rmse;
  }

  rmse << 0, 0, 0, 0; // modify rmse from the 0;
  for(int i = 0; i < estimations.size(); ++i)
  {
    VectorXd temp = estimations[i] - ground_truth[i];
    VectorXd square = temp.array() * temp.array();
    rmse += square;
  }

  rmse /= estimations.size();
  rmse = rmse.array().sqrt();

  return rmse;
}


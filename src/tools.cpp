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
    
    // check that the estimation vector size equal to ground_truth vector size
    assert(estimations.size() == ground_truth.size());
    // check that the estimation vector size is not zero
    assert(estimations.size() > 0);
    auto n_vectors = estimations.size();
    
    assert(estimations[0].size() == ground_truth[0].size());
    auto dim_vector = estimations[0].size();
    
    //accumulate squared residuals
    VectorXd rmse(dim_vector); rmse.fill(0);
    for (auto i=0; i<n_vectors; ++i){
        //calculate the squared difference of the residuals
        VectorXd sqr_residual = (estimations[i]-ground_truth[i]).array().square();
        //Aggregate the residuals
        rmse += sqr_residual;
    }
    
    //Apply sqrt and average
    rmse = (rmse/n_vectors).array().sqrt();
    
    //Return the result
    return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
}

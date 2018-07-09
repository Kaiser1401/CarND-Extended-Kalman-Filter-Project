#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth)
{
  /**
  TODO:
    * Calculate the RMSE here.
  */
  
  VectorXd rmse(4);
  rmse << 0,0,0,0;
  
  if (estimations.size() == 0 || estimations.size() != ground_truth.size())
  {
    cout << "Estimation or ground_truth invalid" << endl;
    return rmse; 
  }

  
  for(unsigned int i=0; i < estimations.size(); ++i)
  {
    VectorXd residual = estimations[i] - ground_truth[i];
    
    residual = residual.array()*residual.array();
    rmse += residual;    
  }
  
  rmse = rmse / estimations.size();
  
  rmse = rmse.array().sqrt();
  
  return rmse;
  
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state)
{
  /**
  TODO:
    * Calculate a Jacobian here.
  */  
   MatrixXd J(3,4);
   
   float x = x_state(0);
   float y = x_state(1);
   float u = x_state(2);
   float v = x_state(3);
 
   
   float c1 = x*x + y*y;
   float c2 = sqrt(c1);
   float c3 = (c1*c2);    

   if(fabs(c1) < 0.000001)
   {
     cout << "Div0" << endl;     
     return J;     
   }
   
   J << (x/c2), (y/c2), 0, 0,
	-(y/c1), (x/c1), 0, 0,
	y*(u*y - v*x)/c3, x*(x*v - y*u)/c3, x/c2, y/c2;
	
  return J;  
}

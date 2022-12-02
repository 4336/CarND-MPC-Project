#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include <ros/ros.h>

const size_t N = 12;
const double dt = 0.05;
const int latency_ind = 2; //latency  in units of dt (100ms)

struct Solution {

  std::vector<double> X;
  std::vector<double> Y;
  std::vector<double> Psi;
  std::vector<double> V;
  std::vector<double> Cte;
  std::vector<double> EPsi;
  std::vector<double> Delta;
  std::vector<double> A;
};

class MPC {
  public:
    MPC();

    virtual ~MPC();

    // Solve the model given an initial state and polynomial coefficients.
    // Return the first actuations.
    Solution Solve(Eigen::VectorXd x0, Eigen::VectorXd coeffs);

    double delta_prev {0};
    double a_prev {0.1};

};

#endif  // MPC_H

#ifndef MATHLIB_EIGEN_CONVERSION_H
#define MATHLIB_EIGEN_CONVERSION_H


#include "eigen3/Eigen/Dense"

#include "MathLib.h"
using namespace Eigen;


typedef Eigen::VectorXd Vec;
typedef Eigen::MatrixXd Mat;


Mat              M2E_m(MathLib::Matrix mm);
//Mat              M2E_v(MathLib::Vector mm);
Vec              M2E_v(MathLib::Vector mm);
Mat              M2E_v(MathLib::Vector3 mm);


MathLib::Vector  E2M_v(Eigen::VectorXd ev);
MathLib::Vector3 E2M_v3(Eigen::VectorXd ev);
MathLib::Matrix  E2M_m(Eigen::MatrixXd em);
MathLib::Matrix3 E2M_m3(Eigen::MatrixXd em);


#endif // MATHLIB_EIGEN_CONVERSION_H

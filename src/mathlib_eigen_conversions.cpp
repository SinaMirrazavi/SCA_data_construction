
/*
 * Copyright (C) 2016 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
 * Author: Sina Mirrazavi
 * email:   sina.mirrazavi@epfl.ch
 * website: lasa.epfl.ch
 *
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#include "mathlib_eigen_conversions.h"


Mat M2E_m(MathLib::Matrix mm) {
  Eigen::Map<Mat> ei( mm.Array(), mm.ColumnSize(), mm.RowSize());

  return ei;
}

//Mat M2E_v(MathLib::Vector mm) {
Vec M2E_v(MathLib::Vector mm) {
  Eigen::Map<Vec> ei( mm.Array(), mm.Size() );

  return ei;
}

Mat M2E_v(MathLib::Vector3 mm) {
  Eigen::Map<Vec> ei(mm.Array(), 3);

  return ei;
}

MathLib::Vector E2M_v(Eigen::VectorXd ev) {
  MathLib::Vector mv( ev.rows() );

  mv.Set( ev.data(), ev.rows() );
  return mv;
}

MathLib::Vector3 E2M_v3(Eigen::VectorXd ev) {
  //  ROS_ASSERT_MSG(ev.rows() == 3 , "Trying to convert MathLib::Vector3 to Eigen but wrong size !");
  MathLib::Vector3 mv;

  mv.Set( ev(0), ev(1), ev(2) );
  return mv;
}

MathLib::Matrix E2M_m(Eigen::MatrixXd em) {
  MathLib::Matrix mm( em.rows(), em.cols() );

  mm.Set( em.data(), em.rows(), em.cols() );
  return mm;
}

MathLib::Matrix3 E2M_m3(Eigen::MatrixXd em) {
  //  ROS_ASSERT_MSG((em.cols() == 3) && (em.rows() == 3) , "Trying to convert MathLib::Matrix3 to Eigen but wrong size !");
  MathLib::Matrix3 mm;

  mm.Set( em.data() );
  return mm;
}

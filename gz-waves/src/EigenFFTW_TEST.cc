// Copyright (C) 2022  Rhys Mainwaring
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#include <gtest/gtest.h>

#include <Eigen/Dense>

#include <iostream>
#include <memory>
#include <string>

using Eigen::MatrixXd;

//////////////////////////////////////////////////
TEST(EigenFFWT, EigenFFT1D)
{

  // Eigen::MatrixXd dsxdx = Eigen::MatrixXd::Zero(n2, 1);
  // Eigen::MatrixXd dsydy = Eigen::MatrixXd::Zero(n2, 1);
  // Eigen::MatrixXd dsxdy = Eigen::MatrixXd::Zero(n2, 1);

  // EXPECT_EQ(dsxdy.size(), n2);
  // EXPECT_DOUBLE_EQ(dsxdx(i, 0), ref_dsxdx(i, 0));
}

//////////////////////////////////////////////////
// Run tests

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}


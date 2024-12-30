// Copyright (C) 2019  Rhys Mainwaring
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

#include <array>
#include <iostream>
#include <string>
#include <vector>

#include "gz/waves/Algorithm.hh"
#include "gz/waves/Types.hh"

using gz::waves::Index;
using gz::waves::algorithm::sort_indexes;

//////////////////////////////////////////////////
TEST(Algorithm, SortIndexArray2)
{
  { // Case 1
    std::array<double, 2> v {0, 1};
    std::array<Index, 2> idx = sort_indexes(v);
    EXPECT_EQ(v[idx[0]], 1.0);
    EXPECT_EQ(v[idx[1]], 0.0);
  }

  { // Case 2
    std::array<double, 2> v {1, 0};
    std::array<Index, 2> idx = sort_indexes(v);
    EXPECT_EQ(v[idx[0]], 1.0);
    EXPECT_EQ(v[idx[1]], 0.0);
  }
}

//////////////////////////////////////////////////
TEST(Algorithm, SortIndexVector2)
{
  { // Case 1
    std::vector<double> v {0, 1};
    std::vector<Index> idx = sort_indexes(v);
    EXPECT_EQ(v[idx[0]], 1.0);
    EXPECT_EQ(v[idx[1]], 0.0);
  }

  { // Case 2
    std::vector<double> v {1, 0};
    std::vector<Index> idx = sort_indexes(v);
    EXPECT_EQ(v[idx[0]], 1.0);
    EXPECT_EQ(v[idx[1]], 0.0);
  }
}

//////////////////////////////////////////////////
TEST(Algorithm, SortIndex3)
{
  { // Case 1
    std::vector<double> v {0, 1, 2};
    std::vector<Index> idx = sort_indexes(v);
    EXPECT_EQ(v[idx[0]], 2.0);
    EXPECT_EQ(v[idx[1]], 1.0);
    EXPECT_EQ(v[idx[2]], 0.0);
  }

  { // Case 2
    std::vector<double> v {0, 2, 1};
    std::vector<Index> idx = sort_indexes(v);
    EXPECT_EQ(v[idx[0]], 2.0);
    EXPECT_EQ(v[idx[1]], 1.0);
    EXPECT_EQ(v[idx[2]], 0.0);
  }

  { // Case 3
    std::vector<double> v {1, 0, 2};
    std::vector<Index> idx = sort_indexes(v);
    EXPECT_EQ(v[idx[0]], 2.0);
    EXPECT_EQ(v[idx[1]], 1.0);
    EXPECT_EQ(v[idx[2]], 0.0);
  }

  { // Case 4
    std::vector<double> v {1, 2, 0};
    std::vector<Index> idx = sort_indexes(v);
    EXPECT_EQ(v[idx[0]], 2.0);
    EXPECT_EQ(v[idx[1]], 1.0);
    EXPECT_EQ(v[idx[2]], 0.0);
  }

  { // Case 5
    std::vector<double> v {2, 0, 1};
    std::vector<Index> idx = sort_indexes(v);
    EXPECT_EQ(v[idx[0]], 2.0);
    EXPECT_EQ(v[idx[1]], 1.0);
    EXPECT_EQ(v[idx[2]], 0.0);
  }

  { // Case 6
    std::vector<double> v {2, 1, 0};
    std::vector<Index> idx = sort_indexes(v);
    EXPECT_EQ(v[idx[0]], 2.0);
    EXPECT_EQ(v[idx[1]], 1.0);
    EXPECT_EQ(v[idx[2]], 0.0);
  }
}

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}


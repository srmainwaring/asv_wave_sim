// Copyright (C) 2020  Rhys Mainwaring
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

#include "wave_sim_config/server.h"
#include <gtest/gtest.h>

// Test fixture
class WaveSimConfigServerTestSuite : public ::testing::Test
{
protected:
  ~WaveSimConfigServerTestSuite() override
  {
  }

  WaveSimConfigServerTestSuite()
  {
  }

  void SetUp() override
  {        
  }

  void TearDown() override
  {
  }
};

// Define tests
TEST_F(WaveSimConfigServerTestSuite, test1)
{
  EXPECT_EQ(1, 1) << "1 != 1";
}

// Run tests
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

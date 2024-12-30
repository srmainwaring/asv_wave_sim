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

#include <gnuplot-iostream.h>

#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

int main(int /*argc*/, const char **/*argv*/)
{
  try
  {
    std::cout << "PLOT_GnuPlotExample\n";

    {
      std::string s = "pkill gnuplot_qt";
      std::system(s.c_str());
    }

    {
      double A = 1;
      double T = 12;
      double phase = 0 * M_PI / 180;
      double beta = 180 * M_PI / 180;
      double k = pow(2 * M_PI / T, 2) / 9.81;

      std::vector<double> pts_t;
      std::vector<double> pts_eta;
      double x = 0.0;
      double y = 0.0;
      double xx = x * cos(beta) + y * sin(beta);
      for (double t = 0.0; t < 4.0 * T; t += 0.1)
      {
        pts_t.push_back(t);
        pts_eta.push_back(A * cos(k * xx - 2 * M_PI * t / T + phase));
      }
      Gnuplot gp;
      gp << "set term qt title 'Wave Elevation at Origin'\n";
      gp << "set grid\n";
      gp << "set xlabel 'time (s)'\n";
      gp << "set ylabel '(m)'\n";
      gp << "plot '-' w l title 'eta'\n";
      gp.send1d(std::make_tuple(pts_t, pts_eta));
    }
  }
  catch(...)
  {
    std::cerr << "Unknown exception\n";
    return -1;
  }
  return 0;
}


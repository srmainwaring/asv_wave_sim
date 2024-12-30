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

#include <Eigen/Dense>

#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <gz/waves/WaveSimulation.hh>
#include <gz/waves/LinearRandomWaveSimulation.hh>

using gz::waves::Index;
using gz::waves::LinearRandomWaveSimulation;

int main(int /*argc*/, const char **/*argv*/)
{
  try
  {
    std::cout << "PLOT_LinearRandomFFTWaves\n";

    {
      std::string s = "pkill gnuplot_qt";
      std::system(s.c_str());
    }

    double lx = 256.0;
    double ly = 256.0;
    Index nx = 128;
    Index ny = 128;

    { // wave elevation vs time
      std::unique_ptr<LinearRandomWaveSimulation> wave_sim(
          new LinearRandomWaveSimulation(lx, ly, nx, ny));
      wave_sim->SetNumWaves(300);
      wave_sim->SetMaxOmega(6.0);
      wave_sim->SetWindVelocity(10.0, 0.0);
      wave_sim->SetTime(5.0);

      double sample_time = 60.0;
      std::vector<double> pts_t;
      std::vector<double> pts_eta;
      for (double t = 0.0; t < sample_time; t += 0.1)
      {
        double eta;
        wave_sim->SetTime(t);
        wave_sim->ElevationAt(0, 0, eta);

        pts_t.push_back(t);
        pts_eta.push_back(eta);
      }
      Gnuplot gp;
      gp << "set term qt title 'Linear Random Wave Elevation at Origin'\n";
      gp << "set grid\n";
      gp << "set xlabel 'time (s)'\n";
      gp << "set ylabel '(m)'\n";
      gp << "plot '-' w l title 'eta'"
         << "\n";
      gp.send1d(std::make_tuple(pts_t, pts_eta));
    }

    { // wave elevation vs position
      std::unique_ptr<LinearRandomWaveSimulation> wave_sim(
          new LinearRandomWaveSimulation(lx, ly, nx, ny));
      wave_sim->SetNumWaves(300);
      wave_sim->SetMaxOmega(6.0);
      wave_sim->SetWindVelocity(10.0, 0.0);
      wave_sim->SetTime(5.0);

      Eigen::ArrayXXd h(nx * ny, 1);
      wave_sim->ElevationAt(h);

      std::vector<double> pts_x;
      std::vector<double> pts_eta1;
      std::vector<double> pts_eta2;
      for (Index ix = 0; ix < nx; ++ix)
      {
        Index iy = ny / 2;
        Index idx = iy * nx + ix;
        double eta1 = 0.0;
        double eta2 = h(idx, 0);
        wave_sim->ElevationAt(ix, iy, eta1);

        double x = -0.5 * lx + ix * lx / nx;
        pts_x.push_back(x);
        pts_eta1.push_back(eta1);
        pts_eta2.push_back(eta2);
      }
      Gnuplot gp;
      gp << "set term qt title 'Linear Random Wave Elevation'\n";
      gp << "set grid\n";
      gp << "set xlabel 'x (m)'\n";
      gp << "set ylabel '(m)'\n";
      gp << "plot '-' w l title 'eta1'"
         << ",'-' w l title 'eta2'"
         << "\n";
      gp.send1d(std::make_tuple(pts_x, pts_eta1));
      gp.send1d(std::make_tuple(pts_x, pts_eta2));
    }

    { // wave pressure vs position
      Index nz = 10;
      double lz = 50.0;

      std::unique_ptr<LinearRandomWaveSimulation> wave_sim(
          new LinearRandomWaveSimulation(lx, ly, lz, nx, ny, nz));
      wave_sim->SetNumWaves(300);
      wave_sim->SetMaxOmega(6.0);
      wave_sim->SetWindVelocity(10.0, 0.0);
      wave_sim->SetTime(5.0);

      std::vector<double> pts_x;
      std::vector<std::vector<double>> pts_p(nz);
      for (Index ix = 0; ix < nx; ++ix)
      {
        Index iy = ny / 2;
        for (Index iz = 0; iz < nz; ++iz)
        {
          double p;
          wave_sim->PressureAt(ix, iy, iz, p);
          pts_p[iz].push_back(p);
        }
        double x = -0.5 * lx + ix * lx / nx;
        pts_x.push_back(x);
      }

      // assume we always have at least one plot
      std::string plot_str("plot '-' w l title 'p0'");
      for (Index iz = 1; iz < nz; ++iz)
      {
        plot_str.append(",'-' w l title 'p")
          .append(std::to_string(iz)).append("'");
      }
      plot_str.append("\n");

      Gnuplot gp;
      gp << "set term qt title 'Linear Random Wave Pressure'\n";
      gp << "set grid\n";
      gp << "set xlabel 'x (m)'\n";
      gp << "set ylabel '(m)'\n";
      gp << plot_str;

      for (Index iz = 0; iz < nz; ++iz)
      {
        gp.send1d(std::make_tuple(pts_x, pts_p[iz]));
      }
    }
  }
  catch(...)
  {
    std::cerr << "Unknown exception\n";
    return -1;
  }
  return 0;
}


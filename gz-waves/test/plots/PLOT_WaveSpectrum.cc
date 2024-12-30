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
#include <gz/waves/WaveSpectrum.hh>

using gz::waves::Index;
using gz::waves::ECKVWaveSpectrum;
using gz::waves::PiersonMoskowitzWaveSpectrum;

// https://stackoverflow.com/questions/16605967/set-precision-of-stdto-string-when-converting-floating-point-values
template <typename T>
std::string to_string_with_precision(const T a_value, const int n = 6)
{
    std::ostringstream out;
    out.precision(n);
    out << std::fixed << a_value;
    return out.str();
}

int main(int /*argc*/, const char **/*argv*/)
{
  try
  {
    std::cout << "PLOT_WaveSpectrum\n";

    {
      std::string s = "pkill gnuplot_qt";
      std::system(s.c_str());
    }

    {
      ECKVWaveSpectrum spectrum;

      Index nk = 200;
      Eigen::ArrayXd k =
          Eigen::pow(10.0, Eigen::ArrayXd::LinSpaced(nk, -3.0, 4.0));

      Index nu = 5;
      Eigen::ArrayXd u10 = Eigen::ArrayXd::LinSpaced(nu, 0.0, 20.0);

      std::vector<double> pts_k;
      std::vector<std::vector<double>> pts_s(u10.size());

      for (Index ik = 0; ik < nk; ++ik)
      {
        pts_k.push_back(k(ik));

        for (Index iu = 0; iu < nu; ++iu)
        {
          spectrum.SetU10(u10(iu));
          double s = spectrum.Evaluate(k(ik));
          pts_s[iu].push_back(s);
        }
      }


      // assume we always have at least one plot
      std::string plot_str("plot '-' w l title 'u10 = ");
      plot_str.append(to_string_with_precision(u10(0), 1)).append("'");
      for (Index iu = 1; iu < nu; ++iu)
      {
        plot_str.append(",'-' w l title 'u10 = ")
          .append(to_string_with_precision(u10(iu), 1)).append("'");
      }
      plot_str.append("\n");

      Gnuplot gp;
      gp << "set term qt title 'ECKV Wave Spectrum'\n";
      gp << "set grid\n";
      gp << "set logscale xy\n";
      gp << "set xrange [1.0E-3:1.0E4]\n";
      gp << "set yrange [1.0E-15:1.0E3]\n";
      gp << "set xlabel 'spatial frequency k (rad/m)'\n";
      gp << "set ylabel 'variance spectrum S(k) (m^2/(rad/m))'\n";
      gp << plot_str;

      for (Index iu = 0; iu < nu; ++iu)
      {
        gp.send1d(std::make_tuple(pts_k, pts_s[iu]));
      }
    }

    {
      PiersonMoskowitzWaveSpectrum spectrum;

      Index nk = 200;
      Eigen::ArrayXd k =
          Eigen::pow(10.0, Eigen::ArrayXd::LinSpaced(nk, -3.0, 4.0));

      Index nu = 5;
      Eigen::ArrayXd u19 = Eigen::ArrayXd::LinSpaced(nu, 0.0, 20.0);

      std::vector<double> pts_k;
      std::vector<std::vector<double>> pts_s(u19.size());

      for (Index ik = 0; ik < nk; ++ik)
      {
        pts_k.push_back(k(ik));

        for (Index iu = 0; iu < nu; ++iu)
        {
          spectrum.SetU19(u19(iu));
          double s = spectrum.Evaluate(k(ik));
          pts_s[iu].push_back(s);
        }
      }

      // assume we always have at least one plot
      std::string plot_str("plot '-' w l title 'u19 = ");
      plot_str.append(to_string_with_precision(u19(0), 1)).append("'");
      for (Index iu = 1; iu < nu; ++iu)
      {
        plot_str.append(",'-' w l title 'u19 = ")
          .append(to_string_with_precision(u19(iu), 1)).append("'");
      }
      plot_str.append("\n");

      Gnuplot gp;
      gp << "set term qt title 'Pierson-Moskowitz Wave Spectrum'\n";
      gp << "set grid\n";
      gp << "set logscale xy\n";
      gp << "set xrange [1.0E-3:1.0E4]\n";
      gp << "set yrange [1.0E-15:1.0E3]\n";
      gp << "set xlabel 'spatial frequency k (rad/m)'\n";
      gp << "set ylabel 'variance spectrum S(k) (m^2/(rad/m))'\n";
      gp << plot_str;

      for (Index iu = 0; iu < nu; ++iu)
      {
        gp.send1d(std::make_tuple(pts_k, pts_s[iu]));
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


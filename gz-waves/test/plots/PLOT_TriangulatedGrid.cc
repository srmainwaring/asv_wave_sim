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

#include <array>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>

#include "gz/waves/TriangulatedGrid.hh"
#include "gz/waves/Types.hh"

namespace cgal
{
using gz::cgal::Point3;
}  // namespace cgal

using gz::waves::Index;
using gz::waves::Point3Range;
using gz::waves::TriangulatedGrid;

void plotTriangulation(const TriangulatedGrid& tri_grid)
{
  auto [lx, ly] = tri_grid.TileSize();

  // https://stackoverflow.com/questions/42784369/drawing-triangular-mesh-using-gnuplot
  Gnuplot gp;

  gp << "#set title 'Triangulation'\n";
  gp << "#set xlabel 'X'\n";
  gp << "#set ylabel 'Y'\n";
  gp << "#set zlabel 'Z'\n";

  // sets background color
  gp << "set object 1 rectangle from screen -0.1,-0.1 to screen 1.1,1.1"
      << " fillcolor rgb '#ffffff' behind\n";

  // allows rendering of polygons with hidden line removal
  gp << "set hidden3d back offset 0 trianglepattern 3"
      << " undefined 1 altdiagonal bentover\n";

  // displays borders 0x7F = 0b1111111
  gp << "set border 0x7F linecolor rgb '#555555'\n";

  // displays the x, y and z axis
  gp << "set xzeroaxis linewidth 0.5 linetype 1\n";
  gp << "set yzeroaxis linewidth 0.5 linetype 2\n";
  gp << "set zzeroaxis linewidth 0.5 linetype 3\n";

  // displays the x, y and z grid
  gp << "set grid xtics linecolor rgb '#888888' linewidth 0.2 linetype 9\n";
  gp << "set grid ytics linecolor rgb '#888888' linewidth 0.2 linetype 9\n";
  gp << "set grid ztics linecolor rgb '#888888' linewidth 0.2 linetype 9\n";

  // # moves the x, y grid to 0
  gp << "set xyplane at 0\n";

  // makes the x, y, and z axis proportional
  // gp << "set view equal xyz\n";

  // view from top down
  gp << "set view projection xy\n";

  // sets the axis range
  gp << "set xrange [" << -lx/2*1.1 << ":" << lx/2*1.1 << "]\n";
  gp << "set yrange [" << -ly/2*1.1 << ":" << ly/2*1.1 << "]\n";
  gp << "set zrange [-10:10]\n";

  // moves the key out of the graph
  gp << "set key outside vertical bottom right\n";

  // hides the key
  gp << "set key off\n";

  // temp file for plot data
  gp << "$DATA << EOD\n";
  int count = 1;
  for (auto& index : tri_grid.Indices())
  {
    auto& p1 = tri_grid.Points()[index[0]];
    auto& p2 = tri_grid.Points()[index[1]];
    auto& p3 = tri_grid.Points()[index[2]];
    gp << p1[0] << " " << p1[1] << " " << p1[2] << "\n";
    gp << p2[0] << " " << p2[1] << " " << p2[2] << "\n";
    gp << "\n";
    gp << p3[0] << " " << p3[1] << " " << p3[2] << "\n";
    gp << p3[0] << " " << p3[1] << " " << p3[2] << "\n";
    gp << "\n";
    gp << "\n";
    count++;
  }
  gp << "EOD\n";

  gp << "splot '$DATA' title 'ico' with lines"
      << " linewidth 2.1 linecolor rgb '#88FF88'\n";
}

int main(int /*argc*/, const char **/*argv*/)
{
  try
  {
    std::cout << "PLOT_TriangulatedGrid\n";

    {
      std::string s = "pkill gnuplot_qt";
      std::system(s.c_str());
    }

    // create triangulation
    double lx = 100.0;
    double ly = 100.0;
    Index nx = 16;
    Index ny = 8;
    Index nx_plus1 = nx + 1;
    Index ny_plus1 = ny + 1;
    auto tri_grid = TriangulatedGrid::Create(nx, ny, lx, ly);

    // for (auto& index : tri_grid->Indices())
    // {
    //   for (auto idx : index)
    //   {
    //     auto& pt = tri_grid->Points()[idx];
    //     std::cout << "idx: " << idx << ", pt: " << pt << "\n";
    //   }
    // }

    plotTriangulation(*tri_grid);

    // modify points
    std::vector<cgal::Point3> points = tri_grid->Points();
    for (auto&& p : points)
    {
      double a = 1.5;
      double d = std::sqrt(p[0]*p[0] + p[1]*p[1]);
      double dx = a * std::sin(0.2 * 2 * M_PI * d);
      double dy = a * std::cos(0.3 * 2 * M_PI * d);
      p = cgal::Point3(p[0] + dx, p[1] + dy, p[2]);
    }

    tri_grid->UpdatePoints(points);
    plotTriangulation(*tri_grid);

  }
  catch(...)
  {
    std::cerr << "Unknown exception\n";
    return -1;
  }
  return 0;
}


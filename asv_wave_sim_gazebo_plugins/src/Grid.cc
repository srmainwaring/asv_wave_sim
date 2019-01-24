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

#include "asv_wave_sim_gazebo_plugins/Grid.hh"
#include "asv_wave_sim_gazebo_plugins/CGALTypes.hh"
#include "asv_wave_sim_gazebo_plugins/Geometry.hh"
#include "asv_wave_sim_gazebo_plugins/MeshTools.hh"

#include <CGAL/number_utils.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>

#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>

#include <array>
#include <iostream>
#include <iterator>
#include <string>
#include <vector>

namespace asv 
{

///////////////////////////////////////////////////////////////////////////////
// GridPrivate

  class GridPrivate
  {
    /// \brief The size of the grid in each direction
    public: std::array<double, 2> size;

    /// \brief The number of cells in each direction
    public: std::array<size_t, 2> cellCount;
    
    /// \brief The position of the grid center
    public: Point3 center;

    /// \brief The grid mesh
    public: std::shared_ptr<Mesh> mesh;

    /// \brief The grid normals (for each face)
    public: std::vector<Vector3> normals;
  };

///////////////////////////////////////////////////////////////////////////////
// Grid

  Grid::Grid(
    const std::array<double, 2>& _size,
    const std::array<size_t, 2>& _cellCount
  ) : data(new GridPrivate()) 
  {
    this->data->size = _size;
    this->data->cellCount = _cellCount;
    this->data->center = CGAL::ORIGIN;
    this->data->mesh = std::make_shared<Mesh>();

    // Grid dimensions
    const size_t nx = this->data->cellCount[0];
    const size_t ny = this->data->cellCount[1];
    const double Lx = this->data->size[0];
    const double Ly = this->data->size[1];
    const double lx = Lx/nx;
    const double ly = Ly/ny;

    auto& mesh = *this->data->mesh;
    auto& normals = this->data->normals;

    // Add vertices
    for (size_t iy=0; iy<=ny; ++iy)
    {
      double py = iy * ly - Ly/2.0;
      for (size_t ix=0; ix<=nx; ++ix)
      {
        double px = ix * lx - Lx/2.0;
        mesh.add_vertex(Point3(px, py, 0));
      }
    }

    // Add faces
    for (size_t iy=0; iy<ny; ++iy)
    {
      for (size_t ix=0; ix<nx; ++ix)
      {
        // Get the vertices in the cell coordinates
        const size_t idx0 = iy * (nx+1) + ix;
        const size_t idx1 = iy * (nx+1) + ix + 1;
        const size_t idx2 = (iy+1) * (nx+1) + ix + 1;
        const size_t idx3 = (iy+1) * (nx+1) + ix;

        // Vertex iterators
        auto v0 = std::begin(mesh.vertices());
        auto v1 = std::begin(mesh.vertices());
        auto v2 = std::begin(mesh.vertices());
        auto v3 = std::begin(mesh.vertices());
        std::advance(v0, idx0);
        std::advance(v1, idx1);
        std::advance(v2, idx2);
        std::advance(v3, idx3);

        // Faces
        mesh.add_face(*v0, *v1, *v2);
        mesh.add_face(*v0, *v2, *v3);

        // Face Normals
        Point3 p0(mesh.point(*v0));
        Point3 p1(mesh.point(*v1));
        Point3 p2(mesh.point(*v2));
        Point3 p3(mesh.point(*v3));
        normals.push_back(Geometry::Normal(p0, p1, p2));
        normals.push_back(Geometry::Normal(p0, p2, p3));
      }
    }
  }

  Grid::Grid(const Grid& _other) :
    data(new GridPrivate()) 
  {
    this->data->size = _other.data->size;
    this->data->cellCount = _other.data->cellCount;
    this->data->mesh.reset(new Mesh(*_other.data->mesh));
  }

  Grid& Grid::operator=(const Grid& _other)
  {
    // Check for self assignment
    if (&_other == this)
      return *this;

    // Copy
    this->data->size = _other.data->size;
    this->data->cellCount = _other.data->cellCount;
    this->data->mesh.reset(new Mesh(*_other.data->mesh));
    return *this;
  }

  std::shared_ptr<const Mesh> Grid::GetMesh() const
  {
    return this->data->mesh;
  }

  std::shared_ptr<Mesh> Grid::GetMesh()
  {
    return this->data->mesh;
  }

  const Mesh& Grid::GetMeshByRef() const
  {
    return *this->data->mesh;
  }

  const std::array<double, 2>& Grid::GetSize() const
  {
    return this->data->size;
  } 

  const std::array<size_t, 2>& Grid::GetCellCount() const
  {
    return this->data->cellCount;
  } 

  size_t Grid::GetVertexCount() const
  {
    return this->data->mesh->number_of_vertices();
  }

  size_t Grid::GetFaceCount() const
  {
    return this->data->mesh->number_of_faces();
  }

  const Point3& Grid::GetPoint(size_t _i) const
  {
    auto vb = std::begin(this->data->mesh->vertices());
    std::advance(vb, _i);
    return this->data->mesh->point(*vb);
  }

  void Grid::SetPoint(size_t _i, const Point3& _v)
  {
    auto vb = std::begin(this->data->mesh->vertices());
    std::advance(vb, _i);
    this->data->mesh->point(*vb) = _v;
  }

  Triangle Grid::GetTriangle(size_t _ix, size_t _iy, size_t _k) const
  {
    // Original lookup using cell indexing - keep for index arithmetic
    // // Grid dimensions
    // const size_t nx = this->data->cellCount[0];
    // const size_t ny = this->data->cellCount[1];

    // // Get the vertices in the cell coordinates
    // const size_t idx0 = _iy * (nx+1) + _ix;
    // const size_t idx1 = _iy * (nx+1) + _ix + 1;
    // const size_t idx2 = (_iy+1) * (nx+1) + _ix + 1;
    // const size_t idx3 = (_iy+1) * (nx+1) + _ix;

    // switch (_k) 
    // {
    // case 0:
    //   return Triangle(
    //     this->GetPoint(idx0),
    //     this->GetPoint(idx1),
    //     this->GetPoint(idx2)
    //   );
    // case 1:
    //   return Triangle(
    //     this->GetPoint(idx0),
    //     this->GetPoint(idx2),
    //     this->GetPoint(idx3)
    //   );      
    // default:
    //   gzthrow("Index too large: " << _k << " > 1");
    // }

    // Optimised lookup using face indexing
    // Face index
    const size_t nx = this->data->cellCount[0];
    const size_t ny = this->data->cellCount[1];
    const size_t idx = 2 * (nx * _iy + _ix) + _k;

    // Make triangle from face descriptor
    auto& mesh = *this->data->mesh;
    auto fb = std::begin(mesh.faces());
    std::advance(fb, idx);
    return Geometry::MakeTriangle(mesh, *fb);
  }

  FaceIndex Grid::GetFace(size_t _ix, size_t _iy, size_t _k) const
  {
    // Face index
    const size_t nx = this->data->cellCount[0];
    const size_t ny = this->data->cellCount[1];
    const size_t idx = 2 * (nx * _iy + _ix) + _k;

    // Make triangle from face descriptor
    auto& mesh = *this->data->mesh;
    auto fb = std::begin(mesh.faces());
    std::advance(fb, idx);
    return *fb;
  }

  const Vector3& Grid::GetNormal(size_t _ix, size_t _iy, size_t _k) const
  {
    // Face index
    const size_t nx = this->data->cellCount[0];
    const size_t ny = this->data->cellCount[1];
    const size_t idx = 2 * (nx * _iy + _ix) + _k;

    return this->data->normals[idx];
  }

  const Vector3& Grid::GetNormal(size_t _idx) const
  {
    return this->data->normals[_idx];
  }

  void Grid::RecalculateNormals()
  {
    auto& mesh = *this->data->mesh;
    unsigned long idx = 0;
    for(auto&& face : mesh.faces())
    {
      Vector3 normal = Geometry::Normal(mesh, face);
      this->data->normals[idx++] = normal;
    }
  }

  const Point3& Grid::GetCenter() const
  {
    return this->data->center;
  }

  void Grid::SetCenter(const Point3& _center)
  {
    this->data->center = _center;
  }

  void Grid::DebugPrint() const
  {
    auto&& mesh = *this->data->mesh;

    gzmsg << "Center " << std::endl;    
    gzmsg << "c0:  " << this->data->center << std::endl;

    gzmsg << "Vertices " << std::endl;    
    for(auto&& vertex : mesh.vertices())
    {
      gzmsg << vertex << ": " << mesh.point(vertex) << std::endl;
    }
    gzmsg << "Faces " << std::endl;
    for(auto&& face : mesh.faces())
    {
      Triangle tri = Geometry::MakeTriangle(mesh, face);
      gzmsg << face << ": " << tri << std::endl;
    }
  }


///////////////////////////////////////////////////////////////////////////////
// GridTools

  bool GridTools::FindIntersectionIndex(
    const Grid& _grid,
    double _x, double _y,
    std::array<size_t, 3>& _index
  )
  {
    const size_t nx = _grid.GetCellCount()[0];
    const size_t ny = _grid.GetCellCount()[1];
    const double Lx = _grid.GetSize()[0];
    const double Ly = _grid.GetSize()[1];
    const double lx = Lx/nx;
    const double ly = Ly/ny;
    const double lowerX = -Lx/2.0 + _grid.GetCenter().x();
    const double upperX =  Lx/2.0 + _grid.GetCenter().x();
    const double lowerY = -Ly/2.0 + _grid.GetCenter().y();
    const double upperY =  Ly/2.0 + _grid.GetCenter().y();

    // Check x bounds
    if (_x < lowerX || _x > upperX)
      return false;

    // Check y bounds
    if (_y < lowerY || _y > upperY)
      return false;

    // Cell
    size_t ix = static_cast<size_t>(std::floor((_x - lowerX)/lx));
    size_t iy = static_cast<size_t>(std::floor((_y - lowerY)/ly));

    // Face / triangle
    double x0 = ix * lx + lowerX;
    double y0 = iy * ly + lowerY;
    double m = ly/lx;
    double c = y0 - m * x0;
    size_t k = _y > (m * _x + c) ? 1 : 0;

    // @DEBUG_INFO
    // gzmsg << "ix: " << ix << std::endl;
    // gzmsg << "iy: " << iy << std::endl;
    // gzmsg << "x0: " << x0 << std::endl;
    // gzmsg << "y0: " << y0 << std::endl;
    // gzmsg << "m:  " << m  << std::endl;
    // gzmsg << "c:  " << c  << std::endl;
    // gzmsg << "k:  " << k  << std::endl;

    _index[0] = ix;
    _index[1] = iy;
    _index[2] = k;

    return true;    
  }

  bool GridTools::FindIntersectionTriangle(
    const Grid& _grid,
    const Point3& _origin,
    const Direction3& _direction,
    const std::array<size_t, 3>& _index,
    Point3& _intersection
  )
  {
    // FaceIndex version: the ByRef vs shared_ptr access makes 
    // a difference (50% of this functions execution time!)
    const auto& mesh = _grid.GetMeshByRef();
    auto face = _grid.GetFace(_index[0], _index[1], _index[2]);
    HalfedgeIndex hf = mesh.halfedge(face);
    const Point3& p0 = mesh.point(mesh.target(hf));
    hf = mesh.next(hf);
    const Point3& p1 = mesh.point(mesh.target(hf));
    hf = mesh.next(hf);
    const Point3& p2 = mesh.point(mesh.target(hf));
    hf = mesh.next(hf);

    return Geometry::LineIntersectsTriangle(
      _origin, _direction, p0, p1, p2, _intersection); 
  }

  bool GridTools::FindIntersectionCell(
    const Grid& _grid,
    const Point3& _origin,
    const Direction3& _direction,
    std::array<size_t, 3>& _index,
    Point3& _intersection
  )
  {
    // Search each of the two triangles comprising each cell 
    bool isFound = FindIntersectionTriangle(
      _grid, _origin, _direction, _index, _intersection); 
    if (isFound)
    {
      return true;  
    }
    _index[2] = (_index[2] == 0) ? 1 : 0;
    return FindIntersectionTriangle(
      _grid, _origin, _direction, _index, _intersection); 
  }

  // NOTE: This routine as written must use 'int' rather than 'unsigned int'
  // otherwise the index arithmetic will be incorrect.
  bool GridTools::FindIntersectionGrid(
    const Grid& _grid,
    const Point3& _origin,
    const Direction3& _direction,
    std::array<size_t, 3>& _index,
    Point3& _point
  )
  {
    // The flag 'isDone' is true if there are no remaining cells to search.
    bool isDone = false;

    // index of the first cell searched.
    isDone = FindIntersectionCell(_grid, _origin, _direction, _index, _point);
    if (isDone)
    {
      return isDone;
    }
    
    // Grid dimensions
    int nx = _grid.GetCellCount()[0];
    int ny = _grid.GetCellCount()[1];

    // indexes for the grid boundaries
    int kxmin = 0;
    int kxmax = nx - 1;
    int kymin = 0;
    int kymax = ny - 1;
            
    // grid boundary indexes for the first cell. 
    int kx   = _index[0];
    int ky   = _index[1];
    int kxm0 = kx;
    int kxp0 = kxm0;
    int kym0 = ky;
    int kyp0 = kym0;

    // loop searches the cells in an expanding shell about the area already searched. 
    while (!isDone)
    {
      isDone = true;

      // boundary of the next shell
      int kxm = std::max(kxm0 - 1, kxmin);
      int kxp = std::min(kxp0 + 1, kxmax);
      int kym = std::max(kym0 - 1, kymin);
      int kyp = std::min(kyp0 + 1, kymax);
      
      // row0 : lower limit offset for the column loop is either 0 or 1 
      int row0 = 0;
      if (kxm != kxm0)
      {
        isDone = false;
        row0 = 1;
        for (int j=kym; j<=kyp; ++j)
        {
          _index[0] = kxm;
          _index[1] = j;
          isDone = FindIntersectionCell(_grid, _origin, _direction, _index, _point);
          if (isDone)
            return true;
        }
      }
      
      // row1 : upper limit offset for the column loop is either 0 or 1.
      int row1 = 0;
      if (kxp != kxp0)
      {
        isDone = false;
        row1 = 1;
        for (int j=kym; j<=kyp; ++j)
        {
          _index[0] = kxp;
          _index[1] = j;
          isDone = FindIntersectionCell(_grid, _origin, _direction, _index, _point);
          if (isDone)
            return true;
        }
      }
      
      // col0
      if (kym != kym0)
      {
        isDone = false;
        for (int i=kxm+row0; i<=kxp-row1; ++i)
        {
          _index[0] = i;
          _index[1] = kym;
          isDone = FindIntersectionCell(_grid, _origin, _direction, _index, _point);
          if (isDone)
            return true;
        }
      }
      
      // col1
      if (kyp != kyp0)
      {
        isDone = false;
        for (int i=kxm+row0; i<=kxp-row1; ++i)
        {
          _index[0] = i;
          _index[1] = kyp;
          isDone = FindIntersectionCell(_grid, _origin, _direction, _index, _point);
          if (isDone)
            return true;
        }
      }
      
      // update
      kxm0 = kxm;
      kxp0 = kxp;
      kym0 = kym;
      kyp0 = kyp;
    }

    // Fail to find an intersection after searching all cells 
    return false;
  }

///////////////////////////////////////////////////////////////////////////////

} // namespace asv

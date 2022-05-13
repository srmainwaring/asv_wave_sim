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

#include "gz/marine/MeshTools.hh"
#include "gz/marine/CGALTypes.hh"

#include <gz/common.hh>
#include <gz/math.hh>

#include <array>
#include <iostream>
#include <iterator>
#include <string>

namespace ignition
{
namespace marine
{
///////////////////////////////////////////////////////////////////////////////
// MeshTools

  /// Vertex and Index conventions used by ignition::common::Mesh 
  ///
  /// Mesh::GetVertexCount()
  ///   returns the number of vertices in the mesh, which means that
  ///   the size of _vertices will be 3 * Mesh::GetVertexCount()
  ///
  /// Mesh::GetIndexCount()
  ///   returns the number of indices in the mesh, which means that
  ///   the size of _indices will be Mesh::GetIndexCount()
  ///
  void MeshTools::FillArrays(
    const common::Mesh& _source,
    std::vector<float>& _vertices,
    std::vector<int>& _indices
  )
  {    
    double *vertices = nullptr;
    int   *indices  = nullptr;

    // No leaks...
    try
    {
      size_t nv = _source.VertexCount();
      size_t ni = _source.IndexCount();

      // @DEBUG_INFO
      // ignmsg << "nv: " << nv << std::endl; 
      // ignmsg << "ni: " << ni << std::endl; 

      _source.FillArrays(&vertices, &indices);
      
      _vertices.insert(_vertices.end(), vertices, vertices + 3 * nv);
      _indices.insert(_indices.end(), indices, indices + ni);
    }
    catch(...) 
    {
      ignerr << "Unknown Error in Mesh::FillArrays" << std::endl;
    }
    // Clean up
    if (vertices)
      delete[] vertices;
    if (indices)
      delete[] indices;
  }

  void MeshTools::MakeSurfaceMesh(const common::Mesh& _source, cgal::Mesh& _target)
  {
    std::vector<float> vertices;
    std::vector<int>   indices;

    FillArrays(_source, vertices, indices);

    // Vertices
    for (size_t i=0; i<vertices.size(); )
    {
      auto&& v0 = vertices[i++];
      auto&& v1 = vertices[i++];
      auto&& v2 = vertices[i++];
      cgal::Point3 p(v0, v1, v2);
      auto&& v = _target.add_vertex(p);
      
      // @DEBUG_INFO
      // ignmsg << v << ": " << _target.point(v) << std::endl; 
    }

    // Faces
    for (size_t i=0; i<indices.size(); )
    {
      // @DEBUG_INFO
      // ignmsg << "face" << i/3 << std::endl; 

      auto v0 = _target.vertices().begin();
      auto v1 = _target.vertices().begin(); 
      auto v2 = _target.vertices().begin();
      auto i0 = indices[i++];
      auto i1 = indices[i++];
      auto i2 = indices[i++];
      std::advance(v0, i0);
      std::advance(v1, i1);
      std::advance(v2, i2);
      _target.add_face(*v0, *v1, *v2);  

      // @DEBUG_INFO
      // ignmsg << i0 << ", " << i1 << ", " << i2 << std::endl; 
      // ignmsg << *v0 << ": " << _target.point(*v0) << std::endl; 
      // ignmsg << *v1 << ": " << _target.point(*v1) << std::endl; 
      // ignmsg << *v2 << ": " << _target.point(*v2) << std::endl; 
    }
  }

///////////////////////////////////////////////////////////////////////////////

}
}

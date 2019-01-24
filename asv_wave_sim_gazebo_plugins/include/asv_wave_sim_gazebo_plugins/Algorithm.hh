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

/// \file Algorithm.hh
/// \brief Methods for sorting indexes into arrays and vectors.

#ifndef _ASV_WAVE_SIM_GAZEBO_PLUGINS_ALGORITHM_HH_
#define _ASV_WAVE_SIM_GAZEBO_PLUGINS_ALGORITHM_HH_

#include <algorithm>
#include <array>
#include <numeric>
#include <vector>

namespace asv
{
  /// \brief A small collection of static template methods for sorting arrays and vectors.
  namespace algorithm
  {
    /// \brief Sort and keep track of indexes (largest first)
    ///
    /// See:
    /// <https://stackoverflow.com/questions/1577475/c-sorting-and-keeping-track-of-indexes>
    ///
    /// Usage:
    /// \code
    /// for (auto i: sort_indexes(v)) {
    ///   cout << v[i] << endl;
    /// }
    /// \endcode
    ///
    /// \param[in] _v   The array to be indexed.
    /// \return         A vector of indexes in to the input array.
    template <typename T>
    std::vector<size_t> sort_indexes(const std::vector<T>& _v)
    {
      // initialize original index locations
      std::vector<size_t> idx(_v.size());
      std::iota(idx.begin(), idx.end(), 0);

      // sort indexes based on comparing values in _v
      std::sort(idx.begin(), idx.end(),
          [&_v](size_t i1, size_t i2) {return _v[i1] > _v[i2];});

      return idx;
    }

    /// \brief Sort and keep track of indexes (largest first)
    ///
    /// This version is for sorting std::array<T, N>
    /// \param[in] _v   The array to be indexed.
    /// \return         An array of indexes in to the input array.
    template <typename T, std::size_t N>
    std::array<size_t, N> sort_indexes(const std::array<T, N>& _v)
    {
      // initialize original index locations
      std::array<size_t, N> idx;
      std::iota(idx.begin(), idx.end(), 0);

      // sort indexes based on comparing values in _v
      std::sort(idx.begin(), idx.end(),
          [&_v](size_t i1, size_t i2) {return _v[i1] > _v[i2];});

      return idx;
    }

  } // namespace algorithm 
} // namespace asv

#endif // _ASV_WAVE_SIM_GAZEBO_PLUGINS_ALGORITHM_HH_

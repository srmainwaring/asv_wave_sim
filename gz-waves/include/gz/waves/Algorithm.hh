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

#ifndef GZ_WAVES_ALGORITHM_HH_
#define GZ_WAVES_ALGORITHM_HH_

#include <algorithm>
#include <array>
#include <numeric>
#include <unordered_map>
#include <utility>
#include <vector>

#include "gz/waves/Types.hh"

namespace gz
{
namespace waves
{
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
/// \param[in] v   The array to be indexed.
/// \return         A vector of indexes in to the input array.
template <typename T>
std::vector<Index> sort_indexes(const std::vector<T>& v)
{
  // initialize original index locations
  std::vector<Index> idx(v.size());
  std::iota(idx.begin(), idx.end(), 0);

  // sort indexes based on comparing values in v
  std::sort(idx.begin(), idx.end(),
      [&v](Index i1, Index i2) {return v[i1] > v[i2];});

  return idx;
}

/// \brief Sort and keep track of indexes (largest first)
///
/// This version is for sorting std::array<T, N>
/// \param[in] v    The array to be indexed.
/// \return         An array of indexes in to the input array.
template <typename T, std::size_t N>
std::array<Index, N> sort_indexes(const std::array<T, N>& v) {
  // initialize original index locations
  std::array<Index, N> idx;
  std::iota(idx.begin(), idx.end(), 0);

  // sort indexes based on comparing values in v
  std::sort(idx.begin(), idx.end(),
      [&v](Index i1, Index i2) {return v[i1] > v[i2];});

  return idx;
}

/// \brief C++ equivalent of numpy.unique
///
/// Adapted from homer512's SO answer.
/// https://stackoverflow.com/questions/70868307/c-equivalent-of-numpy-unique-on-stdvector-with-return-index-and-return-inver
///
template<typename T, typename Iterator, typename Idx>
void unordered_unique(
    Iterator first, Iterator last,
    std::vector<T>* unique,
    std::vector<Idx>* index = nullptr,
    std::vector<Idx>* inverse = nullptr,
    std::vector<Idx>* count = nullptr)
{
  using index_map = std::unordered_map<T, Idx>;
  using map_iter = typename index_map::iterator;
  using map_value = typename index_map::value_type;
  for (auto&& arg : {index, inverse, count}) {
    if (arg) {
      arg->clear();
    }
  }
  index_map map;
  std::size_t cur_idx = 0;
  for (auto i = first; i != last; ++cur_idx, ++i) {
    const std::pair<map_iter, bool> inserted =
      map.emplace(*i, unique->size());
    map_value& ival = *inserted.first;
    if (inserted.second) {
      unique->push_back(ival.first);
      if (index) {
        index->push_back(cur_idx);
      }
      if (count) {
        count->push_back(1);
      }
    } else if (count) {
      (*count)[ival.second] += 1;
    }
    if (inverse) {
      inverse->push_back(ival.second);
    }
  }
}

}  // namespace algorithm
}  // namespace waves
}  // namespace gz

#endif  // GZ_WAVES_ALGORITHM_HH_

// Copyright 2024 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef AUTOWARE_UNIVERSE_UTILS_GEOMETRY_FACD_2D_HPP_
#define AUTOWARE_UNIVERSE_UTILS_GEOMETRY_FACD_2D_HPP_

#include "autoware/universe_utils/geometry/boost_geometry.hpp"

namespace autoware::universe_utils::facd
{

/// @brief Decomposes a concave polygon into a set of convex polygons using FACD.
/// @param polygon The input concave polygon.
/// @return A collection of convex polygons.
autoware::universe_utils::MultiPolygon2d fast_approximate_convex_decomposition(const Polygon2d & polygon);

}  // namespace autoware::universe_utils::facd

#endif  // AUTOWARE_UNIVERSE_UTILS_GEOMETRY_FACD_2D_HPP_

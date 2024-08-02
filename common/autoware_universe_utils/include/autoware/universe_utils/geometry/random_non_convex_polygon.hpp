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

#ifndef AUTOWARE__UNIVERSE_UTILS__GEOMETRY__RANDOM_POLYGON_HPP_
#define AUTOWARE__UNIVERSE_UTILS__GEOMETRY__RANDOM_POLYGON_HPP_

#include <autoware/universe_utils/geometry/geometry.hpp>

namespace autoware::universe_utils
{
/// @brief Generate a random non-convex polygon
/// @param vertices Number of vertices for the desired polygon
/// @param max Points will be generated in the range [-max, max]
/// @details This function generates a simple non-convex polygon by randomly placing vertices and
///          then applying random perturbations to create intersections or adjust the shape.
Polygon2d random_non_convex_polygon(const size_t vertices, const double max);

}  // namespace autoware::universe_utils

#endif  // AUTOWARE__UNIVERSE_UTILS__GEOMETRY__RANDOM_POLYGON_HPP_

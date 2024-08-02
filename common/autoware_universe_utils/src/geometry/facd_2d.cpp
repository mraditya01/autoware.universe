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

#include "autoware/universe_utils/geometry/facd_2d.hpp"

#include <vector>
#include <algorithm>

namespace autoware::universe_utils::facd
{

namespace
{
/// @brief calculate the dot product between two points
double dot_product(const Point2d & p1, const Point2d & p2)
{
  return p1.x() * p2.x() + p1.y() * p2.y();
}

/// @brief calculate the cross product of two points
double cross_product(const Point2d & p1, const Point2d & p2)
{
  return p1.x() * p2.y() - p1.y() * p2.x();
}

/// @brief check if a point is inside a triangle
bool is_point_in_triangle(const Point2d & pt, const Point2d & v1, const Point2d & v2, const Point2d & v3)
{
  auto d1 = cross_product(v2 - v1, pt - v1);
  auto d2 = cross_product(v3 - v2, pt - v2);
  auto d3 = cross_product(v1 - v3, pt - v3);
  bool has_neg = (d1 < 0) || (d2 < 0) || (d3 < 0);
  bool has_pos = (d1 > 0) || (d2 > 0) || (d3 > 0);
  return !(has_neg && has_pos);
}

/// @brief triangulate a simple polygon using ear clipping
std::vector<Polygon2d> triangulate(const Polygon2d & polygon)
{
  std::vector<Polygon2d> triangles;

  if (polygon.outer().size() < 3) {
    return triangles;
  }

  std::vector<Point2d> points = polygon.outer();

  while (points.size() > 3) {
    bool ear_found = false;
    for (size_t i = 0; i < points.size(); ++i) {
      size_t prev = (i + points.size() - 1) % points.size();
      size_t next = (i + 1) % points.size();

      if (cross_product(points[next] - points[i], points[prev] - points[i]) <= 0) {
        continue;
      }

      bool is_ear = true;
      for (size_t j = 0; j < points.size(); ++j) {
        if (j == i || j == prev || j == next) {
          continue;
        }

        if (is_point_in_triangle(points[j], points[prev], points[i], points[next])) {
          is_ear = false;
          break;
        }
      }

      if (is_ear) {
        Polygon2d triangle;
        triangle.outer().push_back(points[prev]);
        triangle.outer().push_back(points[i]);
        triangle.outer().push_back(points[next]);
        triangles.push_back(triangle);

        points.erase(points.begin() + i);
        ear_found = true;
        break;
      }
    }

    if (!ear_found) {
      break;  // failed to find an ear, polygon might be self-intersecting or not simple
    }
  }

  if (points.size() == 3) {
    Polygon2d triangle;
    triangle.outer() = points;
    triangles.push_back(triangle);
  }

  return triangles;
}

/// @brief Merge adjacent triangles into convex polygons
std::vector<Polygon2d> merge_triangles(const std::vector<Polygon2d> & triangles)
{
  std::vector<Polygon2d> convex_polygons = triangles;

  // TODO: Implement actual merging logic to create larger convex polygons

  return convex_polygons;
}

/// @brief Decompose a concave polygon into convex polygons using an approximation method
void decompose(const Polygon2d & polygon, std::vector<Polygon2d> & convex_polygons)
{
  // Step 1: Triangulate the polygon
  auto triangles = triangulate(polygon);

  // Step 2: Merge triangles into larger convex polygons
  convex_polygons = merge_triangles(triangles);
}

}  // namespace

std::vector<Polygon2d> fast_approximate_convex_decomposition(const Polygon2d & polygon)
{
  std::vector<Polygon2d> convex_polygons;
  decompose(polygon, convex_polygons);
  return convex_polygons;
}

}  // namespace autoware::universe_utils::facd

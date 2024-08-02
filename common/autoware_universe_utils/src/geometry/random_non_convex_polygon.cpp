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

#include "autoware/universe_utils/geometry/random_convex_polygon.hpp"
#include <boost/geometry/algorithms/correct.hpp>
#include <boost/geometry/algorithms/is_simple.hpp>
#include <boost/geometry/algorithms/is_valid.hpp>
#include <algorithm>
#include <random>

namespace autoware::universe_utils
{
namespace
{
std::vector<Point2d> generate_random_vertices(const size_t vertices, const double max)
{
    std::random_device r;
    std::default_random_engine random_engine(r());
    std::uniform_real_distribution<double> uniform_dist(-max, max);
    std::vector<Point2d> points;
    for (size_t i = 0; i < vertices; ++i) {
        points.emplace_back(uniform_dist(random_engine), uniform_dist(random_engine));
    }
    return points;
}

Polygon2d create_simple_polygon(const std::vector<Point2d>& vertices)
{
    LinearRing2d ring;
    std::vector<Point2d> sorted_vertices = vertices;
    std::sort(sorted_vertices.begin(), sorted_vertices.end(), [](const Point2d& a, const Point2d& b) {
        return std::atan2(a.y(), a.x()) < std::atan2(b.y(), b.x());
    });

    for (const auto& p : sorted_vertices) {
        ring.emplace_back(p.x(), p.y());
    }
    Polygon2d poly;
    poly.outer() = ring;
    return poly;
}

Polygon2d introduce_non_convexity(const Polygon2d& poly) 
{
    Polygon2d poly_with_intersections = poly;
    bool valid_polygon = false;
    std::random_device r;
    std::default_random_engine random_engine(r());
    std::uniform_real_distribution<double> dist(-2.0, 2.0); // Reduced perturbation range

    size_t max_attempts = 100; // Limit attempts to avoid infinite loops
    size_t attempts = 0;

    while (!valid_polygon && attempts < max_attempts) {
        // Introduce non-convexity with some random perturbation
        if (poly_with_intersections.outer().size() > 3) {
            // Randomly perturb vertices
            for (auto& p : poly_with_intersections.outer()) {
                p.x() += dist(random_engine);
                p.y() += dist(random_engine);
            }

            // Optionally, insert a new vertex to increase complexity
            poly_with_intersections.outer().insert(poly_with_intersections.outer().begin() + 2, 
                Point2d(poly_with_intersections.outer()[1].x() + 2, // Reduced insertion offset
                        poly_with_intersections.outer()[1].y() + 2));
        }

        // Ensure the polygon is simple and valid after perturbation
        boost::geometry::correct(poly_with_intersections);
        if (boost::geometry::is_simple(poly_with_intersections) && boost::geometry::is_valid(poly_with_intersections)) {
            valid_polygon = true; // Exit the loop if the polygon is valid
        } else {
            // If not valid, clear and reinitialize the polygon to avoid infinite loop
            poly_with_intersections = poly;
        }

        ++attempts;
    }

    if (!valid_polygon) {
        // Handle failure to produce a valid polygon within the attempt limit
        std::cerr << "Warning: Failed to generate a valid non-convex polygon after " << max_attempts << " attempts." << std::endl;
    }

    return poly_with_intersections;
}

}  // namespace

Polygon2d random_non_convex_polygon(const size_t vertices, const double max)
{
    auto random_vertices = generate_random_vertices(vertices, max);
    auto simple_polygon = create_simple_polygon(random_vertices);
    auto non_convex_polygon = introduce_non_convexity(simple_polygon);

    return non_convex_polygon;
}
}  // namespace autoware::universe_utils

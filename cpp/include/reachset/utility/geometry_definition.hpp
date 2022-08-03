#pragma once

#include <boost/geometry/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

namespace bg = boost::geometry;
using GeometryPoint = bg::model::d2::point_xy<double>;
using GeometryRing = bg::model::ring<GeometryPoint, false>;
using GeometryPolygon = bg::model::polygon<GeometryPoint>;
using GeometryBox = bg::model::box<GeometryPoint>;
using GeometryPolygonPtr = std::shared_ptr<GeometryPolygon>;


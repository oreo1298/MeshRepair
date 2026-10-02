// MeshRepair - manifold repair for triangle meshes.
//
// Core geometric types. The library is dependency free (C++17 standard library
// only) so that it can be embedded into other applications such as slicers.

#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

namespace meshrepair {

struct Vec3
{
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    constexpr Vec3() = default;
    constexpr Vec3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}

    double  operator[](int i) const { return i == 0 ? x : (i == 1 ? y : z); }
    double& operator[](int i) { return i == 0 ? x : (i == 1 ? y : z); }

    Vec3  operator+(const Vec3& o) const { return { x + o.x, y + o.y, z + o.z }; }
    Vec3  operator-(const Vec3& o) const { return { x - o.x, y - o.y, z - o.z }; }
    Vec3  operator-() const { return { -x, -y, -z }; }
    Vec3  operator*(double s) const { return { x * s, y * s, z * s }; }
    Vec3  operator/(double s) const { return { x / s, y / s, z / s }; }
    Vec3& operator+=(const Vec3& o) { x += o.x; y += o.y; z += o.z; return *this; }
    Vec3& operator-=(const Vec3& o) { x -= o.x; y -= o.y; z -= o.z; return *this; }
    Vec3& operator*=(double s) { x *= s; y *= s; z *= s; return *this; }
    Vec3& operator/=(double s) { x /= s; y /= s; z /= s; return *this; }
    bool  operator==(const Vec3& o) const { return x == o.x && y == o.y && z == o.z; }
    bool  operator!=(const Vec3& o) const { return !(*this == o); }

    double dot(const Vec3& o) const { return x * o.x + y * o.y + z * o.z; }
    Vec3   cross(const Vec3& o) const { return { y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x }; }
    double squared_norm() const { return dot(*this); }
    double norm() const { return std::sqrt(squared_norm()); }
    Vec3   normalized() const
    {
        const double n = norm();
        return n > 0.0 ? *this / n : Vec3();
    }
    bool is_finite() const { return std::isfinite(x) && std::isfinite(y) && std::isfinite(z); }
};

inline Vec3 operator*(double s, const Vec3& v) { return v * s; }
inline Vec3 cwise_min(const Vec3& a, const Vec3& b) { return { std::min(a.x, b.x), std::min(a.y, b.y), std::min(a.z, b.z) }; }
inline Vec3 cwise_max(const Vec3& a, const Vec3& b) { return { std::max(a.x, b.x), std::max(a.y, b.y), std::max(a.z, b.z) }; }

// Vertex indices of a triangle, counter-clockwise when looking at the outside.
using Triangle = std::array<int, 3>;

struct BoundingBox
{
    Vec3 min { std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), std::numeric_limits<double>::max() };
    Vec3 max { std::numeric_limits<double>::lowest(), std::numeric_limits<double>::lowest(), std::numeric_limits<double>::lowest() };

    bool valid() const { return min.x <= max.x && min.y <= max.y && min.z <= max.z; }
    void extend(const Vec3& p)
    {
        min = cwise_min(min, p);
        max = cwise_max(max, p);
    }
    void extend(const BoundingBox& b)
    {
        if (b.valid()) {
            extend(b.min);
            extend(b.max);
        }
    }
    void inflate(double d)
    {
        min -= Vec3(d, d, d);
        max += Vec3(d, d, d);
    }
    Vec3   size() const { return valid() ? max - min : Vec3(); }
    Vec3   center() const { return valid() ? (min + max) * 0.5 : Vec3(); }
    double diagonal() const { return size().norm(); }
    bool   contains(const Vec3& p) const
    {
        return p.x >= min.x && p.x <= max.x && p.y >= min.y && p.y <= max.y && p.z >= min.z && p.z <= max.z;
    }
    bool contains(const BoundingBox& b) const { return contains(b.min) && contains(b.max); }
    bool intersects(const BoundingBox& b) const
    {
        return min.x <= b.max.x && max.x >= b.min.x && min.y <= b.max.y && max.y >= b.min.y && min.z <= b.max.z &&
               max.z >= b.min.z;
    }
};

// Indexed triangle set. This mirrors the layout of the indexed_triangle_set
// used by PrusaSlicer / Bambu Studio / OrcaSlicer so that conversion is trivial.
struct Mesh
{
    std::vector<Vec3>     vertices;
    std::vector<Triangle> faces;

    bool   empty() const { return faces.empty(); }
    void   clear()
    {
        vertices.clear();
        faces.clear();
    }
};

BoundingBox bounding_box(const Mesh& mesh);

// Twice the area vector of a triangle (cross product of two edges).
inline Vec3 triangle_normal(const Vec3& a, const Vec3& b, const Vec3& c) { return (b - a).cross(c - a); }
inline double triangle_area(const Vec3& a, const Vec3& b, const Vec3& c) { return 0.5 * triangle_normal(a, b, c).norm(); }

Vec3   face_normal(const Mesh& mesh, size_t face_idx); // unit length, zero for degenerate faces
double face_area(const Mesh& mesh, size_t face_idx);

// Signed volume enclosed by the mesh. Only meaningful for closed meshes.
double signed_volume(const Mesh& mesh);
double surface_area(const Mesh& mesh);

// Removes vertices not referenced by any face and renumbers the faces.
// Returns the number of removed vertices.
size_t remove_unreferenced_vertices(Mesh& mesh);

// Reverses the orientation of all faces.
void flip_all_faces(Mesh& mesh);

} // namespace meshrepair

#include "meshrepair/Mesh.hpp"

namespace meshrepair {

BoundingBox bounding_box(const Mesh& mesh)
{
    BoundingBox bb;
    for (const Triangle& f : mesh.faces)
        for (int v : f)
            if (v >= 0 && size_t(v) < mesh.vertices.size() && mesh.vertices[v].is_finite())
                bb.extend(mesh.vertices[v]);
    return bb;
}

Vec3 face_normal(const Mesh& mesh, size_t face_idx)
{
    const Triangle& f = mesh.faces[face_idx];
    return triangle_normal(mesh.vertices[f[0]], mesh.vertices[f[1]], mesh.vertices[f[2]]).normalized();
}

double face_area(const Mesh& mesh, size_t face_idx)
{
    const Triangle& f = mesh.faces[face_idx];
    return triangle_area(mesh.vertices[f[0]], mesh.vertices[f[1]], mesh.vertices[f[2]]);
}

double signed_volume(const Mesh& mesh)
{
    // Use the bounding box center as origin to limit cancellation errors.
    const Vec3 o      = bounding_box(mesh).center();
    double     volume = 0.0;
    for (const Triangle& f : mesh.faces) {
        const Vec3 a = mesh.vertices[f[0]] - o;
        const Vec3 b = mesh.vertices[f[1]] - o;
        const Vec3 c = mesh.vertices[f[2]] - o;
        volume += a.dot(b.cross(c));
    }
    return volume / 6.0;
}

double surface_area(const Mesh& mesh)
{
    double area = 0.0;
    for (size_t i = 0; i < mesh.faces.size(); ++i)
        area += face_area(mesh, i);
    return area;
}

size_t remove_unreferenced_vertices(Mesh& mesh)
{
    // Keeps the relative order of the remaining vertices.
    std::vector<int> remap(mesh.vertices.size(), -1);
    for (const Triangle& f : mesh.faces)
        for (int v : f)
            remap[v] = 0;
    std::vector<Vec3> vertices;
    vertices.reserve(mesh.vertices.size());
    for (size_t i = 0; i < remap.size(); ++i)
        if (remap[i] == 0) {
            remap[i] = int(vertices.size());
            vertices.push_back(mesh.vertices[i]);
        }
    for (Triangle& f : mesh.faces)
        for (int& v : f)
            v = remap[v];
    const size_t removed = mesh.vertices.size() - vertices.size();
    mesh.vertices.swap(vertices);
    return removed;
}

void flip_all_faces(Mesh& mesh)
{
    for (Triangle& f : mesh.faces)
        std::swap(f[1], f[2]);
}

} // namespace meshrepair

// 3D view of a mesh with its defects highlighted.

#pragma once

#include "GL.hpp"

#include <meshrepair/Diagnostics.hpp>
#include <meshrepair/Mesh.hpp>

#include <cstdint>
#include <string>
#include <vector>

namespace mrgui {

struct Mat4
{
    float m[16]; // column major
    static Mat4 identity();
    Mat4        operator*(const Mat4& o) const;
};

// Orbit camera around a target point, Z up (3D printing convention).
class Camera
{
public:
    void fit(const meshrepair::BoundingBox& box);
    void orbit(double dx_px, double dy_px);
    void pan(double dx_px, double dy_px, int viewport_h);
    void zoom(double steps);
    void set_view(double yaw_deg, double pitch_deg);

    Mat4            view() const;
    Mat4            projection(double aspect) const;
    meshrepair::Vec3 eye() const;

    meshrepair::Vec3 target;
    double           distance = 10.0;
    double           yaw      = -60.0; // degrees, around Z
    double           pitch    = 25.0;  // degrees above the XY plane
    double           fov      = 35.0;  // vertical, degrees
    double           radius   = 1.0;   // scene radius, for clipping planes
};

struct ViewSettings
{
    bool  wireframe      = false;
    bool  show_issues    = true; // open / non-manifold edges and vertices
    bool  xray_issues    = true; // also show issues hidden behind surfaces
    bool  show_new_faces = true; // tint faces created by the repair
    bool  show_grid      = true;
    float issue_line_width = 2.5f;
};

// Face classes used for colouring.
enum FaceClass : uint8_t {
    FaceNormal   = 0,
    FaceNew      = 1, // added by hole filling
    FaceSplit    = 2, // split while closing cracks
    FaceInverted = 3, // part of an inside-out shell
};

class Viewer
{
public:
    ~Viewer();
    bool init(std::string& error);
    void destroy();

    // Uploads a mesh. face_class may be empty. The detail lists are drawn as
    // overlays (open edges red, non-manifold edges yellow, ...). Visible back
    // faces are drawn red when alert_back_faces is set (mesh with defects).
    void set_mesh(const meshrepair::Mesh& mesh, const std::vector<uint8_t>& face_class,
                  const meshrepair::DiagnosticsDetail& detail, bool alert_back_faces);
    void clear_mesh();
    bool has_mesh() const { return m_tri_vertices > 0; }

    const meshrepair::BoundingBox& bounds() const { return m_bounds; } // in centered coordinates

    void draw(const Camera& cam, int x, int y, int w, int h, const ViewSettings& vs);

    // Renders into an offscreen buffer and returns RGBA pixels (top row first).
    bool render_offscreen(const Camera& cam, int w, int h, const ViewSettings& vs, std::vector<uint8_t>& rgba);

private:
    void build_grid();

    gl::GLuint m_prog_mesh  = 0;
    gl::GLuint m_prog_lines = 0;
    gl::GLuint m_prog_bg    = 0;

    gl::GLuint m_vao_mesh = 0, m_vbo_mesh = 0;
    gl::GLuint m_vao_lines = 0, m_vbo_lines = 0;
    gl::GLuint m_vao_points = 0, m_vbo_points = 0;
    gl::GLuint m_vao_grid = 0, m_vbo_grid = 0;
    gl::GLuint m_vao_empty = 0;

    bool   m_alert_back     = true;
    size_t m_tri_vertices   = 0;
    size_t m_line_vertices  = 0;
    size_t m_point_vertices = 0;
    size_t m_grid_vertices  = 0;

    meshrepair::BoundingBox m_bounds; // centered coordinates
    meshrepair::Vec3        m_offset; // model coordinates = centered + offset
};

} // namespace mrgui

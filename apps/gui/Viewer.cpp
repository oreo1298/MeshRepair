#include "Viewer.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

using namespace meshrepair;

namespace mrgui {

namespace {

constexpr double kPi = 3.14159265358979323846;

const char* kMeshVS = R"(#version 330 core
layout(location = 0) in vec3 a_pos;
layout(location = 1) in vec3 a_normal;
layout(location = 2) in float a_class;
uniform mat4 u_mvp;
uniform mat4 u_view;
out vec3 v_normal;
out vec3 v_pos;
out vec3 v_bary;
flat out int v_class;
void main()
{
    gl_Position = u_mvp * vec4(a_pos, 1.0);
    v_normal    = mat3(u_view) * a_normal;
    v_pos       = (u_view * vec4(a_pos, 1.0)).xyz;
    v_class     = int(a_class + 0.5);
    int k       = gl_VertexID % 3;
    v_bary      = vec3(k == 0 ? 1.0 : 0.0, k == 1 ? 1.0 : 0.0, k == 2 ? 1.0 : 0.0);
}
)";

const char* kMeshFS = R"(#version 330 core
in vec3 v_normal;
in vec3 v_pos;
in vec3 v_bary;
flat in int v_class;
uniform int u_show_classes;
uniform int u_wire;
uniform int u_alert_back;
out vec4 o_color;
void main()
{
    vec3 base = vec3(0.62, 0.68, 0.78);
    if (u_show_classes != 0) {
        if (v_class == 1)
            base = vec3(0.36, 0.82, 0.46); // filled hole
        else if (v_class == 2)
            base = vec3(0.45, 0.75, 0.90); // split to close a crack
        else if (v_class == 3)
            base = vec3(0.95, 0.58, 0.22); // inside-out shell
    }
    vec3 n = normalize(v_normal);
    if (dot(n, n) < 0.5)
        n = vec3(0.0, 0.0, 1.0);
    if (!gl_FrontFacing) {
        // Looking at the back side: through a hole or at a flipped face.
        // Inside-out shells keep their own colour.
        n = -n;
        if (u_show_classes == 0 || v_class != 3)
            base = u_alert_back != 0 ? vec3(0.86, 0.26, 0.30) : base * 0.6;
    }
    vec3  v    = normalize(-v_pos);
    vec3  l1   = normalize(vec3(0.35, 0.55, 1.0));
    vec3  l2   = normalize(vec3(-0.6, -0.2, 0.4));
    float diff = max(dot(n, l1), 0.0) * 0.75 + max(dot(n, l2), 0.0) * 0.25;
    float spec = pow(max(dot(reflect(-l1, n), v), 0.0), 40.0) * 0.18;
    vec3  c    = base * (0.28 + 0.72 * diff) + vec3(spec);
    if (u_wire != 0) {
        vec3  d = fwidth(v_bary);
        vec3  a = smoothstep(vec3(0.0), d * 1.25, v_bary);
        float e = min(min(a.x, a.y), a.z);
        c       = mix(c * 0.35, c, e);
    }
    o_color = vec4(c, 1.0);
}
)";

const char* kLinesVS = R"(#version 330 core
layout(location = 0) in vec3 a_pos;
layout(location = 1) in vec4 a_color;
uniform mat4 u_mvp;
uniform float u_point_size;
out vec4 v_color;
void main()
{
    gl_Position  = u_mvp * vec4(a_pos, 1.0);
    gl_PointSize = u_point_size;
    v_color      = a_color;
}
)";

const char* kLinesFS = R"(#version 330 core
in vec4 v_color;
uniform float u_alpha;
uniform int u_round;
out vec4 o_color;
void main()
{
    if (u_round != 0) {
        vec2 p = gl_PointCoord * 2.0 - 1.0;
        if (dot(p, p) > 1.0)
            discard;
    }
    o_color = vec4(v_color.rgb, v_color.a * u_alpha);
}
)";

const char* kBackgroundVS = R"(#version 330 core
out float v_t;
void main()
{
    vec2 p      = vec2((gl_VertexID << 1) & 2, gl_VertexID & 2) * 2.0 - 1.0;
    v_t         = p.y * 0.5 + 0.5;
    gl_Position = vec4(p, 0.999, 1.0);
}
)";

const char* kBackgroundFS = R"(#version 330 core
in float v_t;
out vec4 o_color;
void main()
{
    vec3 bottom = vec3(0.105, 0.115, 0.135);
    vec3 top    = vec3(0.215, 0.235, 0.275);
    o_color     = vec4(mix(bottom, top, v_t), 1.0);
}
)";

gl::GLuint compile(gl::GLenum type, const char* src, std::string& error)
{
    const gl::GLuint s = gl::CreateShader(type);
    gl::ShaderSource(s, 1, &src, nullptr);
    gl::CompileShader(s);
    gl::GLint ok = 0;
    gl::GetShaderiv(s, gl::COMPILE_STATUS, &ok);
    if (!ok) {
        char log[2048];
        gl::GetShaderInfoLog(s, sizeof(log), nullptr, log);
        error = std::string("Shader compilation failed: ") + log;
        gl::DeleteShader(s);
        return 0;
    }
    return s;
}

gl::GLuint link(const char* vs, const char* fs, std::string& error)
{
    const gl::GLuint v = compile(gl::VERTEX_SHADER, vs, error);
    if (!v)
        return 0;
    const gl::GLuint f = compile(gl::FRAGMENT_SHADER, fs, error);
    if (!f) {
        gl::DeleteShader(v);
        return 0;
    }
    const gl::GLuint p = gl::CreateProgram();
    gl::AttachShader(p, v);
    gl::AttachShader(p, f);
    gl::LinkProgram(p);
    gl::DeleteShader(v);
    gl::DeleteShader(f);
    gl::GLint ok = 0;
    gl::GetProgramiv(p, gl::LINK_STATUS, &ok);
    if (!ok) {
        char log[2048];
        gl::GetProgramInfoLog(p, sizeof(log), nullptr, log);
        error = std::string("Shader link failed: ") + log;
        gl::DeleteProgram(p);
        return 0;
    }
    return p;
}

Mat4 look_at(const Vec3& eye, const Vec3& target, const Vec3& up)
{
    const Vec3 f = (target - eye).normalized();
    Vec3       s = f.cross(up).normalized();
    if (s.squared_norm() == 0.0)
        s = Vec3(1, 0, 0);
    const Vec3 u = s.cross(f);
    Mat4       m = Mat4::identity();
    m.m[0]  = float(s.x);
    m.m[4]  = float(s.y);
    m.m[8]  = float(s.z);
    m.m[1]  = float(u.x);
    m.m[5]  = float(u.y);
    m.m[9]  = float(u.z);
    m.m[2]  = float(-f.x);
    m.m[6]  = float(-f.y);
    m.m[10] = float(-f.z);
    m.m[12] = float(-s.dot(eye));
    m.m[13] = float(-u.dot(eye));
    m.m[14] = float(f.dot(eye));
    return m;
}

Mat4 perspective(double fovy_deg, double aspect, double znear, double zfar)
{
    const double f = 1.0 / std::tan(fovy_deg * kPi / 360.0);
    Mat4         m {};
    m.m[0]  = float(f / aspect);
    m.m[5]  = float(f);
    m.m[10] = float((zfar + znear) / (znear - zfar));
    m.m[11] = -1.0f;
    m.m[14] = float(2.0 * zfar * znear / (znear - zfar));
    return m;
}

struct LineVertex
{
    float x, y, z;
    float r, g, b, a;
};

} // namespace

Mat4 Mat4::identity()
{
    Mat4 r {};
    r.m[0] = r.m[5] = r.m[10] = r.m[15] = 1.0f;
    return r;
}

Mat4 Mat4::operator*(const Mat4& o) const
{
    Mat4 r {};
    for (int c = 0; c < 4; ++c)
        for (int rr = 0; rr < 4; ++rr) {
            float s = 0.0f;
            for (int k = 0; k < 4; ++k)
                s += m[k * 4 + rr] * o.m[c * 4 + k];
            r.m[c * 4 + rr] = s;
        }
    return r;
}

// ---------------------------------------------------------------------------

void Camera::fit(const BoundingBox& box)
{
    if (!box.valid())
        return;
    target           = box.center();
    radius           = std::max(box.diagonal() * 0.5, 1e-6);
    const double fit = fov * 0.5 * kPi / 180.0;
    distance         = radius / std::sin(fit) * 1.05;
}

void Camera::orbit(double dx, double dy)
{
    yaw -= dx * 0.35;
    pitch += dy * 0.35;
    pitch = std::max(-89.0, std::min(89.0, pitch));
    if (yaw > 180.0)
        yaw -= 360.0;
    if (yaw < -180.0)
        yaw += 360.0;
}

void Camera::pan(double dx, double dy, int viewport_h)
{
    const Vec3   e     = eye();
    const Vec3   f     = (target - e).normalized();
    Vec3         right = f.cross(Vec3(0, 0, 1)).normalized();
    if (right.squared_norm() == 0.0)
        right = Vec3(1, 0, 0);
    const Vec3   up    = right.cross(f);
    const double scale = 2.0 * distance * std::tan(fov * kPi / 360.0) / std::max(1, viewport_h);
    target -= right * (dx * scale);
    target += up * (dy * scale);
}

void Camera::zoom(double steps)
{
    distance *= std::pow(0.88, steps);
    distance = std::max(distance, radius * 1e-3);
    distance = std::min(distance, radius * 1e3);
}

void Camera::set_view(double yaw_deg, double pitch_deg)
{
    yaw   = yaw_deg;
    pitch = pitch_deg;
}

Vec3 Camera::eye() const
{
    const double y = yaw * kPi / 180.0, p = pitch * kPi / 180.0;
    return target + Vec3(std::cos(p) * std::cos(y), std::cos(p) * std::sin(y), std::sin(p)) * distance;
}

Mat4 Camera::view() const { return look_at(eye(), target, Vec3(0, 0, 1)); }

Mat4 Camera::projection(double aspect) const
{
    const double dist_to_center = (eye() - target).norm();
    const double znear          = std::max(dist_to_center - radius * 4.0, distance * 1e-3);
    const double zfar           = dist_to_center + radius * 4.0;
    return perspective(fov, aspect, znear, zfar);
}

// ---------------------------------------------------------------------------

Viewer::~Viewer() = default;

bool Viewer::init(std::string& error)
{
    m_prog_mesh = link(kMeshVS, kMeshFS, error);
    if (!m_prog_mesh)
        return false;
    m_prog_lines = link(kLinesVS, kLinesFS, error);
    if (!m_prog_lines)
        return false;
    m_prog_bg = link(kBackgroundVS, kBackgroundFS, error);
    if (!m_prog_bg)
        return false;

    gl::GenVertexArrays(1, &m_vao_mesh);
    gl::GenBuffers(1, &m_vbo_mesh);
    gl::BindVertexArray(m_vao_mesh);
    gl::BindBuffer(gl::ARRAY_BUFFER, m_vbo_mesh);
    gl::EnableVertexAttribArray(0);
    gl::VertexAttribPointer(0, 3, gl::FLOAT, gl::FALSE_, 7 * sizeof(float), reinterpret_cast<void*>(0));
    gl::EnableVertexAttribArray(1);
    gl::VertexAttribPointer(1, 3, gl::FLOAT, gl::FALSE_, 7 * sizeof(float), reinterpret_cast<void*>(3 * sizeof(float)));
    gl::EnableVertexAttribArray(2);
    gl::VertexAttribPointer(2, 1, gl::FLOAT, gl::FALSE_, 7 * sizeof(float), reinterpret_cast<void*>(6 * sizeof(float)));

    for (auto [vao, vbo] : { std::pair<gl::GLuint*, gl::GLuint*> { &m_vao_lines, &m_vbo_lines },
                             { &m_vao_points, &m_vbo_points },
                             { &m_vao_grid, &m_vbo_grid } }) {
        gl::GenVertexArrays(1, vao);
        gl::GenBuffers(1, vbo);
        gl::BindVertexArray(*vao);
        gl::BindBuffer(gl::ARRAY_BUFFER, *vbo);
        gl::EnableVertexAttribArray(0);
        gl::VertexAttribPointer(0, 3, gl::FLOAT, gl::FALSE_, sizeof(LineVertex), reinterpret_cast<void*>(0));
        gl::EnableVertexAttribArray(1);
        gl::VertexAttribPointer(1, 4, gl::FLOAT, gl::FALSE_, sizeof(LineVertex),
                                reinterpret_cast<void*>(3 * sizeof(float)));
    }
    gl::GenVertexArrays(1, &m_vao_empty);
    gl::BindVertexArray(0);
    return true;
}

void Viewer::destroy()
{
    for (gl::GLuint* b : { &m_vbo_mesh, &m_vbo_lines, &m_vbo_points, &m_vbo_grid })
        if (*b) {
            gl::DeleteBuffers(1, b);
            *b = 0;
        }
    for (gl::GLuint* a : { &m_vao_mesh, &m_vao_lines, &m_vao_points, &m_vao_grid, &m_vao_empty })
        if (*a) {
            gl::DeleteVertexArrays(1, a);
            *a = 0;
        }
    for (gl::GLuint* p : { &m_prog_mesh, &m_prog_lines, &m_prog_bg })
        if (*p) {
            gl::DeleteProgram(*p);
            *p = 0;
        }
}

void Viewer::clear_mesh()
{
    m_tri_vertices = m_line_vertices = m_point_vertices = m_grid_vertices = 0;
    m_bounds = BoundingBox();
}

void Viewer::set_mesh(const Mesh& mesh, const std::vector<uint8_t>& face_class, const DiagnosticsDetail& detail,
                      bool alert_back_faces)
{
    m_alert_back = alert_back_faces;
    BoundingBox model_box = bounding_box(mesh);
    m_offset              = model_box.valid() ? model_box.center() : Vec3();
    m_bounds              = BoundingBox();

    const size_t       nv = mesh.vertices.size();
    std::vector<float> data;
    data.reserve(mesh.faces.size() * 21);
    for (size_t f = 0; f < mesh.faces.size(); ++f) {
        const Triangle& t     = mesh.faces[f];
        bool            valid = true;
        for (int v : t)
            valid &= v >= 0 && size_t(v) < nv && mesh.vertices[v].is_finite();
        if (!valid)
            continue;
        const Vec3  n   = face_normal(mesh, f);
        const float cls = f < face_class.size() ? float(face_class[f]) : 0.0f;
        for (int k = 0; k < 3; ++k) {
            const Vec3 p = mesh.vertices[t[k]] - m_offset;
            m_bounds.extend(p);
            data.insert(data.end(), { float(p.x), float(p.y), float(p.z), float(n.x), float(n.y), float(n.z), cls });
        }
    }
    m_tri_vertices = data.size() / 7;
    gl::BindBuffer(gl::ARRAY_BUFFER, m_vbo_mesh);
    gl::BufferData(gl::ARRAY_BUFFER, gl::GLsizeiptr(data.size() * sizeof(float)), data.data(), gl::STATIC_DRAW);

    // Issue overlays.
    std::vector<LineVertex> lines;
    auto add_edges = [&](const std::vector<Edge>& edges, float r, float g, float b) {
        for (const Edge& e : edges)
            for (int v : e) {
                const Vec3 p = mesh.vertices[v] - m_offset;
                lines.push_back({ float(p.x), float(p.y), float(p.z), r, g, b, 1.0f });
            }
    };
    add_edges(detail.open_edges, 1.0f, 0.22f, 0.22f);
    add_edges(detail.non_manifold_edges, 1.0f, 0.85f, 0.1f);
    add_edges(detail.inconsistent_edges, 1.0f, 0.5f, 0.0f);
    m_line_vertices = lines.size();
    gl::BindBuffer(gl::ARRAY_BUFFER, m_vbo_lines);
    gl::BufferData(gl::ARRAY_BUFFER, gl::GLsizeiptr(lines.size() * sizeof(LineVertex)), lines.data(), gl::STATIC_DRAW);

    std::vector<LineVertex> points;
    for (int v : detail.non_manifold_vertices) {
        const Vec3 p = mesh.vertices[v] - m_offset;
        points.push_back({ float(p.x), float(p.y), float(p.z), 0.85f, 0.35f, 1.0f, 1.0f });
    }
    m_point_vertices = points.size();
    gl::BindBuffer(gl::ARRAY_BUFFER, m_vbo_points);
    gl::BufferData(gl::ARRAY_BUFFER, gl::GLsizeiptr(points.size() * sizeof(LineVertex)), points.data(),
                   gl::STATIC_DRAW);
    gl::BindBuffer(gl::ARRAY_BUFFER, 0);
    build_grid();
}

void Viewer::build_grid()
{
    std::vector<LineVertex> grid;
    if (m_bounds.valid()) {
        // Spacing: power of ten giving roughly 10-100 lines across the model.
        const Vec3   size   = m_bounds.size();
        const double extent = std::max({ size.x, size.y, 1e-9 }) * 1.6;
        double       step   = std::pow(10.0, std::floor(std::log10(extent / 10.0)));
        if (extent / step > 40)
            step *= 5;
        const Vec3   c    = m_bounds.center();
        const double z    = m_bounds.min.z;
        const double half = std::ceil(extent * 0.5 / step) * step;
        const double x0 = std::floor((c.x - half) / step) * step, x1 = c.x + half;
        const double y0 = std::floor((c.y - half) / step) * step, y1 = c.y + half;
        int          i  = 0;
        for (double x = x0; x <= x1 + 1e-9; x += step, ++i) {
            const float a = (i % 5 == 0) ? 0.22f : 0.10f;
            grid.push_back({ float(x), float(y0), float(z), 0.8f, 0.85f, 0.95f, a });
            grid.push_back({ float(x), float(y1), float(z), 0.8f, 0.85f, 0.95f, a });
        }
        i = 0;
        for (double y = y0; y <= y1 + 1e-9; y += step, ++i) {
            const float a = (i % 5 == 0) ? 0.22f : 0.10f;
            grid.push_back({ float(x0), float(y), float(z), 0.8f, 0.85f, 0.95f, a });
            grid.push_back({ float(x1), float(y), float(z), 0.8f, 0.85f, 0.95f, a });
        }
    }
    m_grid_vertices = grid.size();
    gl::BindBuffer(gl::ARRAY_BUFFER, m_vbo_grid);
    gl::BufferData(gl::ARRAY_BUFFER, gl::GLsizeiptr(grid.size() * sizeof(LineVertex)), grid.data(), gl::STATIC_DRAW);
    gl::BindBuffer(gl::ARRAY_BUFFER, 0);
}

void Viewer::draw(const Camera& cam, int x, int y, int w, int h, const ViewSettings& vs)
{
    if (w <= 0 || h <= 0)
        return;
    gl::Viewport(x, y, w, h);
    gl::Disable(gl::CULL_FACE);

    // Background gradient.
    gl::Disable(gl::DEPTH_TEST);
    gl::UseProgram(m_prog_bg);
    gl::BindVertexArray(m_vao_empty);
    gl::DrawArrays(gl::TRIANGLES, 0, 3);

    if (!has_mesh()) {
        gl::BindVertexArray(0);
        return;
    }

    const Mat4 view = cam.view();
    const Mat4 mvp  = cam.projection(double(w) / double(h)) * view;

    gl::Enable(gl::DEPTH_TEST);
    gl::DepthFunc(gl::LEQUAL);
    gl::Clear(gl::DEPTH_BUFFER_BIT);
    gl::Enable(gl::BLEND);
    gl::BlendFunc(gl::SRC_ALPHA, gl::ONE_MINUS_SRC_ALPHA);

    // Grid (no depth writes, below the model).
    if (vs.show_grid && m_grid_vertices) {
        gl::DepthMask(gl::FALSE_);
        gl::UseProgram(m_prog_lines);
        gl::UniformMatrix4fv(gl::GetUniformLocation(m_prog_lines, "u_mvp"), 1, gl::FALSE_, mvp.m);
        gl::Uniform1f(gl::GetUniformLocation(m_prog_lines, "u_alpha"), 1.0f);
        gl::Uniform1i(gl::GetUniformLocation(m_prog_lines, "u_round"), 0);
        gl::Uniform1f(gl::GetUniformLocation(m_prog_lines, "u_point_size"), 1.0f);
        gl::BindVertexArray(m_vao_grid);
        gl::LineWidth(1.0f);
        gl::DrawArrays(gl::LINES, 0, gl::GLsizei(m_grid_vertices));
        gl::DepthMask(gl::TRUE_);
    }

    // Surface, pushed back slightly so that edges on it stay visible. Back
    // faces (seen through holes or on flipped faces) are drawn separately and
    // pushed back further, so that they never bleed through at silhouettes.
    gl::Enable(gl::POLYGON_OFFSET_FILL);
    gl::UseProgram(m_prog_mesh);
    gl::UniformMatrix4fv(gl::GetUniformLocation(m_prog_mesh, "u_mvp"), 1, gl::FALSE_, mvp.m);
    gl::UniformMatrix4fv(gl::GetUniformLocation(m_prog_mesh, "u_view"), 1, gl::FALSE_, view.m);
    gl::Uniform1i(gl::GetUniformLocation(m_prog_mesh, "u_show_classes"), vs.show_new_faces ? 1 : 0);
    gl::Uniform1i(gl::GetUniformLocation(m_prog_mesh, "u_wire"), vs.wireframe ? 1 : 0);
    gl::Uniform1i(gl::GetUniformLocation(m_prog_mesh, "u_alert_back"), m_alert_back ? 1 : 0);
    gl::BindVertexArray(m_vao_mesh);
    gl::Enable(gl::CULL_FACE);
    gl::CullFace(gl::BACK);
    gl::PolygonOffset(1.0f, 1.0f);
    gl::DrawArrays(gl::TRIANGLES, 0, gl::GLsizei(m_tri_vertices));
    gl::CullFace(gl::FRONT);
    gl::PolygonOffset(2.0f, 8.0f);
    gl::DrawArrays(gl::TRIANGLES, 0, gl::GLsizei(m_tri_vertices));
    gl::Disable(gl::CULL_FACE);
    gl::Disable(gl::POLYGON_OFFSET_FILL);

    if (vs.show_issues && (m_line_vertices || m_point_vertices)) {
        gl::UseProgram(m_prog_lines);
        gl::UniformMatrix4fv(gl::GetUniformLocation(m_prog_lines, "u_mvp"), 1, gl::FALSE_, mvp.m);
        const gl::GLint alpha = gl::GetUniformLocation(m_prog_lines, "u_alpha");
        const gl::GLint round = gl::GetUniformLocation(m_prog_lines, "u_round");
        const gl::GLint psize = gl::GetUniformLocation(m_prog_lines, "u_point_size");
        gl::Enable(gl::PROGRAM_POINT_SIZE);
        // Pass 1: hidden parts, faint. Pass 2: visible parts.
        for (int pass = vs.xray_issues ? 0 : 1; pass < 2; ++pass) {
            gl::DepthFunc(pass == 0 ? gl::ALWAYS : gl::LEQUAL);
            gl::DepthMask(pass == 0 ? gl::FALSE_ : gl::TRUE_);
            gl::Uniform1f(alpha, pass == 0 ? 0.35f : 1.0f);
            if (m_line_vertices) {
                gl::Uniform1i(round, 0);
                gl::LineWidth(vs.issue_line_width);
                gl::BindVertexArray(m_vao_lines);
                gl::DrawArrays(gl::LINES, 0, gl::GLsizei(m_line_vertices));
            }
            if (m_point_vertices) {
                gl::Uniform1i(round, 1);
                gl::Uniform1f(psize, vs.issue_line_width * 3.5f);
                gl::BindVertexArray(m_vao_points);
                gl::DrawArrays(gl::POINTS, 0, gl::GLsizei(m_point_vertices));
            }
        }
        gl::LineWidth(1.0f);
        gl::DepthFunc(gl::LEQUAL);
        gl::DepthMask(gl::TRUE_);
    }
    gl::Disable(gl::BLEND);
    gl::BindVertexArray(0);
    gl::UseProgram(0);
}

bool Viewer::render_offscreen(const Camera& cam, int w, int h, const ViewSettings& vs, std::vector<uint8_t>& rgba)
{
    gl::GLuint fbo = 0, color = 0, depth = 0;
    gl::GenFramebuffers(1, &fbo);
    gl::BindFramebuffer(gl::FRAMEBUFFER, fbo);
    gl::GenRenderbuffers(1, &color);
    gl::BindRenderbuffer(gl::RENDERBUFFER, color);
    gl::RenderbufferStorage(gl::RENDERBUFFER, gl::RGBA8, w, h);
    gl::FramebufferRenderbuffer(gl::FRAMEBUFFER, gl::COLOR_ATTACHMENT0, gl::RENDERBUFFER, color);
    gl::GenRenderbuffers(1, &depth);
    gl::BindRenderbuffer(gl::RENDERBUFFER, depth);
    gl::RenderbufferStorage(gl::RENDERBUFFER, gl::DEPTH_COMPONENT24, w, h);
    gl::FramebufferRenderbuffer(gl::FRAMEBUFFER, gl::DEPTH_ATTACHMENT, gl::RENDERBUFFER, depth);
    const bool ok = gl::CheckFramebufferStatus(gl::FRAMEBUFFER) == gl::FRAMEBUFFER_COMPLETE;
    if (ok) {
        gl::ClearColor(0, 0, 0, 1);
        gl::Clear(gl::COLOR_BUFFER_BIT | gl::DEPTH_BUFFER_BIT);
        draw(cam, 0, 0, w, h, vs);
        rgba.resize(size_t(w) * size_t(h) * 4);
        gl::PixelStorei(gl::PACK_ALIGNMENT, 1);
        gl::ReadBuffer(gl::COLOR_ATTACHMENT0);
        gl::ReadPixels(0, 0, w, h, gl::RGBA, gl::UNSIGNED_BYTE, rgba.data());
        // Flip to top row first.
        const size_t row = size_t(w) * 4;
        for (int yy = 0; yy < h / 2; ++yy)
            std::swap_ranges(rgba.begin() + long(yy * row), rgba.begin() + long((yy + 1) * row),
                             rgba.begin() + long((h - 1 - yy) * row));
    }
    gl::BindFramebuffer(gl::FRAMEBUFFER, 0);
    gl::DeleteRenderbuffers(1, &color);
    gl::DeleteRenderbuffers(1, &depth);
    gl::DeleteFramebuffers(1, &fbo);
    return ok;
}

} // namespace mrgui

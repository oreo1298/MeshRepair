// Test suite: builds meshes with known defects, repairs them and verifies the
// result is closed, manifold and correctly oriented - also after a round trip
// through binary STL, which is how slicers usually receive the result.

#include <meshrepair/Diagnostics.hpp>
#include <meshrepair/IO.hpp>
#include <meshrepair/Repair.hpp>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <map>
#include <random>
#include <string>
#include <vector>

using namespace meshrepair;

static int g_failures = 0;
static int g_checks   = 0;
static std::string g_current;

#define CHECK(cond)                                                                                                   \
    do {                                                                                                              \
        ++g_checks;                                                                                                   \
        if (!(cond)) {                                                                                                \
            ++g_failures;                                                                                             \
            std::printf("  FAILED [%s] %s:%d: %s\n", g_current.c_str(), __FILE__, __LINE__, #cond);                  \
        }                                                                                                             \
    } while (0)

#define CHECK_NEAR(a, b, tol)                                                                                         \
    do {                                                                                                              \
        ++g_checks;                                                                                                   \
        const double _a = (a), _b = (b);                                                                              \
        if (!(std::abs(_a - _b) <= (tol))) {                                                                          \
            ++g_failures;                                                                                             \
            std::printf("  FAILED [%s] %s:%d: %s = %.9g, expected %.9g\n", g_current.c_str(), __FILE__, __LINE__, #a, \
                        _a, _b);                                                                                      \
        }                                                                                                             \
    } while (0)

// ---------------------------------------------------------------------------
// Mesh generators

static void append(Mesh& dst, const Mesh& src)
{
    const int off = int(dst.vertices.size());
    dst.vertices.insert(dst.vertices.end(), src.vertices.begin(), src.vertices.end());
    for (Triangle t : src.faces)
        dst.faces.push_back({ t[0] + off, t[1] + off, t[2] + off });
}

static Mesh cube(Vec3 o = Vec3(0, 0, 0), double s = 1.0)
{
    Mesh m;
    for (int i = 0; i < 8; ++i)
        m.vertices.push_back(o + Vec3((i & 1) ? s : 0, (i & 2) ? s : 0, (i & 4) ? s : 0));
    // Outward CCW quads.
    const int q[6][4] = { { 0, 2, 3, 1 }, { 4, 5, 7, 6 }, { 0, 1, 5, 4 }, { 2, 6, 7, 3 }, { 0, 4, 6, 2 }, { 1, 3, 7, 5 } };
    for (auto& f : q) {
        m.faces.push_back({ f[0], f[1], f[2] });
        m.faces.push_back({ f[0], f[2], f[3] });
    }
    return m;
}

// Cube made of n x n quads per side (shared vertices).
static Mesh grid_cube(int n, Vec3 o = Vec3(0, 0, 0), double s = 1.0)
{
    Mesh                      m;
    std::map<std::array<int, 3>, int> ids;
    auto vid = [&](int i, int j, int k) {
        auto key = std::array<int, 3> { i, j, k };
        auto it  = ids.find(key);
        if (it != ids.end())
            return it->second;
        m.vertices.push_back(o + Vec3(i, j, k) * (s / n));
        ids[key] = int(m.vertices.size() - 1);
        return ids[key];
    };
    for (int axis = 0; axis < 3; ++axis)
        for (int side = 0; side < 2; ++side)
            for (int a = 0; a < n; ++a)
                for (int b = 0; b < n; ++b) {
                    auto p = [&](int u, int v) {
                        int c[3];
                        c[axis]           = side * n;
                        c[(axis + 1) % 3] = u;
                        c[(axis + 2) % 3] = v;
                        return vid(c[0], c[1], c[2]);
                    };
                    int v0 = p(a, b), v1 = p(a + 1, b), v2 = p(a + 1, b + 1), v3 = p(a, b + 1);
                    if (side == 0) {
                        std::swap(v1, v3);
                    }
                    m.faces.push_back({ v0, v1, v2 });
                    m.faces.push_back({ v0, v2, v3 });
                }
    return m;
}

static Mesh icosphere(int subdiv, double r = 1.0, Vec3 c = Vec3(0, 0, 0))
{
    const double t = (1.0 + std::sqrt(5.0)) / 2.0;
    Mesh         m;
    m.vertices = { { -1, t, 0 }, { 1, t, 0 }, { -1, -t, 0 }, { 1, -t, 0 }, { 0, -1, t }, { 0, 1, t },
                   { 0, -1, -t }, { 0, 1, -t }, { t, 0, -1 }, { t, 0, 1 }, { -t, 0, -1 }, { -t, 0, 1 } };
    m.faces    = { { 0, 11, 5 }, { 0, 5, 1 }, { 0, 1, 7 }, { 0, 7, 10 }, { 0, 10, 11 }, { 1, 5, 9 }, { 5, 11, 4 },
                   { 11, 10, 2 }, { 10, 7, 6 }, { 7, 1, 8 }, { 3, 9, 4 }, { 3, 4, 2 }, { 3, 2, 6 }, { 3, 6, 8 },
                   { 3, 8, 9 }, { 4, 9, 5 }, { 2, 4, 11 }, { 6, 2, 10 }, { 8, 6, 7 }, { 9, 8, 1 } };
    for (Vec3& v : m.vertices)
        v = v.normalized();
    for (int s = 0; s < subdiv; ++s) {
        std::map<std::pair<int, int>, int> mid;
        auto midpoint = [&](int a, int b) {
            auto key = std::make_pair(std::min(a, b), std::max(a, b));
            auto it  = mid.find(key);
            if (it != mid.end())
                return it->second;
            m.vertices.push_back(((m.vertices[a] + m.vertices[b]) * 0.5).normalized());
            mid[key] = int(m.vertices.size() - 1);
            return mid[key];
        };
        std::vector<Triangle> nf;
        for (const Triangle& f : m.faces) {
            const int a = midpoint(f[0], f[1]), b = midpoint(f[1], f[2]), c2 = midpoint(f[2], f[0]);
            nf.push_back({ f[0], a, c2 });
            nf.push_back({ f[1], b, a });
            nf.push_back({ f[2], c2, b });
            nf.push_back({ a, b, c2 });
        }
        m.faces.swap(nf);
    }
    for (Vec3& v : m.vertices)
        v = c + v * r;
    return m;
}

// Open tube (no caps) along z.
static Mesh tube(int seg, int rings, double r, double h)
{
    Mesh m;
    const double pi = 3.14159265358979323846;
    for (int j = 0; j <= rings; ++j)
        for (int i = 0; i < seg; ++i)
            m.vertices.push_back({ r * std::cos(2 * pi * i / seg), r * std::sin(2 * pi * i / seg), h * j / rings });
    for (int j = 0; j < rings; ++j)
        for (int i = 0; i < seg; ++i) {
            const int a = j * seg + i, b = j * seg + (i + 1) % seg, c = a + seg, d = b + seg;
            m.faces.push_back({ a, b, d });
            m.faces.push_back({ a, d, c });
        }
    return m;
}

// ---------------------------------------------------------------------------
// Checks

static std::string tmp_path(const std::string& name)
{
    const char* dir = std::getenv("TMPDIR");
    return std::string(dir ? dir : "/tmp") + "/meshrepair_test_" + name;
}

static void print_diag(const MeshDiagnostics& d)
{
    std::printf("    open=%zu nm_edges=%zu nm_verts=%zu inconsistent=%zu inverted=%zu degenerate=%zu dup=%zu shells=%zu "
                "vol=%.6g\n",
                d.open_edges, d.non_manifold_edges, d.non_manifold_vertices, d.inconsistent_edges, d.inverted_shells,
                d.degenerate_faces, d.duplicate_faces, d.shells, d.volume);
}

static void expect_clean(const Mesh& m, bool via_stl = true)
{
    const MeshDiagnostics d = analyze(m);
    CHECK(d.open_edges == 0);
    CHECK(d.non_manifold_edges == 0);
    CHECK(d.non_manifold_vertices == 0);
    CHECK(d.inconsistent_edges == 0);
    CHECK(d.inverted_shells == 0);
    CHECK(d.degenerate_faces == 0);
    CHECK(d.duplicate_faces == 0);
    if (!d.clean())
        print_diag(d);
    if (via_stl) {
        // What the slicer sees after loading the STL (positions merged).
        std::string err;
        const std::string path = tmp_path("roundtrip.stl");
        CHECK(save_stl(path, m, false, err));
        Mesh back;
        CHECK(load_stl(path, back, err));
        const MeshDiagnostics d2 = analyze(back);
        CHECK(d2.slicer_clean());
        if (!d2.slicer_clean()) {
            std::printf("    after STL round trip:\n");
            print_diag(d2);
        }
        std::remove(path.c_str());
    }
}

static RepairResult run(Mesh& m, const RepairOptions& opt = {})
{
    RepairResult r = repair(m, opt);
    CHECK(r.success);
    return r;
}

static void test(const char* name, const std::function<void()>& fn)
{
    g_current       = name;
    const int before = g_failures;
    const auto t0    = std::chrono::steady_clock::now();
    fn();
    const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    std::printf("%s %-50s %8.1f ms\n", g_failures == before ? "[ OK ]" : "[FAIL]", name, ms);
}

// ---------------------------------------------------------------------------

int main()
{
    test("clean cube is left unchanged", [] {
        Mesh m = cube();
        const RepairResult r = run(m);
        CHECK(m.faces.size() == 12);
        CHECK(m.vertices.size() == 8);
        CHECK_NEAR(r.after.volume, 1.0, 1e-5);
        CHECK(r.stats.faces_added == 0);
        CHECK(r.stats.faces_flipped == 0);
        expect_clean(m);
    });

    test("diagnostics of a clean sphere", [] {
        const Mesh m = icosphere(3);
        const MeshDiagnostics d = analyze(m);
        CHECK(d.clean());
        CHECK(d.shells == 1);
        CHECK(d.holes == 0);
    });

    test("missing face is filled", [] {
        Mesh m = cube();
        m.faces.erase(m.faces.begin() + 4);
        const MeshDiagnostics d = analyze(m);
        CHECK(d.open_edges == 3);
        CHECK(d.holes == 1);
        const RepairResult r = run(m);
        CHECK(r.stats.holes_filled == 1);
        CHECK_NEAR(r.after.volume, 1.0, 1e-5);
        expect_clean(m);
    });

    test("whole side missing is filled", [] {
        Mesh m = grid_cube(4);
        // Remove the z = 1 side (faces whose vertices all have z == 1).
        std::vector<Triangle> keep;
        for (const Triangle& t : m.faces)
            if (!(m.vertices[t[0]].z == 1 && m.vertices[t[1]].z == 1 && m.vertices[t[2]].z == 1))
                keep.push_back(t);
        m.faces = keep;
        const RepairResult r = run(m);
        CHECK(r.stats.holes_filled == 1);
        CHECK_NEAR(r.after.volume, 1.0, 1e-5);
        expect_clean(m);
    });

    test("inside out cube is flipped", [] {
        Mesh m = cube();
        flip_all_faces(m);
        CHECK(analyze(m).inverted_shells == 1);
        const RepairResult r = run(m);
        CHECK_NEAR(r.after.volume, 1.0, 1e-5);
        CHECK(r.stats.faces_flipped == 12);
        expect_clean(m);
    });

    test("randomly flipped faces are reoriented", [] {
        Mesh         m = icosphere(3);
        std::mt19937 rng(42);
        for (Triangle& t : m.faces)
            if (rng() % 3 == 0)
                std::swap(t[1], t[2]);
        CHECK(analyze(m).inconsistent_edges > 0);
        const double vol = signed_volume(icosphere(3));
        const RepairResult r = run(m);
        CHECK_NEAR(r.after.volume, vol, 1e-5);
        expect_clean(m);
    });

    test("duplicate faces are removed", [] {
        Mesh m = cube();
        m.faces.push_back(m.faces[0]);
        m.faces.push_back(m.faces[3]);
        m.faces.push_back({ m.faces[5][1], m.faces[5][2], m.faces[5][0] }); // rotated copy
        CHECK(analyze(m).duplicate_faces == 3);
        const RepairResult r = run(m);
        CHECK(r.stats.duplicate_faces_removed == 3);
        CHECK(m.faces.size() == 12);
        CHECK_NEAR(r.after.volume, 1.0, 1e-5);
        expect_clean(m);
    });

    test("unwelded triangle soup is welded", [] {
        Mesh soup;
        Mesh c = icosphere(2);
        for (const Triangle& t : c.faces) {
            const int b = int(soup.vertices.size());
            for (int k = 0; k < 3; ++k)
                soup.vertices.push_back(c.vertices[t[k]]);
            soup.faces.push_back({ b, b + 1, b + 2 });
        }
        CHECK(analyze(soup).open_edges == c.faces.size() * 3);
        const RepairResult r = run(soup);
        CHECK(soup.vertices.size() == c.vertices.size());
        CHECK(r.stats.faces_added == 0);
        expect_clean(soup);
    });

    test("two cubes sharing an edge (non-manifold edge)", [] {
        Mesh m = cube();
        append(m, cube(Vec3(1, 1, 0)));
        merge_identical_vertices(m);
        const MeshDiagnostics d = analyze(m);
        CHECK(d.non_manifold_edges == 1);
        const RepairResult r = run(m);
        CHECK(r.stats.non_manifold_edges_fixed == 1);
        CHECK(r.after.shells == 2);
        CHECK_NEAR(r.after.volume, 2.0, 1e-3);
        CHECK(r.stats.faces_added == 0);
        expect_clean(m);
    });

    test("two cubes sharing a vertex (non-manifold vertex)", [] {
        Mesh m = cube();
        append(m, cube(Vec3(1, 1, 1)));
        merge_identical_vertices(m);
        CHECK(analyze(m).non_manifold_vertices == 1);
        const RepairResult r = run(m);
        CHECK(r.after.shells == 2);
        CHECK_NEAR(r.after.volume, 2.0, 1e-3);
        expect_clean(m);
    });

    test("two cubes sharing a face become one solid", [] {
        Mesh m = cube();
        append(m, cube(Vec3(1, 0, 0)));
        merge_identical_vertices(m);
        const MeshDiagnostics d = analyze(m);
        CHECK(d.non_manifold_edges > 0);
        const RepairResult r = run(m);
        CHECK(r.after.shells == 1);
        CHECK_NEAR(r.after.volume, 2.0, 1e-5);
        expect_clean(m);
    });

    test("stacked cubes, different tessellation of shared face", [] {
        Mesh m = cube();
        append(m, grid_cube(2, Vec3(0, 0, 1)));
        merge_identical_vertices(m);
        CHECK(!analyze(m).slicer_clean());
        const RepairResult r = run(m);
        CHECK_NEAR(r.after.volume, 2.0, 1e-3);
        expect_clean(m);
    });

    test("internal wall inside a closed box is removed", [] {
        Mesh m = grid_cube(2);
        // Partition at x = 0.5 using the existing grid vertices.
        auto find = [&](Vec3 p) {
            for (size_t i = 0; i < m.vertices.size(); ++i)
                if ((m.vertices[i] - p).norm() < 1e-12)
                    return int(i);
            m.vertices.push_back(p);
            return int(m.vertices.size() - 1);
        };
        for (int a = 0; a < 2; ++a)
            for (int b = 0; b < 2; ++b) {
                const int v0 = find({ 0.5, a * 0.5, b * 0.5 }), v1 = find({ 0.5, (a + 1) * 0.5, b * 0.5 });
                const int v2 = find({ 0.5, (a + 1) * 0.5, (b + 1) * 0.5 }), v3 = find({ 0.5, a * 0.5, (b + 1) * 0.5 });
                m.faces.push_back({ v0, v1, v2 });
                m.faces.push_back({ v0, v2, v3 });
            }
        CHECK(analyze(m).non_manifold_edges > 0);
        const RepairResult r = run(m);
        CHECK(r.after.shells == 1);
        CHECK_NEAR(r.after.volume, 1.0, 1e-5);
        expect_clean(m);
    });

    test("open tube gets caps", [] {
        Mesh m = tube(48, 6, 1.0, 2.0);
        CHECK(analyze(m).holes == 2);
        const RepairResult r = run(m);
        CHECK(r.stats.holes_filled == 2);
        const double expect = 0.5 * 48 * std::sin(2 * 3.14159265358979323846 / 48) * 2.0; // polygon area * h
        CHECK_NEAR(r.after.volume, expect, 1e-6);
        expect_clean(m);
    });

    test("sphere with several holes", [] {
        Mesh         m   = icosphere(4);
        const double vol = signed_volume(m);
        std::mt19937 rng(7);
        // Remove clusters of faces around random vertices.
        for (int h = 0; h < 6; ++h) {
            const int             v = int(rng() % m.vertices.size());
            std::vector<Triangle> keep;
            for (const Triangle& t : m.faces)
                if ((m.vertices[t[0]] - m.vertices[v]).norm() > 0.2)
                    keep.push_back(t);
            m.faces = keep;
        }
        remove_unreferenced_vertices(m);
        CHECK(analyze(m).holes >= 3);
        const RepairResult r = run(m);
        CHECK(r.stats.holes_filled >= 3);
        CHECK_NEAR(r.after.volume, vol, 0.03 * vol);
        expect_clean(m);
    });

    test("huge hole (loop splitting path)", [] {
        Mesh m = tube(1500, 2, 10.0, 5.0);
        const RepairResult r = run(m);
        CHECK(r.stats.holes_filled == 2);
        CHECK_NEAR(r.after.volume, 3.14159265358979323846 * 100 * 5, 0.01 * 3.14159265358979323846 * 100 * 5);
        expect_clean(m);
    });

    test("T-junction crack is closed", [] {
        // Two halves of a box; the left half has an extra vertex in the middle
        // of the seam edges that the right half does not have.
        Mesh left  = grid_cube(2, Vec3(0, 0, 0), 1.0);
        Mesh m;
        // Build a box of size 2x1x1 from two separately tessellated halves,
        // without the shared wall: left half grid 2x2, right half 1x1.
        auto keep_side = [](const Mesh& src, int axis, double val, bool drop) {
            Mesh out = src;
            out.faces.clear();
            for (const Triangle& t : src.faces) {
                const bool on = src.vertices[t[0]][axis] == val && src.vertices[t[1]][axis] == val &&
                                src.vertices[t[2]][axis] == val;
                if (on != drop)
                    out.faces.push_back(t);
            }
            remove_unreferenced_vertices(out);
            return out;
        };
        append(m, keep_side(left, 0, 1.0, true));
        append(m, keep_side(cube(Vec3(1, 0, 0)), 0, 1.0, true));
        merge_identical_vertices(m);
        const MeshDiagnostics d = analyze(m);
        CHECK(d.open_edges > 0);
        const RepairResult r = run(m);
        CHECK(r.stats.t_junctions_fixed > 0);
        CHECK(r.stats.holes_filled == 0);
        CHECK_NEAR(r.after.volume, 2.0, 1e-5);
        expect_clean(m);
    });

    test("crack with slightly offset vertices is stitched", [] {
        Mesh m = icosphere(3);
        // Duplicate the vertices of one hemisphere boundary and offset them a bit.
        const size_t n = m.vertices.size();
        std::vector<int> copy(n, -1);
        for (Triangle& t : m.faces) {
            const Vec3 c = (m.vertices[t[0]] + m.vertices[t[1]] + m.vertices[t[2]]) / 3.0;
            if (c.z < 0)
                continue;
            for (int& v : t) {
                if (copy[v] < 0) {
                    copy[v] = int(m.vertices.size());
                    m.vertices.push_back(m.vertices[v] + Vec3(2e-5, -1e-5, 1e-5));
                }
                v = copy[v];
            }
        }
        CHECK(analyze(m).open_edges > 0);
        const RepairResult r = run(m);
        CHECK(r.stats.boundary_vertices_stitched > 0);
        CHECK(r.stats.holes_filled == 0);
        expect_clean(m);
    });

    test("hollow cube keeps its cavity", [] {
        Mesh m     = cube(Vec3(0, 0, 0), 3.0);
        Mesh inner = cube(Vec3(1, 1, 1), 1.0);
        flip_all_faces(inner);
        append(m, inner);
        const RepairResult r = run(m);
        CHECK(r.after.shells == 2);
        CHECK_NEAR(r.after.volume, 26.0, 1e-5);
        expect_clean(m);
    });

    test("cavity with an open hole keeps its cavity", [] {
        Mesh m     = cube(Vec3(0, 0, 0), 3.0);
        Mesh inner = grid_cube(2, Vec3(1, 1, 1), 1.0);
        flip_all_faces(inner);
        inner.faces.pop_back();
        append(m, inner);
        const RepairResult r = run(m);
        CHECK_NEAR(r.after.volume, 26.0, 1e-5);
        expect_clean(m);
    });

    test("overlapping solid inside another stays a solid", [] {
        Mesh m = cube(Vec3(0, 0, 0), 3.0);
        append(m, cube(Vec3(1, 1, 1), 1.0));
        const RepairResult r = run(m);
        CHECK_NEAR(r.after.volume, 28.0, 1e-5);
        expect_clean(m);
    });

    test("cap triangle (collinear vertices) is flipped away", [] {
        Mesh m = cube();
        // Split the edge (0,2) of the face {0,2,3}: add vertex in the middle of
        // the diagonal and a zero area triangle.
        m.vertices.push_back((m.vertices[0] + m.vertices[3]) * 0.5); // on the diagonal 0-3 of the bottom quad
        const int c = int(m.vertices.size() - 1);
        // bottom faces {0,2,3} and {0,3,1}: split both at c.
        m.faces[0] = { 0, 2, c };
        m.faces[1] = { c, 2, 3 };
        m.faces.push_back({ 0, c, 1 });
        m.faces.push_back({ c, 3, 1 });
        // Fold a cap: replace {0,c,1} + {c,3,1} by {0,3,1} + zero area {0,c,3}
        m.faces.pop_back();
        m.faces.pop_back();
        m.faces.push_back({ 0, 3, 1 });
        m.faces.push_back({ 0, c, 3 });
        // Now {0,2,c},{c,2,3} share c; {0,c,3} is a cap on edge 0-3 of {0,3,1}.
        const MeshDiagnostics d = analyze(m);
        CHECK(d.degenerate_faces == 1);
        CHECK(d.slicer_clean());
        const RepairResult r = run(m);
        CHECK_NEAR(r.after.volume, 1.0, 1e-5);
        expect_clean(m);
    });

    test("fin attached to a closed surface", [] {
        Mesh m = cube();
        // A single triangle sticking out of edge (0,1) of the cube.
        m.vertices.push_back({ 0.5, -1.0, -1.0 });
        m.faces.push_back({ 0, 1, 8 });
        CHECK(analyze(m).non_manifold_edges == 1);
        const RepairResult r = run(m);
        CHECK_NEAR(r.after.volume, 1.0, 1e-5);
        expect_clean(m);
    });

    test("Moebius strip does not break the repair", [] {
        Mesh         m;
        const int    seg = 40;
        const double pi  = 3.14159265358979323846;
        for (int i = 0; i < seg; ++i) {
            const double a = 2 * pi * i / seg;
            for (int s = -1; s <= 1; s += 2) {
                const double w = 0.3 * s;
                m.vertices.push_back({ (1 + w * std::cos(a / 2)) * std::cos(a), (1 + w * std::cos(a / 2)) * std::sin(a),
                                       w * std::sin(a / 2) });
            }
        }
        for (int i = 0; i < seg; ++i) {
            const int a = 2 * i, b = 2 * i + 1;
            int       c = 2 * ((i + 1) % seg), d = c + 1;
            if (i == seg - 1)
                std::swap(c, d); // the twist
            m.faces.push_back({ a, c, d });
            m.faces.push_back({ a, d, b });
        }
        const RepairResult r = run(m);
        CHECK(r.after.non_manifold_edges == 0);
        CHECK(r.after.inconsistent_edges == 0);
        CHECK(r.after.non_manifold_vertices == 0);
    });

    test("random triangle soup does not break the repair", [] {
        std::mt19937                           rng(1234);
        std::uniform_real_distribution<double> u(0, 1);
        for (int round = 0; round < 20; ++round) {
            Mesh m;
            const int nv = 30 + round * 5;
            for (int i = 0; i < nv; ++i)
                m.vertices.push_back({ std::floor(u(rng) * 4), std::floor(u(rng) * 4), std::floor(u(rng) * 4) });
            for (int i = 0; i < nv * 2; ++i)
                m.faces.push_back({ int(rng() % nv), int(rng() % nv), int(rng() % nv) });
            const RepairResult r = run(m);
            CHECK(r.after.non_manifold_edges == 0);
            CHECK(r.after.inconsistent_edges == 0);
            CHECK(r.after.non_manifold_vertices == 0);
            CHECK(r.after.open_edges == 0);
        }
    });

    test("stray far away vertices do not blow up tolerances", [] {
        // A real model with a crack, plus garbage triangles with absurd
        // coordinates (as in corrupted files): must stay fast and still fix
        // the real model.
        Mesh m = grid_cube(8, Vec3(0, 0, 0), 20.0);
        m.faces.erase(m.faces.begin() + 10);
        std::mt19937                           rng(17);
        std::uniform_real_distribution<double> u(-1, 1);
        for (int i = 0; i < 200; ++i) {
            const int b = int(m.vertices.size());
            for (int k = 0; k < 3; ++k)
                m.vertices.push_back(Vec3(u(rng), u(rng), u(rng)) * 1e30);
            m.faces.push_back({ b, b + 1, b + 2 });
        }
        const auto         t0  = std::chrono::steady_clock::now();
        const RepairResult r   = run(m);
        const double       sec = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        CHECK(sec < 10.0);
        CHECK(r.stats.holes_filled >= 1);
        // Tolerances follow the real model (20 mm cube), not the garbage.
        CHECK(r.weld_tolerance_used < 0.01);
        CHECK(r.stitch_tolerance_used < 1.0);
        // The garbage sheets are gone, the cube is closed again.
        CHECK(r.after.open_edges == 0);
        CHECK_NEAR(r.after.volume, 8000.0, 1.0);
    });

    test("far away flat closed sheet counts as zero volume despite rounding", [] {
        // A closed but flat shell (both sides of a tilted square, split along
        // different diagonals) far away: its triple products do not cancel
        // exactly, the computed volume is pure rounding noise.
        Mesh      m = grid_cube(8, Vec3(0, 0, 0), 20.0);
        const int b = int(m.vertices.size());
        for (const auto& xy : { std::make_pair(0.3, 0.1), std::make_pair(0.9, 0.2), std::make_pair(0.8, 0.7),
                                std::make_pair(0.2, 0.9) })
            m.vertices.push_back(Vec3(xy.first, xy.second, 0.75 * xy.first + 0.5 * xy.second) * 3.7e29);
        for (const Triangle& t : { Triangle { 0, 1, 2 }, Triangle { 0, 2, 3 }, Triangle { 0, 3, 1 }, Triangle { 3, 2, 1 } })
            m.faces.push_back({ b + t[0], b + t[1], b + t[2] });
        const RepairResult r = run(m);
        CHECK(r.stats.shells_removed == 1);
        CHECK(r.after.shells == 1);
        CHECK_NEAR(r.after.volume, 8000.0, 1.0);
    });

    test("invalid faces and NaN coordinates are dropped", [] {
        Mesh m = cube();
        m.vertices.push_back({ std::nan(""), 0, 0 });
        m.faces.push_back({ 0, 1, 8 });
        m.faces.push_back({ 0, 1, 42 });
        const RepairResult r = run(m);
        CHECK(r.stats.invalid_faces_removed == 2);
        expect_clean(m);
    });

    test("empty mesh", [] {
        Mesh               m;
        const RepairResult r = run(m);
        CHECK(m.faces.empty());
        (void)r;
    });

    test("cancel leaves the mesh untouched", [] {
        Mesh       m    = icosphere(3);
        m.faces.resize(m.faces.size() - 10);
        const Mesh orig = m;
        RepairResult r  = repair(m, {}, {}, [] { return true; });
        CHECK(r.canceled);
        CHECK(!r.success);
        CHECK(m.faces.size() == orig.faces.size());
    });

    test("STL ascii and binary round trip", [] {
        const Mesh  m = icosphere(2);
        std::string err;
        for (bool ascii : { false, true }) {
            const std::string path = tmp_path(ascii ? "a.stl" : "b.stl");
            CHECK(save_stl(path, m, ascii, err));
            Mesh back;
            CHECK(load_stl(path, back, err));
            CHECK(back.faces.size() == m.faces.size());
            CHECK(back.vertices.size() == m.vertices.size());
            CHECK(analyze(back).clean());
            std::remove(path.c_str());
        }
    });

    test("OBJ round trip", [] {
        const Mesh  m = icosphere(2);
        std::string err;
        const std::string path = tmp_path("c.obj");
        CHECK(save_mesh(path, m, err));
        Mesh back;
        CHECK(load_mesh(path, back, err));
        CHECK(back.faces.size() == m.faces.size());
        CHECK(analyze(back).clean());
        std::remove(path.c_str());
    });

    test("3MF round trip (multiple objects, names, split vertices kept)", [] {
        std::vector<NamedMesh> objs;
        objs.push_back({ "Cube & <friends>", cube() });
        Mesh two = cube();
        append(two, cube(Vec3(1, 1, 0)));
        merge_identical_vertices(two);
        repair(two);
        objs.push_back({ "Repaired pair", two });
        std::string       err;
        const std::string path = tmp_path("rt.3mf");
        CHECK(save_3mf(path, objs, err));
        std::vector<NamedMesh> back;
        CHECK(load_3mf(path, back, err));
        CHECK(back.size() == 2);
        if (back.size() == 2) {
            CHECK(back[0].name == "Cube & <friends>");
            CHECK(back[1].name == "Repaired pair");
            CHECK(back[1].mesh.vertices.size() == two.vertices.size());
            CHECK(back[1].mesh.faces == two.faces);
            CHECK(analyze(back[1].mesh).slicer_clean());
            CHECK_NEAR(signed_volume(back[0].mesh), 1.0, 1e-6);
        }
        std::remove(path.c_str());
    });

    test("3MF loader errors are reported", [] {
        std::vector<NamedMesh> objs;
        std::string            err;
        const std::string      path = tmp_path("bad.3mf");
        FILE*                  f    = std::fopen(path.c_str(), "wb");
        std::fputs("this is not a zip file at all, just some text that is long enough", f);
        std::fclose(f);
        CHECK(!load_3mf(path, objs, err));
        CHECK(!err.empty());
        std::remove(path.c_str());
    });

    test("large mesh performance (330k faces, holes, flips)", [] {
        Mesh         m = icosphere(7);
        std::mt19937 rng(99);
        for (Triangle& t : m.faces)
            if (rng() % 50 == 0)
                std::swap(t[1], t[2]);
        std::vector<Triangle> keep;
        for (size_t i = 0; i < m.faces.size(); ++i)
            if (rng() % 500 != 0)
                keep.push_back(m.faces[i]);
        m.faces = keep;
        const auto         t0 = std::chrono::steady_clock::now();
        const RepairResult r  = run(m);
        const double sec = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        std::printf("    %zu faces repaired in %.2f s (%zu holes filled)\n", m.faces.size(), sec, r.stats.holes_filled);
        expect_clean(m, false);
    });

    std::printf("\n%d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}

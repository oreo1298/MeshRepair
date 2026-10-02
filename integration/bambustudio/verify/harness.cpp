// Checks the MeshRepair backend with Bambu Studio's own mesh diagnostics
// (libslic3r/MeshDiagnostics.cpp, which drives the "non-manifold edges" and
// "reversed faces" warnings): every mesh is repaired through the patched
// Slic3r::fix_mesh_by_win10_sdk() and checked before and after, also after
// merging identical positions the way Bambu Studio loads an STL.
#include "libslic3r/MeshDiagnostics.hpp"
#include "libslic3r/Win10ModelRepair.hpp"
#include <meshrepair/IO.hpp>
#include <boost/log/core.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/trivial.hpp>
#include <cstdio>
using namespace Slic3r;

static indexed_triangle_set to_its(const meshrepair::Mesh& m)
{
    indexed_triangle_set its;
    for (const auto& v : m.vertices) its.vertices.emplace_back(float(v.x), float(v.y), float(v.z));
    for (const auto& f : m.faces) its.indices.emplace_back(f[0], f[1], f[2]);
    return its;
}

// What Bambu does when it loads an STL: identical float positions are one vertex.
static indexed_triangle_set weld_like_stl(const indexed_triangle_set& its)
{
    meshrepair::Mesh m;
    for (const auto& v : its.vertices) m.vertices.emplace_back(v.x(), v.y(), v.z());
    for (const auto& f : its.indices) m.faces.push_back({ f(0), f(1), f(2) });
    meshrepair::merge_identical_vertices(m);
    return to_its(m);
}

static bool clean(const MeshDiagnosticStats& s)
{
    return s.open_edges == 0 && s.non_manifold_edges == 0 && s.non_manifold_vertices == 0 && !s.has_reversed_faces;
}

int main(int argc, char** argv)
{
    boost::log::core::get()->set_filter(boost::log::trivial::severity >= boost::log::trivial::warning);
    int bad = 0, total = 0, fixed = 0, failed_repairs = 0;
    for (int i = 1; i < argc; ++i) {
        meshrepair::Mesh m;
        std::string      err;
        if (!meshrepair::load_mesh(argv[i], m, err) || m.faces.empty())
            continue;
        ++total;
        const indexed_triangle_set in     = to_its(m);
        const MeshDiagnosticStats  before = its_mesh_diagnostics(in);
        indexed_triangle_set       out;
        std::string                msg;
        unsigned                   last_progress = 0;
        const bool ok = fix_mesh_by_win10_sdk(in, out, [&](const char*, unsigned p) { last_progress = p; }, [] { return false; }, &msg);
        if (!ok) {
            ++failed_repairs;
            std::printf("%-45s repair refused: %s\n", argv[i], msg.c_str());
            continue;
        }
        const MeshDiagnosticStats after  = its_mesh_diagnostics(out);
        const MeshDiagnosticStats welded = its_mesh_diagnostics(weld_like_stl(out));
        const bool good = clean(after) && clean(welded) && last_progress == 100;
        if (!clean(before) && good)
            ++fixed;
        if (!good)
            ++bad;
        std::printf("%-45s before: open %zu nm_e %zu nm_v %zu rev %d | after: open %zu nm_e %zu nm_v %zu rev %d | as STL: %s %s\n",
                    argv[i], before.open_edges, before.non_manifold_edges, before.non_manifold_vertices, before.has_reversed_faces,
                    after.open_edges, after.non_manifold_edges, after.non_manifold_vertices, after.has_reversed_faces,
                    clean(welded) ? "clean" : "NOT CLEAN", good ? "" : "  <== PROBLEM");
    }
    std::printf("\n%d meshes, %d broken ones fixed, %d not clean afterwards, %d refused (no volume)\n", total, fixed, bad, failed_repairs);
    return bad ? 1 : 0;
}

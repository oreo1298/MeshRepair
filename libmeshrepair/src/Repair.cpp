// The repair pipeline.

#include "meshrepair/Repair.hpp"

#include "Internal.hpp"
#include "meshrepair/IO.hpp"

#include <cstdio>
#include <sstream>
#include <stdexcept>
#include <unordered_set>

namespace meshrepair {

using namespace detail;

namespace {

// Edge flips for "cap" triangles: zero area faces whose three vertices are
// (nearly) collinear. The longest edge is flipped with the neighbouring face,
// which removes the degenerate face without changing the surface.
void fix_cap_triangles(WorkMesh& m, Context& ctx, double h_eps)
{
    std::unordered_map<uint64_t, int> he; // directed edge -> face
    he.reserve(m.F.size() * 3);
    for (size_t f = 0; f < m.F.size(); ++f)
        for (int c = 0; c < 3; ++c)
            he[directed_edge_key(m.F[f][c], m.F[f][(c + 1) % 3])] = int(f);
    auto set_face = [&](int f, const Triangle& t) {
        for (int c = 0; c < 3; ++c) {
            auto it = he.find(directed_edge_key(m.F[f][c], m.F[f][(c + 1) % 3]));
            if (it != he.end() && it->second == f)
                he.erase(it);
        }
        m.F[f] = t;
        for (int c = 0; c < 3; ++c)
            he[directed_edge_key(t[c], t[(c + 1) % 3])] = f;
    };

    for (int pass = 0; pass < 8; ++pass) {
        ctx.check_cancel();
        bool changed = false;
        for (size_t f = 0; f < m.F.size(); ++f) {
            const Triangle t = m.F[f];
            double         lmax = -1.0;
            int            c    = 0;
            for (int k = 0; k < 3; ++k) {
                const double l = (m.P[t[(k + 1) % 3]] - m.P[t[k]]).norm();
                if (l > lmax) {
                    lmax = l;
                    c    = k;
                }
            }
            if (!(lmax > 0.0) || m.normal(f).norm() / lmax > h_eps)
                continue;
            const int a = t[c], b = t[(c + 1) % 3], v = t[(c + 2) % 3];
            auto      it = he.find(directed_edge_key(b, a));
            if (it == he.end())
                continue;
            const int       g  = it->second;
            const Triangle& tg = m.F[g];
            const int       d  = tg[(corner_of(tg, b) + 2) % 3];
            if (d == v || he.count(directed_edge_key(v, d)) || he.count(directed_edge_key(d, v)))
                continue;
            const Triangle n1 { v, a, d };
            const Triangle n2 { d, b, v };
            // The new faces must keep the orientation of the neighbour.
            const Vec3 ng = m.normal(size_t(g));
            const Vec3 a1 = triangle_normal(m.P[v], m.P[a], m.P[d]);
            const Vec3 a2 = triangle_normal(m.P[d], m.P[b], m.P[v]);
            if (a1.dot(ng) <= 0.0 || a2.dot(ng) <= 0.0)
                continue;
            const double l1 = std::max({ (m.P[a] - m.P[v]).norm(), (m.P[d] - m.P[a]).norm(), (m.P[v] - m.P[d]).norm() });
            const double l2 = std::max({ (m.P[b] - m.P[d]).norm(), (m.P[v] - m.P[b]).norm(), (m.P[d] - m.P[v]).norm() });
            if (a1.norm() / l1 <= h_eps || a2.norm() / l2 <= h_eps)
                continue;
            set_face(int(f), n1);
            set_face(g, n2);
            m.flipped[f] = m.flipped[g];
            ++ctx.stats->degenerate_faces_fixed;
            changed = true;
        }
        if (!changed)
            break;
    }
}

struct ShellInfo
{
    std::vector<std::vector<int>> faces;
    std::vector<char>             closed;
    std::vector<double>           volume;
    std::vector<double>           area;
};

ShellInfo compute_shells(const WorkMesh& m)
{
    ShellInfo        s;
    int              n    = 0;
    std::vector<int> comp = face_components(m.F, n, false);
    s.faces.resize(n);
    for (size_t f = 0; f < m.F.size(); ++f)
        s.faces[comp[f]].push_back(int(f));
    s.closed.assign(n, 1);
    const std::vector<EdgeRec> recs = build_edge_records(m.F);
    for_each_edge_group(recs, [&](size_t b, size_t e) {
        if (e - b != 2)
            for (size_t i = b; i < e; ++i)
                s.closed[comp[recs[i].face]] = 0;
    });
    s.volume.resize(n);
    s.area.assign(n, 0.0);
    for (int i = 0; i < n; ++i) {
        s.volume[i] = shell_signed_volume(m.P, m.F, s.faces[i]);
        for (int f : s.faces[i])
            s.area[i] += m.area(size_t(f));
    }
    return s;
}

// Orients every shell. Outermost shells point outwards. Shells nested in other
// shells keep the orientation that most of their input faces had (a cavity
// stays a cavity, an overlapping solid stays a solid); shells made only of new
// faces follow the nesting parity.
void orient_shells(WorkMesh& m, Context& ctx)
{
    const ShellInfo        s     = compute_shells(m);
    const std::vector<int> depth = nesting_depths(m.P, m.F, s.faces, s.closed);
    for (size_t i = 0; i < s.faces.size(); ++i) {
        ctx.check_cancel();
        const double vol = s.volume[i];
        if (vol == 0.0)
            continue;
        int desired = 1;
        if (depth[i] > 0) {
            double same = 0.0, reversed = 0.0;
            for (int f : s.faces[i]) {
                if (m.origin[f] == FaceFilled)
                    continue;
                (m.flipped[f] ? reversed : same) += m.area(size_t(f));
            }
            if (same + reversed > 0.0) {
                // Sign of the volume the input orientation had.
                const int current = vol > 0.0 ? 1 : -1;
                desired           = same >= reversed ? current : -current;
            } else
                desired = (depth[i] % 2 == 0) ? 1 : -1;
        }
        if ((vol > 0.0 ? 1 : -1) != desired) {
            for (int f : s.faces[i])
                m.flip_face(size_t(f));
            ++ctx.stats->shells_reoriented;
        }
    }
}

bool is_zero_volume(const ShellInfo& s, size_t i, double thickness_eps)
{
    // The "thickness" 2V/A of the closed shell is negligible.
    return s.closed[i] && 2.0 * std::abs(s.volume[i]) <= thickness_eps * s.area[i];
}

// Throws when nothing printable is left: every shell is a closed surface
// enclosing no volume (single sided sheets made closed by hole filling).
void check_has_volume(const WorkMesh& m, double thickness_eps)
{
    const ShellInfo s = compute_shells(m);
    for (size_t i = 0; i < s.faces.size(); ++i)
        if (!is_zero_volume(s, i, thickness_eps))
            return;
    throw std::runtime_error("The model has no volume: it only consists of zero-thickness surfaces "
                             "(or of faces that cancel each other out), so no printable solid could be built. "
                             "If the surfaces are separated by gaps, try a larger crack tolerance.");
}

// Final check from the outside, like Bambu Studio does: a shell that is only
// ever seen from behind is inside out (e.g. a "cavity" that is not enclosed
// after all), so it is flipped. Shells seen from both sides fold over
// themselves; flipping them would not help, they are left alone.
void flip_shells_seen_from_behind(WorkMesh& m, Context& ctx)
{
    for (int pass = 0; pass < 3; ++pass) {
        std::vector<std::pair<int, bool>> hits;
        if (visible_back_faces(m.P, m.F, &hits) == 0)
            return;
        int              n    = 0;
        std::vector<int> comp = face_components(m.F, n, false);
        std::vector<int> back(n, 0), front(n, 0);
        for (const auto& h : hits)
            (h.second ? back : front)[comp[h.first]]++;
        bool changed = false;
        for (int c = 0; c < n; ++c)
            if (back[c] > 0 && front[c] == 0) {
                for (size_t f = 0; f < m.F.size(); ++f)
                    if (comp[f] == c)
                        m.flip_face(f);
                ++ctx.stats->shells_reoriented;
                changed = true;
            }
        if (!changed)
            return;
    }
}

void remove_shells(WorkMesh& m, Context& ctx, double thickness_eps)
{
    const RepairOptions& opt = *ctx.options;
    if (!opt.remove_zero_volume_shells && !(opt.min_shell_volume_ratio > 0.0))
        return;
    const ShellInfo s = compute_shells(m);
    if (s.faces.size() < 1)
        return;
    double vmax = 0.0;
    for (double v : s.volume)
        vmax = std::max(vmax, std::abs(v));
    std::vector<char> remove(m.F.size(), 0);
    size_t            removed_shells = 0;
    for (size_t i = 0; i < s.faces.size(); ++i) {
        const double v    = std::abs(s.volume[i]);
        bool         drop = false;
        if (opt.remove_zero_volume_shells && is_zero_volume(s, i, thickness_eps))
            drop = true;
        if (opt.min_shell_volume_ratio > 0.0 && v < opt.min_shell_volume_ratio * vmax)
            drop = true;
        if (drop) {
            ++removed_shells;
            for (int f : s.faces[i])
                remove[f] = 1;
        }
    }
    // Never remove everything: a model made only of sheets stays as it is.
    if (removed_shells > 0 && removed_shells < s.faces.size()) {
        m.remove_faces(remove);
        ctx.stats->shells_removed += removed_shells;
    }
}

// Groups of vertices that share a position once rounded to single precision.
std::vector<std::vector<int>> float_coincident_groups(const std::vector<Vec3>& P)
{
    struct Key
    {
        float x, y, z;
        int   v;
    };
    std::vector<Key> keys(P.size());
    for (size_t v = 0; v < P.size(); ++v)
        keys[v] = { float(P[v].x) + 0.0f, float(P[v].y) + 0.0f, float(P[v].z) + 0.0f, int(v) };
    auto same = [](const Key& a, const Key& b) { return a.x == b.x && a.y == b.y && a.z == b.z; };
    std::sort(keys.begin(), keys.end(), [](const Key& a, const Key& b) {
        if (a.x != b.x)
            return a.x < b.x;
        if (a.y != b.y)
            return a.y < b.y;
        if (a.z != b.z)
            return a.z < b.z;
        return a.v < b.v;
    });
    std::vector<std::vector<int>> groups;
    for (size_t i = 0; i < keys.size();) {
        size_t j = i + 1;
        while (j < keys.size() && same(keys[i], keys[j]))
            ++j;
        if (j - i > 1) {
            groups.emplace_back();
            for (size_t k = i; k < j; ++k)
                groups.back().push_back(keys[k].v);
        }
        i = j;
    }
    return groups;
}

// Splitting non-manifold vertices leaves several vertices at one position.
// File formats without connectivity (STL) merge them again on load, so they
// are moved apart by a few micrometres. Every copy moves into the region its
// shell encloses (into the material for a solid, into the void for a cavity),
// so touching shells separate and nested shells stay nested.
void separate_coincident_vertices(WorkMesh& m, Context& ctx)
{
    m.compact_vertices();
    static const Vec3 axes[6] = { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 }, { -1, 0, 0 }, { 0, -1, 0 }, { 0, 0, -1 } };
    std::vector<char> counted(m.P.size(), 0);
    // Orientation sign of the shell each vertex belongs to.
    std::vector<double> shell_sign(m.P.size(), 1.0);
    {
        const ShellInfo s = compute_shells(m);
        for (size_t i = 0; i < s.faces.size(); ++i)
            if (s.volume[i] < 0.0)
                for (int f : s.faces[i])
                    for (int v : m.F[f])
                        shell_sign[v] = -1.0;
    }
    for (int round = 0; round < 4; ++round) {
        const std::vector<std::vector<int>> groups = float_coincident_groups(m.P);
        if (groups.empty())
            return;
        std::vector<char> affected(m.P.size(), 0);
        for (const auto& g : groups)
            for (int v : g)
                affected[v] = 1;
        // Area weighted normal and direction towards the fan for each vertex.
        std::vector<Vec3>   nsum(m.P.size()), csum(m.P.size());
        std::vector<double> min_edge(m.P.size(), std::numeric_limits<double>::max());
        for (size_t f = 0; f < m.F.size(); ++f) {
            const Triangle& t = m.F[f];
            const Vec3      n = m.normal(f);
            const Vec3      c = (m.P[t[0]] + m.P[t[1]] + m.P[t[2]]) / 3.0;
            for (int k = 0; k < 3; ++k) {
                const int v = t[k];
                if (!affected[v])
                    continue;
                nsum[v] += n;
                csum[v] += (c - m.P[v]).normalized() * n.norm();
                // Zero length edges (to another copy at the same position)
                // do not limit the step, they are what is being fixed.
                for (int o : { t[(k + 1) % 3], t[(k + 2) % 3] }) {
                    const double l = (m.P[o] - m.P[v]).norm();
                    if (l > 0.0)
                        min_edge[v] = std::min(min_edge[v], l);
                }
            }
        }
        for (const auto& g : groups)
            for (size_t k = 0; k < g.size(); ++k) {
                const int v = g[k];
                // Into the region enclosed by the shell (against the normal
                // of a solid, along the normal of a cavity).
                Vec3 d = (nsum[v] * -shell_sign[v]).normalized();
                if (d.squared_norm() == 0.0)
                    d = csum[v].normalized();
                if (round > 0 || d.squared_norm() == 0.0)
                    d = (d + axes[(k + size_t(round)) % 6] * 0.5).normalized();
                const Vec3&  p   = m.P[v];
                const double mag = std::max({ std::abs(p.x), std::abs(p.y), std::abs(p.z), ctx.diag });
                // Well above float resolution, far below printer resolution.
                double step = std::max(1e-5 * ctx.diag, 64.0 * mag * double(std::numeric_limits<float>::epsilon()));
                step        = std::min(step * (round + 1), 0.25 * min_edge[v]);
                m.P[v] += d * step;
                if (!counted[v]) {
                    counted[v] = 1;
                    ++ctx.stats->vertices_separated;
                }
            }
    }
}

// The mesh as a slicer sees it after loading an STL: single precision
// coordinates, identical positions merged.
Mesh float_welded(const Mesh& mesh)
{
    Mesh out = mesh;
    for (Vec3& p : out.vertices)
        p = Vec3(double(float(p.x)), double(float(p.y)), double(float(p.z)));
    merge_identical_vertices(out);
    return out;
}

} // namespace

RepairResult repair(Mesh& mesh, const RepairOptions& options, const ProgressFn& progress, const CancelFn& cancel)
{
    RepairResult res;
    auto         report = [&](const char* stage, int percent) {
        if (progress)
            progress(stage, percent);
    };

    try {
        report("Analyzing mesh", 0);
        res.before = analyze(mesh);

        // Import, dropping faces with invalid indices or coordinates.
        WorkMesh m;
        m.P = mesh.vertices;
        m.vorig.resize(m.P.size());
        const int nv = int(mesh.vertices.size());
        for (const Triangle& t : mesh.faces) {
            bool ok = true;
            for (int v : t)
                ok &= v >= 0 && v < nv && mesh.vertices[v].is_finite();
            if (ok)
                m.add_face(t, FaceOriginal, 0);
            else
                ++res.stats.invalid_faces_removed;
        }

        BoundingBox bb;
        for (const Triangle& t : m.F)
            for (int v : t)
                bb.extend(m.P[v]);
        const double diag = bb.valid() ? bb.diagonal() : 0.0;

        Context ctx;
        ctx.options    = &options;
        ctx.stats      = &res.stats;
        ctx.cancel     = &cancel;
        ctx.diag       = diag;
        ctx.weld_tol   = options.weld_tolerance >= 0.0 ? options.weld_tolerance : diag * 1e-6;
        ctx.stitch_tol = options.stitch_tolerance >= 0.0 ? options.stitch_tolerance : diag * 1e-4;
        ctx.stitch_tol = std::max(ctx.stitch_tol, ctx.weld_tol);
        res.weld_tolerance_used   = ctx.weld_tol;
        res.stitch_tolerance_used = options.stitch_cracks ? ctx.stitch_tol : 0.0;
        const double eps          = std::max(ctx.weld_tol, diag * 1e-6);

        report("Merging duplicate vertices", 5);
        res.stats.vertices_welded = weld_vertices(m, ctx.weld_tol);
        ctx.check_cancel();

        // Crack closing and topology repair, alternating until nothing
        // changes. Cracks are closed first, so that vertices on a crack are
        // not mistaken for non-manifold vertices.
        remove_degenerate_and_duplicate_faces(m, ctx);
        if (options.stitch_cracks) {
            report("Closing cracks", 10);
            if (stitch_boundaries(m, ctx))
                remove_degenerate_and_duplicate_faces(m, ctx);
        }
        const int iterations = std::max(1, options.max_iterations);
        for (int iter = 0; iter < iterations; ++iter) {
            report("Fixing non-manifold edges and orientation", 15 + 40 * iter / iterations);
            make_manifold(m, ctx);
            if (!options.stitch_cracks || iter + 1 == iterations)
                break;
            report("Closing cracks", 20 + 40 * iter / iterations);
            if (!stitch_boundaries(m, ctx))
                break;
            remove_degenerate_and_duplicate_faces(m, ctx);
        }

        if (options.fill_holes) {
            for (int pass = 0; pass < 3; ++pass) {
                report("Filling holes", 60 + 5 * pass);
                const size_t before = res.stats.holes_filled;
                fill_holes(m, ctx);
                if (res.stats.holes_filled == before)
                    break;
                // Filling never creates non-manifold edges, but face removal
                // in make_manifold could open new (small) holes. A filled
                // single sheet is two opposite copies of the same faces, keep
                // them (zero volume shells are handled below).
                remove_degenerate_and_duplicate_faces(m, ctx, false);
                make_manifold(m, ctx);
            }
        }

        if (options.fix_degenerate_faces) {
            report("Fixing degenerate faces", 80);
            fix_cap_triangles(m, ctx, eps);
        }

        report("Orienting shells", 85);
        if (!m.F.empty() && options.fill_holes)
            check_has_volume(m, eps);
        remove_shells(m, ctx, eps);
        if (options.orient_outward) {
            orient_shells(m, ctx);
            flip_shells_seen_from_behind(m, ctx);
        }

        if (options.separate_coincident_vertices) {
            report("Separating touching shells", 92);
            separate_coincident_vertices(m, ctx);
        }

        m.compact_vertices();
        if (m.F.empty() && res.before.faces > res.before.invalid_faces)
            throw std::runtime_error("The model has no volume: it only consists of zero-thickness surfaces "
                                     "(or of faces that cancel each other out), so no printable solid could be built.");
        for (size_t f = 0; f < m.F.size(); ++f)
            if (m.flipped[f] && m.origin[f] != FaceFilled)
                ++res.stats.faces_flipped;
        ctx.check_cancel();

        mesh.vertices    = std::move(m.P);
        mesh.faces       = std::move(m.F);
        res.face_origin  = std::move(m.origin);

        report("Verifying result", 96);
        // Verified as written to STL / seen by the slicer.
        res.after   = analyze(options.separate_coincident_vertices ? float_welded(mesh) : mesh);
        res.success = true;
        report("Done", 100);
    } catch (const Canceled&) {
        res.canceled = true;
        res.error    = "Repair canceled";
    } catch (const std::exception& ex) {
        res.error = ex.what();
    }
    return res;
}

std::string format_diagnostics(const MeshDiagnostics& d)
{
    std::ostringstream ss;
    auto               line = [&ss](const char* label, size_t v) { ss << "  " << label << ": " << v << "\n"; };
    line("vertices", d.vertices);
    line("faces", d.faces);
    line("shells", d.shells);
    line("open edges", d.open_edges);
    line("holes", d.holes);
    line("non-manifold edges", d.non_manifold_edges);
    line("non-manifold vertices", d.non_manifold_vertices);
    line("inconsistently oriented edges", d.inconsistent_edges);
    line("inverted shells", d.inverted_shells);
    line("views seeing a back face first (Bambu ray test)", d.visible_back_faces);
    line("degenerate faces", d.degenerate_faces);
    line("duplicate faces", d.duplicate_faces);
    if (d.invalid_faces)
        line("invalid faces", d.invalid_faces);
    char buf[256];
    std::snprintf(buf, sizeof(buf), "  volume: %.6g\n  surface area: %.6g\n", d.volume, d.area);
    ss << buf;
    if (d.bbox.valid()) {
        const Vec3 s = d.bbox.size();
        std::snprintf(buf, sizeof(buf), "  size: %.4g x %.4g x %.4g\n", s.x, s.y, s.z);
        ss << buf;
    }
    ss << "  status: "
       << (d.clean() ? "clean" : (d.slicer_clean() ? "printable (minor issues)" : "needs repair")) << "\n";
    return ss.str();
}

std::string format_report(const RepairResult& r)
{
    std::ostringstream ss;
    if (!r.success) {
        ss << "Repair failed: " << r.error << "\n";
        return ss.str();
    }
    const RepairStats& s    = r.stats;
    auto               line = [&ss](const char* label, size_t v) {
        if (v)
            ss << "  " << label << ": " << v << "\n";
    };
    ss << "Repairs:\n";
    line("vertices merged", s.vertices_welded);
    line("invalid faces removed", s.invalid_faces_removed);
    line("degenerate faces removed", s.degenerate_faces_removed);
    line("degenerate faces fixed by edge flips", s.degenerate_faces_fixed);
    line("duplicate faces removed", s.duplicate_faces_removed);
    line("non-manifold edges fixed", s.non_manifold_edges_fixed);
    line("vertices split", s.non_manifold_vertices_split);
    line("faces flipped", s.faces_flipped);
    line("unresolvable faces removed", s.nonorientable_faces_removed);
    line("T-junctions fixed", s.t_junctions_fixed);
    line("crack vertices stitched", s.boundary_vertices_stitched);
    line("holes filled", s.holes_filled);
    line("faces added", s.faces_added);
    line("holes left open", s.holes_left_open);
    line("shells reoriented", s.shells_reoriented);
    line("shells removed", s.shells_removed);
    line("touching vertices separated", s.vertices_separated);
    ss << "Before:\n" << format_diagnostics(r.before);
    ss << "After:\n" << format_diagnostics(r.after);
    return ss.str();
}

} // namespace meshrepair

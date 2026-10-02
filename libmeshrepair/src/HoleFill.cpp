// Hole filling.
//
// Every boundary loop is triangulated with the dynamic programming scheme of
// Liepa ("Filling Holes in Meshes", SGP 2003): among all triangulations of the
// loop pick the one minimising the largest dihedral angle (between new
// triangles and between new triangles and the surrounding surface), with the
// total area as tie breaker. This gives flat caps for planar holes and smooth
// patches for curved ones. Very large loops are split into smaller ones first.

#include "Internal.hpp"

#include <cmath>
#include <unordered_set>

namespace meshrepair {
namespace detail {

namespace {

constexpr size_t kMaxDpLoop = 400;
constexpr double kPi        = 3.14159265358979323846;

struct Loop
{
    std::vector<int>  verts;  // patch orientation: the patch uses edges verts[i] -> verts[i+1]
    std::vector<Vec3> adj;    // adj[i]: unit normal of the face across edge i (zero if unknown)
};

double dihedral(const Vec3& n_new, const Vec3& n_adj)
{
    if (n_adj.squared_norm() == 0.0)
        return 0.0; // nothing to compare against
    if (n_new.squared_norm() == 0.0)
        return kPi; // degenerate new triangle: worst possible
    const double c = std::max(-1.0, std::min(1.0, n_new.dot(n_adj)));
    return std::acos(c);
}

class LoopFiller
{
public:
    LoopFiller(const std::vector<Vec3>& P, const std::unordered_set<uint64_t>& edges) : m_P(P), m_edges(edges) {}

    bool fill(const Loop& loop, std::vector<Triangle>& out)
    {
        const size_t n = loop.verts.size();
        if (n < 3)
            return false;
        if (n == 3) {
            out.push_back({ loop.verts[0], loop.verts[1], loop.verts[2] });
            return true;
        }
        if (n <= kMaxDpLoop) {
            if (dp(loop, out))
                return true;
            // DP impossible (every triangulation would duplicate an existing
            // edge): fall back to a fan around a new center vertex.
            return false;
        }
        Loop a, b;
        if (!split(loop, a, b))
            return false;
        std::vector<Triangle> ta, tb;
        if (!fill(a, ta) || !fill(b, tb))
            return false;
        out.insert(out.end(), ta.begin(), ta.end());
        out.insert(out.end(), tb.begin(), tb.end());
        return true;
    }

private:
    bool is_edge(int a, int b) const { return m_edges.count(edge_key(a, b)) != 0; }

    bool dp(const Loop& loop, std::vector<Triangle>& out)
    {
        const size_t               n   = loop.verts.size();
        const std::vector<int>&    L   = loop.verts;
        const double               INF = std::numeric_limits<double>::infinity();
        std::vector<double>        w_ang(n * n, INF);
        std::vector<double>        w_area(n * n, INF);
        std::vector<int>           best(n * n, -1);
        std::vector<Vec3>          tri_n(n * n);
        auto                       at = [n](size_t i, size_t k) { return i * n + k; };

        for (size_t i = 0; i + 1 < n; ++i) {
            w_ang[at(i, i + 1)]  = 0.0;
            w_area[at(i, i + 1)] = 0.0;
        }
        for (size_t len = 2; len < n; ++len) {
            for (size_t i = 0; i + len < n; ++i) {
                const size_t k   = i + len;
                const bool   top = i == 0 && k == n - 1;
                // A chord must not duplicate an edge already in the mesh.
                if (!top && is_edge(L[i], L[k]))
                    continue;
                double bang  = INF;
                double barea = INF;
                int    bm    = -1;
                Vec3   bn;
                for (size_t m = i + 1; m < k; ++m) {
                    const double a1 = w_ang[at(i, m)];
                    const double a2 = w_ang[at(m, k)];
                    if (a1 == INF || a2 == INF)
                        continue;
                    const Vec3   nrm = triangle_normal(m_P[L[i]], m_P[L[m]], m_P[L[k]]);
                    const double nn  = nrm.norm();
                    const Vec3   un  = nn > 0.0 ? nrm / nn : Vec3();
                    double       ang = std::max(a1, a2);
                    ang = std::max(ang, dihedral(un, m == i + 1 ? loop.adj[i] : tri_n[at(i, m)]));
                    ang = std::max(ang, dihedral(un, k == m + 1 ? loop.adj[m] : tri_n[at(m, k)]));
                    if (top)
                        ang = std::max(ang, dihedral(un, loop.adj[n - 1]));
                    const double area = 0.5 * nn + w_area[at(i, m)] + w_area[at(m, k)];
                    if (ang < bang - 1e-9 || (ang <= bang + 1e-9 && area < barea)) {
                        bang  = ang;
                        barea = area;
                        bm    = int(m);
                        bn    = un;
                    }
                }
                if (bm >= 0) {
                    w_ang[at(i, k)]  = bang;
                    w_area[at(i, k)] = barea;
                    best[at(i, k)]   = bm;
                    tri_n[at(i, k)]  = bn;
                }
            }
        }
        if (best[at(0, n - 1)] < 0)
            return false;

        std::vector<std::pair<size_t, size_t>> stack { { 0, n - 1 } };
        while (!stack.empty()) {
            const auto [i, k] = stack.back();
            stack.pop_back();
            const size_t m = size_t(best[at(i, k)]);
            out.push_back({ L[i], L[m], L[k] });
            if (m > i + 1)
                stack.push_back({ i, m });
            if (k > m + 1)
                stack.push_back({ m, k });
        }
        return true;
    }

    // Splits a large loop along a short chord into two loops of similar size.
    bool split(const Loop& loop, Loop& a, Loop& b)
    {
        const size_t n      = loop.verts.size();
        const size_t min_gap = std::max<size_t>(3, n / 4);
        const size_t stride = std::max<size_t>(1, n / 256);
        double       best   = std::numeric_limits<double>::infinity();
        size_t       bi = 0, bj = 0;
        for (size_t i = 0; i < n; i += stride)
            for (size_t j = i + min_gap; j < n; ++j) {
                if (n - (j - i) < min_gap)
                    break;
                if (is_edge(loop.verts[i], loop.verts[j]))
                    continue;
                const double d = (m_P[loop.verts[i]] - m_P[loop.verts[j]]).squared_norm();
                if (d < best) {
                    best = d;
                    bi   = i;
                    bj   = j;
                }
            }
        if (best == std::numeric_limits<double>::infinity())
            return false;
        a = Loop();
        b = Loop();
        for (size_t k = bi; k <= bj; ++k) {
            a.verts.push_back(loop.verts[k]);
            a.adj.push_back(k < bj ? loop.adj[k] : Vec3());
        }
        for (size_t k = bj; k != bi; k = (k + 1) % n) {
            b.verts.push_back(loop.verts[k]);
            b.adj.push_back(loop.adj[k]);
        }
        b.verts.push_back(loop.verts[bi]);
        b.adj.push_back(Vec3());
        return true;
    }

    const std::vector<Vec3>&            m_P;
    const std::unordered_set<uint64_t>& m_edges;
};

} // namespace

void fill_holes(WorkMesh& m, Context& ctx)
{
    const std::vector<EdgeRec> recs = build_edge_records(m.F);

    // Boundary half edges a->b (as used by their face); the patch uses b->a.
    std::unordered_map<int, int>  next;   // patch loop: vertex -> next vertex
    std::unordered_map<int, Vec3> adj_n;  // vertex (start of patch edge) -> adjacent face normal
    std::unordered_set<int>       broken; // vertices with more than one boundary successor
    std::unordered_set<uint64_t>  edges;
    edges.reserve(recs.size());
    for_each_edge_group(recs, [&](size_t b, size_t e) {
        edges.insert(edge_key(recs[b].lo, recs[b].hi));
        if (e - b != 1)
            return;
        const EdgeRec&  r  = recs[b];
        const Triangle& t  = m.F[r.face];
        const int       va = t[r.corner];
        const int       vb = t[(r.corner + 1) % 3];
        if (next.count(vb))
            broken.insert(vb);
        next[vb]  = va;
        adj_n[vb] = m.normal(r.face).normalized();
    });
    if (next.empty())
        return;

    // Collect loops.
    std::vector<Loop>       loops;
    std::unordered_set<int> visited;
    std::vector<int>        starts;
    starts.reserve(next.size());
    for (const auto& kv : next)
        starts.push_back(kv.first);
    std::sort(starts.begin(), starts.end());
    for (int s : starts) {
        if (visited.count(s))
            continue;
        Loop loop;
        int  v  = s;
        bool ok = true;
        while (true) {
            if (broken.count(v)) {
                ok = false;
            }
            visited.insert(v);
            loop.verts.push_back(v);
            loop.adj.push_back(adj_n[v]);
            auto it = next.find(v);
            if (it == next.end()) {
                ok = false;
                break;
            }
            v = it->second;
            if (v == s)
                break;
            if (visited.count(v)) {
                ok = false;
                break;
            }
        }
        if (ok)
            loops.push_back(std::move(loop));
        else
            ++ctx.stats->holes_left_open;
    }

    LoopFiller filler(m.P, edges);
    size_t     done = 0;
    for (const Loop& loop : loops) {
        ctx.check_cancel();
        ++done;
        if (ctx.options->max_hole_edges > 0 && loop.verts.size() > ctx.options->max_hole_edges) {
            ++ctx.stats->holes_left_open;
            continue;
        }
        std::vector<Triangle> tris;
        if (!filler.fill(loop, tris)) {
            // Fallback: fan around the centroid of the loop.
            tris.clear();
            Vec3 c;
            for (int v : loop.verts)
                c += m.P[v];
            c /= double(loop.verts.size());
            const int cv = m.add_vertex(c, m.new_orig());
            for (size_t i = 0; i < loop.verts.size(); ++i)
                tris.push_back({ loop.verts[i], loop.verts[(i + 1) % loop.verts.size()], cv });
        }
        for (const Triangle& t : tris) {
            m.add_face(t, FaceFilled, 0);
            edges.insert(edge_key(t[0], t[1]));
            edges.insert(edge_key(t[1], t[2]));
            edges.insert(edge_key(t[2], t[0]));
        }
        ++ctx.stats->holes_filled;
        ctx.stats->faces_added += tris.size();
    }
}

} // namespace detail
} // namespace meshrepair

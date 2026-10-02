// Crack closing: T-junction removal and merging of nearby boundary vertices.
//
// Cracks are the most common source of "open edges" in exported CAD models:
// two neighbouring surface patches are tessellated independently, so the
// vertices along their common border either do not coincide exactly or one
// side has vertices in the middle of an edge of the other side.

#include "Internal.hpp"

#include <unordered_map>

namespace meshrepair {
namespace detail {

namespace {

std::vector<int> boundary_records(const std::vector<EdgeRec>& recs)
{
    std::vector<int> bnd;
    for_each_edge_group(recs, [&](size_t b, size_t e) {
        if (e - b == 1)
            bnd.push_back(int(b));
    });
    return bnd;
}

// Inserts boundary vertices lying on boundary edges of other faces into those
// edges. Returns the number of inserted vertices.
size_t split_t_junctions(WorkMesh& m, double tol)
{
    const std::vector<EdgeRec> recs = build_edge_records(m.F);
    const std::vector<int>     bnd  = boundary_records(recs);
    if (bnd.empty())
        return 0;

    std::vector<char> is_bv(m.P.size(), 0);
    std::vector<BoundingBox> boxes(bnd.size());
    for (size_t k = 0; k < bnd.size(); ++k) {
        const EdgeRec& r = recs[bnd[k]];
        is_bv[r.lo] = is_bv[r.hi] = 1;
        boxes[k].extend(m.P[r.lo]);
        boxes[k].extend(m.P[r.hi]);
        boxes[k].inflate(tol);
    }
    AABBTree tree;
    tree.build(std::move(boxes));

    struct Split
    {
        int    face;
        int    corner;
        double t;
        int    v;
    };
    // A vertex close to many boundary edges is ambiguous (or the tolerance is
    // far too large for this mesh): such vertices are left alone, and the
    // total amount of work is bounded.
    constexpr size_t   kMaxCandidates = 16;
    const size_t       max_splits     = 4 * bnd.size() + 64;
    std::vector<Split> splits;
    std::vector<Split> local;
    for (size_t v = 0; v < m.P.size(); ++v) {
        if (!is_bv[v])
            continue;
        const Vec3& p = m.P[v];
        BoundingBox q;
        q.extend(p);
        local.clear();
        size_t candidates = 0;
        tree.query_box(q, [&](int k) -> bool {
            if (++candidates > kMaxCandidates)
                return false;
            const EdgeRec&  r = recs[bnd[k]];
            const Triangle& t = m.F[r.face];
            if (corner_of(t, int(v)) >= 0)
                return true;
            const int a = t[r.corner];
            const int b = t[(r.corner + 1) % 3];
            if (m.vorig[v] == m.vorig[a] || m.vorig[v] == m.vorig[b])
                return true;
            const Vec3   ab = m.P[b] - m.P[a];
            const double l2 = ab.squared_norm();
            if (!(l2 > 0.0))
                return true;
            const double tt = (p - m.P[a]).dot(ab) / l2;
            if (tt <= 0.0 || tt >= 1.0)
                return true;
            // Vertex to vertex proximity is handled by merging.
            if ((p - m.P[a]).norm() <= tol || (p - m.P[b]).norm() <= tol)
                return true;
            if ((p - (m.P[a] + ab * tt)).norm() > tol)
                return true;
            local.push_back({ r.face, r.corner, tt, int(v) });
            return true;
        });
        if (candidates > kMaxCandidates)
            continue;
        splits.insert(splits.end(), local.begin(), local.end());
        if (splits.size() > max_splits)
            return 0;
    }
    if (splits.empty())
        return 0;

    std::sort(splits.begin(), splits.end(), [](const Split& a, const Split& b) {
        if (a.face != b.face)
            return a.face < b.face;
        if (a.corner != b.corner)
            return a.corner < b.corner;
        if (a.t != b.t)
            return a.t < b.t;
        return a.v < b.v;
    });

    std::vector<char> remove(m.F.size(), 0);
    size_t            inserted = 0;
    const size_t      nf       = m.F.size();
    for (size_t i = 0; i < splits.size();) {
        const int face = splits[i].face;
        size_t    j    = i;
        std::vector<int> on_edge[3];
        while (j < splits.size() && splits[j].face == face) {
            std::vector<int>& list = on_edge[splits[j].corner];
            if (std::find(list.begin(), list.end(), splits[j].v) == list.end())
                list.push_back(splits[j].v);
            ++j;
        }
        i = j;

        const Triangle t       = m.F[face];
        const uint8_t  origin  = m.origin[face] == FaceFilled ? uint8_t(FaceFilled) : uint8_t(FaceSplit);
        const uint8_t  flipped = m.flipped[face];
        int            edges_with_splits = 0;
        int            split_edge        = -1;
        for (int c = 0; c < 3; ++c)
            if (!on_edge[c].empty()) {
                ++edges_with_splits;
                split_edge = c;
                inserted += on_edge[c].size();
            }
        remove[face] = 1;
        if (edges_with_splits == 1) {
            // Fan from the vertex opposite to the split edge.
            const int        c    = split_edge;
            const int        apex = t[(c + 2) % 3];
            std::vector<int> chain;
            chain.push_back(t[c]);
            chain.insert(chain.end(), on_edge[c].begin(), on_edge[c].end());
            chain.push_back(t[(c + 1) % 3]);
            for (size_t k = 0; k + 1 < chain.size(); ++k)
                m.add_face({ apex, chain[k], chain[k + 1] }, origin, flipped);
        } else {
            // Fan from a new vertex at the centroid.
            std::vector<int> poly;
            for (int c = 0; c < 3; ++c) {
                poly.push_back(t[c]);
                poly.insert(poly.end(), on_edge[c].begin(), on_edge[c].end());
            }
            const int cv = m.add_vertex((m.P[t[0]] + m.P[t[1]] + m.P[t[2]]) / 3.0, m.new_orig());
            for (size_t k = 0; k < poly.size(); ++k)
                m.add_face({ cv, poly[k], poly[(k + 1) % poly.size()] }, origin, flipped);
        }
    }
    remove.resize(m.F.size(), 0);
    (void)nf;
    m.remove_faces(remove);
    return inserted;
}

// Merges boundary vertices closer than tol. Vertices that were deliberately
// split apart (same vorig) are never merged again.
size_t merge_boundary_vertices(WorkMesh& m, double tol)
{
    const std::vector<EdgeRec> recs = build_edge_records(m.F);
    const std::vector<int>     bnd  = boundary_records(recs);
    if (bnd.empty())
        return 0;
    std::vector<char> is_bv(m.P.size(), 0);
    // Neighbours along boundary edges (usually two per vertex).
    std::unordered_map<int, std::vector<int>> bnb;
    for (int r : bnd) {
        is_bv[recs[r].lo] = is_bv[recs[r].hi] = 1;
        bnb[recs[r].lo].push_back(recs[r].hi);
        bnb[recs[r].hi].push_back(recs[r].lo);
    }
    std::vector<int> ids;
    for (size_t v = 0; v < m.P.size(); ++v)
        if (is_bv[v])
            ids.push_back(int(v));
    // Copies of a vertex split apart earlier may only be merged again when
    // that glues an edge (they share a boundary neighbour), otherwise the
    // non-manifold vertex would just come back.
    auto rejoins_edge = [&](int a, int b) {
        for (int x : bnb[a])
            for (int y : bnb[b])
                if (x == y)
                    return true;
        return false;
    };

    struct Pair
    {
        int    a, b;
        double d;
    };
    // A crack pairs every boundary vertex with only a few others; far more
    // pairs mean the tolerance is too large for this mesh: do nothing then.
    std::vector<Pair> pairs;
    const size_t      max_pairs = 8 * ids.size() + 64;
    bool              too_many  = false;
    for_each_close_pair(m.P, ids, tol, [&](int a, int b, double d) {
        if (too_many)
            return;
        if (m.vorig[a] != m.vorig[b] || rejoins_edge(a, b))
            pairs.push_back({ a, b, d });
        too_many = pairs.size() > max_pairs;
    });
    if (pairs.empty() || too_many)
        return 0;
    std::sort(pairs.begin(), pairs.end(), [](const Pair& x, const Pair& y) {
        if (x.d != y.d)
            return x.d < y.d;
        if (x.a != y.a)
            return x.a < y.a;
        return x.b < y.b;
    });

    UnionFind                     uf(m.P.size());
    std::vector<std::vector<int>> origs(m.P.size());
    size_t                        merged = 0;
    for (const Pair& p : pairs) {
        int ra = uf.find(p.a);
        int rb = uf.find(p.b);
        if (ra == rb)
            continue;
        std::vector<int>& oa = origs[ra];
        std::vector<int>& ob = origs[rb];
        if (oa.empty())
            oa.push_back(m.vorig[ra]);
        if (ob.empty())
            ob.push_back(m.vorig[rb]);
        bool conflict = false;
        for (int x : oa)
            for (int y : ob)
                conflict |= x == y;
        if (conflict && !rejoins_edge(p.a, p.b))
            continue;
        uf.unite(ra, rb);
        const int r     = uf.find(ra);
        const int other = r == ra ? rb : ra;
        std::vector<int> joined(origs[r]);
        joined.insert(joined.end(), origs[other].begin(), origs[other].end());
        origs[other].clear();
        origs[r].swap(joined);
        ++merged;
    }
    if (merged == 0)
        return 0;

    // New position = average of the cluster, the root keeps its index.
    std::vector<Vec3> sum(m.P.size(), Vec3());
    std::vector<int>  cnt(m.P.size(), 0);
    for (int v : ids) {
        const int r = uf.find(v);
        sum[r] += m.P[v];
        ++cnt[r];
    }
    for (int v : ids) {
        const int r = uf.find(v);
        if (r == v && cnt[r] > 1)
            m.P[r] = sum[r] / double(cnt[r]);
    }
    for (Triangle& t : m.F)
        for (int& v : t)
            v = uf.find(v);
    return merged;
}

} // namespace

bool stitch_boundaries(WorkMesh& m, Context& ctx)
{
    const double tol = ctx.stitch_tol;
    if (!(tol > 0.0) || m.F.empty())
        return false;
    ctx.check_cancel();
    const size_t inserted = split_t_junctions(m, tol);
    ctx.stats->t_junctions_fixed += inserted;
    ctx.check_cancel();
    const size_t merged = merge_boundary_vertices(m, tol);
    ctx.stats->boundary_vertices_stitched += merged;
    return inserted + merged > 0;
}

} // namespace detail
} // namespace meshrepair

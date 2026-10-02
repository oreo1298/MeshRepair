// Topological repair: vertex welding, removal of degenerate / duplicate faces,
// splitting of non-manifold edges and vertices and consistent orientation.

#include "Internal.hpp"

#include <cmath>
#include <deque>
#include <unordered_set>

namespace meshrepair {
namespace detail {

// ---------------------------------------------------------------------------
// Vertex welding

// Groups points closer than tol (transitively).
static UnionFind cluster_points(const std::vector<Vec3>& pts, const std::vector<int>& ids, double tol)
{
    UnionFind uf(pts.size());
    if (ids.empty())
        return uf;
    if (tol <= 0.0) {
        std::vector<int> order(ids);
        std::sort(order.begin(), order.end(), [&](int a, int b) {
            const Vec3& p = pts[a];
            const Vec3& q = pts[b];
            if (p.x != q.x)
                return p.x < q.x;
            if (p.y != q.y)
                return p.y < q.y;
            return p.z < q.z;
        });
        for (size_t i = 1; i < order.size(); ++i)
            if (pts[order[i]] == pts[order[i - 1]])
                uf.unite(order[i], order[i - 1]);
        return uf;
    }
    // Identical positions first (sorting), so that heaps of coincident points
    // do not make the grid search quadratic; then the grid on unique points.
    std::vector<int> order(ids);
    std::sort(order.begin(), order.end(), [&](int a, int b) {
        const Vec3& p = pts[a];
        const Vec3& q = pts[b];
        if (p.x != q.x)
            return p.x < q.x;
        if (p.y != q.y)
            return p.y < q.y;
        return p.z < q.z;
    });
    std::vector<int> unique;
    unique.reserve(order.size());
    for (size_t i = 0; i < order.size(); ++i) {
        if (i > 0 && pts[order[i]] == pts[order[i - 1]])
            uf.unite(order[i], order[i - 1]);
        else
            unique.push_back(order[i]);
    }
    for_each_close_pair(pts, unique, tol, [&](int i, int j, double) { uf.unite(i, j); });
    return uf;
}

size_t weld_vertices(WorkMesh& m, double tol)
{
    m.compact_vertices();
    const size_t     n = m.P.size();
    std::vector<int> all(n);
    std::iota(all.begin(), all.end(), 0);
    UnionFind uf = cluster_points(m.P, all, tol);

    // The cluster takes the position of its lowest index vertex.
    std::vector<int> new_id(n, -1);
    std::vector<int> root_id(n, -1);
    std::vector<Vec3> P;
    P.reserve(n);
    for (size_t i = 0; i < n; ++i) {
        const int r = uf.find(int(i));
        if (root_id[r] < 0) {
            root_id[r] = int(P.size());
            P.push_back(m.P[i]);
        }
        new_id[i] = root_id[r];
    }
    for (Triangle& t : m.F)
        for (int& v : t)
            v = new_id[v];
    const size_t merged = n - P.size();
    m.P.swap(P);
    m.vorig.resize(m.P.size());
    std::iota(m.vorig.begin(), m.vorig.end(), 0);
    m.next_orig = int(m.P.size());
    return merged;
}

// ---------------------------------------------------------------------------
// Degenerate and duplicate faces

void remove_degenerate_and_duplicate_faces(WorkMesh& m, Context& ctx, bool cancel_opposite)
{
    std::vector<char> remove(m.F.size(), 0);
    size_t            degenerate = 0;
    for (size_t f = 0; f < m.F.size(); ++f) {
        const Triangle& t = m.F[f];
        if (t[0] == t[1] || t[1] == t[2] || t[2] == t[0]) {
            remove[f] = 1;
            ++degenerate;
        }
    }

    // Duplicates: faces with the same vertex set. Equally oriented copies are
    // reduced to one face. Oppositely oriented pairs (typically the shared wall
    // of two solids exported together) cancel each other out.
    struct Key
    {
        Triangle sorted;
        bool     even; // orientation parity relative to the sorted order
        int      face;
    };
    std::vector<Key> keys;
    keys.reserve(m.F.size());
    for (size_t f = 0; f < m.F.size(); ++f) {
        if (remove[f])
            continue;
        Triangle t = m.F[f];
        // Rotate so that the smallest index comes first, then the parity is
        // given by the order of the remaining two.
        while (t[0] > t[1] || t[0] > t[2])
            t = { t[1], t[2], t[0] };
        const bool even = t[1] < t[2];
        Triangle   s    = t;
        if (!even)
            std::swap(s[1], s[2]);
        keys.push_back({ s, even, int(f) });
    }
    std::sort(keys.begin(), keys.end(), [](const Key& a, const Key& b) {
        if (a.sorted != b.sorted)
            return a.sorted < b.sorted;
        return a.face < b.face;
    });
    size_t duplicates = 0;
    for (size_t i = 0; i < keys.size();) {
        size_t j = i + 1;
        while (j < keys.size() && keys[j].sorted == keys[i].sorted)
            ++j;
        if (j - i > 1) {
            int pos = 0, neg = 0;
            for (size_t k = i; k < j; ++k)
                keys[k].even ? ++pos : ++neg;
            // Keep a single face of the majority orientation, or nothing on a
            // tie. Without cancel_opposite one face per orientation is kept.
            bool keep_even = pos > 0, keep_odd = neg > 0;
            if (cancel_opposite) {
                keep_even = pos > neg;
                keep_odd  = neg > pos;
            }
            for (size_t k = i; k < j; ++k) {
                bool& keep = keys[k].even ? keep_even : keep_odd;
                if (keep) {
                    keep = false;
                    continue;
                }
                remove[keys[k].face] = 1;
                ++duplicates;
            }
        }
        i = j;
    }

    if (degenerate + duplicates > 0)
        m.remove_faces(remove);
    ctx.stats->degenerate_faces_removed += degenerate;
    ctx.stats->duplicate_faces_removed += duplicates;
}

// ---------------------------------------------------------------------------
// Non-manifold edges

namespace {

struct Fin
{
    int    rec;   // edge record index
    double theta; // angle around the edge axis
    bool   open;  // true: the solid lies on the side of increasing theta
    int    comp;  // orientation component of the face
};

// Pairs fins like parentheses on a circle: an "open" fin starts a wedge of
// solid material, a "close" fin ends it. Fins are expected sorted by angle.
void match_fins_circular(const std::vector<Fin>& fins, const std::vector<int>& order, std::vector<char>& matched,
                         std::vector<std::pair<int, int>>& pairs)
{
    std::vector<int>  stack;
    std::vector<char> in_stack(fins.size(), 0);
    for (int pass = 0; pass < 2; ++pass)
        for (int i : order) {
            if (matched[i])
                continue;
            if (fins[i].open) {
                if (!in_stack[i]) {
                    stack.push_back(i);
                    in_stack[i] = 1;
                }
            } else if (!stack.empty()) {
                const int o = stack.back();
                stack.pop_back();
                in_stack[o] = 0;
                matched[o] = matched[i] = 1;
                pairs.push_back({ fins[o].rec, fins[i].rec });
            }
        }
}

// Decides which faces around a non-manifold edge belong together.
void pair_non_manifold_edge(const WorkMesh& m, const std::vector<EdgeRec>& recs, size_t b, size_t e,
                            const std::vector<int>& comp, const std::vector<char>& is_sheet,
                            std::vector<std::pair<int, int>>& glue)
{
    const Vec3& A = m.P[recs[b].lo];
    const Vec3& B = m.P[recs[b].hi];
    Vec3        u = B - A;
    const double len = u.norm();
    if (!(len > 0.0))
        return;
    u /= len;
    // Basis perpendicular to the edge.
    Vec3 e1 = std::abs(u.x) < 0.9 ? Vec3(1, 0, 0) : Vec3(0, 1, 0);
    e1      = (e1 - u * e1.dot(u)).normalized();
    const Vec3 e2 = u.cross(e1);

    std::vector<Fin> fins;
    for (size_t i = b; i < e; ++i) {
        const Triangle& t   = m.F[recs[i].face];
        const int       opp = t[(recs[i].corner + 2) % 3];
        Vec3            w   = m.P[opp] - A;
        w -= u * w.dot(u);
        if (w.norm() <= len * 1e-12)
            continue; // degenerate face, no direction
        const bool forward = edge_forward(m.F, recs[i]);
        // A face running A->B has its normal pointing towards increasing
        // theta, so its solid side is towards decreasing theta.
        fins.push_back({ int(i), std::atan2(w.dot(e2), w.dot(e1)), !forward, comp[recs[i].face] });
    }
    if (fins.size() < 2)
        return;

    std::vector<int> order(fins.size());
    std::iota(order.begin(), order.end(), 0);
    std::sort(order.begin(), order.end(), [&](int a, int c) {
        if (fins[a].theta != fins[c].theta)
            return fins[a].theta < fins[c].theta;
        // Coincident fins: close before open, so that two solids touching
        // along a face are separated by an empty wedge of zero width.
        if (fins[a].open != fins[c].open)
            return !fins[a].open;
        return a < c;
    });

    std::vector<char>                matched(fins.size(), 0);
    std::vector<std::pair<int, int>> pairs;

    // 1) Fins of the same connected surface belong together first. This keeps
    //    a closed surface intact when a stray sheet (internal wall) touches it.
    std::vector<int> comps;
    for (const Fin& f : fins)
        comps.push_back(f.comp);
    std::sort(comps.begin(), comps.end());
    comps.erase(std::unique(comps.begin(), comps.end()), comps.end());
    if (comps.size() < fins.size())
        for (int c : comps) {
            std::vector<int> sub;
            for (int i : order)
                if (fins[i].comp == c)
                    sub.push_back(i);
            if (sub.size() >= 2)
                match_fins_circular(fins, sub, matched, pairs);
        }
    // 2) Fins of surfaces enclosing a volume. Flat sheets (internal walls,
    //    stray fins) are matched last, so that they do not cut a solid apart.
    {
        std::vector<int> solid;
        for (int i : order)
            if (!is_sheet[fins[i].comp])
                solid.push_back(i);
        if (solid.size() >= 2 && solid.size() < fins.size())
            match_fins_circular(fins, solid, matched, pairs);
    }
    // 3) Remaining fins across surfaces.
    match_fins_circular(fins, order, matched, pairs);

    for (const auto& p : pairs)
        glue.push_back(p);
}

// A component is a "sheet" when closing its boundary would enclose no volume:
// planar internal walls, fins and similar single sided leftovers.
std::vector<char> classify_sheets(const WorkMesh& m, const std::vector<EdgeRec>& recs, const std::vector<int>& comp,
                                  int ncomp)
{
    // Reference point: centroid of the component's boundary vertices (edges
    // where the component contributes a single face).
    std::vector<Vec3>   bsum(ncomp);
    std::vector<int>    bcnt(ncomp, 0);
    std::vector<int>    local;
    for_each_edge_group(recs, [&](size_t b, size_t e) {
        local.clear();
        for (size_t i = b; i < e; ++i)
            local.push_back(comp[recs[i].face]);
        std::sort(local.begin(), local.end());
        for (size_t i = 0; i < local.size();) {
            size_t j = i + 1;
            while (j < local.size() && local[j] == local[i])
                ++j;
            if (j - i == 1) {
                bsum[local[i]] += m.P[recs[b].lo] + m.P[recs[b].hi];
                bcnt[local[i]] += 2;
            }
            i = j;
        }
    });
    std::vector<BoundingBox> boxes(ncomp);
    for (size_t f = 0; f < m.F.size(); ++f)
        for (int v : m.F[f])
            boxes[comp[f]].extend(m.P[v]);
    std::vector<Vec3> ref(ncomp);
    for (int c = 0; c < ncomp; ++c)
        ref[c] = bcnt[c] > 0 ? bsum[c] / double(bcnt[c]) : boxes[c].center();
    std::vector<double> vol(ncomp, 0.0), area(ncomp, 0.0);
    for (size_t f = 0; f < m.F.size(); ++f) {
        const int  c = comp[f];
        const Vec3 a = m.P[m.F[f][0]] - ref[c];
        const Vec3 b = m.P[m.F[f][1]] - ref[c];
        const Vec3 d = m.P[m.F[f][2]] - ref[c];
        vol[c] += a.dot(b.cross(d)) / 6.0;
        area[c] += m.area(f);
    }
    std::vector<char> sheet(ncomp, 0);
    for (int c = 0; c < ncomp; ++c)
        sheet[c] = std::abs(vol[c]) <= 1e-6 * area[c] * std::sqrt(area[c]);
    return sheet;
}

} // namespace

void make_manifold(WorkMesh& m, Context& ctx)
{
    // Repeats until the result is manifold; every repetition removes faces,
    // so this terminates.
    for (;;) {
        ctx.check_cancel();
        const size_t nf = m.F.size();
        if (nf == 0)
            return;

        // --- Phase A: orient faces consistently across two-face edges -------
        std::vector<EdgeRec> recs = build_edge_records(m.F);
        std::vector<int>     rec_of_corner(nf * 3, -1);
        std::vector<int>     partner(recs.size(), -1);
        for (size_t i = 0; i < recs.size(); ++i)
            rec_of_corner[recs[i].face * 3 + recs[i].corner] = int(i);
        for_each_edge_group(recs, [&](size_t b, size_t e) {
            if (e - b == 2) {
                partner[b]     = int(b + 1);
                partner[b + 1] = int(b);
            }
        });

        std::vector<int>             comp(nf, -1);
        std::vector<char>            flip(nf, 0);
        std::unordered_set<uint64_t> cut_edges;
        int                          ncomp = 0;
        std::deque<int>              queue;
        for (size_t seed = 0; seed < nf; ++seed) {
            if (comp[seed] >= 0)
                continue;
            comp[seed] = ncomp;
            queue.push_back(int(seed));
            while (!queue.empty()) {
                const int f = queue.front();
                queue.pop_front();
                for (int c = 0; c < 3; ++c) {
                    const int r = rec_of_corner[f * 3 + c];
                    if (r < 0 || partner[r] < 0)
                        continue;
                    const int  o    = partner[r];
                    const int  g    = recs[o].face;
                    const bool same = edge_forward(m.F, recs[r]) == edge_forward(m.F, recs[o]);
                    const char want = char(flip[f] ^ (same ? 1 : 0));
                    if (comp[g] < 0) {
                        comp[g] = ncomp;
                        flip[g] = want;
                        queue.push_back(g);
                    } else if (flip[g] != want) {
                        // Non-orientable: do not glue across this edge.
                        cut_edges.insert(edge_key(recs[r].lo, recs[r].hi));
                    }
                }
            }
            ++ncomp;
        }
        for (size_t f = 0; f < nf; ++f)
            if (flip[f])
                m.flip_face(f);

        // Orient each component outwards (heuristically, the shells are not
        // closed yet). This makes the orientation of different surfaces meeting
        // at a non-manifold edge comparable.
        {
            std::vector<BoundingBox> boxes(ncomp);
            for (size_t f = 0; f < nf; ++f)
                for (int v : m.F[f])
                    boxes[comp[f]].extend(m.P[v]);
            std::vector<double> vol(ncomp, 0.0);
            for (size_t f = 0; f < nf; ++f) {
                const Vec3 o = boxes[comp[f]].center();
                const Vec3 a = m.P[m.F[f][0]] - o;
                const Vec3 b = m.P[m.F[f][1]] - o;
                const Vec3 c = m.P[m.F[f][2]] - o;
                vol[comp[f]] += a.dot(b.cross(c));
            }
            for (size_t f = 0; f < nf; ++f)
                if (vol[comp[f]] < 0.0)
                    m.flip_face(f);
        }

        // --- Phase B: decide which faces to glue along every edge -----------
        recs = build_edge_records(m.F);
        const std::vector<char> is_sheet = classify_sheets(m, recs, comp, ncomp);
        std::vector<std::pair<int, int>> glue;
        size_t                           nm_edges = 0;
        for_each_edge_group(recs, [&](size_t b, size_t e) {
            const size_t cnt = e - b;
            if (cnt == 2) {
                if (!cut_edges.count(edge_key(recs[b].lo, recs[b].hi)))
                    glue.push_back({ int(b), int(b + 1) });
            } else if (cnt > 2) {
                ++nm_edges;
                pair_non_manifold_edge(m, recs, b, e, comp, is_sheet, glue);
            }
        });

        // --- Phase C: rebuild vertices from glued corners ---------------------
        UnionFind        corners(nf * 3);
        std::vector<int> glued_with(nf * 3, -1); // corner (face*3+edge corner) -> partner face
        for (const auto& g : glue) {
            const EdgeRec& ra = recs[g.first];
            const EdgeRec& rb = recs[g.second];
            for (int v : { ra.lo, ra.hi })
                corners.unite(ra.face * 3 + corner_of(m.F[ra.face], v), rb.face * 3 + corner_of(m.F[rb.face], v));
            glued_with[ra.face * 3 + ra.corner] = rb.face;
            glued_with[rb.face * 3 + rb.corner] = ra.face;
        }
        std::vector<int>      new_id(nf * 3, -1);
        std::vector<Vec3>     P;
        std::vector<int>      vorig;
        std::vector<Triangle> F(nf);
        std::vector<int>      copies(m.P.size(), 0);
        for (size_t f = 0; f < nf; ++f)
            for (int k = 0; k < 3; ++k) {
                const int r = corners.find(int(f * 3 + k));
                if (new_id[r] < 0) {
                    const int v = m.F[f][k];
                    new_id[r]   = int(P.size());
                    P.push_back(m.P[v]);
                    vorig.push_back(m.vorig[v]);
                    ++copies[v];
                }
                F[f][k] = new_id[r];
            }

        // --- Verification ----------------------------------------------------
        // Gluing decisions are local; in rare configurations vertex fans
        // connect elsewhere and an edge still ends up shared by the wrong faces.
        // Such faces are removed (the holes get filled later).
        const std::vector<EdgeRec> out = build_edge_records(F);
        std::vector<char>          remove(nf, 0);
        size_t                     bad = 0;
        for_each_edge_group(out, [&](size_t b, size_t e) {
            const size_t cnt = e - b;
            if (cnt == 2) {
                if (edge_forward(F, out[b]) == edge_forward(F, out[b + 1])) {
                    const int f0 = out[b].face, f1 = out[b + 1].face;
                    remove[m.area(f0) < m.area(f1) ? f0 : f1] = 1;
                    ++bad;
                }
            } else if (cnt > 2) {
                // Keep one pair of faces that were glued to each other (or the
                // largest face if there is none), remove the others. Every
                // pass removes at least one face, so this always terminates.
                size_t keep_a = e, keep_b = e;
                for (size_t i = b; i < e && keep_a == e; ++i) {
                    const int partner_face = glued_with[out[i].face * 3 + out[i].corner];
                    for (size_t j = i + 1; j < e; ++j)
                        if (out[j].face == partner_face) {
                            keep_a = i;
                            keep_b = j;
                            break;
                        }
                }
                if (keep_a == e) {
                    keep_a = b;
                    for (size_t i = b + 1; i < e; ++i)
                        if (m.area(out[i].face) > m.area(out[keep_a].face))
                            keep_a = i;
                }
                for (size_t i = b; i < e; ++i)
                    if (i != keep_a && i != keep_b)
                        remove[out[i].face] = 1;
                ++bad;
            }
        });
        if (bad > 0) {
            ctx.stats->nonorientable_faces_removed += m.remove_faces(remove);
            continue;
        }

        size_t split = 0;
        for (int c : copies)
            if (c > 1)
                split += size_t(c - 1);
        ctx.stats->non_manifold_edges_fixed += nm_edges;
        ctx.stats->non_manifold_vertices_split += split;
        m.P.swap(P);
        m.vorig.swap(vorig);
        m.F.swap(F);
        return;
    }
}

} // namespace detail
} // namespace meshrepair

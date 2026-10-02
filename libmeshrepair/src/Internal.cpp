#include "Internal.hpp"

#include <cmath>
#include <memory>

namespace meshrepair {
namespace detail {

std::vector<EdgeRec> build_edge_records(const std::vector<Triangle>& faces)
{
    std::vector<EdgeRec> recs;
    recs.reserve(faces.size() * 3);
    for (size_t f = 0; f < faces.size(); ++f) {
        const Triangle& t = faces[f];
        for (int c = 0; c < 3; ++c) {
            const int a = t[c];
            const int b = t[(c + 1) % 3];
            if (a == b)
                continue;
            recs.push_back({ std::min(a, b), std::max(a, b), int(f), c });
        }
    }
    std::sort(recs.begin(), recs.end(), [](const EdgeRec& l, const EdgeRec& r) {
        if (l.lo != r.lo)
            return l.lo < r.lo;
        if (l.hi != r.hi)
            return l.hi < r.hi;
        return l.face < r.face;
    });
    return recs;
}

namespace {

struct CellKey
{
    int64_t x, y, z;
    bool    operator==(const CellKey& o) const { return x == o.x && y == o.y && z == o.z; }
    bool    operator<(const CellKey& o) const
    {
        if (x != o.x)
            return x < o.x;
        if (y != o.y)
            return y < o.y;
        return z < o.z;
    }
};

struct CellKeyHash
{
    size_t operator()(const CellKey& k) const
    {
        uint64_t h = uint64_t(k.x) * 0x9E3779B97F4A7C15ull;
        h ^= uint64_t(k.y) * 0xC2B2AE3D27D4EB4Full + (h << 6) + (h >> 2);
        h ^= uint64_t(k.z) * 0x165667B19E3779F9ull + (h << 6) + (h >> 2);
        return size_t(h);
    }
};

} // namespace

void for_each_close_pair(const std::vector<Vec3>& pts, const std::vector<int>& ids, double tol,
                         const std::function<void(int, int, double)>& fn)
{
    if (ids.empty() || !(tol > 0.0))
        return;
    const double inv = 1.0 / tol;
    auto         key = [inv](const Vec3& p) {
        return CellKey { int64_t(std::floor(p.x * inv)), int64_t(std::floor(p.y * inv)), int64_t(std::floor(p.z * inv)) };
    };
    std::vector<std::pair<CellKey, int>> cells;
    cells.reserve(ids.size());
    for (int i : ids)
        cells.push_back({ key(pts[i]), i });
    std::sort(cells.begin(), cells.end(), [](const auto& a, const auto& b) {
        if (!(a.first == b.first))
            return a.first < b.first;
        return a.second < b.second;
    });
    std::unordered_map<CellKey, std::pair<int, int>, CellKeyHash> range;
    range.reserve(cells.size());
    for (size_t i = 0; i < cells.size();) {
        size_t j = i + 1;
        while (j < cells.size() && cells[j].first == cells[i].first)
            ++j;
        range[cells[i].first] = { int(i), int(j) };
        i                     = j;
    }
    const double tol2 = tol * tol;
    for (const auto& c : cells) {
        const int   i = c.second;
        const Vec3& p = pts[i];
        for (int64_t dx = -1; dx <= 1; ++dx)
            for (int64_t dy = -1; dy <= 1; ++dy)
                for (int64_t dz = -1; dz <= 1; ++dz) {
                    auto it = range.find({ c.first.x + dx, c.first.y + dy, c.first.z + dz });
                    if (it == range.end())
                        continue;
                    for (int k = it->second.first; k < it->second.second; ++k) {
                        const int j = cells[k].second;
                        if (j <= i)
                            continue;
                        const double d2 = (pts[j] - p).squared_norm();
                        if (d2 <= tol2)
                            fn(i, j, std::sqrt(d2));
                    }
                }
    }
}

size_t WorkMesh::remove_faces(const std::vector<char>& remove)
{
    size_t out = 0;
    for (size_t f = 0; f < F.size(); ++f) {
        if (remove[f])
            continue;
        F[out]       = F[f];
        origin[out]  = origin[f];
        flipped[out] = flipped[f];
        ++out;
    }
    const size_t removed = F.size() - out;
    F.resize(out);
    origin.resize(out);
    flipped.resize(out);
    return removed;
}

void WorkMesh::compact_vertices()
{
    std::vector<int> remap(P.size(), -1);
    for (const Triangle& t : F)
        for (int v : t)
            remap[v] = 0;
    size_t next = 0;
    for (size_t i = 0; i < P.size(); ++i)
        if (remap[i] == 0) {
            remap[i]    = int(next);
            P[next]     = P[i];
            vorig[next] = vorig[i];
            ++next;
        }
    P.resize(next);
    vorig.resize(next);
    for (Triangle& t : F)
        for (int& v : t)
            v = remap[v];
}

// ---------------------------------------------------------------------------

void AABBTree::build(std::vector<BoundingBox> boxes)
{
    m_boxes = std::move(boxes);
    m_nodes.clear();
    m_prims.resize(m_boxes.size());
    std::iota(m_prims.begin(), m_prims.end(), 0);
    m_centers.resize(m_boxes.size());
    for (size_t i = 0; i < m_boxes.size(); ++i)
        m_centers[i] = m_boxes[i].center();
    if (m_boxes.empty())
        return;
    m_nodes.reserve(2 * m_boxes.size() / 4 + 2);
    build_node(0, int(m_prims.size()), 0);
    m_centers.clear();
    m_centers.shrink_to_fit();
}

int AABBTree::build_node(int begin, int end, int depth)
{
    const int idx = int(m_nodes.size());
    m_nodes.emplace_back();
    BoundingBox box;
    for (int i = begin; i < end; ++i)
        box.extend(m_boxes[m_prims[i]]);
    m_nodes[idx].box = box;
    if (end - begin <= 4 || depth > 60) {
        m_nodes[idx].begin = begin;
        m_nodes[idx].end   = end;
        return idx;
    }
    BoundingBox cbox;
    for (int i = begin; i < end; ++i)
        cbox.extend(m_centers[m_prims[i]]);
    const Vec3 s    = cbox.size();
    const int  axis = (s.x >= s.y && s.x >= s.z) ? 0 : (s.y >= s.z ? 1 : 2);
    const int  mid  = (begin + end) / 2;
    std::nth_element(m_prims.begin() + begin, m_prims.begin() + mid, m_prims.begin() + end,
                     [this, axis](int a, int b) { return m_centers[a][axis] < m_centers[b][axis]; });
    const int left        = build_node(begin, mid, depth + 1);
    const int right       = build_node(mid, end, depth + 1);
    m_nodes[idx].left     = left;
    m_nodes[idx].right    = right;
    return idx;
}

bool AABBTree::ray_hits_box(const Vec3& o, const Vec3& inv, const BoundingBox& b)
{
    double tmin = 0.0;
    double tmax = std::numeric_limits<double>::infinity();
    for (int a = 0; a < 3; ++a) {
        double t1 = (b.min[a] - o[a]) * inv[a];
        double t2 = (b.max[a] - o[a]) * inv[a];
        if (std::isnan(t1) || std::isnan(t2)) {
            // Ray parallel to the slab and origin on its boundary.
            if (o[a] < b.min[a] || o[a] > b.max[a])
                return false;
            continue;
        }
        if (t1 > t2)
            std::swap(t1, t2);
        tmin = std::max(tmin, t1);
        tmax = std::min(tmax, t2);
        if (tmin > tmax)
            return false;
    }
    return true;
}

bool ray_triangle(const Vec3& o, const Vec3& d, const Vec3& a, const Vec3& b, const Vec3& c, double tmin, double& t)
{
    const Vec3   e1  = b - a;
    const Vec3   e2  = c - a;
    const Vec3   p   = d.cross(e2);
    const double det = e1.dot(p);
    if (std::abs(det) < 1e-300)
        return false;
    const double inv = 1.0 / det;
    const Vec3   s   = o - a;
    const double u   = s.dot(p) * inv;
    if (u < 0.0 || u > 1.0)
        return false;
    const Vec3   q = s.cross(e1);
    const double v = d.dot(q) * inv;
    if (v < 0.0 || u + v > 1.0)
        return false;
    t = e2.dot(q) * inv;
    return t > tmin;
}

// ---------------------------------------------------------------------------

std::vector<int> face_components(const std::vector<Triangle>& F, int& num_components, bool through_any_edge)
{
    UnionFind                  uf(F.size());
    const std::vector<EdgeRec> recs = build_edge_records(F);
    for_each_edge_group(recs, [&](size_t b, size_t e) {
        if (e - b == 2 || (through_any_edge && e - b > 2))
            for (size_t i = b + 1; i < e; ++i)
                uf.unite(recs[b].face, recs[i].face);
    });
    std::vector<int> comp(F.size(), -1);
    std::vector<int> root_id(F.size(), -1);
    num_components = 0;
    for (size_t f = 0; f < F.size(); ++f) {
        const int r = uf.find(int(f));
        if (root_id[r] < 0)
            root_id[r] = num_components++;
        comp[f] = root_id[r];
    }
    return comp;
}

double shell_signed_volume(const std::vector<Vec3>& P, const std::vector<Triangle>& F, const std::vector<int>& faces)
{
    BoundingBox bb;
    for (int f : faces)
        for (int v : F[f])
            bb.extend(P[v]);
    const Vec3 o   = bb.center();
    double     vol = 0.0;
    for (int f : faces) {
        const Vec3 a = P[F[f][0]] - o;
        const Vec3 b = P[F[f][1]] - o;
        const Vec3 c = P[F[f][2]] - o;
        vol += a.dot(b.cross(c));
    }
    return vol / 6.0;
}

std::vector<int> nesting_depths(const std::vector<Vec3>& P, const std::vector<Triangle>& F,
                                const std::vector<std::vector<int>>& shells, const std::vector<char>& closed)
{
    const size_t     n = shells.size();
    std::vector<int> depth(n, 0);
    if (n < 2)
        return depth;

    std::vector<BoundingBox> boxes(n);
    BoundingBox              all;
    for (size_t s = 0; s < n; ++s) {
        for (int f : shells[s])
            for (int v : F[f])
                boxes[s].extend(P[v]);
        all.extend(boxes[s]);
    }
    const double eps = std::max(all.diagonal(), 1e-30) * 1e-9;

    // Containers must have a larger bounding box.
    std::vector<int> order(n);
    std::iota(order.begin(), order.end(), 0);
    auto box_volume = [&](int s) {
        const Vec3 sz = boxes[s].size();
        return sz.x * sz.y * sz.z + sz.norm() * 1e-12;
    };
    std::sort(order.begin(), order.end(), [&](int a, int b) { return box_volume(a) > box_volume(b); });

    std::vector<std::unique_ptr<AABBTree>> trees(n);
    auto tree_of = [&](int s) -> const AABBTree& {
        if (!trees[s]) {
            trees[s] = std::make_unique<AABBTree>();
            std::vector<BoundingBox> fb(shells[s].size());
            for (size_t i = 0; i < shells[s].size(); ++i)
                for (int v : F[shells[s][i]])
                    fb[i].extend(P[v]);
            trees[s]->build(std::move(fb));
        }
        return *trees[s];
    };

    static const Vec3 dirs[3] = { Vec3(0.5377, 0.6115, 0.5803).normalized(), Vec3(-0.7124, 0.3317, -0.6185).normalized(),
                                  Vec3(0.2291, -0.8836, 0.4083).normalized() };

    auto point_inside = [&](int shell, const Vec3& p) -> int {
        const AABBTree& tree   = tree_of(shell);
        int             in     = 0;
        int             out    = 0;
        for (const Vec3& d : dirs) {
            int hits = 0;
            tree.query_ray(p, d, [&](int i) {
                const Triangle& t = F[shells[shell][i]];
                double          tt;
                if (ray_triangle(p, d, P[t[0]], P[t[1]], P[t[2]], eps, tt))
                    ++hits;
            });
            (hits & 1) ? ++in : ++out;
        }
        return in > out ? 1 : 0;
    };

    for (size_t oi = 0; oi < n; ++oi) {
        const int s = order[oi];
        if (shells[s].empty())
            continue;
        // Sample points: centroids of faces spread over the shell.
        std::vector<Vec3> samples;
        const size_t      fs   = shells[s].size();
        const size_t      want = std::min<size_t>(5, fs);
        for (size_t k = 0; k < want; ++k) {
            const Triangle& t = F[shells[s][(k * fs) / want]];
            samples.push_back((P[t[0]] + P[t[1]] + P[t[2]]) / 3.0);
        }
        for (size_t oj = 0; oj < oi; ++oj) {
            const int c = order[oj];
            if (!closed[c] || c == s)
                continue;
            BoundingBox cb = boxes[c];
            cb.inflate(eps);
            if (!cb.contains(boxes[s]))
                continue;
            int inside = 0;
            for (const Vec3& p : samples)
                inside += point_inside(c, p);
            if (2 * inside > int(samples.size()))
                ++depth[s];
        }
    }
    return depth;
}

} // namespace detail
} // namespace meshrepair

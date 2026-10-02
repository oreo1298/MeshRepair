// Internal helpers shared by the repair stages. Not part of the public API.

#pragma once

#include "meshrepair/Mesh.hpp"
#include "meshrepair/Repair.hpp"

#include <algorithm>
#include <cstdint>
#include <functional>
#include <numeric>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

namespace meshrepair {
namespace detail {

// ---------------------------------------------------------------------------
// Union find with path halving and union by size.
class UnionFind
{
public:
    explicit UnionFind(size_t n = 0) { reset(n); }
    void reset(size_t n)
    {
        m_parent.resize(n);
        std::iota(m_parent.begin(), m_parent.end(), 0);
        m_size.assign(n, 1);
    }
    int find(int x)
    {
        while (m_parent[x] != x) {
            m_parent[x] = m_parent[m_parent[x]];
            x           = m_parent[x];
        }
        return x;
    }
    bool unite(int a, int b)
    {
        a = find(a);
        b = find(b);
        if (a == b)
            return false;
        if (m_size[a] < m_size[b])
            std::swap(a, b);
        m_parent[b] = a;
        m_size[a] += m_size[b];
        return true;
    }
    size_t size() const { return m_parent.size(); }

private:
    std::vector<int> m_parent;
    std::vector<int> m_size;
};

// ---------------------------------------------------------------------------
// Edge records: one record per face edge, sorted so that all records of the
// same undirected edge are adjacent.
struct EdgeRec
{
    int lo;     // smaller vertex index
    int hi;     // larger vertex index
    int face;   // face index
    int corner; // the edge runs from face[corner] to face[(corner + 1) % 3]

    bool same_edge(const EdgeRec& o) const { return lo == o.lo && hi == o.hi; }
};

inline bool edge_forward(const std::vector<Triangle>& F, const EdgeRec& e) { return F[e.face][e.corner] == e.lo; }

std::vector<EdgeRec> build_edge_records(const std::vector<Triangle>& faces);

// Calls fn(begin, end) for every range of records sharing one undirected edge.
template<class Fn>
void for_each_edge_group(const std::vector<EdgeRec>& recs, Fn&& fn)
{
    for (size_t i = 0; i < recs.size();) {
        size_t j = i + 1;
        while (j < recs.size() && recs[j].same_edge(recs[i]))
            ++j;
        fn(i, j);
        i = j;
    }
}

inline uint64_t edge_key(int a, int b)
{
    if (a > b)
        std::swap(a, b);
    return (uint64_t(uint32_t(a)) << 32) | uint64_t(uint32_t(b));
}
inline uint64_t directed_edge_key(int a, int b) { return (uint64_t(uint32_t(a)) << 32) | uint64_t(uint32_t(b)); }

// Index of vertex v within face f, or -1.
inline int corner_of(const Triangle& f, int v) { return f[0] == v ? 0 : (f[1] == v ? 1 : (f[2] == v ? 2 : -1)); }

// ---------------------------------------------------------------------------
// Size of the model for relative tolerances: the diagonal of the bounding box
// of the referenced vertices, ignoring the most extreme 0.5 % on every axis so
// that a few stray vertices far away do not blow up all tolerances.
double robust_diagonal(const std::vector<Vec3>& P, const std::vector<Triangle>& F);

// Model size used for all relative tolerances: the robust diagonal, but at
// most 1000 median edge lengths, which stays meaningful even when a large
// part of the file is garbage (e.g. a corrupted STL).
double model_scale(const std::vector<Vec3>& P, const std::vector<Triangle>& F);

// ---------------------------------------------------------------------------
// Spatial hashing: calls fn(i, j, distance) for every pair of points i < j
// (taken from ids) closer than tol. tol must be positive.
void for_each_close_pair(const std::vector<Vec3>& pts, const std::vector<int>& ids, double tol,
                         const std::function<void(int, int, double)>& fn);

// ---------------------------------------------------------------------------
// Working mesh with per element bookkeeping used throughout the pipeline.
struct WorkMesh
{
    std::vector<Vec3>     P;       // vertex positions
    std::vector<int>      vorig;   // vertex: welded input vertex it was split from
    std::vector<Triangle> F;       // faces
    std::vector<uint8_t>  origin;  // face: FaceOrigin
    std::vector<uint8_t>  flipped; // face: orientation reversed w.r.t. the input
    int                   next_orig = 0; // next unused vorig id (for new vertices)

    size_t num_faces() const { return F.size(); }
    int    new_orig() { return next_orig++; }

    int add_vertex(const Vec3& p, int orig)
    {
        P.push_back(p);
        vorig.push_back(orig);
        return int(P.size() - 1);
    }
    void add_face(const Triangle& t, uint8_t orig, uint8_t flip)
    {
        F.push_back(t);
        origin.push_back(orig);
        flipped.push_back(flip);
    }
    void flip_face(size_t f)
    {
        std::swap(F[f][1], F[f][2]);
        flipped[f] ^= 1;
    }
    // Removes all faces with remove[f] != 0. Returns number of removed faces.
    size_t remove_faces(const std::vector<char>& remove);
    // Drops vertices not referenced by any face.
    void compact_vertices();

    Vec3   normal(size_t f) const { return triangle_normal(P[F[f][0]], P[F[f][1]], P[F[f][2]]); }
    double area(size_t f) const { return 0.5 * normal(f).norm(); }
};

// ---------------------------------------------------------------------------
// Axis aligned bounding box tree over arbitrary primitives.
class AABBTree
{
public:
    void build(std::vector<BoundingBox> boxes);
    bool empty() const { return m_nodes.empty(); }

    // visit(prim) for every primitive whose box overlaps the query box. If
    // visit returns bool, returning false stops the query.
    template<class Fn>
    void query_box(const BoundingBox& box, Fn&& visit) const
    {
        if (m_nodes.empty())
            return;
        int stack[128];
        int sp      = 0;
        stack[sp++] = 0;
        while (sp > 0) {
            const Node& n = m_nodes[stack[--sp]];
            if (!n.box.intersects(box))
                continue;
            if (n.left < 0) {
                for (int i = n.begin; i < n.end; ++i)
                    if (m_boxes[m_prims[i]].intersects(box)) {
                        if constexpr (std::is_same_v<decltype(visit(0)), bool>) {
                            if (!visit(m_prims[i]))
                                return;
                        } else
                            visit(m_prims[i]);
                    }
            } else {
                stack[sp++] = n.left;
                stack[sp++] = n.right;
            }
        }
    }

    // visit(prim) for every primitive whose box is hit by the ray o + t * d, t >= 0.
    template<class Fn>
    void query_ray(const Vec3& o, const Vec3& d, Fn&& visit) const
    {
        if (m_nodes.empty())
            return;
        const Vec3 inv(1.0 / d.x, 1.0 / d.y, 1.0 / d.z);
        int        stack[128];
        int        sp = 0;
        stack[sp++]   = 0;
        while (sp > 0) {
            const Node& n = m_nodes[stack[--sp]];
            if (!ray_hits_box(o, inv, n.box))
                continue;
            if (n.left < 0) {
                for (int i = n.begin; i < n.end; ++i)
                    visit(m_prims[i]);
            } else {
                stack[sp++] = n.left;
                stack[sp++] = n.right;
            }
        }
    }

private:
    struct Node
    {
        BoundingBox box;
        int         left  = -1;
        int         right = -1;
        int         begin = 0;
        int         end   = 0;
    };
    static bool ray_hits_box(const Vec3& o, const Vec3& inv, const BoundingBox& b);
    int         build_node(int begin, int end, int depth);

    std::vector<Node>        m_nodes;
    std::vector<int>         m_prims;
    std::vector<BoundingBox> m_boxes;
    std::vector<Vec3>        m_centers;
};

// Ray / triangle intersection (Moeller-Trumbore). Returns true for t > tmin.
// Optionally returns the barycentric coordinates (P = (1-u-v)a + ub + vc).
bool ray_triangle(const Vec3& o, const Vec3& d, const Vec3& a, const Vec3& b, const Vec3& c, double tmin, double& t,
                  double* u_out = nullptr, double* v_out = nullptr);

// Port of Bambu Studio's reversed face test (libslic3r/MeshDiagnostics.cpp,
// detect_visible_backfaces): 46 rays shot from outside the bounding box at its
// center; the first surface each ray hits must face the ray. Meant for closed
// meshes. Returns the number of rays seeing a back face first; face_hits
// receives (face, seen_from_behind) for every ray that was not discarded.
int visible_back_faces(const std::vector<Vec3>& P, const std::vector<Triangle>& F,
                       std::vector<std::pair<int, bool>>* face_hits = nullptr);

// ---------------------------------------------------------------------------
// Shell analysis.

// Connected components of faces, connected through edges shared by exactly
// two faces (or any number of faces if through_any_edge is set).
std::vector<int> face_components(const std::vector<Triangle>& F, int& num_components, bool through_any_edge);

// For every shell (list of faces), count how many of the other closed shells
// contain it. Shells that are not closed never contain anything.
std::vector<int> nesting_depths(const std::vector<Vec3>& P, const std::vector<Triangle>& F,
                                const std::vector<std::vector<int>>& shells, const std::vector<char>& closed);

double shell_signed_volume(const std::vector<Vec3>& P, const std::vector<Triangle>& F, const std::vector<int>& faces);

// ---------------------------------------------------------------------------
// Cancellation support.
struct Canceled
{};

struct Context
{
    const RepairOptions* options = nullptr;
    RepairStats*         stats   = nullptr;
    const CancelFn*      cancel  = nullptr;
    double               weld_tol   = 0.0;
    double               stitch_tol = 0.0;
    double               diag       = 0.0;

    void check_cancel() const
    {
        if (cancel && *cancel && (*cancel)())
            throw Canceled {};
    }
};

// ---------------------------------------------------------------------------
// Repair stages (see Repair.cpp for the order in which they run).

// Merges vertices closer than tol. Returns the number of merged vertices.
size_t weld_vertices(WorkMesh& m, double tol);

// Removes faces with repeated vertex indices and duplicated faces. With
// cancel_opposite, two copies of a face with opposite orientation remove each
// other (shared wall of two solids), otherwise one of them is kept.
void remove_degenerate_and_duplicate_faces(WorkMesh& m, Context& ctx, bool cancel_opposite = true);

// Splits non-manifold edges and vertices, orients faces consistently.
// On return every edge has at most two faces, which are consistently oriented,
// and every vertex is manifold.
void make_manifold(WorkMesh& m, Context& ctx);

// Closes cracks by inserting T-junction vertices and merging close boundary
// vertices. Returns true if the mesh changed.
bool stitch_boundaries(WorkMesh& m, Context& ctx);

// Fills boundary loops. Requires a manifold, consistently oriented mesh.
void fill_holes(WorkMesh& m, Context& ctx);

} // namespace detail
} // namespace meshrepair

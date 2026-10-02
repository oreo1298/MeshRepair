#include "meshrepair/Diagnostics.hpp"

#include "Internal.hpp"

namespace meshrepair {

using namespace detail;

MeshDiagnostics analyze(const Mesh& mesh, DiagnosticsDetail* detail)
{
    MeshDiagnostics d;
    d.vertices = mesh.vertices.size();
    d.faces    = mesh.faces.size();
    if (detail)
        *detail = DiagnosticsDetail();

    // Faces taking part in the topological analysis: valid indices, finite
    // coordinates and three distinct vertex indices (Bambu Studio skips faces
    // with repeated indices in the same way).
    std::vector<Triangle> faces;
    std::vector<int>      face_id;
    faces.reserve(mesh.faces.size());
    face_id.reserve(mesh.faces.size());
    const int nv = int(mesh.vertices.size());
    for (size_t i = 0; i < mesh.faces.size(); ++i) {
        const Triangle& t     = mesh.faces[i];
        bool            valid = true;
        for (int v : t)
            if (v < 0 || v >= nv || !mesh.vertices[v].is_finite())
                valid = false;
        if (!valid) {
            ++d.invalid_faces;
            continue;
        }
        d.bbox.extend(mesh.vertices[t[0]]);
        d.bbox.extend(mesh.vertices[t[1]]);
        d.bbox.extend(mesh.vertices[t[2]]);
        if (t[0] == t[1] || t[1] == t[2] || t[2] == t[0]) {
            ++d.degenerate_faces;
            if (detail)
                detail->degenerate_faces.push_back(int(i));
            continue;
        }
        faces.push_back(t);
        face_id.push_back(int(i));
    }

    // Zero area faces with distinct vertices (caps / needles).
    const double diag     = std::min(d.bbox.diagonal(), model_scale(mesh.vertices, faces));
    const double h_thresh = diag * 1e-6;
    for (size_t k = 0; k < faces.size(); ++k) {
        const Vec3&  a  = mesh.vertices[faces[k][0]];
        const Vec3&  b  = mesh.vertices[faces[k][1]];
        const Vec3&  c  = mesh.vertices[faces[k][2]];
        const double l  = std::max({ (b - a).norm(), (c - b).norm(), (a - c).norm() });
        const double a2 = triangle_normal(a, b, c).norm();
        if (l == 0.0 || a2 / l <= h_thresh) {
            ++d.degenerate_faces;
            if (detail)
                detail->degenerate_faces.push_back(face_id[k]);
        }
        d.area += 0.5 * a2;
    }

    // Duplicate faces (same vertex set, any orientation).
    {
        std::vector<Triangle> sorted(faces);
        for (Triangle& t : sorted)
            std::sort(t.begin(), t.end());
        std::sort(sorted.begin(), sorted.end());
        for (size_t i = 1; i < sorted.size(); ++i)
            if (sorted[i] == sorted[i - 1])
                ++d.duplicate_faces;
    }

    // Edge classification and vertex fans.
    const std::vector<EdgeRec> recs = build_edge_records(faces);
    UnionFind                  corners(faces.size() * 3); // corner = face * 3 + k
    UnionFind                  shells(faces.size());
    UnionFind                  boundary(mesh.vertices.size());
    std::vector<char>          on_nm_edge(mesh.vertices.size(), 0);
    std::vector<char>          on_boundary(mesh.vertices.size(), 0);

    for_each_edge_group(recs, [&](size_t b, size_t e) {
        const EdgeRec& r = recs[b];
        for (size_t i = b + 1; i < e; ++i)
            shells.unite(recs[b].face, recs[i].face);
        const size_t cnt = e - b;
        if (cnt == 1) {
            ++d.open_edges;
            boundary.unite(r.lo, r.hi);
            on_boundary[r.lo] = on_boundary[r.hi] = 1;
            if (detail)
                detail->open_edges.push_back({ r.lo, r.hi });
        } else if (cnt == 2) {
            const EdgeRec& s = recs[b + 1];
            if (edge_forward(faces, r) == edge_forward(faces, s)) {
                ++d.inconsistent_edges;
                if (detail)
                    detail->inconsistent_edges.push_back({ r.lo, r.hi });
            }
            for (int v : { r.lo, r.hi })
                corners.unite(r.face * 3 + corner_of(faces[r.face], v), s.face * 3 + corner_of(faces[s.face], v));
        } else {
            ++d.non_manifold_edges;
            on_nm_edge[r.lo] = on_nm_edge[r.hi] = 1;
            if (detail)
                detail->non_manifold_edges.push_back({ r.lo, r.hi });
        }
    });

    // Non-manifold vertices: incident faces split into several fans.
    {
        std::vector<int> first_root(mesh.vertices.size(), -1);
        std::vector<char> counted(mesh.vertices.size(), 0);
        for (size_t f = 0; f < faces.size(); ++f)
            for (int k = 0; k < 3; ++k) {
                const int v = faces[f][k];
                if (on_nm_edge[v] || counted[v])
                    continue;
                const int root = corners.find(int(f * 3 + k));
                if (first_root[v] < 0)
                    first_root[v] = root;
                else if (first_root[v] != root) {
                    counted[v] = 1;
                    ++d.non_manifold_vertices;
                    if (detail)
                        detail->non_manifold_vertices.push_back(v);
                }
            }
    }

    // Holes = connected components of the open edge graph.
    {
        std::vector<char> seen(mesh.vertices.size(), 0);
        for (size_t v = 0; v < mesh.vertices.size(); ++v)
            if (on_boundary[v]) {
                const int r = boundary.find(int(v));
                if (!seen[r]) {
                    seen[r] = 1;
                    ++d.holes;
                }
            }
    }

    // Shells.
    std::vector<int>              shell_of(faces.size());
    std::vector<int>              root_id(faces.size(), -1);
    std::vector<std::vector<int>> shell_faces;
    for (size_t f = 0; f < faces.size(); ++f) {
        const int r = shells.find(int(f));
        if (root_id[r] < 0) {
            root_id[r] = int(shell_faces.size());
            shell_faces.emplace_back();
        }
        shell_of[f] = root_id[r];
        shell_faces[root_id[r]].push_back(int(f));
    }
    d.shells = shell_faces.size();

    // A shell is closed when all its edges have exactly two consistently
    // oriented faces.
    std::vector<char> closed(shell_faces.size(), 1);
    for_each_edge_group(recs, [&](size_t b, size_t e) {
        const int s = shell_of[recs[b].face];
        if (e - b != 2 || edge_forward(faces, recs[b]) == edge_forward(faces, recs[b + 1]))
            closed[s] = 0;
    });

    std::vector<Vec3> P(mesh.vertices);
    for (Vec3& p : P)
        if (!p.is_finite())
            p = Vec3();
    std::vector<double> volumes(shell_faces.size()), volume_errors(shell_faces.size());
    for (size_t s = 0; s < shell_faces.size(); ++s) {
        volumes[s] = shell_signed_volume(P, faces, shell_faces[s], &volume_errors[s]);
        d.volume += volumes[s];
    }

    // Same test as Bambu Studio, for closed meshes only: rays from outside
    // must hit front faces first.
    std::vector<int> seen_front(shell_faces.size(), 0), seen_back(shell_faces.size(), 0);
    if (d.open_edges == 0 && d.non_manifold_edges == 0 && d.inconsistent_edges == 0 && !faces.empty()) {
        std::vector<std::pair<int, bool>> hits;
        d.visible_back_faces = size_t(visible_back_faces(P, faces, &hits));
        for (const auto& h : hits)
            (h.second ? seen_back : seen_front)[shell_of[h.first]]++;
    }

    // Inverted shells: closed outermost shells with negative volume. Nested
    // shells may legitimately be inverted (cavities). A self intersecting
    // shell can have a negative volume and still look right from the outside;
    // like Bambu Studio, trust the outside view then.
    const std::vector<int> depth = nesting_depths(P, faces, shell_faces, closed);
    for (size_t s = 0; s < shell_faces.size(); ++s)
        if (closed[s] && depth[s] == 0 && volumes[s] < -volume_errors[s] && !(seen_front[s] > 0 && seen_back[s] == 0)) {
            ++d.inverted_shells;
            if (detail)
                for (int f : shell_faces[s])
                    detail->inverted_shell_faces.push_back(face_id[f]);
        }

    return d;
}

} // namespace meshrepair

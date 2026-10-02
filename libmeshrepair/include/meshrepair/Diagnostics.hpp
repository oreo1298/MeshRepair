// MeshRepair - mesh diagnostics.
//
// The definitions intentionally match the ones used by Bambu Studio
// (libslic3r/MeshDiagnostics.hpp), so that a mesh reported clean here is also
// reported clean by the slicer:
//   open edge           - undirected edge referenced by exactly one face
//   non-manifold edge   - undirected edge referenced by more than two faces
//   non-manifold vertex - vertex whose incident faces do not form a single fan
//                         when connected through two-face edges (bow-tie)
//   reversed faces      - inconsistently oriented neighbours, or a closed shell
//                         that is inside out

#pragma once

#include "Mesh.hpp"

#include <array>
#include <cstddef>
#include <vector>

namespace meshrepair {

struct MeshDiagnostics
{
    size_t vertices = 0;
    size_t faces    = 0;
    size_t shells   = 0; // connected components (faces connected through shared edges)

    // Topological defects.
    size_t open_edges            = 0;
    size_t non_manifold_edges    = 0;
    size_t non_manifold_vertices = 0;
    size_t inconsistent_edges    = 0; // edges whose faces disagree on orientation
    size_t holes                 = 0; // boundary loops formed by open edges

    // Face level defects.
    size_t degenerate_faces = 0; // repeated vertex index or (near) zero area
    size_t duplicate_faces  = 0; // faces using the same three vertices as another face
    size_t invalid_faces    = 0; // vertex index out of range or non finite coordinates

    // Closed, consistently oriented shells that are inside out.
    size_t inverted_shells = 0;

    double      volume = 0.0;
    double      area   = 0.0;
    BoundingBox bbox;

    bool has_reversed_faces() const { return inconsistent_edges > 0 || inverted_shells > 0; }
    bool watertight() const { return open_edges == 0 && non_manifold_edges == 0; }
    bool manifold() const { return non_manifold_edges == 0 && non_manifold_vertices == 0; }

    // True when Bambu Studio / PrusaSlicer would not flag the mesh.
    bool slicer_clean() const { return watertight() && manifold() && !has_reversed_faces(); }

    // True when no defect of any kind was found.
    bool clean() const
    {
        return slicer_clean() && degenerate_faces == 0 && duplicate_faces == 0 && invalid_faces == 0;
    }
};

using Edge = std::array<int, 2>;

// Lists of defective elements, useful for visualisation.
struct DiagnosticsDetail
{
    std::vector<Edge> open_edges;
    std::vector<Edge> non_manifold_edges;
    std::vector<Edge> inconsistent_edges;
    std::vector<int>  non_manifold_vertices;
    std::vector<int>  degenerate_faces;
    std::vector<int>  inverted_shell_faces; // faces belonging to inside-out shells
};

MeshDiagnostics analyze(const Mesh& mesh, DiagnosticsDetail* detail = nullptr);

} // namespace meshrepair

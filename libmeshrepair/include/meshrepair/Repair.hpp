// MeshRepair - automatic mesh repair.
//
// repair() turns an arbitrary triangle soup into a set of closed, manifold,
// consistently and outward oriented shells, which is what slicers need. It is
// the open replacement for the Windows-only "Fix model" (Windows 10 3D
// Printing SDK / netfabb) function of Bambu Studio and PrusaSlicer.

#pragma once

#include "Diagnostics.hpp"
#include "Mesh.hpp"

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace meshrepair {

struct RepairOptions
{
    // Vertices closer than this are merged before anything else happens.
    // Negative value = automatic (1e-6 of the bounding box diagonal, which only
    // merges float rounding noise).
    double weld_tolerance = -1.0;

    // Close cracks: boundary vertices closer than this are merged and boundary
    // vertices lying on another boundary edge (T-junctions) are inserted into
    // that edge. Negative value = automatic (1e-4 of the bounding box diagonal).
    bool   stitch_cracks    = true;
    double stitch_tolerance = -1.0;

    // Fill the remaining holes. Holes with more boundary edges than
    // max_hole_edges are left open (0 = no limit).
    bool   fill_holes     = true;
    size_t max_hole_edges = 0;

    // Orient every closed shell outwards. Shells nested inside other shells keep
    // the orientation the input mostly had (cavity or overlapping solid).
    bool orient_outward = true;

    // Remove closed shells with (practically) zero volume, e.g. leftovers of
    // single sided sheets or internal walls.
    bool remove_zero_volume_shells = true;

    // Remove shells whose volume is smaller than this fraction of the largest
    // shell (0 = keep all shells).
    double min_shell_volume_ratio = 0.0;

    // Flip edges of zero area "cap" triangles so that no degenerate faces remain.
    bool fix_degenerate_faces = true;

    // Splitting non-manifold edges and vertices creates distinct vertices at the
    // same position. Formats like STL do not store connectivity, so a slicer
    // re-merges them by position and sees the defect again. When enabled, such
    // vertices are moved apart by a few micrometres (towards their own side).
    bool separate_coincident_vertices = true;

    // Maximum number of stitch / re-topology passes.
    int max_iterations = 4;
};

struct RepairStats
{
    size_t vertices_welded              = 0;
    size_t invalid_faces_removed        = 0;
    size_t degenerate_faces_removed     = 0;
    size_t duplicate_faces_removed      = 0;
    size_t non_manifold_edges_fixed     = 0;
    size_t non_manifold_vertices_split  = 0;
    size_t faces_flipped                = 0;
    size_t nonorientable_faces_removed  = 0;
    size_t t_junctions_fixed            = 0;
    size_t boundary_vertices_stitched   = 0;
    size_t holes_filled                 = 0;
    size_t holes_left_open              = 0;
    size_t faces_added                  = 0;
    size_t degenerate_faces_fixed       = 0;
    size_t shells_reoriented            = 0;
    size_t shells_removed               = 0;
    size_t vertices_separated           = 0;
};

// Per face flags of the repaired mesh.
enum FaceOrigin : uint8_t {
    FaceOriginal = 0, // face of the input mesh (possibly flipped)
    FaceSplit    = 1, // part of an input face that was split to close a crack
    FaceFilled   = 2, // created while filling a hole
};

struct RepairResult
{
    bool        success  = false;
    bool        canceled = false;
    std::string error;

    MeshDiagnostics before;
    MeshDiagnostics after;
    RepairStats     stats;

    // FaceOrigin for every face of the repaired mesh.
    std::vector<uint8_t> face_origin;

    double weld_tolerance_used   = 0.0;
    double stitch_tolerance_used = 0.0;
};

// stage: human readable description of the current step, percent: 0..100
using ProgressFn = std::function<void(const char* stage, int percent)>;
// Return true to abort the repair. The mesh is left untouched when canceled.
using CancelFn = std::function<bool()>;

RepairResult repair(Mesh& mesh, const RepairOptions& options = {}, const ProgressFn& progress = {},
                    const CancelFn& cancel = {});

// Human readable multi line summary of what repair() did.
std::string format_report(const RepairResult& result);
std::string format_diagnostics(const MeshDiagnostics& diag);

} // namespace meshrepair

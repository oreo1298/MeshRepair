// "Fix model" backend for platforms without the Windows 10 3D printing SDK,
// implemented with MeshRepair (src/meshrepair). It provides the same entry
// points as the Windows implementation, see Win10ModelRepair.hpp.

#include "Win10ModelRepair.hpp"

#if defined(HAS_MESHREPAIR) && !defined(HAS_WIN10SDK)

#include <meshrepair/Repair.hpp>

#include <boost/log/trivial.hpp>

namespace Slic3r {

bool is_win10_model_repair_available()
{
    return true;
}

bool fix_mesh_by_win10_sdk(const indexed_triangle_set& mesh,
                           indexed_triangle_set&       repaired_mesh,
                           Win10RepairProgressFn       progress_callback,
                           Win10RepairCancelFn         cancel_callback,
                           std::string*                error_message)
{
    meshrepair::Mesh m;
    m.vertices.reserve(mesh.vertices.size());
    for (const stl_vertex& v : mesh.vertices)
        m.vertices.emplace_back(double(v.x()), double(v.y()), double(v.z()));
    m.faces.reserve(mesh.indices.size());
    for (const stl_triangle_vertex_indices& f : mesh.indices)
        m.faces.push_back({ f(0), f(1), f(2) });

    const meshrepair::RepairResult res = meshrepair::repair(
        m, meshrepair::RepairOptions(),
        [&progress_callback](const char* stage, int percent) {
            if (progress_callback)
                progress_callback(stage, unsigned(percent));
        },
        [&cancel_callback]() { return cancel_callback && cancel_callback(); });

    if (!res.success) {
        if (error_message)
            *error_message = res.error;
        BOOST_LOG_TRIVIAL(info) << "MeshRepair: repair " << (res.canceled ? "canceled" : "failed: " + res.error);
        return false;
    }

    BOOST_LOG_TRIVIAL(info) << "MeshRepair: " << meshrepair::format_report(res);

    repaired_mesh.clear();
    repaired_mesh.vertices.reserve(m.vertices.size());
    for (const meshrepair::Vec3& v : m.vertices)
        repaired_mesh.vertices.emplace_back(float(v.x), float(v.y), float(v.z));
    repaired_mesh.indices.reserve(m.faces.size());
    for (const meshrepair::Triangle& f : m.faces)
        repaired_mesh.indices.emplace_back(f[0], f[1], f[2]);
    return true;
}

} // namespace Slic3r

#endif // HAS_MESHREPAIR && !HAS_WIN10SDK

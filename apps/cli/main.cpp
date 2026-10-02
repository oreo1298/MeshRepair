// meshrepair - command line mesh repair tool.

#include <meshrepair/Diagnostics.hpp>
#include <meshrepair/IO.hpp>
#include <meshrepair/Repair.hpp>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <sstream>
#include <string>
#include <vector>

using namespace meshrepair;

namespace {

const char* kUsage =
    "Usage: meshrepair [options] <input> [<input> ...]\n"
    "\n"
    "Repairs triangle meshes for 3D printing: fixes non-manifold edges and\n"
    "vertices, open edges (holes, cracks), flipped normals, duplicate and\n"
    "degenerate faces. Supported formats: STL (ascii/binary), OBJ, 3MF.\n"
    "\n"
    "Options:\n"
    "  -o, --output FILE     output file (default: <input>_repaired.<ext>);\n"
    "                        only valid with a single input\n"
    "  -i, --in-place        overwrite the input file(s)\n"
    "  -c, --check           only analyze, do not write anything\n"
    "      --ascii           write ASCII STL instead of binary\n"
    "      --json            print the report as JSON\n"
    "  -q, --quiet           print nothing but errors\n"
    "\n"
    "Repair options:\n"
    "      --weld-tol MM     merge vertices closer than MM (default: auto)\n"
    "      --stitch-tol MM   close cracks narrower than MM (default: auto)\n"
    "      --no-stitch       do not close cracks\n"
    "      --no-fill         do not fill holes\n"
    "      --max-hole N      only fill holes with at most N edges\n"
    "      --no-orient       do not orient shells outwards\n"
    "      --keep-zero-volume  keep zero volume shells\n"
    "      --min-shell-ratio R  remove shells smaller than R * largest volume\n"
    "      --no-separate     do not move apart vertices of touching shells\n"
    "\n"
    "Exit status: 0 = result is clean, 2 = issues remain / were found,\n"
    "             1 = error.\n";

std::string default_output(const std::string& in)
{
    const size_t slash = in.find_last_of('/');
    const size_t dot   = in.find_last_of('.');
    if (dot == std::string::npos || (slash != std::string::npos && dot < slash))
        return in + "_repaired.stl";
    return in.substr(0, dot) + "_repaired" + in.substr(dot);
}

std::string json_escape(const std::string& s)
{
    std::string out;
    for (char c : s) {
        switch (c) {
        case '"': out += "\\\""; break;
        case '\\': out += "\\\\"; break;
        case '\n': out += "\\n"; break;
        case '\t': out += "\\t"; break;
        default:
            if ((unsigned char)c < 0x20) {
                char buf[8];
                std::snprintf(buf, sizeof(buf), "\\u%04x", c);
                out += buf;
            } else
                out += c;
        }
    }
    return out;
}

std::string json_diag(const MeshDiagnostics& d)
{
    std::ostringstream ss;
    ss << "{\"vertices\":" << d.vertices << ",\"faces\":" << d.faces << ",\"shells\":" << d.shells
       << ",\"open_edges\":" << d.open_edges << ",\"holes\":" << d.holes
       << ",\"non_manifold_edges\":" << d.non_manifold_edges
       << ",\"non_manifold_vertices\":" << d.non_manifold_vertices
       << ",\"inconsistent_edges\":" << d.inconsistent_edges << ",\"inverted_shells\":" << d.inverted_shells
       << ",\"degenerate_faces\":" << d.degenerate_faces << ",\"duplicate_faces\":" << d.duplicate_faces
       << ",\"invalid_faces\":" << d.invalid_faces << ",\"volume\":" << d.volume << ",\"area\":" << d.area
       << ",\"watertight\":" << (d.watertight() ? "true" : "false")
       << ",\"slicer_clean\":" << (d.slicer_clean() ? "true" : "false")
       << ",\"clean\":" << (d.clean() ? "true" : "false") << "}";
    return ss.str();
}

std::string json_stats(const RepairStats& s)
{
    std::ostringstream ss;
    ss << "{\"vertices_welded\":" << s.vertices_welded << ",\"invalid_faces_removed\":" << s.invalid_faces_removed
       << ",\"degenerate_faces_removed\":" << s.degenerate_faces_removed
       << ",\"degenerate_faces_fixed\":" << s.degenerate_faces_fixed
       << ",\"duplicate_faces_removed\":" << s.duplicate_faces_removed
       << ",\"non_manifold_edges_fixed\":" << s.non_manifold_edges_fixed
       << ",\"vertices_split\":" << s.non_manifold_vertices_split << ",\"faces_flipped\":" << s.faces_flipped
       << ",\"unresolvable_faces_removed\":" << s.nonorientable_faces_removed
       << ",\"t_junctions_fixed\":" << s.t_junctions_fixed
       << ",\"boundary_vertices_stitched\":" << s.boundary_vertices_stitched << ",\"holes_filled\":" << s.holes_filled
       << ",\"holes_left_open\":" << s.holes_left_open << ",\"faces_added\":" << s.faces_added
       << ",\"shells_reoriented\":" << s.shells_reoriented << ",\"shells_removed\":" << s.shells_removed
       << ",\"vertices_separated\":" << s.vertices_separated << "}";
    return ss.str();
}

bool parse_double(const char* s, double& out)
{
    char* e;
    out = std::strtod(s, &e);
    return e != s && *e == '\0';
}

} // namespace

int main(int argc, char** argv)
{
    std::vector<std::string> inputs;
    std::string              output;
    bool                     check = false, ascii = false, json = false, quiet = false, in_place = false;
    RepairOptions            opt;

    for (int i = 1; i < argc; ++i) {
        const std::string a    = argv[i];
        auto              next = [&](const char* what) -> const char* {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "meshrepair: %s expects a value\n", what);
                std::exit(1);
            }
            return argv[++i];
        };
        auto number = [&](const char* what) {
            double v;
            if (!parse_double(next(what), v)) {
                std::fprintf(stderr, "meshrepair: invalid number for %s\n", what);
                std::exit(1);
            }
            return v;
        };
        if (a == "-h" || a == "--help") {
            std::fputs(kUsage, stdout);
            return 0;
        } else if (a == "--version") {
            std::printf("meshrepair 0.1.0\n");
            return 0;
        } else if (a == "-o" || a == "--output")
            output = next("--output");
        else if (a == "-i" || a == "--in-place")
            in_place = true;
        else if (a == "-c" || a == "--check")
            check = true;
        else if (a == "--ascii")
            ascii = true;
        else if (a == "--json")
            json = true;
        else if (a == "-q" || a == "--quiet")
            quiet = true;
        else if (a == "--weld-tol")
            opt.weld_tolerance = number("--weld-tol");
        else if (a == "--stitch-tol")
            opt.stitch_tolerance = number("--stitch-tol");
        else if (a == "--no-stitch")
            opt.stitch_cracks = false;
        else if (a == "--no-fill")
            opt.fill_holes = false;
        else if (a == "--max-hole")
            opt.max_hole_edges = size_t(number("--max-hole"));
        else if (a == "--no-orient")
            opt.orient_outward = false;
        else if (a == "--keep-zero-volume")
            opt.remove_zero_volume_shells = false;
        else if (a == "--min-shell-ratio")
            opt.min_shell_volume_ratio = number("--min-shell-ratio");
        else if (a == "--no-separate")
            opt.separate_coincident_vertices = false;
        else if (!a.empty() && a[0] == '-' && a != "-") {
            std::fprintf(stderr, "meshrepair: unknown option %s (see --help)\n", a.c_str());
            return 1;
        } else
            inputs.push_back(a);
    }
    if (inputs.empty()) {
        std::fputs(kUsage, stderr);
        return 1;
    }
    if (!output.empty() && inputs.size() > 1) {
        std::fprintf(stderr, "meshrepair: --output can only be used with a single input\n");
        return 1;
    }

    int  status = 0;
    bool first  = true;
    if (json)
        std::printf("[");
    for (const std::string& in : inputs) {
        std::vector<NamedMesh> objects;
        std::string            error;
        if (!load_objects(in, objects, error)) {
            std::fprintf(stderr, "meshrepair: %s\n", error.c_str());
            status = 1;
            continue;
        }
        if (json)
            std::printf("%s{\"file\":\"%s\",\"objects\":[", first ? "" : ",", json_escape(in).c_str());
        first             = false;
        bool issues       = false;
        bool first_object = true;
        for (NamedMesh& obj : objects) {
            if (check) {
                const MeshDiagnostics d = analyze(obj.mesh);
                issues |= !d.slicer_clean();
                if (json)
                    std::printf("%s{\"name\":\"%s\",\"diagnostics\":%s}", first_object ? "" : ",",
                                json_escape(obj.name).c_str(), json_diag(d).c_str());
                else if (!quiet)
                    std::printf("%s%s:\n%s", in.c_str(), objects.size() > 1 ? (" [" + obj.name + "]").c_str() : "",
                                format_diagnostics(d).c_str());
            } else {
                const auto   t0  = std::chrono::steady_clock::now();
                RepairResult res = repair(obj.mesh, opt);
                const double sec =
                    std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
                if (!res.success) {
                    std::fprintf(stderr, "meshrepair: %s: %s\n", in.c_str(), res.error.c_str());
                    status = 1;
                    continue;
                }
                issues |= !res.after.slicer_clean();
                if (json)
                    std::printf("%s{\"name\":\"%s\",\"seconds\":%.3f,\"before\":%s,\"after\":%s,\"repairs\":%s}",
                                first_object ? "" : ",", json_escape(obj.name).c_str(), sec,
                                json_diag(res.before).c_str(), json_diag(res.after).c_str(),
                                json_stats(res.stats).c_str());
                else if (!quiet)
                    std::printf("%s%s (%.2f s):\n%s", in.c_str(),
                                objects.size() > 1 ? (" [" + obj.name + "]").c_str() : "", sec,
                                format_report(res).c_str());
            }
            first_object = false;
        }
        if (json)
            std::printf("]");
        if (!check) {
            const std::string out = in_place ? in : (output.empty() ? default_output(in) : output);
            FileFormat        fmt = format_from_path(out);
            if (fmt == FileFormat::STL && ascii)
                fmt = FileFormat::STLAscii;
            if (!save_objects(out, objects, error, fmt)) {
                std::fprintf(stderr, "meshrepair: %s\n", error.c_str());
                status = 1;
            } else if (json)
                std::printf(",\"output\":\"%s\"", json_escape(out).c_str());
            else if (!quiet)
                std::printf("Written: %s\n", out.c_str());
        }
        if (json)
            std::printf("}");
        if (issues && status == 0)
            status = 2;
    }
    if (json)
        std::printf("]\n");
    return status;
}

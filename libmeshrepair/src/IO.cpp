#include "meshrepair/IO.hpp"

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <unordered_map>

namespace meshrepair {

namespace {

std::string lower(std::string s)
{
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return char(std::tolower(c)); });
    return s;
}

bool read_file(const std::string& path, std::string& data, std::string& error)
{
    std::ifstream f(path, std::ios::binary);
    if (!f) {
        error = "Cannot open file: " + path;
        return false;
    }
    f.seekg(0, std::ios::end);
    const std::streamoff size = f.tellg();
    if (size < 0) {
        error = "Cannot read file: " + path;
        return false;
    }
    f.seekg(0, std::ios::beg);
    data.resize(size_t(size));
    if (size > 0 && !f.read(&data[0], size)) {
        error = "Cannot read file: " + path;
        return false;
    }
    return true;
}

uint32_t read_u32_le(const unsigned char* p)
{
    return uint32_t(p[0]) | (uint32_t(p[1]) << 8) | (uint32_t(p[2]) << 16) | (uint32_t(p[3]) << 24);
}

float read_f32_le(const unsigned char* p)
{
    const uint32_t u = read_u32_le(p);
    float          f;
    std::memcpy(&f, &u, 4);
    return f;
}

void write_u32_le(unsigned char* p, uint32_t v)
{
    p[0] = uint8_t(v);
    p[1] = uint8_t(v >> 8);
    p[2] = uint8_t(v >> 16);
    p[3] = uint8_t(v >> 24);
}

void write_f32_le(unsigned char* p, float f)
{
    uint32_t u;
    std::memcpy(&u, &f, 4);
    write_u32_le(p, u);
}

// Builds an indexed mesh from a triangle soup, merging identical positions.
class SoupBuilder
{
public:
    void add_triangle(const float* v9)
    {
        Triangle t;
        for (int k = 0; k < 3; ++k)
            t[k] = vertex(v9[3 * k], v9[3 * k + 1], v9[3 * k + 2]);
        mesh.faces.push_back(t);
    }
    Mesh mesh;

private:
    struct Key
    {
        uint32_t x, y, z;
        bool     operator==(const Key& o) const { return x == o.x && y == o.y && z == o.z; }
    };
    struct KeyHash
    {
        size_t operator()(const Key& k) const
        {
            uint64_t h = k.x * 0x9E3779B97F4A7C15ull;
            h ^= (k.y + 0x7F4A7C15ull) * 0xC2B2AE3D27D4EB4Full + (h << 7);
            h ^= (k.z + 0x165667B1ull) * 0x165667B19E3779F9ull + (h >> 3);
            return size_t(h ^ (h >> 29));
        }
    };
    static uint32_t bits(float f)
    {
        f += 0.0f; // -0 -> +0
        uint32_t u;
        std::memcpy(&u, &f, 4);
        return u;
    }
    int vertex(float x, float y, float z)
    {
        const Key k { bits(x), bits(y), bits(z) };
        auto      it = m_map.find(k);
        if (it != m_map.end())
            return it->second;
        const int id = int(mesh.vertices.size());
        mesh.vertices.emplace_back(double(x), double(y), double(z));
        m_map.emplace(k, id);
        return id;
    }
    std::unordered_map<Key, int, KeyHash> m_map;
};

bool parse_ascii_stl(const std::string& data, std::vector<NamedMesh>& objects)
{
    const char* p   = data.c_str();
    const char* end = p + data.size();
    auto        skip_ws = [&]() {
        while (p < end && std::isspace((unsigned char)*p))
            ++p;
    };
    auto token = [&](const char*& b) -> size_t {
        skip_ws();
        b = p;
        while (p < end && !std::isspace((unsigned char)*p))
            ++p;
        return size_t(p - b);
    };
    auto is = [](const char* b, size_t n, const char* kw) { return n == std::strlen(kw) && std::strncmp(b, kw, n) == 0; };

    SoupBuilder        builder;
    std::string        name;
    std::vector<float> poly;
    size_t             facets = 0;
    bool               in_solid = false;
    auto               flush = [&]() {
        if (!builder.mesh.faces.empty())
            objects.push_back({ name, std::move(builder.mesh) });
        builder = SoupBuilder();
    };
    while (p < end) {
        const char* b;
        const size_t n = token(b);
        if (n == 0)
            break;
        if (is(b, n, "solid")) {
            if (in_solid)
                flush();
            in_solid = true;
            // Rest of the line is the name.
            const char* e = p;
            while (e < end && *e != '\n' && *e != '\r')
                ++e;
            name.assign(p, e);
            name.erase(0, name.find_first_not_of(" \t"));
            name.erase(name.find_last_not_of(" \t") + 1);
            p = e;
        } else if (is(b, n, "endsolid")) {
            while (p < end && *p != '\n')
                ++p;
            flush();
            in_solid = false;
        } else if (is(b, n, "facet") || is(b, n, "outer") || is(b, n, "loop") || is(b, n, "normal")) {
            poly.clear();
        } else if (is(b, n, "vertex")) {
            for (int k = 0; k < 3; ++k) {
                skip_ws();
                char*        e = nullptr;
                const double v = std::strtod(p, &e);
                if (e == p)
                    return false;
                p = e;
                poly.push_back(float(v));
            }
        } else if (is(b, n, "endloop")) {
            // Fan triangulation for (non-standard) polygons.
            const size_t nv = poly.size() / 3;
            for (size_t k = 1; k + 1 < nv; ++k) {
                float t[9];
                std::memcpy(t, &poly[0], 3 * sizeof(float));
                std::memcpy(t + 3, &poly[3 * k], 3 * sizeof(float));
                std::memcpy(t + 6, &poly[3 * (k + 1)], 3 * sizeof(float));
                builder.add_triangle(t);
            }
            ++facets;
            poly.clear();
        }
        // Numbers following "normal" and unknown tokens are skipped.
    }
    flush();
    return facets > 0;
}

bool parse_binary_stl(const std::string& data, std::vector<NamedMesh>& objects, std::string& error)
{
    if (data.size() < 84) {
        error = "File too small to be an STL file";
        return false;
    }
    const unsigned char* p = reinterpret_cast<const unsigned char*>(data.data());
    size_t               n = read_u32_le(p + 80);
    if (84 + n * 50 > data.size())
        n = (data.size() - 84) / 50; // truncated file: read what is there
    SoupBuilder builder;
    builder.mesh.faces.reserve(n);
    builder.mesh.vertices.reserve(n / 2 + 16);
    for (size_t i = 0; i < n; ++i) {
        const unsigned char* f = p + 84 + i * 50 + 12;
        float                v[9];
        for (int k = 0; k < 9; ++k)
            v[k] = read_f32_le(f + 4 * k);
        builder.add_triangle(v);
    }
    std::string name(reinterpret_cast<const char*>(p), 80);
    name = name.substr(0, name.find('\0'));
    objects.push_back({ name, std::move(builder.mesh) });
    return true;
}

bool load_stl_objects(const std::string& path, std::vector<NamedMesh>& objects, std::string& error)
{
    std::string data;
    if (!read_file(path, data, error))
        return false;
    bool binary = false;
    if (data.size() >= 84) {
        const uint64_t n = read_u32_le(reinterpret_cast<const unsigned char*>(data.data()) + 80);
        binary           = 84 + n * 50 == data.size();
    }
    if (!binary) {
        size_t i = 0;
        while (i < data.size() && std::isspace((unsigned char)data[i]))
            ++i;
        if (data.compare(i, 5, "solid") == 0 && parse_ascii_stl(data, objects))
            return true;
        objects.clear();
    }
    return parse_binary_stl(data, objects, error);
}

} // namespace

void merge_identical_vertices(Mesh& mesh)
{
    std::vector<int> order(mesh.vertices.size());
    for (size_t i = 0; i < order.size(); ++i)
        order[i] = int(i);
    auto less = [&](int a, int b) {
        const Vec3& p = mesh.vertices[a];
        const Vec3& q = mesh.vertices[b];
        if (p.x != q.x)
            return p.x < q.x;
        if (p.y != q.y)
            return p.y < q.y;
        if (p.z != q.z)
            return p.z < q.z;
        return a < b;
    };
    std::sort(order.begin(), order.end(), less);
    std::vector<int> remap(mesh.vertices.size());
    for (size_t i = 0; i < order.size(); ++i) {
        if (i > 0 && mesh.vertices[order[i]] == mesh.vertices[order[i - 1]])
            remap[order[i]] = remap[order[i - 1]];
        else
            remap[order[i]] = order[i];
    }
    for (Triangle& t : mesh.faces)
        for (int& v : t)
            if (v >= 0 && size_t(v) < remap.size())
                v = remap[v];
    remove_unreferenced_vertices(mesh);
}

FileFormat format_from_path(const std::string& path)
{
    const size_t dot = path.find_last_of('.');
    if (dot == std::string::npos)
        return FileFormat::Unknown;
    const std::string ext = lower(path.substr(dot + 1));
    if (ext == "stl")
        return FileFormat::STL;
    if (ext == "obj")
        return FileFormat::OBJ;
    if (ext == "3mf")
        return FileFormat::ThreeMF;
    return FileFormat::Unknown;
}

bool load_stl(const std::string& path, Mesh& mesh, std::string& error)
{
    std::vector<NamedMesh> objects;
    if (!load_stl_objects(path, objects, error))
        return false;
    mesh.clear();
    for (NamedMesh& o : objects) {
        const int off = int(mesh.vertices.size());
        mesh.vertices.insert(mesh.vertices.end(), o.mesh.vertices.begin(), o.mesh.vertices.end());
        for (Triangle t : o.mesh.faces)
            mesh.faces.push_back({ t[0] + off, t[1] + off, t[2] + off });
    }
    if (objects.size() > 1)
        merge_identical_vertices(mesh);
    return true;
}

bool save_stl(const std::string& path, const Mesh& mesh, bool ascii, std::string& error, const std::string& name)
{
    std::FILE* f = std::fopen(path.c_str(), ascii ? "w" : "wb");
    if (!f) {
        error = "Cannot write file: " + path;
        return false;
    }
    bool ok = true;
    if (ascii) {
        std::fprintf(f, "solid %s\n", name.c_str());
        for (size_t i = 0; i < mesh.faces.size(); ++i) {
            const Vec3 n = face_normal(mesh, i);
            std::fprintf(f, "  facet normal %.9g %.9g %.9g\n    outer loop\n", float(n.x), float(n.y), float(n.z));
            for (int v : mesh.faces[i]) {
                const Vec3& p = mesh.vertices[v];
                std::fprintf(f, "      vertex %.9g %.9g %.9g\n", float(p.x), float(p.y), float(p.z));
            }
            std::fprintf(f, "    endloop\n  endfacet\n");
        }
        std::fprintf(f, "endsolid %s\n", name.c_str());
    } else {
        unsigned char header[84] = {};
        const char*   text       = "Binary STL written by MeshRepair";
        std::memcpy(header, text, std::strlen(text));
        write_u32_le(header + 80, uint32_t(mesh.faces.size()));
        ok = std::fwrite(header, 1, 84, f) == 84;
        std::vector<unsigned char> buf;
        const size_t               chunk = 4096;
        buf.resize(chunk * 50);
        for (size_t i = 0; ok && i < mesh.faces.size(); i += chunk) {
            const size_t cnt = std::min(chunk, mesh.faces.size() - i);
            for (size_t j = 0; j < cnt; ++j) {
                unsigned char* rec = &buf[j * 50];
                // Normal computed from the single precision coordinates.
                float v[3][3];
                for (int k = 0; k < 3; ++k) {
                    const Vec3& p = mesh.vertices[mesh.faces[i + j][k]];
                    v[k][0]       = float(p.x);
                    v[k][1]       = float(p.y);
                    v[k][2]       = float(p.z);
                }
                const Vec3 n = triangle_normal(Vec3(v[0][0], v[0][1], v[0][2]), Vec3(v[1][0], v[1][1], v[1][2]),
                                               Vec3(v[2][0], v[2][1], v[2][2]))
                                   .normalized();
                write_f32_le(rec, float(n.x));
                write_f32_le(rec + 4, float(n.y));
                write_f32_le(rec + 8, float(n.z));
                for (int k = 0; k < 3; ++k)
                    for (int c = 0; c < 3; ++c)
                        write_f32_le(rec + 12 + 12 * k + 4 * c, v[k][c]);
                rec[48] = rec[49] = 0;
            }
            ok = std::fwrite(buf.data(), 1, cnt * 50, f) == cnt * 50;
        }
    }
    ok = (std::fclose(f) == 0) && ok;
    if (!ok)
        error = "Error while writing file: " + path;
    return ok;
}

bool load_obj(const std::string& path, std::vector<NamedMesh>& objects, std::string& error)
{
    std::string data;
    if (!read_file(path, data, error))
        return false;
    std::vector<Vec3> positions;
    struct Object
    {
        std::string           name;
        std::vector<Triangle> faces; // global position indices
    };
    std::vector<Object> objs(1);
    std::vector<int>    poly;
    size_t              line_no = 0;
    size_t              pos     = 0;
    while (pos < data.size()) {
        size_t eol = data.find('\n', pos);
        if (eol == std::string::npos)
            eol = data.size();
        std::string line = data.substr(pos, eol - pos);
        pos              = eol + 1;
        ++line_no;
        // Line continuation.
        while (!line.empty() && (line.back() == '\r' || line.back() == ' ' || line.back() == '\t'))
            line.pop_back();
        while (!line.empty() && line.back() == '\\' && pos < data.size()) {
            line.pop_back();
            size_t e2 = data.find('\n', pos);
            if (e2 == std::string::npos)
                e2 = data.size();
            line += data.substr(pos, e2 - pos);
            pos = e2 + 1;
            while (!line.empty() && (line.back() == '\r'))
                line.pop_back();
        }
        const char* p = line.c_str();
        while (*p == ' ' || *p == '\t')
            ++p;
        if (p[0] == 'v' && (p[1] == ' ' || p[1] == '\t')) {
            char*  e;
            double c[3] = { 0, 0, 0 };
            p += 2;
            for (int k = 0; k < 3; ++k) {
                c[k] = std::strtod(p, &e);
                if (e == p) {
                    error = "Invalid vertex on line " + std::to_string(line_no);
                    return false;
                }
                p = e;
            }
            positions.emplace_back(c[0], c[1], c[2]);
        } else if (p[0] == 'f' && (p[1] == ' ' || p[1] == '\t')) {
            p += 2;
            poly.clear();
            while (*p) {
                while (*p == ' ' || *p == '\t')
                    ++p;
                if (!*p)
                    break;
                char*      e;
                const long idx = std::strtol(p, &e, 10);
                if (e == p) {
                    error = "Invalid face on line " + std::to_string(line_no);
                    return false;
                }
                p = e;
                while (*p && *p != ' ' && *p != '\t')
                    ++p; // skip /vt/vn
                long v = idx > 0 ? idx - 1 : long(positions.size()) + idx;
                poly.push_back(int(v));
            }
            for (size_t k = 1; k + 1 < poly.size(); ++k)
                objs.back().faces.push_back({ poly[0], poly[k], poly[k + 1] });
        } else if (p[0] == 'o' && (p[1] == ' ' || p[1] == '\t')) {
            std::string name(p + 2);
            name.erase(0, name.find_first_not_of(" \t"));
            if (!objs.back().faces.empty())
                objs.emplace_back();
            objs.back().name = name;
        }
    }
    objects.clear();
    for (Object& o : objs) {
        if (o.faces.empty())
            continue;
        NamedMesh nm;
        nm.name = o.name;
        std::unordered_map<int, int> remap;
        for (const Triangle& t : o.faces) {
            Triangle out;
            for (int k = 0; k < 3; ++k) {
                const int g = t[k];
                if (g < 0 || size_t(g) >= positions.size()) {
                    error = "Face references a missing vertex";
                    return false;
                }
                auto it = remap.find(g);
                if (it == remap.end()) {
                    it = remap.emplace(g, int(nm.mesh.vertices.size())).first;
                    nm.mesh.vertices.push_back(positions[g]);
                }
                out[k] = it->second;
            }
            nm.mesh.faces.push_back(out);
        }
        merge_identical_vertices(nm.mesh);
        objects.push_back(std::move(nm));
    }
    if (objects.empty()) {
        error = "No faces found in OBJ file";
        return false;
    }
    return true;
}

bool save_obj(const std::string& path, const std::vector<NamedMesh>& objects, std::string& error)
{
    std::FILE* f = std::fopen(path.c_str(), "w");
    if (!f) {
        error = "Cannot write file: " + path;
        return false;
    }
    std::fprintf(f, "# Written by MeshRepair\n");
    size_t offset = 1;
    for (const NamedMesh& o : objects) {
        if (!o.name.empty())
            std::fprintf(f, "o %s\n", o.name.c_str());
        for (const Vec3& p : o.mesh.vertices)
            std::fprintf(f, "v %.9g %.9g %.9g\n", p.x, p.y, p.z);
        for (const Triangle& t : o.mesh.faces)
            std::fprintf(f, "f %zu %zu %zu\n", t[0] + offset, t[1] + offset, t[2] + offset);
        offset += o.mesh.vertices.size();
    }
    if (std::fclose(f) != 0) {
        error = "Error while writing file: " + path;
        return false;
    }
    return true;
}

bool load_objects(const std::string& path, std::vector<NamedMesh>& objects, std::string& error)
{
    objects.clear();
    switch (format_from_path(path)) {
    case FileFormat::OBJ: return load_obj(path, objects, error);
    case FileFormat::ThreeMF: return load_3mf(path, objects, error);
    default: {
        NamedMesh nm;
        if (!load_stl(path, nm.mesh, error))
            return false;
        const size_t slash = path.find_last_of("/\\");
        nm.name            = path.substr(slash == std::string::npos ? 0 : slash + 1);
        const size_t dot   = nm.name.find_last_of('.');
        if (dot != std::string::npos)
            nm.name.erase(dot);
        objects.push_back(std::move(nm));
        return true;
    }
    }
}

bool load_mesh(const std::string& path, Mesh& mesh, std::string& error)
{
    std::vector<NamedMesh> objects;
    if (!load_objects(path, objects, error))
        return false;
    mesh.clear();
    for (const NamedMesh& o : objects) {
        const int off = int(mesh.vertices.size());
        mesh.vertices.insert(mesh.vertices.end(), o.mesh.vertices.begin(), o.mesh.vertices.end());
        for (Triangle t : o.mesh.faces)
            mesh.faces.push_back({ t[0] + off, t[1] + off, t[2] + off });
    }
    return true;
}

bool save_objects(const std::string& path, const std::vector<NamedMesh>& objects, std::string& error, FileFormat format)
{
    if (format == FileFormat::Unknown)
        format = format_from_path(path);
    switch (format) {
    case FileFormat::OBJ: return save_obj(path, objects, error);
    case FileFormat::ThreeMF: return save_3mf(path, objects, error);
    case FileFormat::STL:
    case FileFormat::STLAscii: {
        Mesh merged;
        for (const NamedMesh& o : objects) {
            const int off = int(merged.vertices.size());
            merged.vertices.insert(merged.vertices.end(), o.mesh.vertices.begin(), o.mesh.vertices.end());
            for (Triangle t : o.mesh.faces)
                merged.faces.push_back({ t[0] + off, t[1] + off, t[2] + off });
        }
        return save_stl(path, merged, format == FileFormat::STLAscii, error,
                        objects.size() == 1 && !objects[0].name.empty() ? objects[0].name : "meshrepair");
    }
    default: error = "Unknown output format (use .stl, .obj or .3mf): " + path; return false;
    }
}

bool save_mesh(const std::string& path, const Mesh& mesh, std::string& error, FileFormat format)
{
    std::string name = path;
    const size_t slash = name.find_last_of("/\\");
    if (slash != std::string::npos)
        name.erase(0, slash + 1);
    const size_t dot = name.find_last_of('.');
    if (dot != std::string::npos)
        name.erase(dot);
    std::vector<NamedMesh> objects { { name, mesh } };
    return save_objects(path, objects, error, format);
}

} // namespace meshrepair

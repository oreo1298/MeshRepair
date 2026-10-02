// 3MF reading and writing.
//
// Supports the core specification (meshes, components, build items,
// transforms, units) and the production extension's p:path components, which
// Bambu Studio, OrcaSlicer and PrusaSlicer use to store objects in separate
// model parts (3D/Objects/*.model).

#include "meshrepair/IO.hpp"

#include "Zip.hpp"

#include <array>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <memory>
#include <set>
#include <sstream>

namespace meshrepair {

namespace {

// Affine transform, 3MF convention: p' = p * M (row vector), stored as 3x4.
struct Transform
{
    // m[r][c] for the rotation part, t for translation.
    double m[3][3] = { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } };
    double t[3]    = { 0, 0, 0 };

    Vec3 apply(const Vec3& p) const
    {
        return { p.x * m[0][0] + p.y * m[1][0] + p.z * m[2][0] + t[0],
                 p.x * m[0][1] + p.y * m[1][1] + p.z * m[2][1] + t[1],
                 p.x * m[0][2] + p.y * m[1][2] + p.z * m[2][2] + t[2] };
    }
    // First this, then o.
    Transform then(const Transform& o) const
    {
        Transform r;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                r.m[i][j] = m[i][0] * o.m[0][j] + m[i][1] * o.m[1][j] + m[i][2] * o.m[2][j];
        for (int j = 0; j < 3; ++j)
            r.t[j] = t[0] * o.m[0][j] + t[1] * o.m[1][j] + t[2] * o.m[2][j] + o.t[j];
        return r;
    }
    double determinant() const
    {
        return m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
               m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
    }
    static bool parse(const std::string& s, Transform& out)
    {
        double      v[12];
        const char* p = s.c_str();
        for (int i = 0; i < 12; ++i) {
            char* e;
            v[i] = std::strtod(p, &e);
            if (e == p)
                return false;
            p = e;
        }
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
                out.m[r][c] = v[r * 3 + c];
        out.t[0] = v[9];
        out.t[1] = v[10];
        out.t[2] = v[11];
        return true;
    }
};

// --- Tiny XML tokenizer, sufficient for 3MF model parts --------------------

struct Tag
{
    std::string                        name; // without namespace prefix
    bool                               closing    = false; // </name>
    bool                               self_close = false; // <name/>
    std::map<std::string, std::string> attrs;              // local names, prefixes kept for p:path etc.
    std::string attr(const std::string& k, const std::string& def = "") const
    {
        auto it = attrs.find(k);
        return it == attrs.end() ? def : it->second;
    }
};

std::string xml_unescape(const std::string& s)
{
    if (s.find('&') == std::string::npos)
        return s;
    std::string out;
    for (size_t i = 0; i < s.size(); ++i) {
        if (s[i] != '&') {
            out += s[i];
            continue;
        }
        const size_t semi = s.find(';', i);
        if (semi == std::string::npos) {
            out += s[i];
            continue;
        }
        const std::string ent = s.substr(i + 1, semi - i - 1);
        if (ent == "amp")
            out += '&';
        else if (ent == "lt")
            out += '<';
        else if (ent == "gt")
            out += '>';
        else if (ent == "quot")
            out += '"';
        else if (ent == "apos")
            out += '\'';
        else if (!ent.empty() && ent[0] == '#') {
            const unsigned long cp = ent.size() > 1 && (ent[1] == 'x' || ent[1] == 'X')
                                         ? std::strtoul(ent.c_str() + 2, nullptr, 16)
                                         : std::strtoul(ent.c_str() + 1, nullptr, 10);
            if (cp < 0x80)
                out += char(cp);
            else if (cp < 0x800) {
                out += char(0xC0 | (cp >> 6));
                out += char(0x80 | (cp & 0x3F));
            } else if (cp < 0x10000) {
                out += char(0xE0 | (cp >> 12));
                out += char(0x80 | ((cp >> 6) & 0x3F));
                out += char(0x80 | (cp & 0x3F));
            } else {
                out += char(0xF0 | (cp >> 18));
                out += char(0x80 | ((cp >> 12) & 0x3F));
                out += char(0x80 | ((cp >> 6) & 0x3F));
                out += char(0x80 | (cp & 0x3F));
            }
        } else
            out += s.substr(i, semi - i + 1);
        i = semi;
    }
    return out;
}

std::string xml_escape(const std::string& s)
{
    std::string out;
    for (char c : s) {
        switch (c) {
        case '&': out += "&amp;"; break;
        case '<': out += "&lt;"; break;
        case '>': out += "&gt;"; break;
        case '"': out += "&quot;"; break;
        default: out += c;
        }
    }
    return out;
}

class XmlScanner
{
public:
    explicit XmlScanner(const std::string& s) : m_s(s) {}

    // Next element tag without parsing its attributes. attrs_begin/end span
    // the raw attribute text. Returns false at the end of the document.
    bool next_raw(std::string& name, bool& closing, bool& self_close, size_t& attrs_begin, size_t& attrs_end)
    {
        for (;;) {
            const size_t lt = m_s.find('<', m_pos);
            if (lt == std::string::npos)
                return false;
            m_pos = lt + 1;
            if (m_s.compare(m_pos, 3, "!--") == 0) {
                const size_t e = m_s.find("-->", m_pos);
                m_pos          = e == std::string::npos ? m_s.size() : e + 3;
                continue;
            }
            if (m_s.compare(m_pos, 8, "![CDATA[") == 0) {
                const size_t e = m_s.find("]]>", m_pos);
                m_pos          = e == std::string::npos ? m_s.size() : e + 3;
                continue;
            }
            if (m_pos < m_s.size() && (m_s[m_pos] == '?' || m_s[m_pos] == '!')) {
                const size_t e = m_s.find('>', m_pos);
                m_pos          = e == std::string::npos ? m_s.size() : e + 1;
                continue;
            }
            closing = m_pos < m_s.size() && m_s[m_pos] == '/';
            if (closing)
                ++m_pos;
            const size_t name_start = m_pos;
            while (m_pos < m_s.size() && !std::isspace((unsigned char)m_s[m_pos]) && m_s[m_pos] != '>' &&
                   m_s[m_pos] != '/')
                ++m_pos;
            size_t colon = m_s.find(':', name_start);
            if (colon == std::string::npos || colon >= m_pos)
                colon = name_start;
            else
                ++colon;
            name.assign(m_s, colon, m_pos - colon);
            // Find the end of the tag, skipping quoted values.
            attrs_begin = m_pos;
            char quote  = 0;
            while (m_pos < m_s.size()) {
                const char c = m_s[m_pos];
                if (quote) {
                    if (c == quote)
                        quote = 0;
                } else if (c == '"' || c == '\'')
                    quote = c;
                else if (c == '>')
                    break;
                ++m_pos;
            }
            if (m_pos >= m_s.size())
                return false;
            attrs_end  = m_pos;
            self_close = attrs_end > attrs_begin && m_s[attrs_end - 1] == '/';
            if (self_close)
                --attrs_end;
            ++m_pos;
            return true;
        }
    }

    // Parses the attributes of a raw span.
    std::map<std::string, std::string> attrs(size_t b, size_t e) const
    {
        std::map<std::string, std::string> out;
        size_t                             p = b;
        while (p < e) {
            while (p < e && std::isspace((unsigned char)m_s[p]))
                ++p;
            const size_t k0 = p;
            while (p < e && m_s[p] != '=' && !std::isspace((unsigned char)m_s[p]))
                ++p;
            const std::string key = m_s.substr(k0, p - k0);
            while (p < e && std::isspace((unsigned char)m_s[p]))
                ++p;
            if (p >= e || m_s[p] != '=') {
                ++p;
                continue;
            }
            ++p;
            while (p < e && std::isspace((unsigned char)m_s[p]))
                ++p;
            if (p >= e || (m_s[p] != '"' && m_s[p] != '\''))
                continue;
            const char   q  = m_s[p++];
            const size_t ve = m_s.find(q, p);
            if (ve == std::string::npos || ve > e)
                break;
            out[key] = xml_unescape(m_s.substr(p, ve - p));
            p        = ve + 1;
        }
        return out;
    }

    // Same, into a Tag.
    bool next(Tag& tag)
    {
        size_t b, e;
        if (!next_raw(tag.name, tag.closing, tag.self_close, b, e))
            return false;
        tag.attrs = attrs(b, e);
        return true;
    }

    const char* data() const { return m_s.c_str(); }

private:
    const std::string& m_s;
    size_t             m_pos = 0;
};

// Fast numeric attribute lookup in a raw attribute span, for the hot
// <vertex>/<triangle> elements which make up nearly all of a model file.
bool fast_attr_double(const char* b, const char* e, const char* key, double& out)
{
    const size_t kl = std::strlen(key);
    for (const char* p = b; p + kl < e; ++p) {
        if ((p == b || std::isspace((unsigned char)p[-1])) && std::strncmp(p, key, kl) == 0) {
            const char* q = p + kl;
            while (q < e && std::isspace((unsigned char)*q))
                ++q;
            if (q >= e || *q != '=')
                continue;
            ++q;
            while (q < e && std::isspace((unsigned char)*q))
                ++q;
            if (q >= e || (*q != '"' && *q != '\''))
                continue;
            ++q;
            char* end;
            out = std::strtod(q, &end);
            return end != q;
        }
    }
    return false;
}

struct ObjectDef
{
    std::string name;
    std::string type = "model";
    Mesh        mesh;
    struct Component
    {
        int         objectid = -1;
        std::string path;
        Transform   transform;
    };
    std::vector<Component> components;
};

struct ModelPart
{
    double                   unit_scale = 1.0; // to millimetres
    std::map<int, ObjectDef> objects;
    struct Item
    {
        int         objectid = -1;
        std::string path;
        Transform   transform;
    };
    std::vector<Item> items;
};

double unit_to_mm(const std::string& unit)
{
    if (unit == "micron")
        return 0.001;
    if (unit == "centimeter")
        return 10.0;
    if (unit == "inch")
        return 25.4;
    if (unit == "foot")
        return 304.8;
    if (unit == "meter")
        return 1000.0;
    return 1.0;
}

bool parse_model(const std::string& xml, ModelPart& part, std::string& error)
{
    XmlScanner  sc(xml);
    const char* base = sc.data();
    std::string name;
    bool        closing = false, self_close = false;
    size_t      ab = 0, ae = 0;
    ObjectDef*  cur          = nullptr;
    bool        in_vertices  = false;
    bool        in_triangles = false;
    while (sc.next_raw(name, closing, self_close, ab, ae)) {
        if (closing) {
            if (name == "object")
                cur = nullptr;
            else if (name == "vertices")
                in_vertices = false;
            else if (name == "triangles")
                in_triangles = false;
            continue;
        }
        if (name == "vertex") {
            if (cur && in_vertices) {
                Vec3 v;
                if (!fast_attr_double(base + ab, base + ae, "x", v.x) || !fast_attr_double(base + ab, base + ae, "y", v.y) ||
                    !fast_attr_double(base + ab, base + ae, "z", v.z)) {
                    error = "Invalid vertex in 3MF model";
                    return false;
                }
                cur->mesh.vertices.push_back(v);
            }
            continue;
        }
        if (name == "triangle") {
            if (cur && in_triangles) {
                double a, b, c;
                if (!fast_attr_double(base + ab, base + ae, "v1", a) || !fast_attr_double(base + ab, base + ae, "v2", b) ||
                    !fast_attr_double(base + ab, base + ae, "v3", c)) {
                    error = "Invalid triangle in 3MF model";
                    return false;
                }
                cur->mesh.faces.push_back({ int(a), int(b), int(c) });
            }
            continue;
        }
        Tag tag;
        tag.name       = name;
        tag.self_close = self_close;
        tag.attrs      = sc.attrs(ab, ae);
        if (tag.name == "model") {
            part.unit_scale = unit_to_mm(tag.attr("unit", "millimeter"));
        } else if (tag.name == "object") {
            const int id = std::atoi(tag.attr("id", "-1").c_str());
            cur          = &part.objects[id];
            cur->name    = tag.attr("name");
            cur->type    = tag.attr("type", "model");
            if (tag.self_close)
                cur = nullptr;
        } else if (tag.name == "vertices") {
            in_vertices = !tag.self_close;
        } else if (tag.name == "triangles") {
            in_triangles = !tag.self_close;
        } else if (tag.name == "component" && cur) {
            ObjectDef::Component c;
            c.objectid = std::atoi(tag.attr("objectid", "-1").c_str());
            c.path     = tag.attr("p:path");
            if (c.path.empty())
                c.path = tag.attr("path");
            const std::string t = tag.attr("transform");
            if (!t.empty())
                Transform::parse(t, c.transform);
            cur->components.push_back(c);
        } else if (tag.name == "item") {
            ModelPart::Item it;
            it.objectid = std::atoi(tag.attr("objectid", "-1").c_str());
            it.path     = tag.attr("p:path");
            const std::string t = tag.attr("transform");
            if (!t.empty())
                Transform::parse(t, it.transform);
            part.items.push_back(it);
        }
    }
    return true;
}

class ThreeMFReader
{
public:
    bool load(const std::string& path, std::vector<NamedMesh>& objects, std::string& error)
    {
        if (!m_zip.open(path, error))
            return false;
        // Root model part from the package relationships.
        std::string root = "3D/3dmodel.model";
        std::string rels;
        if (m_zip.read("_rels/.rels", rels, error)) {
            XmlScanner sc(rels);
            Tag        tag;
            while (sc.next(tag))
                if (tag.name == "Relationship" &&
                    tag.attr("Type").find("3dmodel") != std::string::npos && !tag.attr("Target").empty()) {
                    root = tag.attr("Target");
                    break;
                }
        }
        error.clear();
        ModelPart* main = part(root, error);
        if (!main)
            return false;
        if (main->items.empty()) {
            // No build section: take all top level model objects.
            for (const auto& kv : main->objects)
                main->items.push_back({ kv.first, "", Transform() });
        }
        for (const ModelPart::Item& it : main->items) {
            const std::string item_path = it.path.empty() ? root : it.path;
            ModelPart*        p         = part(item_path, error);
            if (!p)
                return false;
            NamedMesh nm;
            auto      oit = p->objects.find(it.objectid);
            if (oit == p->objects.end()) {
                error = "3MF build item references unknown object " + std::to_string(it.objectid);
                return false;
            }
            nm.name = oit->second.name.empty() ? "Object " + std::to_string(it.objectid) : oit->second.name;
            Transform scale;
            for (int i = 0; i < 3; ++i)
                scale.m[i][i] = p->unit_scale;
            if (!flatten(item_path, it.objectid, scale.then(it.transform), nm.mesh, 0, error))
                return false;
            if (!nm.mesh.faces.empty())
                objects.push_back(std::move(nm));
        }
        if (objects.empty()) {
            error = "The 3MF file contains no meshes";
            return false;
        }
        return true;
    }

private:
    ModelPart* part(const std::string& path, std::string& error)
    {
        std::string key = path;
        if (!key.empty() && key[0] == '/')
            key.erase(0, 1);
        auto it = m_parts.find(key);
        if (it != m_parts.end())
            return it->second.get();
        std::string xml;
        if (!m_zip.read(key, xml, error))
            return nullptr;
        auto mp = std::make_unique<ModelPart>();
        if (!parse_model(xml, *mp, error))
            return nullptr;
        ModelPart* raw = mp.get();
        m_parts[key]   = std::move(mp);
        return raw;
    }

    bool flatten(const std::string& path, int objectid, const Transform& tr, Mesh& out, int depth, std::string& error)
    {
        if (depth > 32) {
            error = "3MF component nesting too deep (cycle?)";
            return false;
        }
        ModelPart* p = part(path, error);
        if (!p)
            return false;
        auto it = p->objects.find(objectid);
        if (it == p->objects.end()) {
            error = "3MF component references unknown object " + std::to_string(objectid) + " in " + path;
            return false;
        }
        const ObjectDef& obj = it->second;
        if (obj.type != "model" && obj.type != "solidsupport" && !obj.type.empty() && obj.type != "other")
            return true; // support / surface objects are not part of the solid
        const int  off    = int(out.vertices.size());
        const bool mirror = tr.determinant() < 0.0;
        for (const Vec3& v : obj.mesh.vertices)
            out.vertices.push_back(tr.apply(v));
        const int nv = int(obj.mesh.vertices.size());
        for (Triangle t : obj.mesh.faces) {
            if (t[0] < 0 || t[1] < 0 || t[2] < 0 || t[0] >= nv || t[1] >= nv || t[2] >= nv) {
                error = "3MF triangle references a missing vertex";
                return false;
            }
            if (mirror)
                std::swap(t[1], t[2]);
            out.faces.push_back({ t[0] + off, t[1] + off, t[2] + off });
        }
        for (const ObjectDef::Component& c : obj.components) {
            const std::string cpath = c.path.empty() ? path : c.path;
            if (!flatten(cpath, c.objectid, c.transform.then(tr), out, depth + 1, error))
                return false;
        }
        return true;
    }

    zip::Reader                                       m_zip;
    std::map<std::string, std::unique_ptr<ModelPart>> m_parts;
};

} // namespace

bool load_3mf(const std::string& path, std::vector<NamedMesh>& objects, std::string& error)
{
    objects.clear();
    ThreeMFReader reader;
    return reader.load(path, objects, error);
}

bool save_3mf(const std::string& path, const std::vector<NamedMesh>& objects, std::string& error)
{
    std::string model;
    model.reserve(1 << 20);
    model += "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
             "<model unit=\"millimeter\" xml:lang=\"en-US\" "
             "xmlns=\"http://schemas.microsoft.com/3dmanufacturing/core/2015/02\">\n"
             " <metadata name=\"Application\">MeshRepair</metadata>\n"
             " <resources>\n";
    char buf[256];
    int  id = 1;
    for (const NamedMesh& o : objects) {
        model += "  <object id=\"" + std::to_string(id) + "\" name=\"" + xml_escape(o.name) +
                 "\" type=\"model\">\n   <mesh>\n    <vertices>\n";
        for (const Vec3& v : o.mesh.vertices) {
            std::snprintf(buf, sizeof(buf), "     <vertex x=\"%.9g\" y=\"%.9g\" z=\"%.9g\"/>\n", v.x, v.y, v.z);
            model += buf;
        }
        model += "    </vertices>\n    <triangles>\n";
        for (const Triangle& t : o.mesh.faces) {
            std::snprintf(buf, sizeof(buf), "     <triangle v1=\"%d\" v2=\"%d\" v3=\"%d\"/>\n", t[0], t[1], t[2]);
            model += buf;
        }
        model += "    </triangles>\n   </mesh>\n  </object>\n";
        ++id;
    }
    model += " </resources>\n <build>\n";
    for (int i = 1; i < id; ++i)
        model += "  <item objectid=\"" + std::to_string(i) + "\"/>\n";
    model += " </build>\n</model>\n";

    const std::string content_types =
        "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
        "<Types xmlns=\"http://schemas.openxmlformats.org/package/2006/content-types\">\n"
        " <Default Extension=\"rels\" ContentType=\"application/vnd.openxmlformats-package.relationships+xml\"/>\n"
        " <Default Extension=\"model\" ContentType=\"application/vnd.ms-package.3dmanufacturing-3dmodel+xml\"/>\n"
        "</Types>\n";
    const std::string rels =
        "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
        "<Relationships xmlns=\"http://schemas.openxmlformats.org/package/2006/relationships\">\n"
        " <Relationship Target=\"/3D/3dmodel.model\" Id=\"rel0\" "
        "Type=\"http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel\"/>\n"
        "</Relationships>\n";

    zip::Writer w;
    w.add("[Content_Types].xml", content_types);
    w.add("_rels/.rels", rels);
    w.add("3D/3dmodel.model", model);
    return w.save(path, error);
}

} // namespace meshrepair

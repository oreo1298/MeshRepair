#include "Zip.hpp"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <fstream>

namespace meshrepair {
namespace zip {

uint32_t crc32(const uint8_t* data, size_t len, uint32_t crc)
{
    static uint32_t table[256];
    static bool     init = false;
    if (!init) {
        for (uint32_t n = 0; n < 256; ++n) {
            uint32_t c = n;
            for (int k = 0; k < 8; ++k)
                c = (c & 1) ? 0xEDB88320u ^ (c >> 1) : c >> 1;
            table[n] = c;
        }
        init = true;
    }
    crc = ~crc;
    for (size_t i = 0; i < len; ++i)
        crc = table[(crc ^ data[i]) & 0xFF] ^ (crc >> 8);
    return ~crc;
}

namespace {

const uint16_t kLenBase[29]  = { 3,  4,  5,  6,  7,  8,  9,  10, 11,  13,  15,  17,  19,  23, 27,
                                 31, 35, 43, 51, 59, 67, 83, 99, 115, 131, 163, 195, 227, 258 };
const uint8_t  kLenExtra[29] = { 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5, 5, 0 };
const uint16_t kDistBase[30] = { 1,   2,   3,   4,   5,   7,    9,    13,   17,   25,   33,   49,   65,    97,    129,
                                 193, 257, 385, 513, 769, 1025, 1537, 2049, 3073, 4097, 6145, 8193, 12289, 16385, 24577 };
const uint8_t  kDistExtra[30] = { 0, 0, 0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 9, 10, 10, 11, 11, 12, 12, 13, 13 };

int bit_reverse(int v, int bits)
{
    int r = 0;
    for (int i = 0; i < bits; ++i) {
        r = (r << 1) | (v & 1);
        v >>= 1;
    }
    return r;
}

// Canonical Huffman decoding table (fast lookup for codes up to 9 bits).
struct Huffman
{
    static constexpr int kFast = 9;
    uint16_t             fast[1 << kFast];
    uint16_t             firstcode[17];
    int                  maxcode[18];
    uint16_t             firstsymbol[17];
    uint8_t              size[320];
    uint16_t             value[320];

    bool build(const uint8_t* lengths, int num)
    {
        int sizes[17] = {};
        int next_code[17];
        std::memset(fast, 0, sizeof(fast));
        for (int i = 0; i < num; ++i)
            ++sizes[lengths[i]];
        sizes[0] = 0;
        for (int i = 1; i < 16; ++i)
            if (sizes[i] > (1 << i))
                return false;
        int code = 0, k = 0;
        for (int i = 1; i < 16; ++i) {
            next_code[i]   = code;
            firstcode[i]   = uint16_t(code);
            firstsymbol[i] = uint16_t(k);
            code += sizes[i];
            if (sizes[i] && code - 1 >= (1 << i))
                return false;
            maxcode[i] = code << (16 - i);
            code <<= 1;
            k += sizes[i];
        }
        maxcode[16] = 0x10000;
        for (int i = 0; i < num; ++i) {
            const int s = lengths[i];
            if (!s)
                continue;
            const int c = next_code[s] - firstcode[s] + firstsymbol[s];
            size[c]     = uint8_t(s);
            value[c]    = uint16_t(i);
            if (s <= kFast) {
                for (int j = bit_reverse(next_code[s], s); j < (1 << kFast); j += 1 << s)
                    fast[j] = uint16_t((s << 9) | i);
            }
            ++next_code[s];
        }
        return true;
    }
};

class Inflater
{
public:
    Inflater(const uint8_t* src, size_t len, std::vector<uint8_t>& out) : m_src(src), m_len(len), m_out(out) {}

    bool run(std::string& error)
    {
        bool final = false;
        do {
            final            = bits(1) != 0;
            const int type   = int(bits(2));
            if (type == 0) {
                if (!stored()) {
                    error = "Corrupt stored block";
                    return false;
                }
            } else if (type == 1 || type == 2) {
                if (type == 1)
                    fixed_tables();
                else if (!dynamic_tables()) {
                    error = "Corrupt Huffman tables";
                    return false;
                }
                if (!codes()) {
                    error = "Corrupt compressed data";
                    return false;
                }
            } else {
                error = "Invalid block type";
                return false;
            }
            if (m_overrun) {
                error = "Truncated compressed data";
                return false;
            }
        } while (!final);
        return true;
    }

private:
    void refill()
    {
        while (m_bitcnt <= 56) {
            if (m_pos < m_len)
                m_bitbuf |= uint64_t(m_src[m_pos++]) << m_bitcnt;
            else
                m_padding += 8; // reading past the end, zeros
            m_bitcnt += 8;
        }
    }
    uint32_t bits(int n)
    {
        if (m_bitcnt < n)
            refill();
        const uint32_t v = uint32_t(m_bitbuf & ((uint64_t(1) << n) - 1));
        consume(n);
        return v;
    }
    void consume(int n)
    {
        m_bitbuf >>= n;
        m_bitcnt -= n;
        if (m_padding > 0 && m_bitcnt < m_padding)
            m_overrun = true;
    }
    int decode(const Huffman& h)
    {
        if (m_bitcnt < 16)
            refill();
        const int b = h.fast[m_bitbuf & ((1 << Huffman::kFast) - 1)];
        if (b) {
            consume(b >> 9);
            return b & 511;
        }
        const int k = bit_reverse(int(m_bitbuf & 0xFFFF), 16);
        int       s = Huffman::kFast + 1;
        while (k >= h.maxcode[s])
            ++s;
        if (s >= 16)
            return -1;
        const int c = (k >> (16 - s)) - h.firstcode[s] + h.firstsymbol[s];
        if (c < 0 || c >= 320 || h.size[c] != s)
            return -1;
        consume(s);
        return h.value[c];
    }

    bool stored()
    {
        // Align to a byte boundary.
        consume(m_bitcnt & 7);
        const uint32_t len  = bits(16);
        const uint32_t nlen = bits(16);
        if ((len ^ 0xFFFF) != nlen)
            return false;
        for (uint32_t i = 0; i < len; ++i)
            m_out.push_back(uint8_t(bits(8)));
        return !m_overrun;
    }

    void fixed_tables()
    {
        uint8_t l[288];
        for (int i = 0; i < 144; ++i)
            l[i] = 8;
        for (int i = 144; i < 256; ++i)
            l[i] = 9;
        for (int i = 256; i < 280; ++i)
            l[i] = 7;
        for (int i = 280; i < 288; ++i)
            l[i] = 8;
        m_lit.build(l, 288);
        uint8_t d[32];
        std::fill(d, d + 32, uint8_t(5));
        m_dist.build(d, 32);
    }

    bool dynamic_tables()
    {
        static const uint8_t order[19] = { 16, 17, 18, 0, 8, 7, 9, 6, 10, 5, 11, 4, 12, 3, 13, 2, 14, 1, 15 };
        const int            hlit      = int(bits(5)) + 257;
        const int            hdist     = int(bits(5)) + 1;
        const int            hclen     = int(bits(4)) + 4;
        uint8_t              cl[19]    = {};
        for (int i = 0; i < hclen; ++i)
            cl[order[i]] = uint8_t(bits(3));
        Huffman clh;
        if (!clh.build(cl, 19))
            return false;
        uint8_t lengths[286 + 32] = {};
        int     n                 = 0;
        while (n < hlit + hdist) {
            const int sym = decode(clh);
            if (sym < 0)
                return false;
            if (sym < 16)
                lengths[n++] = uint8_t(sym);
            else {
                int     rep = 0;
                uint8_t val = 0;
                if (sym == 16) {
                    if (n == 0)
                        return false;
                    val = lengths[n - 1];
                    rep = 3 + int(bits(2));
                } else if (sym == 17)
                    rep = 3 + int(bits(3));
                else
                    rep = 11 + int(bits(7));
                if (n + rep > hlit + hdist)
                    return false;
                std::fill(lengths + n, lengths + n + rep, val);
                n += rep;
            }
        }
        if (lengths[256] == 0)
            return false;
        return m_lit.build(lengths, hlit) && m_dist.build(lengths + hlit, hdist);
    }

    bool codes()
    {
        for (;;) {
            const int sym = decode(m_lit);
            if (sym < 0 || m_overrun)
                return false;
            if (sym < 256) {
                m_out.push_back(uint8_t(sym));
                continue;
            }
            if (sym == 256)
                return true;
            const int li = sym - 257;
            if (li >= 29)
                return false;
            const size_t len = kLenBase[li] + bits(kLenExtra[li]);
            const int    ds  = decode(m_dist);
            if (ds < 0 || ds >= 30)
                return false;
            const size_t dist = kDistBase[ds] + bits(kDistExtra[ds]);
            if (dist > m_out.size())
                return false;
            const size_t start = m_out.size() - dist;
            for (size_t i = 0; i < len; ++i)
                m_out.push_back(m_out[start + i]);
        }
    }

    const uint8_t*        m_src;
    size_t                m_len;
    size_t                m_pos     = 0;
    uint64_t              m_bitbuf  = 0;
    int                   m_bitcnt  = 0;
    int                   m_padding = 0;
    bool                  m_overrun = false;
    std::vector<uint8_t>& m_out;
    Huffman               m_lit, m_dist;
};

// Little endian helpers.
uint16_t rd16(const uint8_t* p) { return uint16_t(p[0] | (p[1] << 8)); }
uint32_t rd32(const uint8_t* p) { return uint32_t(p[0]) | (uint32_t(p[1]) << 8) | (uint32_t(p[2]) << 16) | (uint32_t(p[3]) << 24); }
uint64_t rd64(const uint8_t* p) { return uint64_t(rd32(p)) | (uint64_t(rd32(p + 4)) << 32); }
void     wr16(std::vector<uint8_t>& o, uint32_t v)
{
    o.push_back(uint8_t(v));
    o.push_back(uint8_t(v >> 8));
}
void wr32(std::vector<uint8_t>& o, uint32_t v)
{
    wr16(o, v & 0xFFFF);
    wr16(o, v >> 16);
}

} // namespace

bool inflate(const uint8_t* src, size_t src_len, std::vector<uint8_t>& out, size_t expected_size, std::string& error)
{
    out.clear();
    out.reserve(expected_size);
    Inflater inf(src, src_len, out);
    return inf.run(error);
}

void deflate(const uint8_t* src, size_t n, std::vector<uint8_t>& out)
{
    // LZ77 with hash chains, coded with the fixed Huffman tables (one block).
    uint64_t bitbuf = 0;
    int      bitcnt = 0;
    auto     put    = [&](uint32_t v, int bits) {
        bitbuf |= uint64_t(v) << bitcnt;
        bitcnt += bits;
        while (bitcnt >= 8) {
            out.push_back(uint8_t(bitbuf));
            bitbuf >>= 8;
            bitcnt -= 8;
        }
    };
    auto put_lit = [&](int sym) {
        if (sym < 144)
            put(uint32_t(bit_reverse(0x30 + sym, 8)), 8);
        else if (sym < 256)
            put(uint32_t(bit_reverse(0x190 + sym - 144, 9)), 9);
        else if (sym < 280)
            put(uint32_t(bit_reverse(sym - 256, 7)), 7);
        else
            put(uint32_t(bit_reverse(0xC0 + sym - 280, 8)), 8);
    };
    auto put_match = [&](int len, int dist) {
        int li = 28;
        while (kLenBase[li] > len)
            --li;
        put_lit(257 + li);
        put(uint32_t(len - kLenBase[li]), kLenExtra[li]);
        int di = 29;
        while (kDistBase[di] > dist)
            --di;
        put(uint32_t(bit_reverse(di, 5)), 5);
        put(uint32_t(dist - kDistBase[di]), kDistExtra[di]);
    };

    put(1, 1); // final block
    put(1, 2); // fixed Huffman

    constexpr int    kWindow   = 32768;
    constexpr int    kHashBits = 15;
    constexpr int    kMaxChain = 24;
    std::vector<int> head(1 << kHashBits, -1);
    std::vector<int> prev(kWindow, -1);
    auto             hash = [&](size_t i) {
        return int(((uint32_t(src[i]) << 10) ^ (uint32_t(src[i + 1]) << 5) ^ src[i + 2]) * 2654435761u >> (32 - kHashBits));
    };
    auto insert = [&](size_t i) {
        if (i + 2 < n) {
            const int h          = hash(i);
            prev[i & (kWindow - 1)] = head[h];
            head[h]              = int(i);
        }
    };

    size_t i = 0;
    while (i < n) {
        int best_len = 0, best_dist = 0;
        if (i + 2 < n) {
            const int    h       = hash(i);
            int          cand    = head[h];
            const size_t max_len = std::min<size_t>(258, n - i);
            for (int chain = 0; cand >= 0 && chain < kMaxChain; ++chain) {
                const size_t dist = i - size_t(cand);
                if (dist > size_t(kWindow) - 1)
                    break;
                if (src[cand + best_len] == src[i + best_len]) {
                    size_t l = 0;
                    while (l < max_len && src[cand + l] == src[i + l])
                        ++l;
                    if (int(l) > best_len) {
                        best_len  = int(l);
                        best_dist = int(dist);
                        if (l == max_len)
                            break;
                    }
                }
                cand = prev[size_t(cand) & (kWindow - 1)];
            }
        }
        if (best_len >= 3) {
            put_match(best_len, best_dist);
            for (int k = 0; k < best_len; ++k)
                insert(i + size_t(k));
            i += size_t(best_len);
        } else {
            put_lit(src[i]);
            insert(i);
            ++i;
        }
    }
    put_lit(256);
    if (bitcnt > 0)
        put(0, 8 - bitcnt);
}

// ---------------------------------------------------------------------------

bool Reader::open(const std::string& path, std::string& error)
{
    std::ifstream f(path, std::ios::binary);
    if (!f) {
        error = "Cannot open file: " + path;
        return false;
    }
    m_data.assign(std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>());
    const uint8_t* d = reinterpret_cast<const uint8_t*>(m_data.data());
    const size_t   n = m_data.size();
    if (n < 22) {
        error = "Not a ZIP file";
        return false;
    }
    // End of central directory record (may be followed by a comment).
    size_t eocd = std::string::npos;
    for (size_t i = n - 22 + 1; i-- > 0 && n - i <= 22 + 65535;)
        if (rd32(d + i) == 0x06054b50) {
            eocd = i;
            break;
        }
    if (eocd == std::string::npos) {
        error = "Not a ZIP file (no central directory)";
        return false;
    }
    uint64_t entries = rd16(d + eocd + 10);
    uint64_t cd_off  = rd32(d + eocd + 16);
    // ZIP64 end of central directory.
    if (eocd >= 20 && rd32(d + eocd - 20) == 0x07064b50) {
        const uint64_t z64 = rd64(d + eocd - 20 + 8);
        if (z64 + 56 <= n && rd32(d + z64) == 0x06064b50) {
            entries = rd64(d + z64 + 32);
            cd_off  = rd64(d + z64 + 48);
        }
    }
    size_t p = size_t(cd_off);
    for (uint64_t e = 0; e < entries; ++e) {
        if (p + 46 > n || rd32(d + p) != 0x02014b50) {
            error = "Corrupt ZIP central directory";
            return false;
        }
        Entry en;
        en.method          = rd16(d + p + 10);
        en.crc             = rd32(d + p + 16);
        en.comp_size       = rd32(d + p + 20);
        en.size            = rd32(d + p + 24);
        const size_t nlen  = rd16(d + p + 28);
        const size_t xlen  = rd16(d + p + 30);
        const size_t clen  = rd16(d + p + 32);
        en.local_offset    = rd32(d + p + 42);
        if (p + 46 + nlen + xlen > n) {
            error = "Corrupt ZIP central directory";
            return false;
        }
        en.name.assign(reinterpret_cast<const char*>(d + p + 46), nlen);
        // ZIP64 extended information.
        for (size_t x = p + 46 + nlen; x + 4 <= p + 46 + nlen + xlen;) {
            const uint16_t id = rd16(d + x), sz = rd16(d + x + 2);
            if (id == 0x0001) {
                size_t q = x + 4;
                if (en.size == 0xFFFFFFFFu && q + 8 <= x + 4 + sz) {
                    en.size = rd64(d + q);
                    q += 8;
                }
                if (en.comp_size == 0xFFFFFFFFu && q + 8 <= x + 4 + sz) {
                    en.comp_size = rd64(d + q);
                    q += 8;
                }
                if (en.local_offset == 0xFFFFFFFFu && q + 8 <= x + 4 + sz)
                    en.local_offset = rd64(d + q);
            }
            x += 4 + sz;
        }
        m_names.push_back(en.name);
        m_entries.push_back(std::move(en));
        p += 46 + nlen + xlen + clen;
    }
    return true;
}

const Reader::Entry* Reader::find(const std::string& name) const
{
    std::string n = name;
    if (!n.empty() && n[0] == '/')
        n.erase(0, 1);
    for (const Entry& e : m_entries)
        if (e.name == n)
            return &e;
    // 3MF part names are case insensitive.
    auto lower = [](std::string s) {
        std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return char(std::tolower(c)); });
        return s;
    };
    const std::string ln = lower(n);
    for (const Entry& e : m_entries)
        if (lower(e.name) == ln)
            return &e;
    return nullptr;
}

bool Reader::has(const std::string& name) const { return find(name) != nullptr; }

bool Reader::read(const std::string& name, std::string& data, std::string& error) const
{
    const Entry* e = find(name);
    if (!e) {
        error = "Missing part in archive: " + name;
        return false;
    }
    const uint8_t* d = reinterpret_cast<const uint8_t*>(m_data.data());
    const size_t   n = m_data.size();
    const size_t   p = size_t(e->local_offset);
    if (p + 30 > n || rd32(d + p) != 0x04034b50) {
        error = "Corrupt ZIP entry: " + name;
        return false;
    }
    const size_t start = p + 30 + rd16(d + p + 26) + rd16(d + p + 28);
    if (start + e->comp_size > n) {
        error = "Truncated ZIP entry: " + name;
        return false;
    }
    if (e->method == 0) {
        data.assign(reinterpret_cast<const char*>(d + start), size_t(e->comp_size));
    } else if (e->method == 8) {
        std::vector<uint8_t> out;
        std::string          err;
        if (!inflate(d + start, size_t(e->comp_size), out, size_t(e->size), err)) {
            error = name + ": " + err;
            return false;
        }
        data.assign(reinterpret_cast<const char*>(out.data()), out.size());
    } else {
        error = "Unsupported ZIP compression method " + std::to_string(e->method) + " for " + name;
        return false;
    }
    if (crc32(reinterpret_cast<const uint8_t*>(data.data()), data.size()) != e->crc) {
        error = "Checksum mismatch in " + name;
        return false;
    }
    return true;
}

// ---------------------------------------------------------------------------

void Writer::add(const std::string& name, const std::string& data, bool compress)
{
    Entry e;
    e.name   = name;
    e.size   = data.size();
    e.crc    = crc32(reinterpret_cast<const uint8_t*>(data.data()), data.size());
    e.offset = m_body.size();
    std::vector<uint8_t> payload;
    if (compress && !data.empty()) {
        deflate(reinterpret_cast<const uint8_t*>(data.data()), data.size(), payload);
        e.method = 8;
    }
    if (!compress || data.empty() || payload.size() >= data.size()) {
        payload.assign(data.begin(), data.end());
        e.method = 0;
    }
    e.comp_size = payload.size();
    wr32(m_body, 0x04034b50);
    wr16(m_body, 20);
    wr16(m_body, 0x0800); // UTF-8 names
    wr16(m_body, e.method);
    wr16(m_body, 0);      // time
    wr16(m_body, 0x0021); // date: 1980-01-01
    wr32(m_body, e.crc);
    wr32(m_body, uint32_t(e.comp_size));
    wr32(m_body, uint32_t(e.size));
    wr16(m_body, uint32_t(name.size()));
    wr16(m_body, 0);
    m_body.insert(m_body.end(), name.begin(), name.end());
    m_body.insert(m_body.end(), payload.begin(), payload.end());
    m_entries.push_back(e);
}

bool Writer::save(const std::string& path, std::string& error) const
{
    if (m_body.size() > 0xFFFFFFF0u) {
        error = "File too large for a ZIP archive without ZIP64";
        return false;
    }
    std::vector<uint8_t> cd;
    for (const Entry& e : m_entries) {
        wr32(cd, 0x02014b50);
        wr16(cd, 20);
        wr16(cd, 20);
        wr16(cd, 0x0800);
        wr16(cd, e.method);
        wr16(cd, 0);
        wr16(cd, 0x0021);
        wr32(cd, e.crc);
        wr32(cd, uint32_t(e.comp_size));
        wr32(cd, uint32_t(e.size));
        wr16(cd, uint32_t(e.name.size()));
        wr16(cd, 0);
        wr16(cd, 0);
        wr16(cd, 0);
        wr16(cd, 0);
        wr32(cd, 0);
        wr32(cd, uint32_t(e.offset));
        cd.insert(cd.end(), e.name.begin(), e.name.end());
    }
    std::vector<uint8_t> end;
    wr32(end, 0x06054b50);
    wr16(end, 0);
    wr16(end, 0);
    wr16(end, uint32_t(m_entries.size()));
    wr16(end, uint32_t(m_entries.size()));
    wr32(end, uint32_t(cd.size()));
    wr32(end, uint32_t(m_body.size()));
    wr16(end, 0);

    std::FILE* f = std::fopen(path.c_str(), "wb");
    if (!f) {
        error = "Cannot write file: " + path;
        return false;
    }
    bool ok = std::fwrite(m_body.data(), 1, m_body.size(), f) == m_body.size();
    ok      = ok && std::fwrite(cd.data(), 1, cd.size(), f) == cd.size();
    ok      = ok && std::fwrite(end.data(), 1, end.size(), f) == end.size();
    ok      = (std::fclose(f) == 0) && ok;
    if (!ok)
        error = "Error while writing file: " + path;
    return ok;
}

} // namespace zip
} // namespace meshrepair

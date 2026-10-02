// Minimal ZIP container support (stored and deflate entries), enough for 3MF.

#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace meshrepair {
namespace zip {

uint32_t crc32(const uint8_t* data, size_t len, uint32_t crc = 0);

// Raw DEFLATE (RFC 1951).
bool inflate(const uint8_t* src, size_t src_len, std::vector<uint8_t>& out, size_t expected_size, std::string& error);
void deflate(const uint8_t* src, size_t src_len, std::vector<uint8_t>& out);

class Reader
{
public:
    bool open(const std::string& path, std::string& error);
    bool has(const std::string& name) const;
    bool read(const std::string& name, std::string& data, std::string& error) const;
    const std::vector<std::string>& names() const { return m_names; }

private:
    struct Entry
    {
        std::string name;
        uint16_t    method = 0;
        uint64_t    comp_size = 0;
        uint64_t    size = 0;
        uint64_t    local_offset = 0;
        uint32_t    crc = 0;
    };
    const Entry* find(const std::string& name) const;

    std::string              m_data; // whole file
    std::vector<Entry>       m_entries;
    std::vector<std::string> m_names;
};

class Writer
{
public:
    void add(const std::string& name, const std::string& data, bool compress = true);
    bool save(const std::string& path, std::string& error) const;

private:
    struct Entry
    {
        std::string name;
        uint16_t    method;
        uint32_t    crc;
        uint64_t    size;
        uint64_t    offset;
        uint64_t    comp_size;
    };
    std::vector<Entry>   m_entries;
    std::vector<uint8_t> m_body;
};

} // namespace zip
} // namespace meshrepair

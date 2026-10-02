// Minimal PNG writer (uncompressed deflate blocks), used for screenshots.

#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace mrgui {

// rgba: width * height * 4 bytes, top row first.
bool write_png(const std::string& path, int width, int height, const std::vector<uint8_t>& rgba);

} // namespace mrgui

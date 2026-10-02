// DEFLATE / ZIP round trip tests.

#include "Zip.hpp"

#include <cstdio>
#include <random>
#include <string>
#include <vector>

using namespace meshrepair;

static int failures = 0;

static void check(bool ok, const char* what)
{
    std::printf("[%s] %s\n", ok ? " OK " : "FAIL", what);
    failures += ok ? 0 : 1;
}

static bool roundtrip(const std::vector<uint8_t>& data)
{
    std::vector<uint8_t> packed, unpacked;
    zip::deflate(data.data(), data.size(), packed);
    std::string err;
    if (!zip::inflate(packed.data(), packed.size(), unpacked, data.size(), err)) {
        std::printf("    inflate error: %s\n", err.c_str());
        return false;
    }
    return unpacked == data;
}

int main()
{
    std::mt19937 rng(5);
    check(roundtrip({}), "empty input");
    check(roundtrip({ 'a' }), "single byte");

    std::vector<uint8_t> random(200000);
    for (uint8_t& b : random)
        b = uint8_t(rng());
    check(roundtrip(random), "random bytes");

    std::string text;
    for (int i = 0; i < 50000; ++i)
        text += "     <vertex x=\"" + std::to_string(rng() % 1000) + ".25\" y=\"" + std::to_string(i) + "\" z=\"0\"/>\n";
    std::vector<uint8_t> xml(text.begin(), text.end());
    std::vector<uint8_t> packed;
    zip::deflate(xml.data(), xml.size(), packed);
    std::printf("    xml: %zu -> %zu bytes (%.1f%%)\n", xml.size(), packed.size(), 100.0 * packed.size() / xml.size());
    check(roundtrip(xml), "XML-like text");
    check(packed.size() < xml.size() / 3, "XML compresses well");

    std::vector<uint8_t> runs(100000, 'x');
    check(roundtrip(runs), "long runs (overlapping copies)");

    // Corrupt data must fail cleanly, not crash.
    std::vector<uint8_t> broken = packed;
    broken.resize(broken.size() / 2);
    std::vector<uint8_t> out;
    std::string          err;
    check(!zip::inflate(broken.data(), broken.size(), out, 0, err), "truncated stream is rejected");
    for (int k = 0; k < 200; ++k) {
        std::vector<uint8_t> noise(1 + rng() % 400);
        for (uint8_t& b : noise)
            b = uint8_t(rng());
        zip::inflate(noise.data(), noise.size(), out, 0, err); // must not crash
    }
    check(true, "random garbage does not crash the decoder");

    // ZIP container.
    zip::Writer w;
    w.add("a.txt", text);
    w.add("dir/b.bin", std::string(random.begin(), random.end()));
    w.add("empty", "");
    const char* dir = std::getenv("TMPDIR");
    const std::string path = std::string(dir ? dir : "/tmp") + "/meshrepair_zip_test.zip";
    check(w.save(path, err), "write zip");
    zip::Reader r;
    check(r.open(path, err), "open zip");
    std::string a, b, e;
    check(r.read("a.txt", a, err) && a == text, "read deflated entry");
    check(r.read("/dir/b.bin", b, err) && b == std::string(random.begin(), random.end()), "read stored entry");
    check(r.read("empty", e, err) && e.empty(), "read empty entry");
    check(!r.read("missing", e, err), "missing entry reported");
    std::remove(path.c_str());

    std::printf("%d failures\n", failures);
    return failures ? 1 : 0;
}

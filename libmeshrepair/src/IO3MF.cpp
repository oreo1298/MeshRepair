#include "meshrepair/IO.hpp"

namespace meshrepair {

bool load_3mf(const std::string& path, std::vector<NamedMesh>& objects, std::string& error)
{
    (void)path;
    (void)objects;
    error = "3MF is not supported yet";
    return false;
}

bool save_3mf(const std::string& path, const std::vector<NamedMesh>& objects, std::string& error)
{
    (void)path;
    (void)objects;
    error = "3MF is not supported yet";
    return false;
}

} // namespace meshrepair

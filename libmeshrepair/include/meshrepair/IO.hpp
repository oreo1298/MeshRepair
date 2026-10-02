// MeshRepair - file input / output.

#pragma once

#include "Mesh.hpp"

#include <string>
#include <vector>

namespace meshrepair {

enum class FileFormat {
    Unknown,
    STL,      // binary STL when writing
    STLAscii, // ASCII STL (only meaningful for writing)
    OBJ,
    ThreeMF,
};

// Guesses the format from the file extension.
FileFormat format_from_path(const std::string& path);

// One object of a multi-object file (3MF may contain several).
struct NamedMesh
{
    std::string name;
    Mesh        mesh;
};

// Loads a mesh. Vertices with identical coordinates are merged, as STL is a
// plain triangle soup. Multi-object files are merged into one mesh.
// Returns false and sets error on failure.
bool load_mesh(const std::string& path, Mesh& mesh, std::string& error);

// Loads every object of the file separately (STL / OBJ yield one object).
bool load_objects(const std::string& path, std::vector<NamedMesh>& objects, std::string& error);

// Saves a mesh. With FileFormat::Unknown the format is taken from the extension.
bool save_mesh(const std::string& path, const Mesh& mesh, std::string& error, FileFormat format = FileFormat::Unknown);
bool save_objects(const std::string& path, const std::vector<NamedMesh>& objects, std::string& error,
                  FileFormat format = FileFormat::Unknown);

// Format specific entry points.
bool load_stl(const std::string& path, Mesh& mesh, std::string& error);
bool save_stl(const std::string& path, const Mesh& mesh, bool ascii, std::string& error, const std::string& name = "meshrepair");
bool load_obj(const std::string& path, std::vector<NamedMesh>& objects, std::string& error);
bool save_obj(const std::string& path, const std::vector<NamedMesh>& objects, std::string& error);
bool load_3mf(const std::string& path, std::vector<NamedMesh>& objects, std::string& error);
bool save_3mf(const std::string& path, const std::vector<NamedMesh>& objects, std::string& error);

// Merges vertices with bit-identical coordinates (used by the STL loader).
void merge_identical_vertices(Mesh& mesh);

} // namespace meshrepair

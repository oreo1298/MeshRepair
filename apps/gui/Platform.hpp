// Linux desktop integration helpers: processes, paths, fonts, settings.

#pragma once

#include <map>
#include <string>
#include <vector>

namespace mrgui {

// $XDG_CONFIG_HOME/meshrepair (created on demand).
std::string config_dir();
// $XDG_CACHE_HOME/meshrepair (created on demand).
std::string cache_dir();
std::string home_dir();

// Full path of an executable found in $PATH, or empty.
std::string find_in_path(const std::string& exe);

// Runs a program and captures its standard output. Returns false if it could
// not be started. exit_code receives the exit status.
bool run_capture(const std::vector<std::string>& argv, std::string& output, int& exit_code);

// Starts a program detached from this process.
bool launch_detached(const std::vector<std::string>& argv, std::string& error);

// A slicer the repaired model can be sent to.
struct SlicerApp
{
    std::string              name;    // "Bambu Studio"
    std::string              how;     // "flatpak", "/usr/bin/bambu-studio", ...
    std::vector<std::string> command; // argv; the file is appended (or replaces "@FILE@")
};

// Detects installed slicers (native executables in $PATH and Flatpaks).
// custom_command, if not empty, is offered first ("Custom").
std::vector<SlicerApp> detect_slicers(const std::string& custom_command);

bool open_in_slicer(const SlicerApp& app, const std::string& file, std::string& error);

// Path of a regular sans-serif TTF/OTF font of the desktop, or empty.
std::string find_ui_font();

// Splits a command line on whitespace, honouring simple quotes.
std::vector<std::string> split_command(const std::string& cmd);

// Very small key=value settings file.
class Settings
{
public:
    void        load(const std::string& path);
    bool        save(const std::string& path) const;
    std::string get(const std::string& key, const std::string& def = "") const;
    double      get_double(const std::string& key, double def) const;
    bool        get_bool(const std::string& key, bool def) const;
    void        set(const std::string& key, const std::string& value) { m_values[key] = value; }
    void        set_double(const std::string& key, double v);
    void        set_bool(const std::string& key, bool v) { set(key, v ? "1" : "0"); }

private:
    std::map<std::string, std::string> m_values;
};

} // namespace mrgui

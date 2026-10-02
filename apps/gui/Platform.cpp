#include "Platform.hpp"

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <thread>

#include <fcntl.h>
#include <spawn.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

extern char** environ;

namespace fs = std::filesystem;

namespace mrgui {

namespace {

std::string env(const char* name)
{
    const char* v = std::getenv(name);
    return v ? std::string(v) : std::string();
}

std::string ensure_dir(const std::string& dir)
{
    std::error_code ec;
    fs::create_directories(dir, ec);
    return dir;
}

bool is_executable(const std::string& path)
{
    struct stat st;
    return ::stat(path.c_str(), &st) == 0 && S_ISREG(st.st_mode) && ::access(path.c_str(), X_OK) == 0;
}

std::vector<char*> to_argv(const std::vector<std::string>& args)
{
    std::vector<char*> argv;
    for (const std::string& a : args)
        argv.push_back(const_cast<char*>(a.c_str()));
    argv.push_back(nullptr);
    return argv;
}

std::string lower(std::string s)
{
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return char(std::tolower(c)); });
    return s;
}

} // namespace

std::string home_dir()
{
    std::string h = env("HOME");
    return h.empty() ? std::string("/") : h;
}

std::string config_dir()
{
    std::string base = env("XDG_CONFIG_HOME");
    if (base.empty())
        base = home_dir() + "/.config";
    return ensure_dir(base + "/meshrepair");
}

std::string cache_dir()
{
    std::string base = env("XDG_CACHE_HOME");
    if (base.empty())
        base = home_dir() + "/.cache";
    return ensure_dir(base + "/meshrepair");
}

std::string find_in_path(const std::string& exe)
{
    if (exe.find('/') != std::string::npos)
        return is_executable(exe) ? exe : std::string();
    std::stringstream ss(env("PATH"));
    std::string       dir;
    while (std::getline(ss, dir, ':')) {
        if (dir.empty())
            continue;
        const std::string p = dir + "/" + exe;
        if (is_executable(p))
            return p;
    }
    return {};
}

bool run_capture(const std::vector<std::string>& args, std::string& output, int& exit_code)
{
    output.clear();
    exit_code = -1;
    int pipefd[2];
    if (::pipe(pipefd) != 0)
        return false;
    posix_spawn_file_actions_t fa;
    posix_spawn_file_actions_init(&fa);
    posix_spawn_file_actions_addopen(&fa, 0, "/dev/null", O_RDONLY, 0);
    posix_spawn_file_actions_adddup2(&fa, pipefd[1], 1);
    posix_spawn_file_actions_addclose(&fa, pipefd[0]);
    posix_spawn_file_actions_addclose(&fa, pipefd[1]);
    std::vector<char*> argv = to_argv(args);
    pid_t              pid  = 0;
    const int          rc   = posix_spawnp(&pid, argv[0], &fa, nullptr, argv.data(), environ);
    posix_spawn_file_actions_destroy(&fa);
    ::close(pipefd[1]);
    if (rc != 0) {
        ::close(pipefd[0]);
        return false;
    }
    char buf[4096];
    for (;;) {
        const ssize_t n = ::read(pipefd[0], buf, sizeof(buf));
        if (n > 0)
            output.append(buf, size_t(n));
        else if (n < 0 && errno == EINTR)
            continue;
        else
            break;
    }
    ::close(pipefd[0]);
    int status = 0;
    while (::waitpid(pid, &status, 0) < 0 && errno == EINTR) {
    }
    exit_code = WIFEXITED(status) ? WEXITSTATUS(status) : -1;
    return true;
}

bool launch_detached(const std::vector<std::string>& args, std::string& error)
{
    if (args.empty()) {
        error = "Empty command";
        return false;
    }
    posix_spawn_file_actions_t fa;
    posix_spawn_file_actions_init(&fa);
    posix_spawn_file_actions_addopen(&fa, 0, "/dev/null", O_RDONLY, 0);
    posix_spawn_file_actions_addopen(&fa, 1, "/dev/null", O_WRONLY, 0);
    posix_spawn_file_actions_addopen(&fa, 2, "/dev/null", O_WRONLY, 0);
    posix_spawnattr_t attr;
    posix_spawnattr_init(&attr);
#ifdef POSIX_SPAWN_SETSID
    posix_spawnattr_setflags(&attr, POSIX_SPAWN_SETSID);
#endif
    std::vector<char*> argv = to_argv(args);
    pid_t              pid  = 0;
    const int          rc   = posix_spawnp(&pid, argv[0], &fa, &attr, argv.data(), environ);
    posix_spawn_file_actions_destroy(&fa);
    posix_spawnattr_destroy(&attr);
    if (rc != 0) {
        error = "Could not start " + args[0] + ": " + std::strerror(rc);
        return false;
    }
    // Reap the child when it exits so that it does not linger as a zombie.
    std::thread([pid] {
        int status;
        while (::waitpid(pid, &status, 0) < 0 && errno == EINTR) {
        }
    }).detach();
    return true;
}

std::vector<std::string> split_command(const std::string& cmd)
{
    std::vector<std::string> out;
    std::string              cur;
    char                     quote = 0;
    bool                     have  = false;
    for (char c : cmd) {
        if (quote) {
            if (c == quote)
                quote = 0;
            else
                cur += c;
        } else if (c == '"' || c == '\'') {
            quote = c;
            have  = true;
        } else if (std::isspace((unsigned char)c)) {
            if (have || !cur.empty())
                out.push_back(cur);
            cur.clear();
            have = false;
        } else
            cur += c;
    }
    if (have || !cur.empty())
        out.push_back(cur);
    return out;
}

std::vector<SlicerApp> detect_slicers(const std::string& custom_command)
{
    std::vector<SlicerApp> apps;
    if (!custom_command.empty()) {
        std::vector<std::string> cmd = split_command(custom_command);
        if (!cmd.empty())
            apps.push_back({ "Custom slicer", custom_command, cmd });
    }

    struct Known
    {
        const char*              name;
        std::vector<const char*> executables;
        std::vector<const char*> flatpaks;
        const char*              appimage_pattern; // lower case substring
    };
    const Known known[] = {
        { "Bambu Studio", { "bambu-studio", "bambustudio", "BambuStudio", "bambu_studio" }, { "com.bambulab.BambuStudio" },
          "bambu" },
        { "OrcaSlicer", { "orca-slicer", "orcaslicer", "OrcaSlicer" },
          { "io.github.softfever.OrcaSlicer", "com.orcaslicer.OrcaSlicer" }, "orca" },
        { "PrusaSlicer", { "prusa-slicer", "PrusaSlicer", "prusaslicer" }, { "com.prusa3d.PrusaSlicer" }, "prusaslicer" },
    };

    // Installed flatpaks.
    std::vector<std::string> flatpaks;
    if (!find_in_path("flatpak").empty()) {
        std::string out;
        int         code = 0;
        if (run_capture({ "flatpak", "list", "--app", "--columns=application" }, out, code) && code == 0) {
            std::stringstream ss(out);
            std::string       line;
            while (std::getline(ss, line))
                if (!line.empty())
                    flatpaks.push_back(line);
        }
    }

    // AppImages in the usual places.
    std::vector<std::string> appimages;
    for (const std::string& dir : { home_dir() + "/Applications", home_dir() + "/.local/bin", home_dir() + "/Downloads",
                                    home_dir() + "/AppImages", std::string("/opt") }) {
        std::error_code ec;
        for (fs::directory_iterator it(dir, ec), end; !ec && it != end; it.increment(ec)) {
            const std::string p = it->path().string();
            if (lower(p).size() > 9 && lower(p).compare(lower(p).size() - 9, 9, ".appimage") == 0 && is_executable(p))
                appimages.push_back(p);
        }
    }

    for (const Known& k : known) {
        bool found = false;
        for (const char* exe : k.executables) {
            const std::string p = find_in_path(exe);
            if (!p.empty()) {
                apps.push_back({ k.name, p, { p } });
                found = true;
                break;
            }
        }
        for (const char* id : k.flatpaks)
            if (!found && std::find(flatpaks.begin(), flatpaks.end(), id) != flatpaks.end()) {
                apps.push_back({ k.name, std::string("Flatpak ") + id,
                                 { "flatpak", "run", "--file-forwarding", id, "@@", "@FILE@", "@@" } });
                found = true;
            }
        for (const std::string& ai : appimages) {
            const std::string base = lower(fs::path(ai).filename().string());
            if (!found && base.find(k.appimage_pattern) != std::string::npos) {
                apps.push_back({ k.name, ai, { ai } });
                found = true;
            }
        }
    }
    return apps;
}

bool open_in_slicer(const SlicerApp& app, const std::string& file, std::string& error)
{
    std::vector<std::string> argv     = app.command;
    bool                     replaced = false;
    for (std::string& a : argv)
        if (a == "@FILE@") {
            a        = file;
            replaced = true;
        }
    if (!replaced)
        argv.push_back(file);
    return launch_detached(argv, error);
}

std::string find_ui_font()
{
    auto usable = [](const std::string& p) {
        const std::string l = lower(p);
        return l.size() > 4 && l.compare(l.size() - 4, 4, ".ttf") == 0 && fs::exists(p);
    };
    std::string out;
    int         code = 0;
    if (!find_in_path("fc-match").empty() &&
        run_capture({ "fc-match", "-s", "-f", "%{file}\n", "sans-serif:style=Regular" }, out, code) && code == 0) {
        std::stringstream ss(out);
        std::string       line;
        for (int i = 0; i < 12 && std::getline(ss, line); ++i)
            if (usable(line))
                return line;
    }
    for (const char* p : { "/usr/share/fonts/TTF/DejaVuSans.ttf", "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
                           "/usr/share/fonts/dejavu/DejaVuSans.ttf", "/usr/share/fonts/dejavu-sans-fonts/DejaVuSans.ttf",
                           "/usr/share/fonts/noto/NotoSans-Regular.ttf",
                           "/usr/share/fonts/truetype/noto/NotoSans-Regular.ttf",
                           "/usr/share/fonts/google-noto/NotoSans-Regular.ttf",
                           "/usr/share/fonts/liberation/LiberationSans-Regular.ttf",
                           "/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf" })
        if (usable(p))
            return p;
    return {};
}

// ---------------------------------------------------------------------------

void Settings::load(const std::string& path)
{
    std::ifstream f(path);
    std::string   line;
    while (std::getline(f, line)) {
        const size_t eq = line.find('=');
        if (eq == std::string::npos || line.empty() || line[0] == '#')
            continue;
        m_values[line.substr(0, eq)] = line.substr(eq + 1);
    }
}

bool Settings::save(const std::string& path) const
{
    std::ofstream f(path);
    if (!f)
        return false;
    f << "# MeshRepair settings\n";
    for (const auto& kv : m_values)
        f << kv.first << "=" << kv.second << "\n";
    return bool(f);
}

std::string Settings::get(const std::string& key, const std::string& def) const
{
    auto it = m_values.find(key);
    return it == m_values.end() ? def : it->second;
}

double Settings::get_double(const std::string& key, double def) const
{
    auto it = m_values.find(key);
    if (it == m_values.end())
        return def;
    char*        e;
    const double v = std::strtod(it->second.c_str(), &e);
    return e == it->second.c_str() ? def : v;
}

bool Settings::get_bool(const std::string& key, bool def) const
{
    auto it = m_values.find(key);
    return it == m_values.end() ? def : it->second == "1";
}

void Settings::set_double(const std::string& key, double v)
{
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%.17g", v);
    m_values[key] = buf;
}

} // namespace mrgui

// File open / save dialogs.
//
// Uses the desktop's native dialog through zenity (GNOME, XFCE, ...) or
// kdialog (KDE) when one of them is installed, running it in the background so
// the window keeps redrawing. Falls back to a built-in Dear ImGui browser, so
// the application works on any distribution and any desktop.

#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace mrgui {

struct FileFilter
{
    std::string              description; // "Mesh files"
    std::vector<std::string> extensions;  // { "stl", "obj", "3mf" }
};

class FileDialog
{
public:
    enum class Mode { Open, Save };
    enum class Backend { Auto, Zenity, KDialog, BuiltIn };

    ~FileDialog();

    // Starts a dialog. initial is a directory (open) or a suggested file path (save).
    void start(Mode mode, const std::string& title, const std::string& initial, const std::vector<FileFilter>& filters);

    // Call once per frame (inside the ImGui frame). Returns true once the
    // dialog has finished; path is empty when it was canceled.
    bool update(std::string& path);

    bool active() const { return m_active; }

    Backend backend = Backend::Auto;

private:
    void start_native(const std::vector<std::string>& argv);
    bool draw_builtin(std::string& path);

    Mode                    m_mode   = Mode::Open;
    bool                    m_active = false;
    std::string             m_title;
    std::vector<FileFilter> m_filters;

    // Native dialog running in a thread.
    struct Native
    {
        std::thread       thread;
        std::atomic<bool> done { false };
        std::string       result;
        bool              failed = false;
    };
    std::unique_ptr<Native> m_native;

    // Built-in browser state.
    bool        m_builtin       = false;
    bool        m_open_popup    = false;
    std::string m_dir;
    char        m_dir_buf[1024]  = {};
    char        m_name_buf[512]  = {};
    int         m_filter_index  = 0;
    std::string m_error;
    std::string m_selected;
};

} // namespace mrgui

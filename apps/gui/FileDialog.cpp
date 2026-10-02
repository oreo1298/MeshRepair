#include "FileDialog.hpp"

#include "Platform.hpp"

#include <imgui.h>

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <cstring>
#include <filesystem>

namespace fs = std::filesystem;

namespace mrgui {

namespace {

std::string lower(std::string s)
{
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return char(std::tolower(c)); });
    return s;
}

std::string upper(std::string s)
{
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return char(std::toupper(c)); });
    return s;
}

bool desktop_is_kde()
{
    const char* d = std::getenv("XDG_CURRENT_DESKTOP");
    return d && lower(d).find("kde") != std::string::npos;
}

void copy_to(char* dst, size_t n, const std::string& s)
{
    std::strncpy(dst, s.c_str(), n - 1);
    dst[n - 1] = '\0';
}

} // namespace

FileDialog::~FileDialog()
{
    if (m_native && m_native->thread.joinable()) {
        if (m_native->done) {
            m_native->thread.join();
        } else {
            // The dialog is still open (application quitting): let the thread
            // finish on its own and keep its state alive for it.
            m_native->thread.detach();
            (void)m_native.release();
        }
    }
}

void FileDialog::start(Mode mode, const std::string& title, const std::string& initial,
                       const std::vector<FileFilter>& filters)
{
    if (m_active)
        return;
    m_mode    = mode;
    m_title   = title;
    m_filters = filters;
    m_active  = true;
    m_builtin = false;
    m_error.clear();

    // Initial directory / file name for every backend.
    std::error_code ec;
    fs::path        init(initial.empty() ? home_dir() : initial);
    if (fs::is_directory(init, ec)) {
        m_dir = init.string();
        m_name_buf[0] = '\0';
    } else {
        m_dir = init.has_parent_path() ? init.parent_path().string() : home_dir();
        copy_to(m_name_buf, sizeof(m_name_buf), mode == Mode::Save ? init.filename().string() : std::string());
    }
    if (!fs::is_directory(m_dir, ec))
        m_dir = home_dir();

    Backend b = backend;
    if (b == Backend::Auto) {
        const bool zenity  = !find_in_path("zenity").empty();
        const bool kdialog = !find_in_path("kdialog").empty();
        if (kdialog && (desktop_is_kde() || !zenity))
            b = Backend::KDialog;
        else if (zenity)
            b = Backend::Zenity;
        else
            b = Backend::BuiltIn;
    }

    const std::string init_path = (fs::path(m_dir) / m_name_buf).string();
    if (b == Backend::Zenity) {
        std::vector<std::string> argv { "zenity", "--file-selection", "--title=" + title };
        if (mode == Mode::Save) {
            argv.push_back("--save");
            argv.push_back("--confirm-overwrite");
            argv.push_back("--filename=" + init_path);
        } else
            argv.push_back("--filename=" + m_dir + "/");
        for (const FileFilter& f : filters) {
            std::string pattern;
            for (const std::string& e : f.extensions)
                pattern += " *." + lower(e) + " *." + upper(e);
            argv.push_back("--file-filter=" + f.description + " |" + pattern);
        }
        argv.push_back("--file-filter=All files | *");
        start_native(argv);
    } else if (b == Backend::KDialog) {
        std::string filter;
        for (const FileFilter& f : filters) {
            std::string pattern;
            for (const std::string& e : f.extensions)
                pattern += (pattern.empty() ? "" : " ") + std::string("*.") + lower(e) + " *." + upper(e);
            filter += pattern + "|" + f.description + "\n";
        }
        filter += "*|All files";
        start_native({ "kdialog", "--title", title, mode == Mode::Open ? "--getopenfilename" : "--getsavefilename",
                       mode == Mode::Open ? m_dir : init_path, filter });
    } else {
        m_builtin    = true;
        m_open_popup = true;
        copy_to(m_dir_buf, sizeof(m_dir_buf), m_dir);
    }
}

void FileDialog::start_native(const std::vector<std::string>& argv)
{
    m_native = std::make_unique<Native>();
    Native* n = m_native.get();
    n->thread = std::thread([n, argv] {
        std::string out;
        int         code = 0;
        if (!run_capture(argv, out, code) || (code != 0 && code != 1)) {
            n->failed = true;
        } else if (code == 0) {
            const size_t nl = out.find('\n');
            n->result       = out.substr(0, nl);
        }
        n->done = true;
    });
}

bool FileDialog::update(std::string& path)
{
    if (!m_active)
        return false;
    if (m_native) {
        if (!m_native->done)
            return false;
        m_native->thread.join();
        const bool failed = m_native->failed;
        path              = m_native->result;
        m_native.reset();
        if (failed) {
            // The helper could not run (e.g. no display access): use the
            // built-in dialog instead.
            m_builtin    = true;
            m_open_popup = true;
            copy_to(m_dir_buf, sizeof(m_dir_buf), m_dir);
            return false;
        }
        m_active = false;
        return true;
    }
    if (m_builtin)
        return draw_builtin(path);
    return false;
}

bool FileDialog::draw_builtin(std::string& path)
{
    const char* popup_id = "##meshrepair_file_dialog";
    const std::string id = m_title + popup_id;
    if (m_open_popup) {
        ImGui::OpenPopup(id.c_str());
        m_open_popup = false;
    }
    const ImGuiViewport* vp = ImGui::GetMainViewport();
    ImGui::SetNextWindowSize(ImVec2(vp->Size.x * 0.7f, vp->Size.y * 0.75f), ImGuiCond_Appearing);
    ImGui::SetNextWindowPos(vp->GetCenter(), ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));

    bool finished = false;
    bool open     = true;
    if (ImGui::BeginPopupModal(id.c_str(), &open)) {
        auto navigate = [&](const fs::path& p) {
            std::error_code ec;
            if (fs::is_directory(p, ec)) {
                m_dir = fs::weakly_canonical(p, ec).string();
                copy_to(m_dir_buf, sizeof(m_dir_buf), m_dir);
                m_error.clear();
            } else
                m_error = "Not a directory: " + p.string();
        };

        if (ImGui::Button("Up"))
            navigate(fs::path(m_dir).parent_path());
        ImGui::SameLine();
        if (ImGui::Button("Home"))
            navigate(home_dir());
        for (const char* place : { "Desktop", "Downloads", "Documents" }) {
            const fs::path p = fs::path(home_dir()) / place;
            std::error_code ec;
            if (fs::is_directory(p, ec)) {
                ImGui::SameLine();
                if (ImGui::Button(place))
                    navigate(p);
            }
        }
        ImGui::SetNextItemWidth(-1);
        if (ImGui::InputText("##dir", m_dir_buf, sizeof(m_dir_buf), ImGuiInputTextFlags_EnterReturnsTrue))
            navigate(m_dir_buf);

        // Directory listing.
        std::vector<std::string> dirs, files;
        {
            std::error_code ec;
            const FileFilter* filter =
                m_filter_index < int(m_filters.size()) ? &m_filters[size_t(m_filter_index)] : nullptr;
            for (fs::directory_iterator it(m_dir, ec), end; !ec && it != end; it.increment(ec)) {
                const std::string name = it->path().filename().string();
                if (name.empty() || name[0] == '.')
                    continue;
                std::error_code ec2;
                if (it->is_directory(ec2))
                    dirs.push_back(name);
                else {
                    bool ok = filter == nullptr;
                    if (filter) {
                        const std::string ext = lower(it->path().extension().string());
                        for (const std::string& e : filter->extensions)
                            ok |= ext == "." + lower(e);
                    }
                    if (ok)
                        files.push_back(name);
                }
            }
            if (ec)
                m_error = "Cannot read directory: " + ec.message();
        }
        std::sort(dirs.begin(), dirs.end());
        std::sort(files.begin(), files.end());

        const float footer = ImGui::GetFrameHeightWithSpacing() * (m_mode == Mode::Save ? 3.2f : 2.2f);
        bool        accept = false;
        if (ImGui::BeginChild("##listing", ImVec2(0, -footer), ImGuiChildFlags_Borders)) {
            for (const std::string& d : dirs) {
                const std::string label = "[dir]  " + d;
                if (ImGui::Selectable(label.c_str(), false, ImGuiSelectableFlags_AllowDoubleClick) &&
                    ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) {
                    navigate(fs::path(m_dir) / d);
                    break;
                }
            }
            for (const std::string& f : files) {
                const bool selected = f == m_name_buf;
                if (ImGui::Selectable(f.c_str(), selected, ImGuiSelectableFlags_AllowDoubleClick)) {
                    copy_to(m_name_buf, sizeof(m_name_buf), f);
                    if (ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left))
                        accept = true;
                }
            }
        }
        ImGui::EndChild();

        if (m_mode == Mode::Save) {
            ImGui::TextUnformatted("File name:");
            ImGui::SameLine();
            ImGui::SetNextItemWidth(-1);
            if (ImGui::InputText("##name", m_name_buf, sizeof(m_name_buf), ImGuiInputTextFlags_EnterReturnsTrue))
                accept = true;
        }
        if (!m_filters.empty()) {
            std::vector<std::string> labels;
            for (const FileFilter& f : m_filters) {
                std::string l = f.description + " (";
                for (size_t i = 0; i < f.extensions.size(); ++i)
                    l += (i ? ", *." : "*.") + f.extensions[i];
                labels.push_back(l + ")");
            }
            labels.push_back("All files");
            ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x * 0.45f);
            if (ImGui::BeginCombo("##filter", labels[size_t(std::min<int>(m_filter_index, int(labels.size()) - 1))].c_str())) {
                for (int i = 0; i < int(labels.size()); ++i)
                    if (ImGui::Selectable(labels[size_t(i)].c_str(), i == m_filter_index))
                        m_filter_index = i;
                ImGui::EndCombo();
            }
            ImGui::SameLine();
        }
        if (!m_error.empty()) {
            ImGui::TextColored(ImVec4(1, 0.4f, 0.4f, 1), "%s", m_error.c_str());
            ImGui::SameLine();
        }
        const float bw = ImGui::CalcTextSize("Cancel").x + ImGui::GetStyle().FramePadding.x * 2;
        ImGui::SetCursorPosX(ImGui::GetCursorPosX() + std::max(0.0f, ImGui::GetContentRegionAvail().x - bw * 2.4f));
        if (ImGui::Button(m_mode == Mode::Open ? "Open" : "Save", ImVec2(bw * 1.2f, 0)))
            accept = true;
        ImGui::SameLine();
        if (ImGui::Button("Cancel", ImVec2(bw * 1.1f, 0)) || ImGui::IsKeyPressed(ImGuiKey_Escape)) {
            path.clear();
            finished = true;
        }

        if (accept && m_name_buf[0]) {
            fs::path p = fs::path(m_dir) / m_name_buf;
            std::error_code ec;
            if (m_mode == Mode::Open) {
                if (fs::is_regular_file(p, ec)) {
                    path     = p.string();
                    finished = true;
                } else
                    m_error = "File not found";
            } else {
                // Add the default extension when none is given.
                if (!p.has_extension() && !m_filters.empty() && !m_filters[0].extensions.empty())
                    p += "." + lower(m_filters[0].extensions[0]);
                path     = p.string();
                finished = true;
            }
        }
        if (finished)
            ImGui::CloseCurrentPopup();
        ImGui::EndPopup();
    }
    if (!open) { // closed with the title bar button
        path.clear();
        finished = true;
    }
    if (finished) {
        m_active  = false;
        m_builtin = false;
    }
    return finished;
}

} // namespace mrgui

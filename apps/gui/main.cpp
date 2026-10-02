// meshrepair-gui: open a mesh, see what is wrong with it, repair it, save it or
// send it straight to the slicer.

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#include "FileDialog.hpp"
#include "GL.hpp"
#include "Platform.hpp"
#include "Png.hpp"
#include "Viewer.hpp"

#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

#include <meshrepair/Diagnostics.hpp>
#include <meshrepair/IO.hpp>
#include <meshrepair/Repair.hpp>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <filesystem>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#ifndef MESHREPAIR_VERSION
#define MESHREPAIR_VERSION "0.1.0"
#endif

using namespace meshrepair;
namespace fs = std::filesystem;

namespace mrgui {

namespace {

const std::vector<FileFilter> kOpenFilters { { "Mesh files", { "stl", "obj", "3mf" } } };
const std::vector<FileFilter> kSaveFilters { { "STL", { "stl" } }, { "3MF", { "3mf" } }, { "OBJ", { "obj" } } };

const ImVec4 kRed(0.95f, 0.36f, 0.36f, 1.0f);
const ImVec4 kGreen(0.35f, 0.85f, 0.45f, 1.0f);
const ImVec4 kDim(0.62f, 0.65f, 0.70f, 1.0f);

double now_seconds()
{
    using namespace std::chrono;
    return duration<double>(steady_clock::now().time_since_epoch()).count();
}

std::string format_count(size_t n)
{
    std::string s = std::to_string(n);
    for (int i = int(s.size()) - 3; i > 0; i -= 3)
        s.insert(size_t(i), ",");
    return s;
}

std::string format_number(double v)
{
    char buf[64];
    if (std::abs(v) >= 1e6 || (std::abs(v) < 1e-3 && v != 0.0))
        std::snprintf(buf, sizeof(buf), "%.4g", v);
    else
        std::snprintf(buf, sizeof(buf), "%.2f", v);
    return buf;
}

std::string file_name(const std::string& path) { return fs::path(path).filename().string(); }

std::string stem_of(const std::string& path) { return fs::path(path).stem().string(); }

std::vector<uint8_t> classes_for_original(const Mesh& mesh, const DiagnosticsDetail& detail)
{
    std::vector<uint8_t> cls(mesh.faces.size(), FaceNormal);
    for (int f : detail.inverted_shell_faces)
        if (f >= 0 && size_t(f) < cls.size())
            cls[size_t(f)] = FaceInverted;
    return cls;
}

std::vector<uint8_t> classes_for_repaired(const RepairResult& r)
{
    std::vector<uint8_t> cls(r.face_origin.size(), FaceNormal);
    for (size_t i = 0; i < cls.size(); ++i)
        cls[i] = r.face_origin[i] == meshrepair::FaceFilled ? uint8_t(FaceNew)
                 : r.face_origin[i] == meshrepair::FaceSplit ? uint8_t(mrgui::FaceSplit)
                                                             : uint8_t(FaceNormal);
    return cls;
}

size_t problem_count(const MeshDiagnostics& d)
{
    return d.open_edges + d.non_manifold_edges + d.non_manifold_vertices + d.inconsistent_edges + d.inverted_shells +
           d.visible_back_faces;
}

} // namespace

// ---------------------------------------------------------------------------

struct Model
{
    std::string path;
    std::string name;

    Mesh                 original;
    MeshDiagnostics      diag_original;
    DiagnosticsDetail    detail_original;
    std::vector<uint8_t> class_original;

    bool                 repaired = false;
    Mesh                 fixed;
    RepairResult         result;
    DiagnosticsDetail    detail_fixed;
    std::vector<uint8_t> class_fixed;
    double               repair_seconds = 0.0;
    std::string          saved_path; // where the current repaired mesh was saved
};

// Background work (loading or repairing) with progress reporting.
struct Job
{
    enum class Kind { Load, Repair } kind = Kind::Load;

    std::thread       thread;
    std::atomic<bool> done { false };
    std::atomic<bool> cancel { false };
    std::atomic<int>  percent { 0 };
    std::mutex        mutex;
    std::string       stage;
    double            started = 0.0;

    // Results.
    std::unique_ptr<Model> model; // Load
    Mesh                   fixed; // Repair
    RepairResult           result;
    DiagnosticsDetail      detail;
    std::string            error;

    void set_stage(const char* s, int p)
    {
        std::lock_guard<std::mutex> lk(mutex);
        stage   = s;
        percent = p;
    }
    std::string get_stage()
    {
        std::lock_guard<std::mutex> lk(mutex);
        return stage;
    }
};

class App
{
public:
    bool init(int argc, char** argv);
    int  run();
    void shutdown();

private:
    // Actions.
    void open_file(const std::string& path);
    void start_repair();
    void save_repaired(const std::string& path);
    void send_to_slicer();
    void request_open();
    void request_save();
    void upload_current();
    void set_status(const std::string& msg, bool error = false);
    void add_recent(const std::string& path);

    // Per frame.
    void poll_job();
    void poll_dialog();
    void handle_shortcuts();
    void handle_viewport_input();
    void draw_ui();
    void draw_menu();
    void draw_panel();
    void draw_diagnostics_table();
    void draw_repair_section();
    void draw_options();
    void draw_viewport_overlay();
    void draw_about();
    void render_frame();

    void load_settings();
    void save_settings();

    GLFWwindow*            m_window = nullptr;
    float                  m_scale  = 1.0f;
    Viewer                 m_viewer;
    Camera                 m_camera;
    ViewSettings           m_view;
    RepairOptions          m_options;
    bool                   m_auto_weld   = true;
    bool                   m_auto_stitch = true;
    bool                   m_save_ascii  = false;
    std::unique_ptr<Model> m_model;
    std::unique_ptr<Job>   m_job;
    bool                   m_show_repaired = true;
    FileDialog             m_dialog;
    enum class DialogPurpose { None, Open, Save } m_dialog_purpose = DialogPurpose::None;
    std::vector<SlicerApp> m_slicers;
    int                    m_slicer_index = 0;
    char                   m_custom_slicer[512] = {};
    std::deque<std::string> m_recent;
    std::string            m_last_dir;
    std::string            m_status;
    bool                   m_status_error = false;
    double                 m_status_time  = 0.0;
    bool                   m_show_about   = false;
    bool                   m_quit         = false;
    float                  m_panel_width  = 380.0f;
    Settings               m_settings;
    std::vector<std::string> m_dropped;
    bool                   m_dragging   = false;
    bool                   m_panning    = false;
    double                 m_last_x = 0, m_last_y = 0;

    // Screenshot / batch mode for automated tests and documentation.
    bool        m_open_dialog_at_start = false;
    std::string m_screenshot;
    bool        m_screenshot_repair = false;
    int         m_screenshot_frames = 0;

public:
    void on_drop(int count, const char** paths)
    {
        for (int i = 0; i < count; ++i)
            m_dropped.push_back(paths[i]);
    }
};

// ---------------------------------------------------------------------------

static void glfw_error(int code, const char* desc) { std::fprintf(stderr, "GLFW error %d: %s\n", code, desc); }

bool App::init(int argc, char** argv)
{
    std::string initial_file;
    int         width = 0, height = 0;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--screenshot" && i + 1 < argc)
            m_screenshot = argv[++i];
        else if (a == "--repair")
            m_screenshot_repair = true;
        else if (a == "--open-dialog")
            m_open_dialog_at_start = true;
        else if (a == "--size" && i + 1 < argc)
            std::sscanf(argv[++i], "%dx%d", &width, &height);
        else if (a == "-h" || a == "--help") {
            std::printf("Usage: meshrepair-gui [file.stl|obj|3mf]\n"
                        "       meshrepair-gui --screenshot out.png [--repair] [--size WxH] file\n");
            std::exit(0);
        } else if (a == "--version") {
            std::printf("meshrepair-gui %s\n", MESHREPAIR_VERSION);
            std::exit(0);
        } else
            initial_file = a;
    }

    glfwSetErrorCallback(glfw_error);
    if (!glfwInit()) {
        std::fprintf(stderr, "Could not initialise GLFW (no display?)\n");
        return false;
    }
    load_settings();

    m_scale = ImGui_ImplGlfw_GetContentScaleForMonitor(glfwGetPrimaryMonitor());
    if (!(m_scale > 0.0f))
        m_scale = 1.0f;

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_SAMPLES, 4);
    if (!m_screenshot.empty())
        glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
    // Lets the desktop match the window to io.github.oreo1298.MeshRepair.desktop (icon, name).
    glfwWindowHintString(GLFW_X11_CLASS_NAME, "MeshRepair");
    glfwWindowHintString(GLFW_X11_INSTANCE_NAME, "meshrepair");
#ifdef GLFW_WAYLAND_APP_ID
    glfwWindowHintString(GLFW_WAYLAND_APP_ID, "io.github.oreo1298.MeshRepair");
#endif
    if (width <= 0 || height <= 0) {
        width  = int(m_settings.get_double("window.width", 1360 * m_scale));
        height = int(m_settings.get_double("window.height", 860 * m_scale));
    }
    m_window = glfwCreateWindow(width, height, "MeshRepair", nullptr, nullptr);
    if (!m_window) {
        std::fprintf(stderr, "Could not create an OpenGL 3.3 window\n");
        return false;
    }
    glfwSetWindowUserPointer(m_window, this);
    glfwSetDropCallback(m_window, [](GLFWwindow* w, int count, const char** paths) {
        static_cast<App*>(glfwGetWindowUserPointer(w))->on_drop(count, paths);
    });
    glfwMakeContextCurrent(m_window);
    glfwSwapInterval(1);

    if (const char* missing = gl::load([](const char* name) { return reinterpret_cast<void*>(glfwGetProcAddress(name)); })) {
        std::fprintf(stderr, "OpenGL function %s not available\n", missing);
        return false;
    }

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io    = ImGui::GetIO();
    io.IniFilename = nullptr; // layout is fixed, nothing to remember
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;

    ImGui::StyleColorsDark();
    ImGuiStyle& style       = ImGui::GetStyle();
    style.WindowRounding    = 6.0f;
    style.FrameRounding     = 4.0f;
    style.GrabRounding      = 4.0f;
    style.PopupRounding     = 4.0f;
    style.ChildRounding     = 4.0f;
    style.FramePadding      = ImVec2(8, 5);
    style.ItemSpacing       = ImVec2(8, 6);
    style.WindowBorderSize  = 0.0f;
    ImVec4* c               = style.Colors;
    c[ImGuiCol_WindowBg]    = ImVec4(0.13f, 0.14f, 0.16f, 1.0f);
    c[ImGuiCol_ChildBg]     = ImVec4(0.11f, 0.12f, 0.14f, 1.0f);
    c[ImGuiCol_PopupBg]     = ImVec4(0.15f, 0.16f, 0.18f, 0.98f);
    c[ImGuiCol_Header]      = ImVec4(0.20f, 0.23f, 0.27f, 1.0f);
    c[ImGuiCol_HeaderHovered] = ImVec4(0.26f, 0.30f, 0.35f, 1.0f);
    c[ImGuiCol_HeaderActive]  = ImVec4(0.30f, 0.34f, 0.40f, 1.0f);
    c[ImGuiCol_Button]        = ImVec4(0.22f, 0.25f, 0.29f, 1.0f);
    c[ImGuiCol_ButtonHovered] = ImVec4(0.29f, 0.33f, 0.38f, 1.0f);
    c[ImGuiCol_ButtonActive]  = ImVec4(0.34f, 0.38f, 0.44f, 1.0f);
    c[ImGuiCol_FrameBg]       = ImVec4(0.19f, 0.21f, 0.24f, 1.0f);
    c[ImGuiCol_CheckMark]     = ImVec4(0.0f, 0.75f, 0.35f, 1.0f);
    c[ImGuiCol_PlotHistogram] = ImVec4(0.0f, 0.68f, 0.26f, 1.0f);
    c[ImGuiCol_TableRowBgAlt] = ImVec4(1.0f, 1.0f, 1.0f, 0.03f);
    c[ImGuiCol_TitleBg]       = ImVec4(0.17f, 0.18f, 0.21f, 1.0f);
    c[ImGuiCol_TitleBgActive] = ImVec4(0.20f, 0.22f, 0.26f, 1.0f);
    c[ImGuiCol_MenuBarBg]     = ImVec4(0.11f, 0.12f, 0.14f, 1.0f);
    style.ScaleAllSizes(m_scale);
    style.FontScaleDpi = m_scale;
    style.FontSizeBase = 15.0f;

    const std::string font = find_ui_font();
    if (font.empty() || !io.Fonts->AddFontFromFileTTF(font.c_str()))
        io.Fonts->AddFontDefault();

    ImGui_ImplGlfw_InitForOpenGL(m_window, true);
    ImGui_ImplOpenGL3_Init("#version 330 core");

    std::string error;
    if (!m_viewer.init(error)) {
        std::fprintf(stderr, "%s\n", error.c_str());
        return false;
    }

    std::strncpy(m_custom_slicer, m_settings.get("slicer.custom").c_str(), sizeof(m_custom_slicer) - 1);
    m_slicers = detect_slicers(m_custom_slicer);
    const std::string preferred = m_settings.get("slicer.preferred");
    for (size_t i = 0; i < m_slicers.size(); ++i)
        if (m_slicers[i].name == preferred)
            m_slicer_index = int(i);

    // MESHREPAIR_FILE_DIALOG=builtin|zenity|kdialog overrides the automatic choice.
    if (const char* fd = std::getenv("MESHREPAIR_FILE_DIALOG")) {
        const std::string v = fd;
        m_dialog.backend    = v == "builtin"  ? FileDialog::Backend::BuiltIn
                              : v == "zenity" ? FileDialog::Backend::Zenity
                              : v == "kdialog" ? FileDialog::Backend::KDialog
                                               : FileDialog::Backend::Auto;
    }

    if (!initial_file.empty())
        open_file(initial_file);
    if (!m_screenshot.empty()) {
        // Batch mode: finish loading (and repairing) synchronously.
        while (m_job) {
            m_job->thread.join();
            m_job->done = true;
            poll_job();
            if (m_screenshot_repair && m_model && !m_model->repaired && !m_job)
                start_repair();
        }
    }
    return true;
}

void App::load_settings()
{
    m_settings.load(config_dir() + "/meshrepair.ini");
    m_last_dir                          = m_settings.get("last_dir", home_dir());
    m_options.stitch_cracks             = m_settings.get_bool("repair.stitch", true);
    m_options.fill_holes                = m_settings.get_bool("repair.fill_holes", true);
    m_options.max_hole_edges            = size_t(m_settings.get_double("repair.max_hole_edges", 0));
    m_options.orient_outward            = m_settings.get_bool("repair.orient", true);
    m_options.remove_zero_volume_shells = m_settings.get_bool("repair.remove_zero_volume", true);
    m_options.min_shell_volume_ratio    = m_settings.get_double("repair.min_shell_ratio", 0.0);
    m_options.fix_degenerate_faces      = m_settings.get_bool("repair.fix_degenerate", true);
    m_options.separate_coincident_vertices = m_settings.get_bool("repair.separate", true);
    m_auto_weld                         = m_settings.get_bool("repair.auto_weld", true);
    m_auto_stitch                       = m_settings.get_bool("repair.auto_stitch", true);
    m_options.weld_tolerance            = m_settings.get_double("repair.weld_tol", 0.0001);
    m_options.stitch_tolerance          = m_settings.get_double("repair.stitch_tol", 0.01);
    m_save_ascii                        = m_settings.get_bool("save.ascii", false);
    m_view.show_issues                  = m_settings.get_bool("view.issues", true);
    m_view.xray_issues                  = m_settings.get_bool("view.xray", true);
    m_view.wireframe                    = m_settings.get_bool("view.wireframe", false);
    m_view.show_new_faces               = m_settings.get_bool("view.new_faces", true);
    m_view.show_grid                    = m_settings.get_bool("view.grid", true);
    for (int i = 0; i < 8; ++i) {
        const std::string r = m_settings.get("recent." + std::to_string(i));
        if (!r.empty())
            m_recent.push_back(r);
    }
}

void App::save_settings()
{
    if (!m_screenshot.empty())
        return;
    m_settings.set("last_dir", m_last_dir);
    m_settings.set_bool("repair.stitch", m_options.stitch_cracks);
    m_settings.set_bool("repair.fill_holes", m_options.fill_holes);
    m_settings.set_double("repair.max_hole_edges", double(m_options.max_hole_edges));
    m_settings.set_bool("repair.orient", m_options.orient_outward);
    m_settings.set_bool("repair.remove_zero_volume", m_options.remove_zero_volume_shells);
    m_settings.set_double("repair.min_shell_ratio", m_options.min_shell_volume_ratio);
    m_settings.set_bool("repair.fix_degenerate", m_options.fix_degenerate_faces);
    m_settings.set_bool("repair.separate", m_options.separate_coincident_vertices);
    m_settings.set_bool("repair.auto_weld", m_auto_weld);
    m_settings.set_bool("repair.auto_stitch", m_auto_stitch);
    m_settings.set_double("repair.weld_tol", m_options.weld_tolerance);
    m_settings.set_double("repair.stitch_tol", m_options.stitch_tolerance);
    m_settings.set_bool("save.ascii", m_save_ascii);
    m_settings.set_bool("view.issues", m_view.show_issues);
    m_settings.set_bool("view.xray", m_view.xray_issues);
    m_settings.set_bool("view.wireframe", m_view.wireframe);
    m_settings.set_bool("view.new_faces", m_view.show_new_faces);
    m_settings.set_bool("view.grid", m_view.show_grid);
    m_settings.set("slicer.custom", m_custom_slicer);
    if (m_slicer_index < int(m_slicers.size()))
        m_settings.set("slicer.preferred", m_slicers[size_t(m_slicer_index)].name);
    for (int i = 0; i < 8; ++i)
        m_settings.set("recent." + std::to_string(i), i < int(m_recent.size()) ? m_recent[size_t(i)] : "");
    if (m_window && !glfwGetWindowAttrib(m_window, GLFW_MAXIMIZED)) {
        int w, h;
        glfwGetWindowSize(m_window, &w, &h);
        m_settings.set_double("window.width", w);
        m_settings.set_double("window.height", h);
    }
    m_settings.save(config_dir() + "/meshrepair.ini");
}

void App::shutdown()
{
    if (m_job) {
        m_job->cancel = true;
        if (m_job->thread.joinable())
            m_job->thread.join();
        m_job.reset();
    }
    save_settings();
    m_viewer.destroy();
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    if (m_window)
        glfwDestroyWindow(m_window);
    glfwTerminate();
}

// ---------------------------------------------------------------------------
// Actions

void App::set_status(const std::string& msg, bool error)
{
    m_status       = msg;
    m_status_error = error;
    m_status_time  = now_seconds();
    if (error)
        std::fprintf(stderr, "%s\n", msg.c_str());
}

void App::add_recent(const std::string& path)
{
    std::error_code ec;
    const std::string abs = fs::absolute(path, ec).string();
    m_recent.erase(std::remove(m_recent.begin(), m_recent.end(), abs), m_recent.end());
    m_recent.push_front(abs);
    while (m_recent.size() > 8)
        m_recent.pop_back();
}

void App::open_file(const std::string& path)
{
    if (m_job)
        return;
    m_job          = std::make_unique<Job>();
    m_job->kind    = Job::Kind::Load;
    m_job->started = now_seconds();
    m_job->set_stage("Loading", 0);
    Job* job       = m_job.get();
    job->thread    = std::thread([job, path] {
        auto        model = std::make_unique<Model>();
        std::string error;
        model->path = path;
        model->name = file_name(path);
        if (!load_mesh(path, model->original, error)) {
            job->error = error;
        } else if (model->original.faces.empty()) {
            job->error = "The file contains no triangles: " + path;
        } else {
            job->set_stage("Analyzing", 60);
            model->diag_original  = analyze(model->original, &model->detail_original);
            model->class_original = classes_for_original(model->original, model->detail_original);
            job->model            = std::move(model);
        }
        job->done = true;
    });
}

void App::start_repair()
{
    if (m_job || !m_model)
        return;
    RepairOptions opt = m_options;
    if (m_auto_weld)
        opt.weld_tolerance = -1.0;
    if (m_auto_stitch)
        opt.stitch_tolerance = -1.0;
    m_job          = std::make_unique<Job>();
    m_job->kind    = Job::Kind::Repair;
    m_job->started = now_seconds();
    m_job->set_stage("Starting", 0);
    Job*        job  = m_job.get();
    const Mesh* orig = &m_model->original;
    job->thread      = std::thread([job, orig, opt] {
        job->fixed  = *orig;
        job->result = repair(
            job->fixed, opt, [job](const char* stage, int pct) { job->set_stage(stage, pct); },
            [job] { return job->cancel.load(); });
        if (job->result.success)
            analyze(job->fixed, &job->detail);
        job->done = true;
    });
}

void App::poll_job()
{
    if (!m_job || !m_job->done)
        return;
    if (m_job->thread.joinable())
        m_job->thread.join();
    std::unique_ptr<Job> job = std::move(m_job);
    if (job->kind == Job::Kind::Load) {
        if (!job->model) {
            set_status(job->error, true);
            return;
        }
        m_model = std::move(job->model);
        add_recent(m_model->path);
        m_last_dir = fs::path(m_model->path).parent_path().string();
        m_show_repaired = false;
        upload_current();
        m_camera.fit(m_viewer.bounds());
        m_camera.set_view(-60.0, 25.0);
        const size_t problems = problem_count(m_model->diag_original);
        if (problems == 0)
            set_status("Loaded " + m_model->name + ": no problems found.");
        else
            set_status("Loaded " + m_model->name + ": " + format_count(problems) + " problems found. Press Repair.");
        return;
    }
    // Repair finished.
    if (!m_model)
        return;
    if (job->result.canceled) {
        set_status("Repair canceled.");
        return;
    }
    if (!job->result.success) {
        set_status("Repair failed: " + job->result.error, true);
        return;
    }
    m_model->repaired       = true;
    m_model->fixed          = std::move(job->fixed);
    m_model->result         = std::move(job->result);
    m_model->detail_fixed   = std::move(job->detail);
    m_model->class_fixed    = classes_for_repaired(m_model->result);
    m_model->repair_seconds = now_seconds() - job->started;
    m_model->saved_path.clear();
    m_show_repaired = true;
    upload_current();
    const MeshDiagnostics& a = m_model->result.after;
    char buf[256];
    if (a.slicer_clean())
        std::snprintf(buf, sizeof(buf), "Repaired in %.2f s. The model is now watertight and manifold.",
                      m_model->repair_seconds);
    else if (a.watertight() && a.manifold() && a.inconsistent_edges == 0 && a.inverted_shells == 0)
        std::snprintf(buf, sizeof(buf),
                      "Repaired in %.2f s. Watertight and manifold, but the surface folds over itself in places.",
                      m_model->repair_seconds);
    else
        std::snprintf(buf, sizeof(buf), "Repaired in %.2f s, but %s problems remain.", m_model->repair_seconds,
                      format_count(problem_count(a)).c_str());
    set_status(buf, !a.slicer_clean());
}

void App::upload_current()
{
    if (!m_model) {
        m_viewer.clear_mesh();
        return;
    }
    if (m_show_repaired && m_model->repaired)
        m_viewer.set_mesh(m_model->fixed, m_model->class_fixed, m_model->detail_fixed,
                          !m_model->result.after.slicer_clean());
    else
        m_viewer.set_mesh(m_model->original, m_model->class_original, m_model->detail_original,
                          !m_model->diag_original.slicer_clean());
}

void App::save_repaired(const std::string& path)
{
    if (!m_model || !m_model->repaired)
        return;
    std::string error;
    FileFormat  fmt = format_from_path(path);
    if (fmt == FileFormat::Unknown) {
        set_status("Unknown file type, use .stl, .3mf or .obj: " + path, true);
        return;
    }
    if (fmt == FileFormat::STL && m_save_ascii)
        fmt = FileFormat::STLAscii;
    if (!save_mesh(path, m_model->fixed, error, fmt)) {
        set_status(error, true);
        return;
    }
    m_model->saved_path = path;
    m_last_dir          = fs::path(path).parent_path().string();
    set_status("Saved " + path);
}

void App::send_to_slicer()
{
    if (!m_model || !m_model->repaired || m_slicer_index >= int(m_slicers.size()))
        return;
    std::string file = m_model->saved_path;
    if (file.empty()) {
        // Not saved yet: hand over a copy from the cache directory.
        file = cache_dir() + "/" + stem_of(m_model->name) + "_repaired.stl";
        std::string error;
        if (!save_mesh(file, m_model->fixed, error, FileFormat::STL)) {
            set_status(error, true);
            return;
        }
    }
    const SlicerApp& app = m_slicers[size_t(m_slicer_index)];
    std::string      error;
    if (open_in_slicer(app, file, error))
        set_status("Opening " + file_name(file) + " in " + app.name + "...");
    else
        set_status(error, true);
}

void App::request_open()
{
    if (m_dialog.active() || m_job)
        return;
    m_dialog_purpose = DialogPurpose::Open;
    m_dialog.start(FileDialog::Mode::Open, "Open mesh", m_last_dir, kOpenFilters);
}

void App::request_save()
{
    if (m_dialog.active() || !m_model || !m_model->repaired)
        return;
    std::string ext = fs::path(m_model->name).extension().string();
    if (format_from_path(m_model->name) == FileFormat::Unknown)
        ext = ".stl";
    const std::string dir = fs::path(m_model->path).parent_path().string();
    m_dialog_purpose      = DialogPurpose::Save;
    m_dialog.start(FileDialog::Mode::Save, "Save repaired mesh",
                   (fs::path(dir.empty() ? m_last_dir : dir) / (stem_of(m_model->name) + "_repaired" + ext)).string(),
                   kSaveFilters);
}

void App::poll_dialog()
{
    std::string path;
    if (!m_dialog.update(path))
        return;
    const DialogPurpose purpose = m_dialog_purpose;
    m_dialog_purpose            = DialogPurpose::None;
    if (path.empty())
        return;
    if (purpose == DialogPurpose::Open)
        open_file(path);
    else if (purpose == DialogPurpose::Save)
        save_repaired(path);
}

// ---------------------------------------------------------------------------
// Input

void App::handle_shortcuts()
{
    const ImGuiIO& io = ImGui::GetIO();
    if (ImGui::Shortcut(ImGuiMod_Ctrl | ImGuiKey_O, ImGuiInputFlags_RouteGlobal))
        request_open();
    if (ImGui::Shortcut(ImGuiMod_Ctrl | ImGuiKey_S, ImGuiInputFlags_RouteGlobal))
        request_save();
    if (ImGui::Shortcut(ImGuiMod_Ctrl | ImGuiKey_R, ImGuiInputFlags_RouteGlobal))
        start_repair();
    if (ImGui::Shortcut(ImGuiMod_Ctrl | ImGuiKey_Q, ImGuiInputFlags_RouteGlobal))
        m_quit = true;
    if (m_job && ImGui::IsKeyPressed(ImGuiKey_Escape))
        m_job->cancel = true;
    if (io.WantTextInput || m_dialog.active())
        return;
    if (ImGui::IsKeyPressed(ImGuiKey_F, false) && m_viewer.has_mesh())
        m_camera.fit(m_viewer.bounds());
    if (ImGui::IsKeyPressed(ImGuiKey_W, false))
        m_view.wireframe = !m_view.wireframe;
    if (ImGui::IsKeyPressed(ImGuiKey_I, false))
        m_view.show_issues = !m_view.show_issues;
    if (ImGui::IsKeyPressed(ImGuiKey_Tab, false) && m_model && m_model->repaired) {
        m_show_repaired = !m_show_repaired;
        upload_current();
    }
    struct Preset
    {
        ImGuiKey key;
        double   yaw, pitch;
    };
    for (const Preset& p : { Preset { ImGuiKey_0, -60, 25 }, Preset { ImGuiKey_1, -90, 0 }, Preset { ImGuiKey_2, 90, 0 },
                             Preset { ImGuiKey_3, 0, 0 }, Preset { ImGuiKey_4, 180, 0 }, Preset { ImGuiKey_5, -90, 89 },
                             Preset { ImGuiKey_6, -90, -89 } })
        if (ImGui::IsKeyPressed(p.key, false))
            m_camera.set_view(p.yaw, p.pitch);
}

void App::handle_viewport_input()
{
    const ImGuiIO& io = ImGui::GetIO();
    double         x, y;
    glfwGetCursorPos(m_window, &x, &y);
    const bool over_view = !io.WantCaptureMouse && x > m_panel_width;
    const bool left      = glfwGetMouseButton(m_window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
    const bool right     = glfwGetMouseButton(m_window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;
    const bool middle    = glfwGetMouseButton(m_window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS;
    const bool shift     = io.KeyShift;

    if (!m_dragging && !m_panning && over_view) {
        if (left && !shift)
            m_dragging = true;
        else if (right || middle || (left && shift))
            m_panning = true;
        m_last_x = x;
        m_last_y = y;
    }
    if (m_dragging && !left)
        m_dragging = false;
    if (m_panning && !(right || middle || left))
        m_panning = false;

    const double dx = x - m_last_x, dy = y - m_last_y;
    m_last_x = x;
    m_last_y = y;
    if (m_dragging)
        m_camera.orbit(dx, dy);
    if (m_panning) {
        int w, h;
        glfwGetWindowSize(m_window, &w, &h);
        m_camera.pan(dx, dy, h);
    }
    if (over_view && io.MouseWheel != 0.0f)
        m_camera.zoom(io.MouseWheel);
    if (over_view && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left) && m_viewer.has_mesh())
        m_camera.fit(m_viewer.bounds());
}

// ---------------------------------------------------------------------------
// UI

void App::draw_menu()
{
    if (!ImGui::BeginMainMenuBar())
        return;
    if (ImGui::BeginMenu("File")) {
        if (ImGui::MenuItem("Open...", "Ctrl+O", false, !m_job))
            request_open();
        if (ImGui::BeginMenu("Open recent", !m_recent.empty() && !m_job)) {
            for (const std::string& r : m_recent)
                if (ImGui::MenuItem(r.c_str()))
                    open_file(r);
            ImGui::EndMenu();
        }
        if (ImGui::MenuItem("Reload", nullptr, false, m_model && !m_job))
            open_file(m_model->path);
        ImGui::Separator();
        const bool can_save = m_model && m_model->repaired;
        if (ImGui::MenuItem("Save repaired as...", "Ctrl+S", false, can_save))
            request_save();
        ImGui::MenuItem("Write ASCII STL", nullptr, &m_save_ascii);
        ImGui::Separator();
        if (ImGui::MenuItem("Quit", "Ctrl+Q"))
            m_quit = true;
        ImGui::EndMenu();
    }
    if (ImGui::BeginMenu("Repair")) {
        if (ImGui::MenuItem("Repair", "Ctrl+R", false, m_model && !m_job))
            start_repair();
        if (ImGui::MenuItem("Show original", "Tab", !m_show_repaired, m_model && m_model->repaired)) {
            m_show_repaired = !m_show_repaired;
            upload_current();
        }
        ImGui::EndMenu();
    }
    if (ImGui::BeginMenu("View")) {
        if (ImGui::MenuItem("Fit to window", "F", false, m_viewer.has_mesh()))
            m_camera.fit(m_viewer.bounds());
        ImGui::Separator();
        if (ImGui::MenuItem("Isometric", "0"))
            m_camera.set_view(-60, 25);
        if (ImGui::MenuItem("Front", "1"))
            m_camera.set_view(-90, 0);
        if (ImGui::MenuItem("Back", "2"))
            m_camera.set_view(90, 0);
        if (ImGui::MenuItem("Right", "3"))
            m_camera.set_view(0, 0);
        if (ImGui::MenuItem("Left", "4"))
            m_camera.set_view(180, 0);
        if (ImGui::MenuItem("Top", "5"))
            m_camera.set_view(-90, 89);
        if (ImGui::MenuItem("Bottom", "6"))
            m_camera.set_view(-90, -89);
        ImGui::Separator();
        ImGui::MenuItem("Show problems", "I", &m_view.show_issues);
        ImGui::MenuItem("Show hidden problems (X-ray)", nullptr, &m_view.xray_issues);
        ImGui::MenuItem("Highlight new / inside-out faces", nullptr, &m_view.show_new_faces);
        ImGui::MenuItem("Wireframe", "W", &m_view.wireframe);
        ImGui::MenuItem("Grid", nullptr, &m_view.show_grid);
        ImGui::EndMenu();
    }
    if (ImGui::BeginMenu("Help")) {
        if (ImGui::MenuItem("About MeshRepair"))
            m_show_about = true;
        ImGui::EndMenu();
    }
    ImGui::EndMainMenuBar();
}

void App::draw_diagnostics_table()
{
    const bool               rep = m_model->repaired;
    const MeshDiagnostics&   b   = m_model->diag_original;
    const MeshDiagnostics*   a   = rep ? &m_model->result.after : nullptr;
    const ImGuiTableFlags    tf  = ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp | ImGuiTableFlags_BordersInnerH;
    if (!ImGui::BeginTable("##diag", rep ? 3 : 2, tf))
        return;
    ImGui::TableSetupColumn("", ImGuiTableColumnFlags_WidthStretch, 2.2f);
    ImGui::TableSetupColumn("Original", ImGuiTableColumnFlags_WidthStretch, 1.0f);
    if (rep)
        ImGui::TableSetupColumn("Repaired", ImGuiTableColumnFlags_WidthStretch, 1.0f);
    ImGui::TableHeadersRow();

    auto row_count = [&](const char* label, size_t before, size_t after, bool is_problem, const char* tip) {
        ImGui::TableNextRow();
        ImGui::TableNextColumn();
        ImGui::TextUnformatted(label);
        if (tip && ImGui::IsItemHovered())
            ImGui::SetTooltip("%s", tip);
        ImGui::TableNextColumn();
        if (is_problem && before > 0)
            ImGui::TextColored(kRed, "%s", format_count(before).c_str());
        else
            ImGui::TextUnformatted(format_count(before).c_str());
        if (rep) {
            ImGui::TableNextColumn();
            if (is_problem)
                ImGui::TextColored(after > 0 ? kRed : kGreen, "%s", format_count(after).c_str());
            else
                ImGui::TextUnformatted(format_count(after).c_str());
        }
    };
    auto row_text = [&](const char* label, const std::string& before, const std::string& after) {
        ImGui::TableNextRow();
        ImGui::TableNextColumn();
        ImGui::TextUnformatted(label);
        ImGui::TableNextColumn();
        ImGui::TextUnformatted(before.c_str());
        if (rep) {
            ImGui::TableNextColumn();
            ImGui::TextUnformatted(after.c_str());
        }
    };
    auto status = [](const MeshDiagnostics& d) -> std::pair<const char*, ImVec4> {
        if (d.slicer_clean())
            return { d.clean() ? "OK" : "OK (minor)", kGreen };
        return { "Needs repair", kRed };
    };

    {
        ImGui::TableNextRow();
        ImGui::TableNextColumn();
        ImGui::TextUnformatted("Status");
        ImGui::TableNextColumn();
        auto s = status(b);
        ImGui::TextColored(s.second, "%s", s.first);
        if (rep) {
            ImGui::TableNextColumn();
            auto s2 = status(*a);
            ImGui::TextColored(s2.second, "%s", s2.first);
        }
    }
    const MeshDiagnostics& aa = a ? *a : b;
    row_count("Open edges", b.open_edges, aa.open_edges, true,
              "Edges used by only one face: the surface has a hole or a crack here.\nShown as red lines.");
    row_count("Holes", b.holes, aa.holes, true, "Boundary loops formed by the open edges.");
    row_count("Non-manifold edges", b.non_manifold_edges, aa.non_manifold_edges, true,
              "Edges shared by more than two faces (e.g. two bodies touching along an edge, internal walls).\n"
              "Shown as yellow lines.");
    row_count("Non-manifold vertices", b.non_manifold_vertices, aa.non_manifold_vertices, true,
              "Vertices where separate pieces of surface touch in a single point.\nShown as purple dots.");
    row_count("Flipped faces (edges)", b.inconsistent_edges, aa.inconsistent_edges, true,
              "Edges between neighbouring faces facing opposite ways.\nShown as orange lines.");
    row_count("Inside-out shells", b.inverted_shells, aa.inverted_shells, true,
              "Closed parts whose normals all point inwards.\nShown in orange.");
    row_count("Seen from behind", b.visible_back_faces, aa.visible_back_faces, true,
              "Bambu Studio's reversed face test: of 46 views from outside, how many see the back\n"
              "of a face first. Caused by inside-out parts or by surfaces folding over themselves\n"
              "(e.g. a single sheet that cannot enclose a volume). Shown in red.");
    row_count("Degenerate faces", b.degenerate_faces, aa.degenerate_faces, true,
              "Faces with (almost) zero area. Harmless for slicing in most cases.");
    row_count("Duplicate faces", b.duplicate_faces, aa.duplicate_faces, true, nullptr);
    row_count("Shells (parts)", b.shells, aa.shells, false, "Separate connected pieces of surface.");
    row_count("Faces", b.faces, aa.faces, false, nullptr);
    row_count("Vertices", b.vertices, aa.vertices, false, nullptr);
    row_text("Volume (mm³)", format_number(b.volume), a ? format_number(a->volume) : "");
    ImGui::EndTable();
    if (b.bbox.valid()) {
        const Vec3 s = b.bbox.size();
        ImGui::TextColored(kDim, "Size: %s x %s x %s mm", format_number(s.x).c_str(), format_number(s.y).c_str(),
                           format_number(s.z).c_str());
    }
}

void App::draw_options()
{
    ImGui::Checkbox("Close cracks", &m_options.stitch_cracks);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Merge nearby open edges and fix T-junctions before filling holes.");
    if (m_options.stitch_cracks) {
        ImGui::Indent();
        ImGui::Checkbox("Automatic crack width##stitch", &m_auto_stitch);
        if (!m_auto_stitch) {
            ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x * 0.5f);
            ImGui::InputDouble("Max gap (mm)", &m_options.stitch_tolerance, 0.001, 0.01, "%.4f");
            m_options.stitch_tolerance = std::max(0.0, m_options.stitch_tolerance);
        }
        ImGui::Unindent();
    }
    ImGui::Checkbox("Fill holes", &m_options.fill_holes);
    if (m_options.fill_holes) {
        ImGui::Indent();
        int max_edges = int(m_options.max_hole_edges);
        ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x * 0.5f);
        if (ImGui::InputInt("Max hole edges", &max_edges))
            m_options.max_hole_edges = size_t(std::max(0, max_edges));
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Only fill holes with at most this many edges (0 = all holes).");
        ImGui::Unindent();
    }
    ImGui::Checkbox("Orient normals outwards", &m_options.orient_outward);
    ImGui::Checkbox("Remove zero-volume shells", &m_options.remove_zero_volume_shells);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Remove leftovers such as internal walls or single sided sheets.");
    float ratio = float(m_options.min_shell_volume_ratio * 100.0);
    ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x * 0.5f);
    if (ImGui::SliderFloat("Remove small shells", &ratio, 0.0f, 10.0f, ratio == 0.0f ? "off" : "< %.2f %%"))
        m_options.min_shell_volume_ratio = double(ratio) / 100.0;
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Remove parts smaller than this percentage of the largest part's volume.");
    ImGui::Checkbox("Fix degenerate faces", &m_options.fix_degenerate_faces);
    ImGui::Checkbox("Separate touching parts", &m_options.separate_coincident_vertices);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Parts touching in an edge or a point are moved apart by a few micrometres, so that\n"
                          "they stay manifold when saved as STL (which does not store connectivity).");
    ImGui::Checkbox("Automatic vertex merge distance", &m_auto_weld);
    if (!m_auto_weld) {
        ImGui::Indent();
        ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x * 0.5f);
        ImGui::InputDouble("Merge distance (mm)", &m_options.weld_tolerance, 0.0001, 0.001, "%.5f");
        m_options.weld_tolerance = std::max(0.0, m_options.weld_tolerance);
        ImGui::Unindent();
    }
    if (ImGui::Button("Reset to defaults")) {
        m_options     = RepairOptions();
        m_options.weld_tolerance   = 0.0001;
        m_options.stitch_tolerance = 0.01;
        m_auto_weld   = true;
        m_auto_stitch = true;
    }
}

void App::draw_repair_section()
{
    const float w = ImGui::GetContentRegionAvail().x;
    if (m_job && m_job->kind == Job::Kind::Repair) {
        const std::string stage = m_job->get_stage();
        ImGui::ProgressBar(float(m_job->percent) / 100.0f, ImVec2(w, 0), stage.c_str());
        if (ImGui::Button("Cancel", ImVec2(w, 0)))
            m_job->cancel = true;
        return;
    }
    const bool disabled = !m_model || m_job;
    ImGui::BeginDisabled(disabled);
    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.0f, 0.60f, 0.26f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.0f, 0.70f, 0.31f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.0f, 0.52f, 0.22f, 1.0f));
    const char* label = m_model && m_model->repaired ? "Repair again" : "Repair";
    if (ImGui::Button(label, ImVec2(w, ImGui::GetFrameHeight() * 1.6f)))
        start_repair();
    ImGui::PopStyleColor(3);
    ImGui::EndDisabled();

    if (m_model && m_model->repaired) {
        const RepairStats& s = m_model->result.stats;
        std::vector<std::pair<const char*, size_t>> items = {
            { "Holes filled", s.holes_filled },
            { "Faces added", s.faces_added },
            { "Cracks closed (vertices)", s.boundary_vertices_stitched },
            { "T-junctions fixed", s.t_junctions_fixed },
            { "Non-manifold edges fixed", s.non_manifold_edges_fixed },
            { "Vertices split", s.non_manifold_vertices_split },
            { "Faces flipped", s.faces_flipped },
            { "Shells turned outside-in", s.shells_reoriented },
            { "Duplicate faces removed", s.duplicate_faces_removed },
            { "Degenerate faces removed", s.degenerate_faces_removed },
            { "Degenerate faces fixed", s.degenerate_faces_fixed },
            { "Vertices merged", s.vertices_welded },
            { "Shells removed", s.shells_removed },
            { "Unrepairable faces removed", s.nonorientable_faces_removed },
            { "Invalid faces removed", s.invalid_faces_removed },
            { "Touching parts separated (vertices)", s.vertices_separated },
            { "Holes left open", s.holes_left_open },
        };
        bool any = false;
        if (ImGui::TreeNodeEx("Changes", ImGuiTreeNodeFlags_DefaultOpen)) {
            for (const auto& it : items)
                if (it.second) {
                    ImGui::BulletText("%s: %s", it.first, format_count(it.second).c_str());
                    any = true;
                }
            if (!any)
                ImGui::TextColored(kDim, "Nothing had to be changed.");
            ImGui::TreePop();
        }

        ImGui::Spacing();
        if (ImGui::Button("Save repaired as...", ImVec2(w, 0)))
            request_save();
        // Slicer hand-over.
        if (m_slicers.empty()) {
            ImGui::BeginDisabled();
            ImGui::Button("Open in slicer", ImVec2(w, 0));
            ImGui::EndDisabled();
            if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
                ImGui::SetTooltip("No Bambu Studio, OrcaSlicer or PrusaSlicer installation found.\n"
                                  "Set a custom command under Settings.");
        } else {
            m_slicer_index           = std::min(m_slicer_index, int(m_slicers.size()) - 1);
            const SlicerApp&  app    = m_slicers[size_t(m_slicer_index)];
            const std::string button = "Open in " + app.name;
            const float       combo  = m_slicers.size() > 1 ? ImGui::GetFrameHeight() : 0.0f;
            if (ImGui::Button(button.c_str(), ImVec2(w - combo - (combo > 0 ? ImGui::GetStyle().ItemSpacing.x : 0), 0)))
                send_to_slicer();
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("%s", app.how.c_str());
            if (m_slicers.size() > 1) {
                ImGui::SameLine();
                if (ImGui::BeginCombo("##slicer", nullptr, ImGuiComboFlags_NoPreview | ImGuiComboFlags_PopupAlignLeft)) {
                    for (int i = 0; i < int(m_slicers.size()); ++i) {
                        const std::string l = m_slicers[size_t(i)].name + "  (" + m_slicers[size_t(i)].how + ")";
                        if (ImGui::Selectable(l.c_str(), i == m_slicer_index))
                            m_slicer_index = i;
                    }
                    ImGui::EndCombo();
                }
            }
        }
    }
}

void App::draw_panel()
{
    const ImGuiViewport* vp      = ImGui::GetMainViewport();
    const float          menu_h  = ImGui::GetFrameHeight();
    m_panel_width                = std::max(320.0f * m_scale, std::min(vp->WorkSize.x * 0.4f, 400.0f * m_scale));
    ImGui::SetNextWindowPos(ImVec2(vp->WorkPos.x, vp->WorkPos.y));
    ImGui::SetNextWindowSize(ImVec2(m_panel_width, vp->WorkSize.y));
    (void)menu_h;
    const ImGuiWindowFlags flags = ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
                                   ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoBringToFrontOnFocus;
    ImGui::Begin("##panel", nullptr, flags);

    const float w = ImGui::GetContentRegionAvail().x;
    if (!m_model) {
        ImGui::TextColored(kDim, "No model loaded.");
        ImGui::Spacing();
        ImGui::BeginDisabled(m_job != nullptr);
        if (ImGui::Button("Open mesh...", ImVec2(w, ImGui::GetFrameHeight() * 1.6f)))
            request_open();
        ImGui::EndDisabled();
        ImGui::TextColored(kDim, "or drop an STL, OBJ or 3MF file\non the window.");
        if (!m_recent.empty()) {
            ImGui::Spacing();
            ImGui::SeparatorText("Recent files");
            for (const std::string& r : m_recent) {
                if (ImGui::Selectable(file_name(r).c_str()) && !m_job)
                    open_file(r);
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("%s", r.c_str());
            }
        }
    } else {
        ImGui::TextUnformatted(m_model->name.c_str());
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("%s", m_model->path.c_str());
        ImGui::SameLine(w - ImGui::CalcTextSize("Open...").x - ImGui::GetStyle().FramePadding.x);
        ImGui::BeginDisabled(m_job != nullptr);
        if (ImGui::SmallButton("Open..."))
            request_open();
        ImGui::EndDisabled();
        ImGui::Spacing();
        draw_diagnostics_table();
        ImGui::Spacing();
        draw_repair_section();
    }

    ImGui::Spacing();
    if (ImGui::CollapsingHeader("Repair options"))
        draw_options();

    if (ImGui::CollapsingHeader("View")) {
        if (m_model && m_model->repaired) {
            int which = m_show_repaired ? 1 : 0;
            ImGui::RadioButton("Original", &which, 0);
            ImGui::SameLine();
            ImGui::RadioButton("Repaired", &which, 1);
            if ((which == 1) != m_show_repaired) {
                m_show_repaired = which == 1;
                upload_current();
            }
        }
        ImGui::Checkbox("Show problems (I)", &m_view.show_issues);
        ImGui::Checkbox("Show hidden problems (X-ray)", &m_view.xray_issues);
        ImGui::Checkbox("Highlight new / inside-out faces", &m_view.show_new_faces);
        ImGui::Checkbox("Wireframe (W)", &m_view.wireframe);
        ImGui::Checkbox("Grid", &m_view.show_grid);
        ImGui::TextColored(kDim, "Left drag: rotate   Right drag: pan\nWheel: zoom   F / double click: fit\n"
                                 "0-6: standard views   Tab: original/repaired");
    }
    if (ImGui::CollapsingHeader("Legend", ImGuiTreeNodeFlags_DefaultOpen)) {
        auto swatch = [](ImVec4 col, const char* text) {
            ImGui::ColorButton(text, col, ImGuiColorEditFlags_NoTooltip | ImGuiColorEditFlags_NoDragDrop,
                               ImVec2(ImGui::GetTextLineHeight(), ImGui::GetTextLineHeight()));
            ImGui::SameLine();
            ImGui::TextUnformatted(text);
        };
        swatch(ImVec4(1.0f, 0.22f, 0.22f, 1), "Open edge (hole / crack)");
        swatch(ImVec4(1.0f, 0.85f, 0.1f, 1), "Non-manifold edge");
        swatch(ImVec4(0.85f, 0.35f, 1.0f, 1), "Non-manifold vertex");
        swatch(ImVec4(1.0f, 0.5f, 0.0f, 1), "Flipped neighbour faces");
        swatch(ImVec4(0.86f, 0.26f, 0.30f, 1), "Back side visible (hole / inside out)");
        swatch(ImVec4(0.95f, 0.58f, 0.22f, 1), "Inside-out shell");
        swatch(ImVec4(0.36f, 0.82f, 0.46f, 1), "Faces added by the repair");
    }
    if (ImGui::CollapsingHeader("Settings")) {
        ImGui::TextUnformatted("Custom slicer command:");
        ImGui::SetNextItemWidth(-1);
        if (ImGui::InputTextWithHint("##custom", "e.g. ~/Applications/BambuStudio.AppImage", m_custom_slicer,
                                     sizeof(m_custom_slicer), ImGuiInputTextFlags_EnterReturnsTrue)) {
            m_slicers = detect_slicers(m_custom_slicer);
            m_slicer_index = 0;
        }
        if (ImGui::Button("Detect slicers again")) {
            m_slicers      = detect_slicers(m_custom_slicer);
            m_slicer_index = 0;
        }
        if (m_slicers.empty())
            ImGui::TextColored(kDim, "No slicer found.");
        for (const SlicerApp& s : m_slicers)
            ImGui::BulletText("%s: %s", s.name.c_str(), s.how.c_str());
    }
    ImGui::End();
}

void App::draw_viewport_overlay()
{
    const ImGuiViewport* vp = ImGui::GetMainViewport();
    ImDrawList*          dl = ImGui::GetBackgroundDrawList(); // above the 3D view, below windows
    const ImVec2         origin(vp->WorkPos.x + m_panel_width, vp->WorkPos.y);
    const ImVec2         size(vp->WorkSize.x - m_panel_width, vp->WorkSize.y);
    const float          pad = 12.0f * m_scale;

    if (!m_model && !m_job) {
        const char*  msg = "Drop an STL, OBJ or 3MF file here\nor press Ctrl+O to open one";
        const ImVec2 ts  = ImGui::CalcTextSize(msg);
        dl->AddText(ImVec2(origin.x + (size.x - ts.x) * 0.5f, origin.y + (size.y - ts.y) * 0.5f),
                    ImGui::GetColorU32(kDim), msg);
    }
    if (m_model) {
        const char* which = m_model->repaired ? (m_show_repaired ? "Repaired" : "Original (Tab to compare)") : "Original";
        dl->AddText(ImVec2(origin.x + pad, origin.y + pad), ImGui::GetColorU32(ImVec4(1, 1, 1, 0.75f)), which);
    }
    if (m_job) {
        const std::string text = m_job->get_stage() + "...";
        const ImVec2      ts   = ImGui::CalcTextSize(text.c_str());
        const ImVec2      p(origin.x + (size.x - ts.x) * 0.5f, origin.y + size.y * 0.5f);
        dl->AddRectFilled(ImVec2(p.x - pad, p.y - pad), ImVec2(p.x + ts.x + pad, p.y + ts.y + pad),
                          ImGui::GetColorU32(ImVec4(0, 0, 0, 0.6f)), 6.0f);
        dl->AddText(p, ImGui::GetColorU32(ImVec4(1, 1, 1, 1)), text.c_str());
    }
    if (m_dialog.active() && !ImGui::IsPopupOpen("", ImGuiPopupFlags_AnyPopupId)) {
        const char*  text = "Waiting for the file dialog...";
        const ImVec2 ts   = ImGui::CalcTextSize(text);
        dl->AddText(ImVec2(origin.x + (size.x - ts.x) * 0.5f, origin.y + size.y * 0.5f + ts.y * 2),
                    ImGui::GetColorU32(kDim), text);
    }
    if (!m_status.empty()) {
        const double age   = now_seconds() - m_status_time;
        const float  alpha = m_status_error ? 1.0f : float(std::max(0.0, std::min(1.0, 8.0 - age)));
        if (alpha > 0.0f) {
            const ImVec2 ts = ImGui::CalcTextSize(m_status.c_str());
            const ImVec2 p(origin.x + pad, origin.y + size.y - ts.y - pad);
            dl->AddRectFilled(ImVec2(p.x - pad * 0.5f, p.y - pad * 0.4f), ImVec2(p.x + ts.x + pad * 0.5f, p.y + ts.y + pad * 0.4f),
                              ImGui::GetColorU32(ImVec4(0, 0, 0, 0.55f * alpha)), 4.0f);
            ImVec4 col = m_status_error ? kRed : ImVec4(1, 1, 1, 1);
            col.w      = alpha;
            dl->AddText(p, ImGui::GetColorU32(col), m_status.c_str());
        }
    }
}

void App::draw_about()
{
    if (m_show_about) {
        ImGui::OpenPopup("About MeshRepair");
        m_show_about = false;
    }
    ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(), ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
    if (ImGui::BeginPopupModal("About MeshRepair", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::Text("MeshRepair %s", MESHREPAIR_VERSION);
        ImGui::TextColored(kDim, "Repairs non-manifold, open and inside-out meshes for 3D printing.\n"
                                 "An open replacement for the Windows-only \"Fix model\" (netfabb) function\n"
                                 "of Bambu Studio, OrcaSlicer and PrusaSlicer.");
        ImGui::Spacing();
        ImGui::Text("Dear ImGui %s, GLFW %s", IMGUI_VERSION, glfwGetVersionString());
        if (ImGui::Button("Close"))
            ImGui::CloseCurrentPopup();
        ImGui::EndPopup();
    }
}

void App::draw_ui()
{
    draw_menu();
    draw_panel();
    draw_viewport_overlay();
    draw_about();
}

void App::render_frame()
{
    int fbw, fbh, ww, wh;
    glfwGetFramebufferSize(m_window, &fbw, &fbh);
    glfwGetWindowSize(m_window, &ww, &wh);
    const float sx = ww > 0 ? float(fbw) / float(ww) : 1.0f;
    const float sy = wh > 0 ? float(fbh) / float(wh) : 1.0f;

    gl::Viewport(0, 0, fbw, fbh);
    gl::ClearColor(0.13f, 0.14f, 0.16f, 1.0f);
    gl::Clear(gl::COLOR_BUFFER_BIT | gl::DEPTH_BUFFER_BIT);
    gl::Enable(gl::MULTISAMPLE);

    const ImGuiViewport* vp  = ImGui::GetMainViewport();
    const int            px  = int(m_panel_width * sx);
    const int            top = int((vp->WorkPos.y - vp->Pos.y) * sy); // menu bar
    m_viewer.draw(m_camera, px, 0, fbw - px, fbh - top, m_view);

    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}

int App::run()
{
    while (!glfwWindowShouldClose(m_window) && !m_quit) {
        // Sleep while idle, poll while something is going on.
        if (m_job || m_dialog.active() || !m_screenshot.empty() || m_dragging || m_panning ||
            now_seconds() - m_status_time < 9.0)
            glfwPollEvents();
        else
            glfwWaitEventsTimeout(0.25);
        if (glfwGetWindowAttrib(m_window, GLFW_ICONIFIED)) {
            ImGui_ImplGlfw_Sleep(50);
            continue;
        }

        if (!m_dropped.empty()) {
            const std::string p = m_dropped.front();
            m_dropped.clear();
            if (!m_job)
                open_file(p);
        }
        poll_job();

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();
        if (m_open_dialog_at_start) {
            m_open_dialog_at_start = false;
            request_open();
        }
        handle_shortcuts();
        draw_ui();
        poll_dialog();
        handle_viewport_input();
        render_frame();

        if (!m_screenshot.empty() && ++m_screenshot_frames >= 3) {
            int fbw, fbh;
            glfwGetFramebufferSize(m_window, &fbw, &fbh);
            std::vector<uint8_t> rgba(size_t(fbw) * size_t(fbh) * 4);
            gl::PixelStorei(gl::PACK_ALIGNMENT, 1);
            gl::ReadBuffer(gl::BACK);
            gl::ReadPixels(0, 0, fbw, fbh, gl::RGBA, gl::UNSIGNED_BYTE, rgba.data());
            const size_t row = size_t(fbw) * 4;
            for (int y = 0; y < fbh / 2; ++y)
                std::swap_ranges(rgba.begin() + long(y * row), rgba.begin() + long((y + 1) * row),
                                 rgba.begin() + long((fbh - 1 - y) * row));
            for (size_t i = 3; i < rgba.size(); i += 4)
                rgba[i] = 255;
            const bool ok = write_png(m_screenshot, fbw, fbh, rgba);
            std::printf("%s %s (%dx%d)\n", ok ? "Wrote" : "Could not write", m_screenshot.c_str(), fbw, fbh);
            return ok ? 0 : 1;
        }
        glfwSwapBuffers(m_window);
    }
    return 0;
}

} // namespace mrgui

int main(int argc, char** argv)
{
    mrgui::App app;
    if (!app.init(argc, argv)) {
        app.shutdown();
        return 1;
    }
    const int rc = app.run();
    app.shutdown();
    return rc;
}

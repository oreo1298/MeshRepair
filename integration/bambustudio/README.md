# "Fix model" inside Bambu Studio on Linux

Bambu Studio only offers **Fix model** (right click a model → *Fix model*, or
click the warning icon next to it) on Windows. The function is built on the
Windows 10 3D printing SDK (netfabb), which does not exist on Linux or macOS.
That's why the Windows build has the menu entry and the Linux build doesn't,
and why a model with "non-manifold edges" or "open edges" stays broken there.

This directory adds the same function to Bambu Studio, using MeshRepair
instead of the Windows SDK. It's a small source change that you apply before
building Bambu Studio. Once applied, everything works the same way as on
Windows:

| Feature | Before (Linux) | After |
|---|---|---|
| *Fix model* in the object list context menu | missing | available |
| Click on the warning icon of a broken object to fix it | missing | available |
| "Open edges may be caused by the cut tool, do you want to fix it now?" after cutting | missing | available |
| Mesh repair when importing a textured model (texture to colour) | skipped | repairs before importing |

Painted colour, support and seam data are handled the same way as on Windows,
by Bambu Studio's own code. Since 02.08.02 it re-projects them onto the
repaired mesh; older versions clear them.

Windows builds with the SDK still use the SDK. Nothing changes for them.

## Supported versions

Bambu Studio **02.07 and newer**. The installer has been checked on
v02.07.01.62, v02.08.03.66, v02.08.04.61 and master (2026-09-28). Older
versions use a different repair architecture and the installer refuses them.

## Steps

You need the usual requirements for building Bambu Studio from source (see
their [Linux build instructions](https://github.com/bambulab/BambuStudio/wiki/Linux-Compile-Guide)),
plus Python 3.

```sh
# 1. Bambu Studio and MeshRepair sources
git clone --branch v02.08.04.61 https://github.com/bambulab/BambuStudio.git
git clone https://github.com/oreo1298/MeshRepair.git

# 2. Add MeshRepair to the Bambu Studio sources
python3 MeshRepair/integration/bambustudio/install.py BambuStudio

# 3. Build Bambu Studio as usual
cd BambuStudio
```

The installer is idempotent. Running it again only refreshes the vendored
copy of the library. It checks every edit before writing anything, so an
unexpected source layout leaves the tree untouched. `--check` shows what
would change without changing anything.

During the CMake run you should see:

```
Building with MeshRepair model fixing support
```

To build without it, pass `-DSLIC3R_MESHREPAIR=OFF`.

### Building: pick the route that fits your distribution

**Any distribution (Arch included): AppImage through a container.** Bambu
Studio's own `DockerBuild.sh` builds in an Ubuntu container from your local,
already patched source tree and exports an AppImage, which runs on all
distributions. It needs Docker or Podman.

```sh
./DockerBuild.sh -d   # build the dependencies image
./DockerBuild.sh -i   # build Bambu Studio and export an AppImage
```

**Debian / Ubuntu / Fedora hosts:** Bambu Studio's build script supports these
directly:

```sh
sudo ./BuildLinux.sh -u   # install build dependencies
./BuildLinux.sh -dsi      # dependencies, Bambu Studio, AppImage
```

**Arch Linux, AUR:** if you use an AUR package that builds Bambu Studio from
source (rather than repackaging the AppImage), download its PKGBUILD
(`yay -G <package>` or `git clone https://aur.archlinux.org/<package>.git`) and
make two additions:

```sh
# 1. one more source (keep the existing entries):
source+=("git+https://github.com/oreo1298/MeshRepair.git")
sha256sums+=('SKIP')

# 2. in prepare(), after the existing steps; "BambuStudio" is the directory
#    the PKGBUILD extracts the Bambu Studio sources to:
python "$srcdir/MeshRepair/integration/bambustudio/install.py" "$srcdir/BambuStudio"
```

Then `makepkg -si` as usual. Prebuilt binaries (`-bin` packages, the Flatpak
from Flathub, the official AppImage) can't be changed this way, because
they're compiled already. Use the standalone `meshrepair-gui` with *Open in Bambu
Studio* for those.

**Flatpak:** Bambu Studio's Flatpak is built from source on Flathub. Adding the
`install.py` step to the Bambu Studio module of a local copy of the manifest
works the same way.

### Plain patch instead of the installer

For packaging systems that prefer patch files,
[`bambustudio-meshrepair.patch`](bambustudio-meshrepair.patch) contains the
same changes, generated from v02.08.04.61 (it also applies to master). The
patch does not include the library itself: copy `libmeshrepair/` to
`src/meshrepair/` as well.

```sh
cp -r MeshRepair/libmeshrepair BambuStudio/src/meshrepair
cd BambuStudio && patch -p1 < ../MeshRepair/integration/bambustudio/bambustudio-meshrepair.patch
```

## What the change does

* `src/meshrepair/`: a vendored copy of libmeshrepair (C++17, no
  dependencies), built as a static library.
* `src/libslic3r/MeshRepairFix.cpp` (new): implements
  `is_win10_model_repair_available()` and `fix_mesh_by_win10_sdk()`, the
  functions the Windows SDK code provides. It converts Bambu Studio's
  `indexed_triangle_set` to MeshRepair's mesh, runs the repair with progress
  and cancel support, and converts the result back.
* `CMakeLists.txt`: a new `HAS_MODEL_REPAIR` define means "a repair backend is
  available". It's set together with `HAS_WIN10SDK` on Windows, and together
  with `HAS_MESHREPAIR` elsewhere (option `SLIC3R_MESHREPAIR`, default on).
* `FixModelByWin10.cpp`: only the WinRT parts remain Windows-only. The GUI part
  (progress dialog, worker thread, paint re-projection, undo snapshot) is
  shared by both backends.
* `FixModelByWin10.hpp`, `Win10ModelRepair.hpp`, `GLGizmoAdvancedCut.cpp` and
  `TextureImportDialog.*`: their `HAS_WIN10SDK` guards become
  `HAS_MODEL_REPAIR`.

The function names still contain "win10", which keeps the change small and
easy to carry across Bambu Studio updates.

## How this was verified

A full Bambu Studio build wasn't possible in the environment where this was
developed: some of Bambu's dependency downloads were blocked by its network
policy. What was checked instead:

* All touched GUI files (`FixModelByWin10.cpp`, `GLGizmoAdvancedCut.cpp`,
  `TextureImportDialog.cpp`, plus `GUI_ObjectList.cpp` and `GUI_Factories.cpp`,
  which use the changed header) compile on Linux against the real Bambu Studio
  headers, with Bambu's compile definitions and its precompiled header. This
  was done for v02.07.01.62, v02.08.03.66 and v02.08.04.61, with the feature
  both on and off.
* The CMake changes were exercised in a mock of Bambu's source layout. The
  library builds as `src/meshrepair`, links into `libslic3r`, and a test
  program repairs a broken mesh through `Slic3r::fix_mesh_by_win10_sdk()`.
* The repair results were checked with **Bambu Studio's own
  `MeshDiagnostics.cpp`**, the code behind the "non-manifold edges" and
  "reversed faces" warnings. The input was 197 meshes from the trimesh and
  CGAL test suites, many of them deliberately broken. 134 came out clean (no
  open edges, no non-manifold edges or vertices, no reversed faces), also after
  merging vertices by position the way Bambu Studio loads an STL. 55 flat test
  patches without any volume were rejected, with an error message. The other 8
  are single open sheets: closing those can only produce a surface that folds
  over itself, which Bambu Studio then reports as reversed faces.

To repeat the last check against your own Bambu Studio tree and meshes:

```sh
python3 integration/bambustudio/install.py /path/to/BambuStudio
integration/bambustudio/verify/verify.sh /path/to/BambuStudio broken1.stl broken2.3mf
```

(needs Boost.Log and TBB development packages: `boost tbb` on Arch).

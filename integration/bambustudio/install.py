#!/usr/bin/env python3
"""Add MeshRepair based "Fix model" support to a Bambu Studio source tree.

Bambu Studio only offers "Fix model" (repairing non-manifold / open meshes)
on Windows, because it relies on the Windows 10 3D printing SDK (netfabb).
This script makes the function available on Linux (and macOS) by:

  * vendoring libmeshrepair into src/meshrepair,
  * adding src/libslic3r/MeshRepairFix.cpp, which implements the same
    fix_mesh_by_win10_sdk() entry point with MeshRepair,
  * switching the "#ifdef HAS_WIN10SDK" guards of the repair features to a new
    HAS_MODEL_REPAIR macro, so the Fix model menu, the repair offered after
    cutting and the repair during texture import are enabled.

Windows builds that have the SDK keep using it unchanged.

Usage:  install.py /path/to/BambuStudio        apply (idempotent)
        install.py --check /path/to/BambuStudio only report what would change

Supports Bambu Studio 02.07 and newer.
"""

import argparse
import re
import shutil
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parent.parent
LIBRARY = REPO / "libmeshrepair"
BACKEND = HERE / "files" / "src" / "libslic3r" / "MeshRepairFix.cpp"

ROOT_CMAKE_BLOCK = """
# Model repair ("Fix model"): the Windows 10 3D printing SDK (netfabb) on Windows,
# MeshRepair (src/meshrepair) wherever that SDK is not available (Linux, macOS).
option(SLIC3R_MESHREPAIR "Use MeshRepair for Fix model when the Windows 10 SDK is not available" ON)
if (WIN32 AND WIN10SDK_INCLUDE_PATH)
    add_definitions(-DHAS_MODEL_REPAIR)
elseif (SLIC3R_MESHREPAIR AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/meshrepair/CMakeLists.txt")
    message("Building with MeshRepair model fixing support")
    add_definitions(-DHAS_MESHREPAIR -DHAS_MODEL_REPAIR)
    set(SLIC3R_HAS_MESHREPAIR TRUE)
endif ()
"""

SRC_CMAKE_BLOCK = """if (SLIC3R_HAS_MESHREPAIR)
    add_subdirectory(meshrepair)
endif ()
"""

LIBSLIC3R_CMAKE_BLOCK = """
if (SLIC3R_HAS_MESHREPAIR)
    target_sources(libslic3r PRIVATE MeshRepairFix.cpp)
    target_link_libraries(libslic3r meshrepair)
endif ()
"""


class Failure(Exception):
    pass


class Tree:
    def __init__(self, root: Path, dry_run: bool):
        self.root = root
        self.dry_run = dry_run
        self.changes = []

    def read(self, rel):
        p = self.root / rel
        if not p.is_file():
            raise Failure(f"missing file: {rel}")
        return p.read_text(encoding="utf-8")

    def write(self, rel, text, what):
        self.changes.append(f"{rel}: {what}")
        if not self.dry_run:
            (self.root / rel).write_text(text, encoding="utf-8")


def patch_root_cmake(t: Tree):
    rel = "CMakeLists.txt"
    s = t.read(rel)
    if "HAS_MODEL_REPAIR" in s:
        return
    if "-DHAS_WIN10SDK" not in s:
        raise Failure(f"{rel}: Windows 10 SDK detection not found")
    # Before the source tree is added, after the SDK detection.
    m = re.search(r"^[ \t]*add_subdirectory\(\s*src\s*\)", s, re.M)
    if not m or m.start() < s.index("-DHAS_WIN10SDK"):
        raise Failure(f"{rel}: add_subdirectory(src) not found after the SDK detection")
    s = s[: m.start()] + ROOT_CMAKE_BLOCK.lstrip("\n") + "\n" + s[m.start():]
    t.write(rel, s, "define HAS_MODEL_REPAIR / HAS_MESHREPAIR")


def patch_src_cmake(t: Tree):
    rel = "src/CMakeLists.txt"
    s = t.read(rel)
    if "add_subdirectory(meshrepair)" in s:
        return
    m = re.search(r"^add_subdirectory\(admesh\)[ \t]*\n", s, re.M)
    if not m:
        raise Failure(f"{rel}: add_subdirectory(admesh) not found")
    s = s[: m.end()] + SRC_CMAKE_BLOCK + s[m.end():]
    t.write(rel, s, "build src/meshrepair")


def patch_libslic3r_cmake(t: Tree):
    rel = "src/libslic3r/CMakeLists.txt"
    s = t.read(rel)
    if "MeshRepairFix.cpp" in s:
        return
    m = re.search(r"^target_link_libraries\(libslic3r\s*\n", s, re.M)
    if not m:
        raise Failure(f"{rel}: target_link_libraries(libslic3r ...) block not found")
    close = re.compile(r"^[ \t]*\)[ \t]*\n", re.M).search(s, m.end())
    if not close:
        raise Failure(f"{rel}: end of target_link_libraries(libslic3r ...) not found")
    s = s[: close.end()] + LIBSLIC3R_CMAKE_BLOCK + s[close.end():]
    t.write(rel, s, "compile MeshRepairFix.cpp, link meshrepair")


def rename_guards(t: Tree, rel, required=True):
    p = t.root / rel
    if not p.is_file():
        if required:
            raise Failure(f"missing file: {rel}")
        return
    s = t.read(rel)
    n = len(re.findall(r"\bHAS_WIN10SDK\b", s))
    if n == 0:
        return
    s = re.sub(r"\bHAS_WIN10SDK\b", "HAS_MODEL_REPAIR", s)
    t.write(rel, s, f"{n} HAS_WIN10SDK guard(s) -> HAS_MODEL_REPAIR")


def patch_fix_model_cpp(t: Tree):
    """Keep only the WinRT parts of FixModelByWin10.cpp Windows specific."""
    rel = "src/slic3r/Utils/FixModelByWin10.cpp"
    s = t.read(rel)
    if "HAS_MODEL_REPAIR" in s:
        return
    lines = s.split("\n")
    if lines[0].strip() != "#ifdef HAS_WIN10SDK":
        raise Failure(f"{rel}: unexpected first line {lines[0]!r}")

    def find(pred, start=0, what=""):
        for i in range(start, len(lines)):
            if pred(lines[i]):
                return i
        raise Failure(f"{rel}: {what} not found")

    winrt_last = max(i for i, l in enumerate(lines) if l.startswith("#include <winrt/") or l.startswith("#include <roapi.h>")
                     or l.startswith("#include <wrl/"))
    extern_c = find(lambda l: l.startswith('extern "C"'), 0, 'extern "C" block')
    extern_end = find(lambda l: l.strip() == "}", extern_c + 1, 'end of extern "C" block')
    ns = find(lambda l: l.strip() == "namespace Slic3r {", extern_end, "namespace Slic3r")
    cancel = find(lambda l: l.startswith("class RepairCanceledException"), ns, "class RepairCanceledException")
    last = max(i for i, l in enumerate(lines) if l.strip().startswith("#endif"))
    if not (winrt_last < extern_c < extern_end < ns < cancel < last):
        raise Failure(f"{rel}: unexpected layout")

    out = ["#ifdef HAS_MODEL_REPAIR", "", "#ifdef HAS_WIN10SDK"]
    out += lines[1:winrt_last + 1]
    out += ["#endif /* HAS_WIN10SDK */"]
    out += lines[winrt_last + 1:extern_c]
    out += ["#ifdef HAS_WIN10SDK"] + lines[extern_c:extern_end + 1] + ["#endif /* HAS_WIN10SDK */"]
    out += lines[extern_end + 1:ns + 1]
    out += ["", "#ifdef HAS_WIN10SDK"]
    out += lines[ns + 1:cancel]
    out += ["#else /* HAS_WIN10SDK */",
            "",
            "// Fix model is provided by MeshRepair (libslic3r/MeshRepairFix.cpp), which",
            "// works on every platform.",
            "bool is_windows10() { return true; }",
            "",
            "#endif /* HAS_WIN10SDK */",
            ""]
    out += lines[cancel:last]
    out += ["#endif /* HAS_MODEL_REPAIR */"] + lines[last + 1:]
    t.write(rel, "\n".join(out), "WinRT parts guarded by HAS_WIN10SDK, Fix model GUI shared")


def install_files(t: Tree):
    dst = t.root / "src" / "meshrepair"
    t.changes.append("src/meshrepair: vendored libmeshrepair")
    backend = t.root / "src" / "libslic3r" / "MeshRepairFix.cpp"
    t.changes.append("src/libslic3r/MeshRepairFix.cpp: MeshRepair backend")
    if t.dry_run:
        return
    if dst.exists():
        shutil.rmtree(dst)
    shutil.copytree(LIBRARY, dst, ignore=shutil.ignore_patterns("build*", "*.o", "*.a"))
    shutil.copyfile(BACKEND, backend)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bambu_dir", type=Path, help="Bambu Studio source tree")
    ap.add_argument("--check", action="store_true", help="only report what would change")
    args = ap.parse_args()

    root = args.bambu_dir.resolve()
    if not (root / "src" / "slic3r" / "Utils" / "FixModelByWin10.cpp").is_file():
        print(f"error: {root} does not look like a Bambu Studio source tree", file=sys.stderr)
        return 1
    if not (root / "src" / "libslic3r" / "Win10ModelRepair.hpp").is_file():
        print("error: this Bambu Studio version is too old (needs 02.07 or newer, "
              "src/libslic3r/Win10ModelRepair.hpp is missing)", file=sys.stderr)
        return 1

    def run(t):
        patch_root_cmake(t)
        patch_src_cmake(t)
        patch_libslic3r_cmake(t)
        rename_guards(t, "src/libslic3r/Win10ModelRepair.hpp")
        rename_guards(t, "src/slic3r/Utils/FixModelByWin10.hpp")
        patch_fix_model_cpp(t)
        rename_guards(t, "src/slic3r/GUI/Gizmos/GLGizmoAdvancedCut.cpp", required=False)
        rename_guards(t, "src/slic3r/GUI/TextureImportDialog.cpp", required=False)
        rename_guards(t, "src/slic3r/GUI/TextureImportDialog.hpp", required=False)
        install_files(t)

    # Validate every step without writing anything, then apply.
    try:
        t = Tree(root, True)
        run(t)
        if not args.check:
            t = Tree(root, False)
            run(t)
    except Failure as e:
        print(f"error: {e}", file=sys.stderr)
        print("The source tree was not modified.", file=sys.stderr)
        return 1

    verb = "Would change" if args.check else "Changed"
    print(f"{verb}:")
    for c in t.changes:
        print(f"  {c}")
    if not args.check:
        print("\nDone. Build Bambu Studio as usual (e.g. ./BuildLinux.sh -s). CMake prints\n"
              "\"Building with MeshRepair model fixing support\" when the feature is enabled;\n"
              "disable it with -DSLIC3R_MESHREPAIR=OFF.")
    return 0


if __name__ == "__main__":
    sys.exit(main())

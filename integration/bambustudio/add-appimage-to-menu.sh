#!/bin/sh
# Adds a Bambu Studio AppImage to the application menu, with its icon, the
# usual file types (STL, 3MF, OBJ, STEP) and the "Open in Bambu Studio" links
# of MakerWorld. AppImages don't do this by themselves.
#
#   add-appimage-to-menu.sh /path/to/BambuStudio.AppImage
#   add-appimage-to-menu.sh --remove
#
# The menu entry starts the AppImage where it is, so move it to a permanent
# place (e.g. ~/Applications) first. Run the script again after replacing the
# AppImage with a file of a different name.

set -eu

data=${XDG_DATA_HOME:-$HOME/.local/share}
desktop="$data/applications/bambu-studio.desktop"
icons="$data/icons/hicolor"
sizes="32 128 192"

refresh() {
    update-desktop-database "$data/applications" 2>/dev/null || true
    if [ -f "$icons/icon-theme.cache" ]; then
        gtk-update-icon-cache -q -t -f "$icons" 2>/dev/null || true
    fi
}

if [ "${1:-}" = "--remove" ]; then
    rm -f "$desktop" "$icons/scalable/apps/BambuStudio.svg"
    for s in $sizes; do
        rm -f "$icons/${s}x${s}/apps/BambuStudio.png"
    done
    refresh
    echo "Removed Bambu Studio from the application menu."
    exit 0
fi

if [ $# -ne 1 ] || [ ! -f "$1" ]; then
    echo "usage: $0 /path/to/BambuStudio.AppImage" >&2
    echo "       $0 --remove" >&2
    exit 1
fi

app=$(readlink -f "$1")
chmod +x "$app"

tmp=$(mktemp -d)
trap 'rm -rf "$tmp"' EXIT
(
    cd "$tmp"
    "$app" --appimage-extract 'usr/share/icons/*' > /dev/null
    "$app" --appimage-extract 'resources/images/BambuStudio.svg' > /dev/null
)
root="$tmp/squashfs-root"
if [ ! -f "$root/usr/share/icons/hicolor/128x128/apps/BambuStudio.png" ]; then
    echo "error: no Bambu Studio icon found in $app" >&2
    exit 1
fi

for s in $sizes; do
    src="$root/usr/share/icons/hicolor/${s}x${s}/apps/BambuStudio.png"
    if [ -f "$src" ]; then
        mkdir -p "$icons/${s}x${s}/apps"
        cp "$src" "$icons/${s}x${s}/apps/BambuStudio.png"
    fi
done
if [ -f "$root/resources/images/BambuStudio.svg" ]; then
    mkdir -p "$icons/scalable/apps"
    cp "$root/resources/images/BambuStudio.svg" "$icons/scalable/apps/BambuStudio.svg"
fi

# Quote the path for the Exec key: \ " ` $ need a backslash, which itself has
# to be escaped in a desktop entry value (desktop entry specification).
exec_path=$(printf '%s' "$app" | sed -e 's/\\/\\\\\\\\/g' -e 's/["`$]/\\\\&/g')

# The file is named after the window's application ID (bambu-studio), so
# taskbars and docks show the icon on Wayland too.
mkdir -p "$data/applications"
cat > "$desktop" << EOF
[Desktop Entry]
Type=Application
Name=Bambu Studio
GenericName=3D Printing Software
Comment=Bambu Studio with Fix model (MeshRepair)
Exec="$exec_path" %U
Icon=BambuStudio
Terminal=false
Categories=Graphics;3DGraphics;Engineering;
MimeType=model/stl;model/3mf;application/vnd.ms-3mfdocument;application/prs.wavefront-obj;application/x-amf;model/step;x-scheme-handler/bambustudio;x-scheme-handler/bambustudioopen;
Keywords=3D;Printing;Slicer;slice;printer;gcode;stl;obj;3mf;
StartupNotify=false
StartupWMClass=bambu-studio
EOF

refresh
echo "Added Bambu Studio to the application menu ($desktop)."

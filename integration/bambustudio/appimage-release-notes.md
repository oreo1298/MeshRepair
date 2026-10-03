An **unofficial** Linux build of [Bambu Studio](https://github.com/bambulab/BambuStudio) ${version} that includes **Fix model**, provided by [MeshRepair](${repo_url}). Official builds offer Fix model only on Windows, where it relies on a Windows-only Microsoft service. This build is Bambu Studio's own source with the MeshRepair integration added and nothing else changed. Bambu Lab doesn't make or support it, so please report problems with it [here](${repo_url}/issues), not to Bambu Lab.

## Install

1. Download `${appimage}` below.
2. Make it executable and start it:
   ```sh
   chmod +x ${appimage}
   ./${appimage}
   ```
3. Right-click an object that has mesh errors and choose **Fix model**, or click its warning icon.

It shares settings, presets and the printer login with other Bambu Studio installs (`~/.config/BambuStudio`). It runs on any x86-64 Linux distribution that has:

- **FUSE.** On Arch, install `fuse3`. Without FUSE, start it with `--appimage-extract-and-run`.
- **WebKitGTK** (`${webkit}`), like the official AppImage. On Arch, install `${arch_webkit}`.

To add it to the application menu with its icon, run [`add-appimage-to-menu.sh`](${repo_url}/blob/${meshrepair_commit}/integration/bambustudio/add-appimage-to-menu.sh) with the path of the AppImage.

To check the download: `sha256sum -c ${appimage}.sha256`

## Source code

This build is licensed under the AGPL-3.0, like Bambu Studio. Its complete source is:

- Bambu Studio ${version}: https://github.com/bambulab/BambuStudio/tree/${bambu_commit}
- MeshRepair (MIT license): ${repo_url}/tree/${meshrepair_commit}
- How the two are combined: [`integration/bambustudio/install.py`](${repo_url}/blob/${meshrepair_commit}/integration/bambustudio/install.py), the same changes as [a patch](${repo_url}/blob/${meshrepair_commit}/integration/bambustudio/bambustudio-meshrepair.patch)
- The build log: ${run_url}

#!/usr/bin/env bash
# Sourced only by native GUI launcher scripts, in their child shell.
# Snap VS Code injects GTK/GIO paths whose modules pull in core20's glibc.
unset GTK_PATH GTK_EXE_PREFIX GTK_IM_MODULE_FILE GIO_MODULE_DIR GIO_EXTRA_MODULES
unset QT_PLUGIN_PATH QT_QPA_PLATFORM_PLUGIN_PATH QT_QPA_PLATFORMTHEME
unset QML_IMPORT_PATH QML2_IMPORT_PATH

# Keep ROS/Ogre library directories; remove only Snap runtime directories.
semantic_native_libraries=()
IFS=: read -r -a semantic_library_entries <<< "${LD_LIBRARY_PATH:-}"
for semantic_library_entry in "${semantic_library_entries[@]}"; do
    case "$semantic_library_entry" in
        /snap/*|*/snap/*|/var/lib/snapd/*|'') ;;
        *) semantic_native_libraries+=("$semantic_library_entry") ;;
    esac
done
if (( ${#semantic_native_libraries[@]} )); then
    LD_LIBRARY_PATH="$(IFS=:; echo "${semantic_native_libraries[*]}")"
    export LD_LIBRARY_PATH
else
    unset LD_LIBRARY_PATH
fi
if [[ "${LD_PRELOAD:-}" == *'/snap/'* || "${LD_PRELOAD:-}" == *'/var/lib/snapd/'* ]]; then
    unset LD_PRELOAD
fi
export QT_QPA_PLATFORM="${SEMANTIC_QT_PLATFORM:-xcb}"
unset semantic_native_libraries semantic_library_entries semantic_library_entry

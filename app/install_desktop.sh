#!/bin/bash
# Installs a LIO-SAM Studio launcher into the app grid and onto the Desktop.
# Usage: ./install_desktop.sh            install
#        ./install_desktop.sh --uninstall remove
set -e
APP_DIR="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"
APPS="$HOME/.local/share/applications"
ICONS="$HOME/.local/share/icons/hicolor/scalable/apps"
DESKTOP_DIR="$(xdg-user-dir DESKTOP 2>/dev/null || echo "$HOME/Desktop")"
NAME="lio-sam-studio"

if [ "$1" == "--uninstall" ]; then
    rm -f "$APPS/$NAME.desktop" "$DESKTOP_DIR/$NAME.desktop" "$ICONS/$NAME.svg"
    update-desktop-database "$APPS" 2>/dev/null || true
    echo "LIO-SAM Studio launcher removed."
    exit 0
fi

chmod +x "$APP_DIR/lio_sam_studio.sh" "$APP_DIR/lio_sam_studio.py"
mkdir -p "$APPS" "$ICONS"
cp "$APP_DIR/icons/$NAME.svg" "$ICONS/$NAME.svg"

cat > "$APPS/$NAME.desktop" <<EOF
[Desktop Entry]
Type=Application
Name=LIO-SAM Studio
GenericName=LiDAR SLAM Survey
Comment=Record ROS 2 bags and build LIO-SAM survey maps
Exec="$APP_DIR/lio_sam_studio.sh"
Icon=$ICONS/$NAME.svg
Terminal=false
Categories=Science;Engineering;
Keywords=lidar;slam;ros;survey;point cloud;
StartupNotify=true
StartupWMClass=Lio-sam-studio
EOF
chmod +x "$APPS/$NAME.desktop"
desktop-file-validate "$APPS/$NAME.desktop" 2>/dev/null || true
update-desktop-database "$APPS" 2>/dev/null || true

if [ -d "$DESKTOP_DIR" ]; then
    cp "$APPS/$NAME.desktop" "$DESKTOP_DIR/$NAME.desktop"
    chmod +x "$DESKTOP_DIR/$NAME.desktop"
    # GNOME: mark the desktop icon as trusted so it launches on double-click
    gio set "$DESKTOP_DIR/$NAME.desktop" metadata::trusted true 2>/dev/null || true
fi

echo "Installed LIO-SAM Studio."
echo "  App grid: search 'LIO-SAM Studio' (right-click > Pin to Dash to add it to the dock)"
echo "  Desktop:  $DESKTOP_DIR/$NAME.desktop"

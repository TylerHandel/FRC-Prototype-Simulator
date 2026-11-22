#!/usr/bin/env bash
# Downloads MuJoCo's release archive into third_party/mujoco (the version mujoco-rs binds).
# Run once after cloning: tools/fetch-mujoco.sh
set -euo pipefail
VERSION=3.17.0
cd "$(dirname "$0")/.."
DEST=third_party/mujoco
if [ "$(uname -s)" != Darwin ] && [ -f "$DEST/VERSION" ] && [ "$(cat "$DEST/VERSION")" = "$VERSION" ]; then echo "MuJoCo $VERSION already in $DEST"; exit 0; fi
case "$(uname -s)" in
  Linux) ARCHIVE="mujoco-$VERSION-linux-$(uname -m).tar.gz" ;;
  Darwin) ARCHIVE="mujoco-$VERSION-macos-universal2.dmg" ;;
  *) echo "Use tools/fetch-mujoco.ps1 on Windows"; exit 1 ;;
esac
TMP=$(mktemp -d)
MOUNT="$TMP/volume"
mounted=false
cleanup() {
  if $mounted; then hdiutil detach "$MOUNT" >/dev/null || return 1; fi
  rm -rf "$TMP"
}
trap cleanup EXIT
curl -fsSL -o "$TMP/$ARCHIVE" "https://github.com/google-deepmind/mujoco/releases/download/$VERSION/$ARCHIVE"
mkdir -p "$TMP/stage"
if [[ "$ARCHIVE" == *.tar.gz ]]; then
  tar xzf "$TMP/$ARCHIVE" -C "$TMP"
  cp -r "$TMP/mujoco-$VERSION/lib" "$TMP/mujoco-$VERSION/include" "$TMP/stage/"
else
  mkdir "$MOUNT"
  hdiutil attach -readonly -nobrowse -mountpoint "$MOUNT" "$TMP/$ARCHIVE"
  mounted=true
  framework="$MOUNT/mujoco.framework/Versions/A"
  library="$framework/libmujoco.$VERSION.dylib"
  test -f "$library"
  mkdir -p "$TMP/stage/lib" "$TMP/stage/include/mujoco"
  cp -L "$library" "$TMP/stage/lib/libmujoco.dylib"
  cp -R "$framework/Headers/"*.h "$TMP/stage/include/mujoco/"
  chmod u+w "$TMP/stage/lib/libmujoco.dylib"
  install_name_tool -id @rpath/libmujoco.dylib "$TMP/stage/lib/libmujoco.dylib"
  codesign --force --sign - "$TMP/stage/lib/libmujoco.dylib"
fi
echo "$VERSION" > "$TMP/stage/VERSION"
mkdir -p "$DEST"
cp -R "$TMP/stage/." "$DEST/"
echo "MuJoCo $VERSION in $DEST"

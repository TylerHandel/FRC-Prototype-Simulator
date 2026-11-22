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
cleanup() {
  if mount | grep -Fq " on $MOUNT "; then hdiutil detach "$MOUNT" >/dev/null; fi
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
  framework=$(find "$MOUNT" -type d -name mujoco.framework -print -quit)
  test -f "$framework/mujoco"
  mkdir -p "$TMP/stage/lib" "$TMP/stage/include/mujoco"
  cp -L "$framework/mujoco" "$TMP/stage/lib/libmujoco.dylib"
  cp -RL "$framework/Headers/." "$TMP/stage/include/mujoco/"
  chmod u+w "$TMP/stage/lib/libmujoco.dylib"
  install_name_tool -id @rpath/libmujoco.dylib "$TMP/stage/lib/libmujoco.dylib"
  codesign --force --sign - "$TMP/stage/lib/libmujoco.dylib"
fi
echo "$VERSION" > "$TMP/stage/VERSION"
mkdir -p "$DEST"
cp -R "$TMP/stage/." "$DEST/"
echo "MuJoCo $VERSION in $DEST"

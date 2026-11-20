#!/usr/bin/env bash
# Downloads MuJoCo's release archive into third_party/mujoco (the version mujoco-rs binds).
# Run once after cloning: tools/fetch-mujoco.sh
set -euo pipefail
VERSION=3.17.0
cd "$(dirname "$0")/.."
DEST=third_party/mujoco
if [ -f "$DEST/VERSION" ] && [ "$(cat "$DEST/VERSION")" = "$VERSION" ]; then echo "MuJoCo $VERSION already in $DEST"; exit 0; fi
case "$(uname -s)" in
  Linux) ARCHIVE="mujoco-$VERSION-linux-$(uname -m).tar.gz" ;;
  Darwin) ARCHIVE="mujoco-$VERSION-macos-universal2.dmg" ;;
  *) echo "Use tools/fetch-mujoco.ps1 on Windows"; exit 1 ;;
esac
TMP=$(mktemp -d)
curl -fsSL -o "$TMP/$ARCHIVE" "https://github.com/google-deepmind/mujoco/releases/download/$VERSION/$ARCHIVE"
rm -rf "$DEST"; mkdir -p "$DEST"
if [[ "$ARCHIVE" == *.tar.gz ]]; then
  tar xzf "$TMP/$ARCHIVE" -C "$TMP"
  cp -r "$TMP/mujoco-$VERSION/lib" "$TMP/mujoco-$VERSION/include" "$DEST/"
else
  # macOS: the framework inside the disk image; left for when a Mac build is wanted (PLAN.md D18)
  echo "macOS: mount $TMP/$ARCHIVE and copy mujoco.framework's library into $DEST/lib"; exit 1
fi
echo "$VERSION" > "$DEST/VERSION"
rm -rf "$TMP"
echo "MuJoCo $VERSION in $DEST"

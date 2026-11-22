#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
app="$PWD/dist/FRC Prototype Simulator.app"
exe="$app/Contents/MacOS/frc-prototype-sim"
mkdir -p "$app/Contents/MacOS" "$app/Contents/Frameworks" "$app/Contents/Resources"
cp target/release/frc-prototype-sim "$exe"
ditto dist/cadread "$app/Contents/Resources/cadread"
cp third_party/mujoco/lib/libmujoco.dylib "$app/Contents/Frameworks/"
install_name_tool -delete_rpath "$PWD/third_party/mujoco/lib" -add_rpath @executable_path/../Frameworks "$exe"
version=$(awk -F'"' '/^version = / {print $2; exit}' Cargo.toml)
cat > "$app/Contents/Info.plist" <<PLIST
<?xml version="1.0" encoding="UTF-8"?>
<plist version="1.0"><dict>
<key>CFBundleName</key><string>FRC Prototype Simulator</string>
<key>CFBundleIdentifier</key><string>org.frcprototypesimulator.preview</string>
<key>CFBundleExecutable</key><string>frc-prototype-sim</string>
<key>CFBundlePackageType</key><string>APPL</string>
<key>CFBundleShortVersionString</key><string>$version</string>
<key>CFBundleVersion</key><string>${GITHUB_RUN_NUMBER:-1}</string>
<key>NSHighResolutionCapable</key><true/>
</dict></plist>
PLIST
plutil -lint "$app/Contents/Info.plist"
codesign --force --sign - "$app"
codesign --verify --deep --strict "$app"
zip="$PWD/dist/FRC-Prototype-Simulator-$(uname -m).zip"
ditto -c -k --sequesterRsrc --keepParent "$app" "$zip"
scratch=$(mktemp -d)
trap 'rm -rf "$scratch"' EXIT
ditto -x -k "$zip" "$scratch"
cd "$scratch"
bin="./FRC Prototype Simulator.app/Contents/MacOS"
env -u DYLD_LIBRARY_PATH -u DYLD_FALLBACK_LIBRARY_PATH "$bin/frc-prototype-sim" --self-test
"$bin/../Resources/cadread/cadread" "${RUNNER_TEMP:?}/smoke.step" smoke.cadpack
test -s smoke.cadpack/parts.json

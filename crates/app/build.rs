//! Lets this package's binaries and tests find MuJoCo's shared library where tools/fetch-mujoco put
//! it (Linux and macOS: an rpath; Windows: the DLL sits next to the exe, copied by the fetch script).
fn main() {
    println!("cargo:rerun-if-env-changed=MUJOCO_DYNAMIC_LINK_DIR");
    let os = std::env::var("CARGO_CFG_TARGET_OS").unwrap_or_default();
    if os == "windows" { return; }
    if let Ok(dir) = std::env::var("MUJOCO_DYNAMIC_LINK_DIR") {
        if let Ok(abs) = std::fs::canonicalize(&dir) {
            println!("cargo:rustc-link-arg=-Wl,-rpath,{}", abs.display());
        }
    }
    let origin = if os == "macos" { "@executable_path" } else { "$ORIGIN" };
    println!("cargo:rustc-link-arg=-Wl,-rpath,{origin}");
}

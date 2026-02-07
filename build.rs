fn main() {
    // Use pkg-config to find libhackrf and emit the correct linker search paths.
    // This is needed because the libhackrf Rust crate uses #[link(name = "hackrf")]
    // but doesn't ship a build script to locate the system library.
    match pkg_config::Config::new()
        .atleast_version("0.5")
        .probe("libhackrf")
    {
        Ok(lib) => {
            for path in &lib.link_paths {
                println!("cargo:rustc-link-search=native={}", path.display());
            }
        }
        Err(_) => {
            // Fallback: try common Homebrew / system paths
            println!("cargo:rustc-link-search=native=/usr/local/lib");
            println!("cargo:rustc-link-search=native=/opt/homebrew/lib");
            println!("cargo:rustc-link-search=native=/usr/lib");
        }
    }
}

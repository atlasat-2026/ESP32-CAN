use std::env;
use std::path::{Path, PathBuf};

fn main() {
    let cargo_dir = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap());
    let target_dir = PathBuf::from(env::var("CARGO_BUILD_TARGET_DIR").unwrap());

    run_cbindgen(&cargo_dir, &target_dir);
}

fn run_cbindgen(cargo_dir: &Path, target_dir: &Path) {
    let out = target_dir.join("RustApi.h");

    cbindgen::Builder::new()
        .with_crate(cargo_dir)
        .with_language(cbindgen::Language::C)
        .generate()
        .expect("Unable to generate bindings")
        .write_to_file(&out);

    println!("cargo:rerun-if-changed={}", out.display());
}

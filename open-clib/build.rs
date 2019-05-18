use cbindgen::{Builder, Language};
use std::env;

fn main() {
    let crate_dir = env::var("CARGO_MANIFEST_DIR").unwrap();

    Builder::new()
        .with_crate(crate_dir)
        .with_language(Language::C)
        .generate()
        .expect("Unable to generate bindings")
        .write_to_file("open_clib.h");
}

use cbindgen::{Builder, Language};
use icasadi;
use std::env;

fn main() {
    let crate_dir = env::var("CARGO_MANIFEST_DIR").unwrap();

    Builder::new()
        .with_crate(&crate_dir)
        .with_language(Language::C)
        .with_include_guard("_OPEN_GUARD_")
        .with_header(format!(
            "#define OPEN_NUM_DECISION_VARIABLES {}\n#define OPEN_NUM_STATIC_PARAMETERS {}",
            icasadi::NUM_DECISION_VARIABLES,
            icasadi::NUM_STATIC_PARAMETERS
        )) // Sneaky way of getting around cbindgen limitation
        .generate()
        .expect("Unable to generate bindings")
        .write_to_file("open_clib.h");

    Builder::new()
        .with_crate(&crate_dir)
        .with_language(Language::Cxx)
        .with_include_guard("_OPEN_GUARD_")
        .with_header(format!(
            "#define OPEN_NUM_DECISION_VARIABLES {}\n#define OPEN_NUM_STATIC_PARAMETERS {}",
            icasadi::NUM_DECISION_VARIABLES,
            icasadi::NUM_STATIC_PARAMETERS
        )) // Sneaky way of getting around cbindgen limitation
        .generate()
        .expect("Unable to generate bindings")
        .write_to_file("open_clib.hpp");

    println!("cargo:rerun-if-changed=src/lib.rs");
    println!("cargo:rerun-if-changed=open_clib.h");
    println!("cargo:rerun-if-changed=open_clib.hpp");
}

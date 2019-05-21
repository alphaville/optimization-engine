use cbindgen::{Builder, Language};
use icasadi;
use std::env;

fn main() {
    let crate_dir = env::var("CARGO_MANIFEST_DIR").unwrap();

    let mut header = String::new();
    header.push_str("/* This is an auto-generated file made from optimization engine: ");
    header.push_str("https://crates.io/crates/optimization_engine */\n\n");
    header.push_str(
        "/** This is the size of all the arrays that the solver needs, except params. */\n",
    );
    header.push_str(
        format!(
            "#define OPEN_NUM_DECISION_VARIABLES {}\n\n",
            icasadi::NUM_DECISION_VARIABLES,
        )
        .as_str(),
    );
    header.push_str("/** This is the size of the param arrays that the solver needs. */\n");
    header.push_str(
        format!(
            "#define OPEN_NUM_STATIC_PARAMETERS {}\n",
            icasadi::NUM_STATIC_PARAMETERS
        )
        .as_str(),
    );

    Builder::new()
        .with_crate(&crate_dir)
        .with_language(Language::C)
        .with_header(&header)
        .generate()
        .expect("Unable to generate bindings")
        .write_to_file("open_clib.h");

    Builder::new()
        .with_crate(&crate_dir)
        .with_language(Language::Cxx)
        .with_header(&header)
        .generate()
        .expect("Unable to generate bindings")
        .write_to_file("open_clib.hpp");

    println!("cargo:rerun-if-changed=src/lib.rs");
    println!("cargo:rerun-if-changed=open_clib.h");
    println!("cargo:rerun-if-changed=open_clib.hpp");
}
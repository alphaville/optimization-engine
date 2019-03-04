extern crate cc;

fn main() {
    cc::Build::new()
        .flag_if_supported("-Wall")
        .flag_if_supported("-Wpedantic")
        .flag_if_supported("-Wno-long-long")
        .flag("-Wno-unused-parameter")
        .pic(true)
        .include("src")
        .file("extern/auto_casadi_cost.c")
        .file("extern/auto_casadi_grad.c")
        .file("extern/icasadi.c")
        .compile("icasadi");
}

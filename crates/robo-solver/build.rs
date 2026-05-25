use std::path::PathBuf;

fn main() {
    let manifest_dir = PathBuf::from(std::env::var("CARGO_MANIFEST_DIR").unwrap());
    let repo_root = manifest_dir
        .join("..")
        .join("..")
        .join("..")
        .join("min2phase");
    assert!(
        repo_root.join("include/min2phase/min2phase.hpp").exists(),
        "min2phase repository not found next to app"
    );

    let include_dir = repo_root.join("include");
    let sources = [
        "src/api.cpp",
        "src/coord_cube.cpp",
        "src/coord_cube2l.cpp",
        "src/cubie_cube.cpp",
        "src/cubie_cube2l.cpp",
        "src/search.cpp",
        "src/search2l.cpp",
        "src/tools.cpp",
        "src/util.cpp",
    ];

    let mut build = cc::Build::new();
    build.cpp(true);
    build.flag_if_supported("/std:c++17");
    build.flag_if_supported("-std=c++17");
    build.include(include_dir);
    build.define(
        "MIN2PHASE_SOURCE_DIR",
        format!("\"{}\"", repo_root.display()).as_str(),
    );
    build.file("src/bridge.cpp");
    for source in sources {
        build.file(repo_root.join(source));
    }
    build.compile("min2phase_bridge");

    println!("cargo:rerun-if-changed=src/bridge.cpp");
    println!("cargo:rerun-if-changed={}", repo_root.display());
}

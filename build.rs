fn main() {
    // If the proto files get updated, rerun the script
    // println!("cargo:rerun-if-changed=build.rs");

    prost_build::compile_protos(&["proto/state/ssl_gc_game_event.proto"], &["proto/"])
        .expect("Failed to compile proto services");
}
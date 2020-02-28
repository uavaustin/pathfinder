extern crate protoc_rust;

use protoc_rust::Customize;

fn main() {
    protoc_rust::run(protoc_rust::Args {
        out_dir: "src/process",
        input: &[
            "messages/interop.proto",
            "messages/pathfinder.proto",
            "messages/telemetry.proto",
        ],
        includes: &["messages"],
        customize: Customize {
            ..Default::default()
        },
    })
    .expect("protoc");
}

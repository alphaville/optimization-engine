set -euxo pipefail

main() {
    cargo build --target $TARGET
    cargo test --target $TARGET
    cargo run  --target $TARGET
    cargo doc --no-deps
}

main
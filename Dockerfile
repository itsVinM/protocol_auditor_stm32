VFG # syntax=docker/dockerfile:1.7
#
# faultforge — the whole toolchain in a container, so a clean checkout builds
# with no host Rust, no ARM toolchain and no probe-rs.
#
#   docker build -t faultforge .          # builds AND runs both test suites
#   docker run --rm faultforge --help
#
# Stage graph:
#
#   sender-builder ─┐
#                   ├─> runtime   (debian-slim + one static-ish binary)
#   firmware-builder ┘
#   flash            (separate, built only via --target flash)
#
# The unit tests run *inside* the builder stages, so a green image build is a
# green test run. There is no separate test stage to forget.

ARG RUST_VERSION=1.96
ARG DEBIAN_RELEASE=bookworm

# -----------------------------------------------------------------------------
FROM rust:${RUST_VERSION}-slim AS sender-builder

# The sender builder. BUILDPLATFORM, not TARGETPLATFORM: run the compiler
# natively instead of emulating. On Apple Silicon this avoids QEMU entirely.
# Valid because we only ever build for the host -- this is not a multi-arch
# image, and the sender is a native binary that would need a per-arch builder
# to be cross-distributed.
FROM --platform=$BUILDPLATFORM rust:${RUST_VERSION}-slim AS sender-builder
WORKDIR /app

# serialport links against libudev on Linux, so it needs the headers and
# pkg-config. No clang or lld: the rust-lld shipped with the toolchain links
# the host binary fine.
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/dpkg,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
        pkg-config libudev-dev \
    && rm -rf /var/lib/apt/lists/*

# Cache the cargo registry and git checkouts across builds. The git cache
# matters for the firmware stage, where embassy-* is a git dependency.
# CARGO_HOME is split out of the image so it can be a cache mount: the registry
# is large and rebuildable, and leaving it in a layer bloats every image.
ENV CARGO_HOME=/usr/local/cargo
RUN --mount=type=cache,target=/usr/local/cargo/registry,sharing=locked \
    --mount=type=cache,target=/usr/local/cargo/git,sharing=locked \
    mkdir -p $CARGO_HOME

# Dependency pre-fetch with a stub source, so editing src/ does not invalidate
# the dependency layer. The +1.7 dummy trick is not needed: a stub main() is
# enough here because the real sources land in the very next layer.
COPY Cargo.toml Cargo.lock ./
RUN --mount=type=cache,target=/usr/local/cargo/registry,sharing=locked \
    --mount=type=cache,target=/usr/local/cargo/git,sharing=locked \
    mkdir -p src && echo 'fn main() {}' > src/main.rs \
    && cargo build --release --locked || true

# Real sources. firmware/src is an input here too: the sender compiles
# firmware/src/protocol.rs into its binary via #[path], so that file is a build
# input for the host side as well as for the firmware.
COPY src ./src
COPY firmware/src ./firmware/src

# target/ is a cache mount so it persists between builds without living in a
# layer. The artefact is copied to /out inside the same RUN, because a cache
# mount is not part of the image filesystem and `COPY --from` cannot see it.
RUN --mount=type=cache,target=/usr/local/cargo/registry,sharing=locked \
    --mount=type=cache,target=/usr/local/cargo/git,sharing=locked \
    --mount=type=cache,target=/app/target,sharing=locked \
    set -eux; \
    cargo build --release --locked; \
    cargo test --locked; \
    mkdir -p /out; \
    cp /app/target/release/faultforge /out/faultforge

# -----------------------------------------------------------------------------
FROM --platform=$BUILDPLATFORM rust:${RUST_VERSION}-slim AS firmware-builder
WORKDIR /app

# No gcc-arm-none-eabi. thumbv7em-none-eabihf is a Rust target linked by
# flip-link, which drives the rust-lld bundled with the toolchain -- a C
# cross-compiler is not involved and would be ~200 MB of dead weight.

ENV CARGO_HOME=/usr/local/cargo
RUN rustup target add thumbv7em-none-eabihf

# flip-link is the linker named in firmware/.cargo/config.toml. The install
# output is deliberately *not* a cache mount: it is a build input the later
# firmware stage copies out of this one.
RUN --mount=type=cache,target=/usr/local/cargo/registry,sharing=locked \
    cargo install flip-link --locked

COPY firmware/Cargo.toml firmware/Cargo.lock firmware/build.rs firmware/memory.x ./firmware/
COPY firmware/.cargo ./firmware/.cargo

# Every cargo invocation in this stage must run with the CWD inside firmware/.
# Cargo finds .cargo/config.toml by walking up from the *current directory*, not
# from --manifest-path, and that file is what supplies the thumbv7em target, the
# flip-link linker and the -Tlink.x / -Tdefmt.x linker args. Building from /app
# with --manifest-path silently cross-compiles for the host instead, and dies
# with "`#[panic_handler]` function required, but not found".
WORKDIR /app/firmware
RUN --mount=type=cache,target=/usr/local/cargo/registry,sharing=locked \
    --mount=type=cache,target=/usr/local/cargo/git,sharing=locked \
    mkdir -p src \
    && echo '#![no_std] #![no_main] fn main() {}' > src/main.rs \
    && cargo build --release --locked || true

COPY src ./src

# The protocol suite is pure logic and runs natively, but [build]
# target in .cargo/config.toml applies to them too, so the host triple has to
# be passed explicitly. The bare-metal deps are gated on
# cfg(target_os = "none") precisely so this works.
RUN --mount=type=cache,target=/usr/local/cargo/registry,sharing=locked \
    --mount=type=cache,target=/usr/local/cargo/git,sharing=locked \
    --mount=type=cache,target=/app/firmware/target,sharing=locked \
    set -eux; \
    HOST=$(rustc -vV | sed -n 's/^host: //p'); \
    cargo test --locked --lib --target "$HOST"; \
    cargo build --release --locked; \
    mkdir -p /out; \
    cp /app/firmware/target/thumbv7em-none-eabihf/release/faultforge-firmware /out/firmware.elf

# -----------------------------------------------------------------------------
# Runtime: the sender binary and nothing else. No Rust, no ARM toolchain, no
# probe-rs. libudev is the only thing the sender links at runtime.
FROM debian:${DEBIAN_RELEASE}-slim AS runtime

LABEL org.opencontainers.image.title="faultforge" \
      org.opencontainers.image.description="STM32 HIL fault injection: bench sender and prebuilt firmware" \
      org.opencontainers.image.source="https://github.com/itsVinM/stm32_hil_fault_forge"

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/dpkg,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends libudev1 \
    && rm -rf /var/lib/apt/lists/* \
    && useradd --create-home --shell /usr/sbin/nologin faultforge

COPY --from=sender-builder /out/faultforge /usr/local/bin/faultforge
COPY --from=firmware-builder /out/firmware.elf /app/firmware.elf

USER faultforge
ENTRYPOINT ["faultforge"]
CMD ["--help"]

# -----------------------------------------------------------------------------
# Flashing: a separate stage, never part of the runtime image. probe-rs is a
# host-side tool, and baking it into the image the sender runs in would add a
# full Rust toolchain plus libudev headers to every run for no reason.
#
#   docker build --target flash -t faultforge-flash .
#   docker run --rm --privileged -v /dev/bus/usb:/dev/bus/usb \
#       faultforge-flash download /app/firmware.elf --chip STM32F401RETx
FROM --platform=$BUILDPLATFORM rust:${RUST_VERSION}-slim AS flash

RUN rustup target add thumbv7em-none-eabihf
RUN --mount=type=cache,target=/usr/local/cargo/registry,sharing=locked \
    --mount=type=cache,target=/usr/local/cargo/git,sharing=locked \
    cargo install probe-rs --locked

COPY --from=firmware-builder /out/firmware.elf /app/firmware.elf
ENTRYPOINT ["probe-rs"]
CMD ["run", "/app/firmware.elf", "--chip", "STM32F401RETx"]

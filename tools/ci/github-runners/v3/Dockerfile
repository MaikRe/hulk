FROM rust:latest

RUN apt-get update && apt-get install --no-install-recommends --yes \
  clang \
  cmake \
  git-lfs \
  libasound2-dev \
  libdbus-1-dev \
  libhdf5-dev \
  libluajit-5.1-dev \
  libopusfile-dev \
  libsystemd-dev \
  libudev-dev \
  lua-inspect \
  ninja-build \
  python3 \
  python3-click \
  python3-git \
  rsync \
  zstd \
  && rm --recursive --force /var/lib/apt/lists/*

RUN rustup component add clippy rustfmt

RUN useradd --no-log-init --create-home --home-dir /ci ci
WORKDIR /ci
USER ci:ci

ARG SDK_VERSION
RUN wget --no-verbose https://github.com/HULKs/meta-nao/releases/download/${SDK_VERSION}/HULKs-OS-toolchain-${SDK_VERSION}.sh && \
  chmod +x HULKs-OS-toolchain-${SDK_VERSION}.sh && \
  ./HULKs-OS-toolchain-${SDK_VERSION}.sh -y -d /ci/.naosdk/${SDK_VERSION} && \
  rm HULKs-OS-toolchain-${SDK_VERSION}.sh

COPY cargo.sh.patch /ci
RUN cd /ci/.naosdk/${SDK_VERSION}/sysroots/x86_64-naosdk-linux/environment-setup.d && \
  patch -i /ci/cargo.sh.patch && \
  rm /ci/cargo.sh.patch

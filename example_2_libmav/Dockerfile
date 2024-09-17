# 1. -- Sysroot build stage. Runs in arm64 emulation (QEMU) --
# Install auterion-sdk
FROM auterion/app-base:v2 as sysroot-stage

RUN curl -1sLf 'https://dl.cloudsmith.io/public/auterion/public/setup.deb.sh'| sudo -E bash
RUN apt update && apt install -y \
    ros-humble-auterion-sdk && \
    apt clean && \
    rm -rf /var/lib/apt/lists/*

# 2. -- Cross-compile build stage. Runs native on x86 --
FROM --platform=$BUILDPLATFORM auterion/app-cross-buildenv:v2 as build-stage

# Declare automatically defined global scope Docker arg
ARG TARGETARCH

# Copy sysroot from sysroot-stage
COPY --from=sysroot-stage / /opt/sysroot

# Fix broken symlinks
RUN if [ "$TARGETARCH" = "arm64" ]; then \
      ln -snf /opt/sysroot/usr/lib/aarch64-linux-gnu/lapack/liblapack.so.3 /opt/sysroot/usr/lib/aarch64-linux-gnu/liblapack.so.3 && \
      ln -snf /opt/sysroot/usr/lib/aarch64-linux-gnu/blas/libblas.so.3 /opt/sysroot/usr/lib/aarch64-linux-gnu/libblas.so.3; \
    fi

# Compile the app
COPY src /opt/sysroot/app/src
COPY libmav /opt/sysroot/app/libmav
COPY mavlink /opt/sysroot/app/mavlink

COPY CMakeLists.txt /opt/sysroot/app/CMakeLists.txt
WORKDIR /opt/sysroot/app
RUN . /opt/sysroot/opt/ros/humble/setup.sh && \
    cmake -B build -S . -DCMAKE_TOOLCHAIN_FILE=/Toolchain_$TARGETARCH.cmake && cmake --build build

# 3. -- Runtime stage. Runs in arm64 emulation and on target --
FROM sysroot-stage as run-stage

COPY --from=build-stage /opt/sysroot/app/build/ /opt/sysroot/app/build
COPY --from=build-stage /opt/sysroot/app/mavlink /opt/sysroot/app/mavlink
COPY --from=build-stage /opt/sysroot/app/libmav /opt/sysroot/app/libmav
RUN echo "source /opt/ros/humble/setup.sh" >> ~/.bashrc
WORKDIR /opt/sysroot/app/build
CMD . /opt/ros/humble/setup.sh && \
    /opt/sysroot/app/build/libmav_mode
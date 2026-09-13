#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SOURCE_DIR="${SCRIPT_DIR}/src"
BUILD_DIR="${SCRIPT_DIR}/build"
PREFIX="${PREFIX:-/usr/local}"
JOBS="${JOBS:-$(nproc)}"
BUILD_TYPE="${BUILD_TYPE:-Release}"
BLASFEO_TARGET="${BLASFEO_TARGET:-GENERIC}"

required_sources=(eigen qdldl osqp nlopt ipopt casadi acados)
for dependency in "${required_sources[@]}"; do
  if [[ ! -d "${SOURCE_DIR}/${dependency}" ]]; then
    printf 'Missing source: %s\n' "${SOURCE_DIR}/${dependency}" >&2
    printf 'Run: git submodule update --init\n' >&2
    exit 1
  fi
done

MUMPS_INCLUDE_DIR="${MUMPS_INCLUDE_DIR:-/usr/include}"
MUMPS_SEQ_INCLUDE_DIR="${MUMPS_SEQ_INCLUDE_DIR:-/usr/include/mumps_seq}"
required_mumps_headers=(
  "${MUMPS_INCLUDE_DIR}/mumps_compat.h"
  "${MUMPS_INCLUDE_DIR}/dmumps_c.h"
  "${MUMPS_SEQ_INCLUDE_DIR}/mpi.h"
)
for header in "${required_mumps_headers[@]}"; do
  if [[ ! -f "${header}" ]]; then
    printf 'Missing system MUMPS header: %s\n' "${header}" >&2
    printf 'Install it with: sudo apt install libmumps-seq-dev\n' >&2
    exit 1
  fi
done
for acados_dependency in blasfeo hpipm; do
  if [[ ! -e "${SOURCE_DIR}/acados/external/${acados_dependency}/CMakeLists.txt" ]]; then
    printf 'Missing acados source: external/%s\n' "${acados_dependency}" >&2
    printf 'Run: git -C 3rdpart/src/acados submodule update --init external/blasfeo external/hpipm\n' >&2
    exit 1
  fi
done

mkdir -p "${BUILD_DIR}"

install_cmake() {
  local build_path="$1"
  if [[ -w "${PREFIX}" ]] || [[ ! -e "${PREFIX}" && -w "$(dirname "${PREFIX}")" ]]; then
    cmake --install "${build_path}"
  else
    sudo cmake --install "${build_path}"
  fi
}

install_make() {
  local build_path="$1"
  if [[ -w "${PREFIX}" ]] || [[ ! -e "${PREFIX}" && -w "$(dirname "${PREFIX}")" ]]; then
    make -C "${build_path}" install
  else
    sudo make -C "${build_path}" install
  fi
}

printf '\n[1/6] Eigen 3.4.0 -> %s\n' "${PREFIX}"
cmake -S "${SOURCE_DIR}/eigen" -B "${BUILD_DIR}/eigen" \
  -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" \
  -DCMAKE_INSTALL_PREFIX="${PREFIX}" \
  -DBUILD_TESTING=OFF
cmake --build "${BUILD_DIR}/eigen" --parallel "${JOBS}"
install_cmake "${BUILD_DIR}/eigen"

printf '\n[2/6] OSQP 1.0.0 (using local QDLDL 0.1.8) -> %s\n' "${PREFIX}"
cmake -S "${SOURCE_DIR}/osqp" -B "${BUILD_DIR}/osqp" \
  -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" \
  -DCMAKE_INSTALL_PREFIX="${PREFIX}" \
  -DOSQP_VERSION=1.0.0 \
  -DOSQP_BUILD_SHARED_LIB=ON \
  -DOSQP_BUILD_STATIC_LIB=OFF \
  -DOSQP_BUILD_UNITTESTS=OFF \
  -DOSQP_BUILD_DEMO_EXE=OFF \
  -DFETCHCONTENT_UPDATES_DISCONNECTED=ON \
  -DFETCHCONTENT_SOURCE_DIR_QDLDL="${SOURCE_DIR}/qdldl"
cmake --build "${BUILD_DIR}/osqp" --parallel "${JOBS}"
install_cmake "${BUILD_DIR}/osqp"

printf '\n[3/6] NLopt 2.7.1 -> %s\n' "${PREFIX}"
cmake -S "${SOURCE_DIR}/nlopt" -B "${BUILD_DIR}/nlopt" \
  -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" \
  -DCMAKE_INSTALL_PREFIX="${PREFIX}" \
  -DBUILD_SHARED_LIBS=ON \
  -DNLOPT_PYTHON=OFF \
  -DNLOPT_OCTAVE=OFF \
  -DNLOPT_MATLAB=OFF \
  -DNLOPT_GUILE=OFF \
  -DNLOPT_SWIG=OFF \
  -DNLOPT_TESTS=OFF
cmake --build "${BUILD_DIR}/nlopt" --parallel "${JOBS}"
install_cmake "${BUILD_DIR}/nlopt"

printf '\n[4/6] Ipopt 3.14.11 -> %s\n' "${PREFIX}"
mkdir -p "${BUILD_DIR}/ipopt"
(cd "${BUILD_DIR}/ipopt" && \
  "${SOURCE_DIR}/ipopt/configure" \
    --prefix="${PREFIX}" \
    --disable-java \
    "--with-mumps-cflags=-I${MUMPS_INCLUDE_DIR} -I${MUMPS_SEQ_INCLUDE_DIR}" \
    --with-mumps-lflags=-ldmumps_seq \
    "--with-lapack-lflags=-llapack -lblas")
make -C "${BUILD_DIR}/ipopt" -j"${JOBS}"
install_make "${BUILD_DIR}/ipopt"

printf '\n[5/6] CasADi 3.7.2 with system Ipopt -> %s\n' "${PREFIX}"
export PKG_CONFIG_PATH="${PREFIX}/lib/pkgconfig:${PREFIX}/lib64/pkgconfig:${PKG_CONFIG_PATH:-}"
cmake -S "${SOURCE_DIR}/casadi" -B "${BUILD_DIR}/casadi" \
  -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" \
  -DCMAKE_INSTALL_PREFIX="${PREFIX}" \
  -DCMAKE_PREFIX_PATH="${PREFIX}" \
  -DWITH_IPOPT=ON \
  -DWITH_BUILD_IPOPT=OFF \
  -DWITH_PYTHON=OFF \
  -DWITH_SELFCONTAINED=OFF \
  -DWITH_THREAD=ON
cmake --build "${BUILD_DIR}/casadi" --parallel "${JOBS}"
install_cmake "${BUILD_DIR}/casadi"

printf '\n[6/6] acados 4c23274e4 with HPIPM/BLASFEO -> %s\n' "${PREFIX}"
cmake -S "${SOURCE_DIR}/acados" -B "${BUILD_DIR}/acados" \
  -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" \
  -DCMAKE_INSTALL_PREFIX="${PREFIX}" \
  -DACADOS_INSTALL_DIR="${PREFIX}" \
  -DBUILD_SHARED_LIBS=ON \
  -DBLASFEO_TARGET="${BLASFEO_TARGET}" \
  -DHPIPM_TARGET=GENERIC \
  -DACADOS_EXAMPLES=OFF \
  -DACADOS_UNIT_TESTS=OFF
cmake --build "${BUILD_DIR}/acados" --parallel "${JOBS}"
install_cmake "${BUILD_DIR}/acados"

if command -v ldconfig >/dev/null 2>&1; then
  if [[ "$(id -u)" -eq 0 ]]; then
    ldconfig
  else
    sudo ldconfig
  fi
fi

printf '\nAll source-built dependencies are installed in %s.\n' "${PREFIX}"
printf 'Sources remain in %s; build products remain in %s.\n' "${SOURCE_DIR}" "${BUILD_DIR}"

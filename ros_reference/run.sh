#!/usr/bin/env bash
# Runs the reference of every scenario of the ROS reference harness in the reference
# image, built for the architecture of this host.
#
#   ros_reference/run.sh          writes what the image gives
#   ros_reference/run.sh --check  writes nothing, and fails when what the image gives
#                                 differs from the committed files, printing each
#                                 difference
#
# On amd64, the image gives the reference outputs under reference/, with the SHA-256
# digest of each output in SHA256SUMS and of each file of the repository that the
# references read (each scenario and the files it names) in INPUTS.SHA256SUMS, and the
# version of each of its packages, in packages-amd64.txt. On arm64, it gives the version
# of each of its packages, in packages-arm64.txt, and the parity tests
# (tests/ros_reference_*.rs of each crate) then compare with its outputs, within their
# bounds.
#
# The container runs with no network and one OpenMP thread, reads the repository through
# a read-only mount, and writes its outputs as the user who runs the script. Both mounts
# carry :z, so SELinux lets the container read and write them. Docker builds with buildx
# when the buildx plugin is installed, loading the image into the engine as a builder of
# the docker-container driver needs, and with its legacy builder otherwise. The script
# runs the image by the id that its build gives, so each checkout runs the image of its
# own files.
set -euo pipefail

# On amd64, the CPU features that the glibc.cpu.hwcaps tunable masks. The selectors of
# glibc 2.43's math functions read FMA, FMA4, AVX, AVX2 and SSE4_1 (libm), and SSE4_1,
# FMA, AVX2, AVX512F and AVX512DQ (libmvec). The mask holds all of them, with SSSE3,
# SSE4_2, POPCNT, BMI1, BMI2, LZCNT, MOVBE and AVX512BW, CD, VL, ER and PF, which no math
# selector reads, so every amd64 host runs the SSE2 variants of the math functions, of
# baseline instructions alone, and the outputs hold the same bits.
amd64_hwcaps_mask=-SSSE3,-SSE4_1,-SSE4_2,-POPCNT,-AVX,-AVX2,-BMI1,-BMI2,-FMA,-FMA4,-LZCNT,-MOVBE,-AVX512F,-AVX512BW,-AVX512CD,-AVX512DQ,-AVX512VL,-AVX512ER,-AVX512PF

# Runs in the container: the reference of every scenario, which lists the files it read
# in inputs.txt, the digests of the outputs and of those files, and the version of each
# package.
read -r -d '' container_script <<'EOF' || true
set -euo pipefail
cd /repository
mkdir /outputs/reference
for scenario in ros_reference/scenarios/*.json; do
  ros_reference "$scenario" "/outputs/reference/$(basename "$scenario")" /repository \
    /outputs/inputs.txt
done
sort -u /outputs/inputs.txt | xargs -d '\n' sha256sum > /outputs/reference/INPUTS.SHA256SUMS
rm /outputs/inputs.txt
(cd /outputs/reference && sha256sum *.json > SHA256SUMS)
dpkg-query --show --showformat '${Package} ${Version}\n' | sort > /outputs/packages.txt
EOF

usage() {
  echo "usage: $0 [--check]" >&2
  exit 2
}

case $#:${1-} in
  0:) mode=write ;;
  1:--check) mode=check ;;
  *) usage ;;
esac

case $(uname -m) in
  x86_64) platform=amd64 ;;
  aarch64 | arm64) platform=arm64 ;;
  *)
    echo "$0 runs on an amd64 or an arm64 host, and this host is $(uname -m)" >&2
    exit 2
    ;;
esac

# The options under which the container writes its outputs as the user who runs the
# script: rootless Podman maps that user into the container with keep-id, rootless
# Docker maps the container's root to that user, and a rootful engine runs the container
# as that user. Podman names itself in `docker --version`.
if docker --version | grep -q -i podman; then
  engine=podman
  rootless=$(docker info --format '{{.Host.Security.Rootless}}')
else
  engine=docker
  security_options=$(docker info --format '{{json .SecurityOptions}}')
  rootless=$([[ $security_options == *name=rootless* ]] && echo true || echo false)
fi
case $engine:$rootless in
  podman:true) user_options=(--userns=keep-id) ;;
  docker:true) user_options=() ;;
  *) user_options=(--user "$(id -u):$(id -g)") ;;
esac
if [[ $engine == docker ]] && docker buildx version >/dev/null 2>&1; then
  build=(docker buildx build --load)
else
  build=(docker build)
fi

harness=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
repository=$(dirname "$harness")
packages=$harness/packages-$platform.txt

scratch=$(mktemp -d)
trap 'rm -rf "$scratch"' EXIT
outputs=$scratch/outputs
mkdir "$outputs"

"${build[@]}" --quiet --platform "linux/$platform" --iidfile "$scratch/image" "$harness" \
  >/dev/null

environment=(--env OMP_NUM_THREADS=1)
if [[ $platform == amd64 ]]; then
  environment+=(--env "GLIBC_TUNABLES=glibc.cpu.hwcaps=$amd64_hwcaps_mask")
fi
docker run --rm --platform "linux/$platform" --network none "${user_options[@]}" \
  "${environment[@]}" --volume "$repository:/repository:ro,z" \
  --volume "$outputs:/outputs:z" "$(<"$scratch/image")" bash -c "$container_script"

failed=0
if [[ $mode == check ]]; then
  diff -u "$packages" "$outputs/packages.txt" || failed=1
  if [[ $platform == amd64 ]]; then
    diff -ru "$harness/reference" "$outputs/reference" || failed=1
  fi
  if ((failed)); then
    echo "the image gives the lines marked +, and the committed files hold the lines" \
      "marked -; ros_reference/run.sh writes what the image gives" >&2
  else
    echo "the committed files hold what the $platform image gives"
  fi
else
  cp "$outputs/packages.txt" "$packages"
  if [[ $platform == amd64 ]]; then
    rm -f "$harness"/reference/*
    cp "$outputs"/reference/* "$harness/reference/"
  fi
  echo "wrote what the $platform image gives"
fi

if [[ $platform == arm64 ]]; then
  shopt -s nullglob
  parity_tests=("$repository"/*/tests/ros_reference_*.rs)
  if ((${#parity_tests[@]} == 0)); then
    echo "the repository holds no tests/ros_reference_*.rs to compare the arm64 outputs" >&2
    exit 1
  fi
  for parity_test in "${parity_tests[@]}"; do
    (cd "$(dirname "$(dirname "$parity_test")")" &&
      ROS_REFERENCE_OUTPUTS=$outputs/reference \
        cargo test --locked --test "$(basename "$parity_test" .rs)") || failed=1
  done
fi

exit "$failed"

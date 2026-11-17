#!/usr/bin/env bash
# Runs the reference of every scenario of the ROS reference harness in the reference
# image, built for the architecture of this host.
#
#   ros_reference/run.sh             writes what the image gives
#   ros_reference/run.sh --check     writes no committed file, and fails when what the
#                                    image gives differs from the committed files,
#                                    printing each difference
#   ros_reference/run.sh --datasets  runs the local-only kinds too, in either mode
#
# A local-only kind reads datasets that the repository does not hold. With --datasets, the
# script fetches each archive that datasets.txt lists into the dataset cache,
# ${XDG_CACHE_HOME:-$HOME/.cache}/peppy-ros-reference/<SHA-256>/, checks its digest,
# extracts the files that the references read, extracting them again when they differ
# from the digests taken as they were extracted, and mounts the cache read-only in the
# container, which writes the runs of the local-only references, in either mode, to
# ros_reference/target/results/ of the checkout that holds this script; the crate's
# statistics program then scores them into their reference outputs. Without --datasets,
# the committed outputs of the local-only kinds stand, with the committed digests of the
# inputs that they list.
#
# On amd64, the image gives the reference outputs under reference/, with the SHA-256
# digest of each output in SHA256SUMS and of each file of the repository that an output
# depends on in INPUTS.SHA256SUMS (each scenario and the files it names, which the
# references read, and the sources of each local-only kind), and the version of each of
# its packages, in packages-amd64.txt. On arm64, it gives the version
# of each of its packages, in packages-arm64.txt, and the parity tests
# (tests/ros_reference_*.rs of each crate) then compare with its outputs, within their
# bounds.
#
# The container runs with no network, one OpenMP thread and the OpenBLAS kernels of a
# baseline CPU of the architecture, reads the repository through
# a read-only mount, and writes its outputs as the user who runs the script. Both mounts
# carry :z, so SELinux lets the container read and write them. Docker builds with buildx
# when the buildx plugin is installed, loading the image into the engine as a builder of
# the docker-container driver needs, and with its legacy builder otherwise. The script
# runs the image by the id that its build gives, so each checkout runs the image of its
# own files. It names each container that it starts
# ros_reference-<checkout>-<digest>-<step>, where <checkout> is the path of the checkout
# that holds this script, each character of it outside A-Z, a-z, 0-9, _, . and - written as
# - and its leading dashes dropped, <digest> the first 12 hexadecimal digits of the SHA-256
# of that path, which tells apart paths that the rewriting makes alike, and <step> is
# references or digests.
#
# A run holds an flock on ros_reference/target/run.lock from start to end, and refuses to
# start while another run holds it or a container of either name exists. With --datasets,
# it also holds an flock on the cache's cache.lock until it ends: shared while it reads the
# cache, and exclusive while it changes the cache by a fetch or an extraction, which it does
# only when no other run holds that lock. The kernel releases each lock when the run ends,
# however it ends. When the script exits, on an error or on SIGTERM as well, it removes each
# container that it started and its scratch directory.
#
# When it starts, a run takes the digest of each file of the checkout outside .git and the
# target/ directories. INPUTS.SHA256SUMS takes from them the digest of each file that an
# output the run writes depends on, and the run fails, naming the file, when one of those
# files differs from its digest at the end. The committed outputs of the local-only kinds,
# which a run without --datasets keeps, keep the committed digests of the files they list.
set -euo pipefail

# The kinds whose references read the dataset cache.
local_only_kinds=(room_dataset)

# On amd64, the CPU features that the glibc.cpu.hwcaps tunable masks. The selectors of
# glibc 2.43's math functions read FMA, FMA4, AVX, AVX2 and SSE4_1 (libm), and SSE4_1,
# FMA, AVX2, AVX512F and AVX512DQ (libmvec). The mask holds all of them, with SSSE3,
# SSE4_2, POPCNT, BMI1, BMI2, LZCNT, MOVBE and AVX512BW, CD, VL, ER and PF, which no math
# selector reads, so every amd64 host runs the SSE2 variants of the math functions, of
# baseline instructions alone, and the outputs hold the same bits.
amd64_hwcaps_mask=-SSSE3,-SSE4_1,-SSE4_2,-POPCNT,-AVX,-AVX2,-BMI1,-BMI2,-FMA,-FMA4,-LZCNT,-MOVBE,-AVX512F,-AVX512BW,-AVX512CD,-AVX512DQ,-AVX512VL,-AVX512ER,-AVX512PF

# The OpenBLAS core whose kernels the references run on each architecture: its baseline,
# which every host has, so OpenBLAS picks the same kernels on every host.
amd64_openblas_core=Prescott
arm64_openblas_core=ARMV8

# Runs in the container: the reference of every scenario but the local-only ones that
# LOCAL_ONLY names, each listing the files it read in inputs.txt, and the version of each
# package. With DATASETS=1, the reference of a local-only scenario runs too, and writes
# its output to /results, from which the statistics program scores it.
read -r -d '' references_script <<'EOF' || true
set -euo pipefail
cd /repository
for scenario in ros_reference/scenarios/*.json; do
  name=$(basename "$scenario")
  if [[ " $LOCAL_ONLY " == *" $name "* ]]; then
    if [[ $DATASETS == 1 ]]; then
      ros_reference "$scenario" "/results/$name" /repository /outputs/inputs.txt
    fi
    continue
  fi
  ros_reference "$scenario" "/outputs/reference/$name" /repository /outputs/inputs.txt
done
dpkg-query --show --showformat '${Package} ${Version}\n' | sort > /outputs/packages.txt
EOF

# Runs in the container, after digest_line_of: the digests of the outputs and of the files
# that they depend on, each input that carried.SHA256SUMS lists with the digest it gives
# there and every other with its digest when the run started, which start.SHA256SUMS
# gives and the file still has.
read -r -d '' digests_script <<'EOF' || true
set -euo pipefail
cd /repository
sort -u /outputs/inputs.txt | while IFS= read -r input; do
  carried=$(digest_line_of /outputs/carried.SHA256SUMS "$input")
  if [[ -n $carried ]]; then
    echo "$carried"
    continue
  fi
  started=$(digest_line_of /outputs/start.SHA256SUMS "$input")
  if [[ -z $started ]]; then
    echo "$input, which an output depends on, is no regular file of the checkout outside" \
      ".git and the target/ directories, whose digests the run takes when it starts" >&2
    exit 1
  fi
  if ! sha256sum --check --status <<<"$started"; then
    echo "$input, which an output depends on, changed during the run; run $RUN_COMMAND" \
      "again, and edit no file of the checkout while it runs" >&2
    exit 1
  fi
  echo "$started"
done >/outputs/reference/INPUTS.SHA256SUMS
rm /outputs/inputs.txt /outputs/carried.SHA256SUMS /outputs/start.SHA256SUMS
(cd /outputs/reference && sha256sum *.json > SHA256SUMS)
EOF

# The line of the sha256sum list $1 that gives the digest of the file $2. A line of the list
# holds a digest, two spaces and the file's path.
digest_line_of() {
  awk -v file="$2" 'substr($0, length($1) + 3) == file { print; exit }' "$1"
}

usage() {
  echo "usage: $0 [--check] [--datasets]" >&2
  exit 2
}

mode=write
datasets=0
for argument in "$@"; do
  case $argument in
    --check) mode=check ;;
    --datasets) datasets=1 ;;
    *) usage ;;
  esac
done
# The command line of this run, and of the run that writes what this one compares.
run_command="ros_reference/run.sh${*:+ $*}"
write_command=ros_reference/run.sh
if ((datasets)); then
  write_command+=" --datasets"
fi

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
container_prefix=ros_reference-$(sed 's/[^A-Za-z0-9_.-]/-/g; s/^-*//' <<<"$repository")
container_prefix+=-$(printf '%s' "$repository" | sha256sum | cut -c1-12)
packages=$harness/packages-$platform.txt
cache=${XDG_CACHE_HOME:-$HOME/.cache}/peppy-ros-reference
cache_lock=$cache/cache.lock
results=$harness/target/results
run_lock=$harness/target/run.lock

# The scratch directory of this run, which the exit path removes with each container that
# the run started.
scratch=$(mktemp -d)
remove_what_this_run_started() {
  local id ids
  ids=$(docker ps --all --quiet --filter "label=ros_reference.run=$scratch" || true)
  for id in $ids; do
    docker rm --force "$id" >/dev/null || true
  done
  rm -rf -- "$scratch"
}
trap remove_what_this_run_started EXIT

# The checkout's lock: this run holds an flock on run.lock through descriptor 8, which the
# processes it starts inherit, until it and they end, and writes its pid there, which the
# refusal of another run names.
mkdir -p "$harness/target"
exec 8<>"$run_lock"
if ! flock --exclusive --nonblock 8; then
  holder=$(<"$run_lock")
  if ps -p "$holder" >/dev/null 2>&1; then
    echo "another run of ros_reference/run.sh runs in this checkout, as pid $holder;" \
      "wait for it to end, or stop it with kill $holder" >&2
  else
    echo "processes that a run of ros_reference/run.sh in this checkout started, as pid" \
      "$holder, run on and hold $run_lock; fuser -v $run_lock lists them" >&2
  fi
  exit 1
fi
printf '%s\n' "$$" >"$run_lock"
for step in references digests; do
  if docker container inspect "$container_prefix-$step" >/dev/null 2>&1; then
    echo "the container $container_prefix-$step, of a run of ros_reference/run.sh in this" \
      "checkout that ended without removing it, exists; remove it with" \
      "docker rm --force $container_prefix-$step and run the script again" >&2
    exit 1
  fi
done

# The scenarios of the local-only kinds, by file name: each scenario holds its kind on a
# line of its own, where the layout of the scenarios puts it.
local_only=()
for scenario in "$harness"/scenarios/*.json; do
  for kind in "${local_only_kinds[@]}"; do
    if grep -q -x "  \"kind\": \"$kind\"," "$scenario"; then
      local_only+=("$(basename "$scenario")")
    fi
  done
done

# Extracts the depth/ and groundtruth.txt of the archive $2 into extracted/ of the
# directory $1, with extracted.SHA256SUMS beside it, the digest of each file extracted.
extract_dataset() {
  local directory=$1 archive=$2
  rm -rf -- "${directory:?}/extracting"
  mkdir "$directory/extracting"
  tar --extract --gzip --file "$archive" --directory "$directory/extracting" \
    --strip-components 1 --wildcards '*/depth/*' '*/groundtruth.txt'
  (cd "$directory/extracting" && find . -type f -print0 | LC_ALL=C sort -z |
    xargs -0 sha256sum) >"$directory/extracting.SHA256SUMS"
  rm -rf -- "$directory/extracted"
  mv "$directory/extracting" "$directory/extracted"
  mv "$directory/extracting.SHA256SUMS" "$directory/extracted.SHA256SUMS"
}

# Whether extracted/ of the directory $1 holds the files that extracted.SHA256SUMS lists
# and no other, each with its digest there.
extracted_intact() {
  local directory=$1
  [[ -d $directory/extracted && -f $directory/extracted.SHA256SUMS ]] &&
    (cd "$directory/extracted" &&
      diff -q <(find . -type f | LC_ALL=C sort) \
        <(awk '{ print substr($0, length($1) + 3) }' ../extracted.SHA256SUMS) >/dev/null &&
      sha256sum --check --quiet --status ../extracted.SHA256SUMS)
}

# Turns this run's flock on the cache exclusive, unless it is, and exits when another run
# holds that lock: $1 says how this run changes the cache.
holds_the_cache_exclusively=0
change_the_cache() {
  if ((holds_the_cache_exclusively)); then
    return
  fi
  if ! flock --exclusive --nonblock 9; then
    echo "the dataset cache needs $1, and another run of ros_reference/run.sh reads it;" \
      "wait for that run to end, then run $run_command again" >&2
    exit 1
  fi
  holds_the_cache_exclusively=1
}

# Fetches each archive of datasets.txt into the cache, unless the cache holds it, checks
# its SHA-256, and extracts its depth/ and groundtruth.txt into extracted/ beside it,
# unless extracted/ is intact.
fetch_datasets() {
  local sha256 url name directory archive
  while read -r sha256 url name; do
    if [[ -z $sha256 || $sha256 == \#* ]]; then
      continue
    fi
    directory=$cache/$sha256
    archive=$directory/$(basename "$url")
    mkdir -p "$directory"
    if [[ ! -f $archive ]]; then
      change_the_cache "the dataset $name fetched"
      echo "fetching the dataset $name from $url" >&2
      curl --fail --location --retry 3 --silent --show-error --output "$archive.part" "$url"
      mv "$archive.part" "$archive"
    fi
    if ! echo "$sha256  $archive" | sha256sum --check --status; then
      echo "$archive, the dataset $name, does not have the SHA-256 that" \
        "ros_reference/datasets.txt gives; remove it and run $run_command again" >&2
      exit 1
    fi
    if ! extracted_intact "$directory"; then
      change_the_cache "the dataset $name extracted again"
      echo "extracting the dataset $name into $directory/extracted" >&2
      extract_dataset "$directory" "$archive"
    fi
  done <"$harness/datasets.txt"
}

# With --datasets, this run holds an flock on the cache through descriptor 9 until it ends:
# shared while it reads the cache, and exclusive while it fetches or extracts a dataset.
if ((datasets)); then
  mkdir -p "$cache"
  exec 9<>"$cache_lock"
  if ! flock --shared --nonblock 9; then
    echo "another run of ros_reference/run.sh changes the dataset cache; wait for that" \
      "run to end, then run $run_command again" >&2
    exit 1
  fi
  fetch_datasets
  if ((holds_the_cache_exclusively)); then
    flock --shared 9
    holds_the_cache_exclusively=0
  fi
  mkdir -p "$results"
fi

outputs=$scratch/outputs
mkdir "$outputs" "$outputs/reference"

# The digest of each file of the checkout as the run starts, outside .git and the target/
# directories, by its path from the repository root: the digests step takes the digest of
# each input from it.
(cd "$repository" && find . -path ./.git -prune -o -type d -name target -prune -o -type f \
  -print0 | LC_ALL=C sort -z | xargs -0 sha256sum |
  sed 's/^\([0-9a-f]*\)  \.\//\1  /') >"$outputs/start.SHA256SUMS"

# Without --datasets, the outputs of the local-only scenarios are their committed ones,
# with the inputs that each lists and the committed digest of each.
touch "$outputs/carried.SHA256SUMS"
if ((!datasets)); then
  for name in "${local_only[@]}"; do
    if [[ ! -f $harness/reference/$name ]]; then
      echo "ros_reference/reference/ holds no output of the local-only scenario $name;" \
        "ros_reference/run.sh --datasets writes it" >&2
      exit 1
    fi
    cp "$harness/reference/$name" "$outputs/reference/$name"
    while IFS= read -r input; do
      digest=$(digest_line_of "$harness/reference/INPUTS.SHA256SUMS" "$input")
      if [[ -z $digest ]]; then
        echo "ros_reference/reference/INPUTS.SHA256SUMS holds no digest of $input, which" \
          "ros_reference/reference/$name lists; ros_reference/run.sh --datasets writes" \
          "both" >&2
        exit 1
      fi
      echo "$input" >>"$outputs/inputs.txt"
      echo "$digest" >>"$outputs/carried.SHA256SUMS"
    done < <(sed -n 's/^  "inputs": \[\(.*\)\],$/\1/p' "$harness/reference/$name" |
      tr ',' '\n' | tr -d '"')
  done
fi

"${build[@]}" --quiet --platform "linux/$platform" --iidfile "$scratch/image" "$harness" \
  >/dev/null

environment=(--env OMP_NUM_THREADS=1 --env "LOCAL_ONLY=${local_only[*]}"
  --env "DATASETS=$datasets" --env "RUN_COMMAND=$run_command")
if [[ $platform == amd64 ]]; then
  environment+=(--env "GLIBC_TUNABLES=glibc.cpu.hwcaps=$amd64_hwcaps_mask"
    --env "OPENBLAS_CORETYPE=$amd64_openblas_core")
else
  environment+=(--env "OPENBLAS_CORETYPE=$arm64_openblas_core")
fi
volumes=(--volume "$repository:/repository:ro,z" --volume "$outputs:/outputs:z")
if ((datasets)); then
  environment+=(--env ROS_REFERENCE_DATASETS=/datasets --env ROS_REFERENCE_RESULTS=/results)
  volumes+=(--volume "$cache:/datasets:ro,z" --volume "$results:/results:z")
fi
# Runs the script $2 in the image, in the container of the step $1.
run_in_image() {
  docker run --rm --name "$container_prefix-$1" --label "ros_reference.run=$scratch" \
    --platform "linux/$platform" --network none "${user_options[@]}" "${environment[@]}" \
    "${volumes[@]}" "$(<"$scratch/image")" bash -c "$2"
}
run_in_image references "$references_script"

# With --datasets, the outputs of the local-only scenarios are the statistics of their runs,
# which the crate's statistics program computes on the host, on amd64 under the container's
# glibc.cpu.hwcaps mask.
if ((datasets)); then
  statistics_environment=()
  if [[ $platform == amd64 ]]; then
    statistics_environment=("GLIBC_TUNABLES=glibc.cpu.hwcaps=$amd64_hwcaps_mask")
  fi
  (cd "$harness" &&
    env "${statistics_environment[@]}" cargo run --locked --release --quiet \
      --bin statistics -- "$results" "$outputs/reference" "$outputs/inputs.txt")
fi
run_in_image digests "$(declare -f digest_line_of)"$'\n'"$digests_script"

failed=0
if [[ $mode == check ]]; then
  diff -u "$packages" "$outputs/packages.txt" || failed=1
  if [[ $platform == amd64 ]]; then
    diff -ru "$harness/reference" "$outputs/reference" || failed=1
  fi
  if ((failed)); then
    echo "the image gives the lines marked +, and the committed files hold the lines" \
      "marked -; $write_command writes what the image gives" >&2
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

# ROS reference harness

A parity test of this repository checks a library that follows ROS code: it states what
the library guarantees, and the agreement of the library's outputs with those of the ROS
code, within a stated bound, is the evidence. The ROS code runs with ROS 2, in a reference
image: developers run it through `run.sh`, and the parity tests read the outputs that it
wrote.

A parity test reads a **scenario**, an input that the test and the reference both read,
and the **reference output** that the reference wrote for it. The reference outputs are
committed with the digests of them and of their inputs, so the parity tests run in the
routine `cargo test` of each crate, with no ROS on the machine, and fail on an output, a
scenario or a URDF that differs from its digest. A change of the digests is a change of
the reference, which `run.sh --check` proves.

| Kind of scenario | Reference | Parity test |
|---|---|---|
| `forward_kinematics` | MoveIt's robot state: the global transform of each link of a chain | `chain_kinematics/tests/ros_reference_forward_kinematics.rs` |

## Layout

| Path | What it holds |
|---|---|
| `scenarios/` | One JSON file per scenario. Its `kind` names the reference that runs on it. |
| `reference/` | The reference output of each scenario, under the scenario's file name; `SHA256SUMS`, the SHA-256 digest of each output; and `INPUTS.SHA256SUMS`, the SHA-256 digest of each file of the repository that the references read (each scenario and the files it names), by its path from the repository root, as the reference program lists them. Every number of an output is an IEEE-754 bit pattern, `0x` and 16 hexadecimal digits, so a test reads the reference's own bits. |
| `packages-amd64.txt`, `packages-arm64.txt` | The version of every package of the reference image, on each architecture. |
| `reference_programs/` | The reference program `ros_reference`: one function per kind of scenario. |
| `src/` | The Rust crate `ros_reference`: one module per kind of scenario, which lists the scenarios of the kind and reads each with its reference output, checked with the scenario and its URDF against their digests, for the parity test. |
| `tools/` | The scripts that write the scenarios, one per kind, and `drawing.py`, what they share: the paths, the generator of each scenario's draws, the joint limits of a URDF, the configurations of a chain, and `write`, which writes a scenario. |
| `Dockerfile`, `CMakeLists.txt`, `ros-snapshots.asc` | The reference image and the build of the reference program in it. |
| `run.sh` | Runs the reference of every scenario in the reference image. |

## Running the references

```sh
ros_reference/run.sh          # writes what the reference image gives
ros_reference/run.sh --check  # fails when that differs from the committed files
```

The script builds the reference image for the architecture of the host, and runs the
reference program on each scenario in it. It runs the image by the id that its build
gives, so each checkout runs the image of its own files, and `docker image prune` removes
the images of earlier builds. On a host without the base image, the build fetches about
434 MB on amd64 (the base image, 341 MB; the apt indexes, 54.5 MB; and 37.9 MB of
packages) and about 418 MB on arm64 (326 MB, 55.0 MB and 37.2 MB), and the image takes
about 1.9 GB of disk on each.
The reference outputs and their checks run on the developer's machine; the Tests
workflow runs the parity tests on the committed outputs.

The host runs bash and a container engine as the `docker` command: Docker, rootful or
rootless, with buildx (its default builder or one of the docker-container driver) or with
its legacy builder; or Podman, rootful or rootless, which names itself in
`docker --version`, as the podman-docker package installs it. The container writes its
outputs as the user who runs the script. Both mounts carry `:z`, so on a host with SELinux
the host relabels the whole checkout for containers to share, and the checkout must be
one that the user lets containers read. On arm64, the host runs cargo too.

On an amd64 host, `run.sh` writes `reference/`, with its `SHA256SUMS` and
`INPUTS.SHA256SUMS`, and `packages-amd64.txt`, and `run.sh --check` fails when a file
differs from what the image gives, and prints each differing line. The reference outputs
reproduce bit for bit on every amd64 host. Of the libraries that the reference program
loads, libc and libm alone hold IFUNCs, the functions that glibc resolves by CPU feature,
and the selectors of glibc 2.43's math functions read FMA, FMA4, AVX, AVX2 and SSE4_1
(libm) and SSE4_1, FMA, AVX2, AVX512F and AVX512DQ (libmvec). `run.sh` runs the reference
program with `GLIBC_TUNABLES=glibc.cpu.hwcaps=` set to mask all of them, with SSSE3,
SSE4_2, POPCNT, BMI1, BMI2, LZCNT, MOVBE and AVX512BW, CD, VL, ER and PF, so glibc runs
the SSE2 variants of the math functions, of baseline instructions alone, on every amd64
host. glibc 2.43 also masks CMOV, CX8, SSE2, HTT, IBT, SHSTK, RTM, ERMS, XSAVE, XSAVEC
and OSXSAVE by name; no math selector reads them, and the mask leaves them.

On an arm64 host, `run.sh` writes `packages-arm64.txt`, and `run.sh --check` fails when it
differs from what the image gives. After the write or the check, both run each parity
test, `tests/ros_reference_*.rs` of each crate, on the outputs of the arm64 image, which
the environment variable `ROS_REFERENCE_OUTPUTS` names to the test, and fail when a test
does. The committed reference outputs are those of the amd64 image, and tests compare bits
on amd64 alone.

## Rules

1. A reference runs in the reference program's process. A reference with a plain C++ API
   is called directly. A reference that reads its parameters from a ROS node creates that
   node in the reference program's process, sets the scenario's parameters on it, and
   calls the reference on the program's own thread.
2. A reference gives the same output on every run: the container has no network and one
   OpenMP thread, every random generator of a reference is seeded from its scenario, and
   a search ends on an iteration bound that the scenario gives.
3. A reference that is statistical by nature (a sampling planner, a particle filter) runs
   over the seeds that its scenario lists, and its committed output holds the statistic,
   with its interval. Like every output, it changes only when the image changes.
4. A parity test states what the library guarantees, and the bound within which its
   outputs agree with the reference, or the statistic it compares, and why. A test that
   compares the exact bits of an output runs under `#[cfg(target_arch = "x86_64")]`,
   since the committed outputs hold amd64 bits.
5. One rule lays out every scenario and every reference output: a value goes on one line
   when it is a scalar, an array of scalars, or a record, an object whose members are
   each a scalar or an array of scalars; every other array or object spreads over lines,
   one member a line, indented by two spaces. So a forward kinematics scenario holds one
   configuration a line, and its output one link pose a line. The reference program
   writes every output by it through `write_json`, and every scenario tool writes every
   scenario by it through `write` of `tools/drawing.py`.
6. A kind commits only the few cases that exercise its behaviors: for forward
   kinematics, eight configurations a chain, all joints at zero, at their lower limits,
   at their upper limits, and five drawn inside them. A large drawn set runs locally and
   is never committed: draw it into `scenarios/` (for forward kinematics,
   `python3 ros_reference/tools/draw_forward_kinematics.py --drawn 500`), run
   `ros_reference/run.sh` and the parity tests, and restore the committed files with
   `git restore ros_reference/scenarios ros_reference/reference`.

## Pins

Every input of the image is pinned, so a rebuild gives the same packages:

- the base image `docker.io/library/ros:lyrical-ros-base`, by the digest of its index of
  amd64 and arm64;
- the ROS packages, by a dated snapshot of the ROS repository (`ROS_SNAPSHOT`), signed by
  the key in `ros-snapshots.asc`, which is valid until 2027-06-01;
- the Ubuntu packages, by a dated snapshot of the primary Ubuntu archive
  (`UBUNTU_SNAPSHOT`), which holds amd64 and arm64. The Dockerfile names the primary
  archive in place of the ports archive that the arm64 base image names.

The Ubuntu snapshot is 2026-10-27T00:00:00Z, and the base image was built at
2026-10-27T00:32:46Z for amd64 and at 2026-10-27T00:32:38Z for arm64. On both
architectures, the two snapshots hold the version of every package of the base image, and
apt keeps those versions.

`packages-amd64.txt` and `packages-arm64.txt` record the version of each package that
results. To move a pin, change the Dockerfile, run `run.sh` on an amd64 host and on an
arm64 host, and review the diff of the package lists and of `reference/` in the same pull
request: a reference output that changes is a change of the reference.

### Renewing the ROS snapshot key

`ros-snapshots.asc` is valid until 2027-06-01. To renew it, fetch the key from the
Ubuntu key server and print it:

```sh
(
  export GNUPGHOME=$(mktemp -d)
  gpg --keyserver hkps://keyserver.ubuntu.com --recv-keys 4B63CF8FDE49746E98FA01DDAD19BAB3CBF125EA
  gpg --armor --export 4B63CF8FDE49746E98FA01DDAD19BAB3CBF125EA > ros_reference/ros-snapshots.asc
  gpg --show-keys ros_reference/ros-snapshots.asc
  rm -rf "$GNUPGHOME"
)
```

The fingerprint that the last command prints must be
`4B63 CF8F DE49 746E 98FA 01DD AD19 BAB3 CBF1 25EA`, the key that
<https://wiki.ros.org/SnapshotRepository> names. Write the expiry date that it prints in
this file and in the Dockerfile, and run `run.sh --check` on an amd64 host and on an
arm64 host.

## Adding a scenario

1. Write the scenario under `scenarios/`. For forward kinematics, add the robot to
   `ROBOTS` in `tools/draw_forward_kinematics.py` and run
   `python3 ros_reference/tools/draw_forward_kinematics.py`, which writes every scenario
   of the kind, each drawn from a generator seeded by its own name, so the others come
   out unchanged.
2. Run `ros_reference/run.sh` on an amd64 host, which writes its reference output and the
   digests, and `ros_reference/run.sh --check` on an arm64 host, which runs the parity
   tests on the outputs of the arm64 image.
3. The parity test of its kind lists the scenarios of the kind from `scenarios/`, so it
   runs on the new one. The forward kinematics parity test poses chains of 1 to 7 joints.

A routine test of `ros_reference` fails when a scenario has no reference output, a
reference output has no scenario or no digest, or a scenario or a URDF differs from its
digest.

## Adding a kind of scenario

1. In `src/`, add the kind to `Kind` in `lib.rs`, and a module with a `kind` tag type of
   its own, which its scenario and its reference output hold, so each refuses another
   kind. The module lists the scenarios of the kind with `scenario_names`, reads each with
   `read_scenario` and its reference output with `read_reference`, and every other file
   of the repository that the reference reads with `read_input`, each checked against its
   digest, and pairs them for the parity test.
2. In `reference_programs/`, add the function that runs the reference on a scenario of
   the kind, reading every file of the repository through `Inputs::read`, which lists it
   for `INPUTS.SHA256SUMS`. Add the function to `references` in `main.cpp` and to
   `CMakeLists.txt`, and install the ROS packages it needs in the Dockerfile.
3. In `tools/`, add a script that writes the scenarios through `write` of `drawing.py`,
   its random draws from the `generator` of each scenario, seeded by the scenario's name.
4. Run `run.sh` on an amd64 host, which writes their reference outputs and
   `packages-amd64.txt`, and on an arm64 host, which writes `packages-arm64.txt`.
5. In the crate under test, add `ros_reference` as a dev-dependency and a parity test,
   `tests/ros_reference_<kind>.rs`, that loads every scenario of the kind with its
   reference output and compares, by the rules above. Add the kind to the table at the top
   of this file.

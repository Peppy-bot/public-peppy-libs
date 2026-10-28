# The reference image of the ROS reference harness: ROS 2 Lyrical with the packages that
# the reference programs use, moveit_core of MoveIt 2 with srdfdom and urdf, and the
# reference program built against them, for amd64 and arm64. Developers run it through
# run.sh, and the parity tests read the outputs that it wrote.
#
# Every input is pinned, so a rebuild gives the same packages: the base image by the
# digest of its index of both architectures, the ROS packages by a dated snapshot of the
# ROS repository, and the Ubuntu packages by a dated snapshot of the primary Ubuntu
# archive, which holds both architectures. The two snapshots hold the version of every
# package of the base image (built 2026-10-27T00:32:46Z), and apt keeps those versions.
# ros-snapshots.asc is the key that signs the ROS snapshots (fingerprint 4B63 CF8F DE49
# 746E 98FA 01DD AD19 BAB3 CBF1 25EA, valid until 2027-06-01); the README says how to
# renew it.
FROM docker.io/library/ros:lyrical-ros-base@sha256:3224ee150cb02eb89a5e0811bc267b57ed3a91b9c10191df57810726a7e916d7

ARG ROS_SNAPSHOT=2026-10-05
ARG UBUNTU_SNAPSHOT=20261027T000000Z

# The Ubuntu snapshots serve the primary archive, which holds arm64 too, so the sources of
# the arm64 base image name the primary archive in place of the ports archive. The update
# fails when it cannot fetch an index (--error-on=any), so an outage of a snapshot server
# fails the build there, with apt's fetch error.
COPY ros-snapshots.asc /usr/share/keyrings/ros-snapshots.asc
RUN rm /etc/apt/sources.list.d/ros2.sources \
 && sed -i 's|http://ports.ubuntu.com/ubuntu-ports/|http://archive.ubuntu.com/ubuntu/|' \
      /etc/apt/sources.list.d/ubuntu.sources \
 && printf 'Types: deb\nURIs: http://snapshots.ros.org/lyrical/%s/ubuntu\nSuites: resolute\nComponents: main\nSigned-By: /usr/share/keyrings/ros-snapshots.asc\n' \
      "$ROS_SNAPSHOT" > /etc/apt/sources.list.d/ros2-snapshot.sources \
 && apt-get update --error-on=any --snapshot "$UBUNTU_SNAPSHOT" \
 && apt-get install --yes --no-install-recommends --snapshot "$UBUNTU_SNAPSHOT" \
      build-essential \
      cmake \
      nlohmann-json3-dev \
      ros-lyrical-moveit-core \
      ros-lyrical-srdfdom \
      ros-lyrical-urdf \
 && rm -rf /var/lib/apt/lists/*

COPY CMakeLists.txt /src/
COPY reference_programs /src/reference_programs
RUN . /opt/ros/lyrical/setup.sh \
 && cmake -S /src -B /tmp/build -DCMAKE_BUILD_TYPE=Release \
 && cmake --build /tmp/build --parallel 4 \
 && cmake --install /tmp/build --prefix /usr/local \
 && rm -rf /tmp/build

# The reference program loads the ROS libraries through the environment that
# setup.sh exports.
ENTRYPOINT ["/ros_entrypoint.sh"]

#!/bin/sh
# REVIEW ONLY: executed solely inside newly approved private namespace.
set -eu
mountpoint=$1
seccomp_fd=$2
quota=$3
probe=$4
host_net_inode=$6
host_ipc_inode=$7
arg=${5:---reviewed-initialization-diagnostic}
[ "$quota" = 1073741824 ] || exit 91
mount --make-rprivate /
mount -t tmpfs -o "size=$quota,nr_inodes=32768,nosuid,nodev,noexec" tmpfs "$mountpoint"
mkdir -p "$mountpoint"/tmp "$mountpoint"/run "$mountpoint"/dev/shm "$mountpoint"/home "$mountpoint"/cache "$mountpoint"/roslog "$mountpoint"/empty
ip link set lo up
exec bwrap --unshare-pid --as-pid-1 \
 --ro-bind /usr /usr --ro-bind /lib /lib --ro-bind /lib64 /lib64 \
 --dir /etc --ro-bind /etc/ld.so.cache /etc/ld.so.cache --dir /etc/alternatives \
 --ro-bind /usr/lib/x86_64-linux-gnu/blas/libblas.so.3.10.0 /etc/alternatives/libblas.so.3-x86_64-linux-gnu \
 --ro-bind /usr/lib/x86_64-linux-gnu/lapack/liblapack.so.3.10.0 /etc/alternatives/liblapack.so.3-x86_64-linux-gnu \
 --ro-bind /home/robot/experiment_data/a2b_portable_delivery_20261109_v2 /isolated_bundle \
 --ro-bind /home/robot/experiment_data/a2b_closeout_candidate_20261110_r0/portable/supplement /supplement \
 --ro-bind /home/robot/experiment_data/a2b_closeout_candidate_20261110_r0/portable/src /review \
 --bind "$mountpoint" /mnt --bind "$mountpoint/tmp" /tmp --bind "$mountpoint/tmp" /var/tmp \
 --bind "$mountpoint/run" /run --bind "$mountpoint/dev" /dev --bind "$mountpoint/home" /home \
 --dev-bind /dev/null /dev/null --dev-bind /dev/zero /dev/zero \
 --dev-bind /dev/urandom /dev/urandom --dev-bind /dev/random /dev/random \
 --ro-bind "$mountpoint/empty" /readonly_audit \
 --proc /proc --ro-bind "$mountpoint/empty" /proc/sys --ro-bind /proc/sysrq-trigger /proc/sysrq-trigger \
 --remount-ro / --cap-drop ALL --new-session --die-with-parent --clearenv \
 --setenv HOST_NET_NS_INODE "$host_net_inode" --setenv HOST_IPC_NS_INODE "$host_ipc_inode" --setenv PATH /usr/bin:/bin --setenv HOME /home --setenv TMPDIR /tmp --setenv ROS_LOG_DIR /mnt/roslog \
 --setenv ROS_HOME /home --setenv XDG_CACHE_HOME /mnt/cache --setenv PYTHONDONTWRITEBYTECODE 1 \
 --setenv PYTHONNOUSERSITE 1 --setenv OPENBLAS_NUM_THREADS 1 --setenv OMP_NUM_THREADS 1 --setenv MKL_NUM_THREADS 1 \
 --setenv LD_LIBRARY_PATH /supplement/lib:/isolated_bundle/ros/lib --setenv AMENT_PREFIX_PATH /supplement:/isolated_bundle/ros \
 --setenv ROS_DOMAIN_ID 229 --setenv ROS_LOCALHOST_ONLY 0 \
 --setenv FASTRTPS_DEFAULT_PROFILES_FILE /isolated_bundle/runtime/resources/semantic/config/fastdds_large_map.xml \
 --chdir /mnt --seccomp "$seccomp_fd" \
 /usr/bin/python3 -B -I -S -X pycache_prefix=/tmp/bytecode_disabled "$probe" "$arg"

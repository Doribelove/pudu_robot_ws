#!/usr/bin/env bash
# Local-validation only: invoked exclusively on disposable CI runners.
set -euo pipefail
[[ ${TRILAYER_DEB_VALIDATION_REVIEWED:-} == yes && ${GITHUB_ACTIONS:-} == true && ${RUNNER_OS:-} == Linux ]] || { echo 'Requires reviewed disposable GitHub Linux runner'; exit 2; }
source /etc/os-release
[[ $ID == ubuntu && $VERSION_ID == 22.04 ]] || exit 2
case "$(uname -m)" in x86_64|aarch64) ;; *) exit 2 ;; esac
src=$(realpath "${1:?source checkout root}")
out=$(realpath -m "${2:?NEW output directory outside source checkout}")
[[ ! -e $out && $out != "$src"/* ]] || { echo 'Refuse existing output or source modification'; exit 2; }
mkdir -p "$out/evidence" "$out/build" "$out/debs"
script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
exec > >(tee "$out/evidence/driver.log") 2>&1
unset PYTHONPATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH LD_LIBRARY_PATH DEB_BUILD_PROFILES
export PATH=/usr/bin:/bin PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1
export DEB_BUILD_OPTIONS=parallel=2 CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2
export CTEST_PARALLEL_LEVEL=1
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=178
export TMPDIR=$(mktemp -d /tmp/tri-deb-XXXXXXXX)
stage=bootstrap
collect_exit() {
 rc=$?; trap - EXIT; set +e
 printf 'exit_code=%s\nstage=%s\ntemporary_evidence=%s\n' "$rc" "$stage" "$TMPDIR" > "$out/evidence/EXIT_STATUS.txt"
 python3 "$script_dir/collect_evidence.py" "$out" "$TMPDIR" > "$out/evidence/collection.log" 2>&1
 dpkg-query -W > "$out/evidence/runner-packages-exit.tsv" 2>&1
 echo "Validation exit=$rc stage=$stage; upload artifact path: $out/evidence (always, including failure)."
 exit "$rc"
}
trap collect_exit EXIT
# Runner must already have official signed ROS2 Humble apt source configured.
# This draft never installs keys with apt-key or disables signature verification.
sudo apt-get update
sudo apt-get install -y --no-install-recommends python3-bloom python3-rosdep python3-yaml python3-catkin-pkg debhelper dh-python dpkg-dev fakeroot build-essential cmake git
python3 "$script_dir/prepare_graph.py" "$src" "$out/evidence"
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then sudo rosdep init; fi
# Only this disposable runner is changed; these are transparent local CI mappings.
printf 'yaml file://%s/evidence/CI_ONLY_LOCAL_ROSDEP.yaml\n' "$out" | sudo tee /etc/ros/rosdep/sources.list.d/10-trilayer-local-deb.list
rosdep update --rosdistro humble
rosdep install --from-paths "$src/src" --ignore-src --rosdistro humble -y
rosdep check --from-paths "$src/src" --ignore-src --rosdistro humble
set +u; source /opt/ros/humble/setup.bash; set -u
# The single-package Bloom CLI cannot infer unpublished workspace peers.
# Above explicit local keys are needed for generated control dependencies.
dpkg-query -W > "$out/evidence/runner-packages-before.tsv"
bloom-generate rosdebian --help > "$out/evidence/bloom-help.txt"
solver_root=''
readonly deb_revision=0localvalidation2
while IFS= read -r name; do
 stage="build:$name"
 debname="ros-humble-${name//_/-}"
 if dpkg-query -W -f='${Status}' "$debname" 2>/dev/null | grep -q 'install ok installed'; then
  echo "Refuse testing own package against an old installed copy: $debname"; exit 1
 fi
 parent="$out/build/$name"; mkdir -p "$parent"; cp -a "$src/src/$name" "$parent/source"; pkg="$parent/source"
 (
  cd "$pkg"
  bloom-generate rosdebian --os-name ubuntu --os-version jammy --ros-distro humble -i "$deb_revision"
  version=$(dpkg-parsechangelog -S Version)
  dpkg --validate-version "$version"
  printf '%s\n' "$version" > "$out/evidence/$name.deb-version.txt"
  [[ $version == *-"${deb_revision}jammy" ]] || { echo "Unexpected Bloom version $version"; exit 1; }
  patch_args=()
  if [[ -n $solver_root ]]; then patch_args=(--solver-root "$solver_root"); fi
  python3 "$script_dir/patch_rules.py" "$pkg" "$name" "${patch_args[@]}"
  cp debian/control "$out/evidence/$name.control"
  cp debian/LOCAL_VALIDATION_RULES.patch "$out/evidence/$name.rules.patch"
  dpkg-checkbuilddeps
  # No nocheck, no -d, no skipped tests. Bloom's swallowed-test result was removed.
  # Binary-only local validation: this is not a bloom release repository or .dsc release.
  dpkg-buildpackage -us -uc -b -rfakeroot
 ) 2>&1 | tee "$out/evidence/$name.build.log"
 mapfile -t candidates < <(find "$parent" -maxdepth 1 -name '*.deb' -print)
 selected=''
 for f in "${candidates[@]}"; do
  if [[ $(dpkg-deb -f "$f" Package) == "$debname" ]]; then selected=$f; fi
 done
 [[ -n $selected ]] || { echo "No expected binary $debname"; exit 1; }
 cp "$selected" "$out/debs/"
 dpkg-deb --info "$selected" > "$out/evidence/$name.deb-info.txt"
 dpkg-deb --contents "$selected" > "$out/evidence/$name.deb-contents.txt"
 sudo apt-get install -y "$selected"
 set +u; source /opt/ros/humble/setup.bash; set -u
 # Derive path from the Debian payload; do not assume colcon's local/lib layout.
 if dpkg-query -W -f='${Status}' ros-humble-trilayer-osqp-vendor 2>/dev/null | grep -q 'install ok installed'; then
  init=$(dpkg-query -L ros-humble-trilayer-osqp-vendor | grep '/osqp/__init__.py$')
  solver_root=$(dirname "$(dirname "$init")")
  [[ $solver_root == /opt/ros/humble/* ]] || exit 1
 fi
done < "$out/evidence/order.txt"
sha256sum "$out"/debs/*.deb > "$out/evidence/deb-SHA256SUMS"
dpkg-query -W > "$out/evidence/runner-packages-after.tsv"
stage=postinstall-smoke
bash "$script_dir/postinstall-smoke-draft.sh" "$src" "$out" "$solver_root"
stage=complete
echo 'LOCAL DEB VALIDATION COMPLETE ONLY; not official ROS apt publication'

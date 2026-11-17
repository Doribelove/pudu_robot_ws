#!/usr/bin/env bash
# REVIEW DRAFT ONLY. Execute only after reviewed local Debian build/install stage.
set -euo pipefail
[[ ${TRILAYER_DEB_VALIDATION_REVIEWED:-} == yes && ${GITHUB_ACTIONS:-} == true ]] || exit 2
src=$(realpath "${1:?source root}");out=$(realpath "${2:?artifact root}");solver_root=${3:?installed solver directory}
[[ $solver_root == /opt/ros/humble/* && -d $solver_root ]] || exit 2
unset PYTHONPATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH LD_LIBRARY_PATH TRILAYER_TEST_PREFIX
export PATH=/usr/bin:/bin PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1
export ROS_LOCALHOST_ONLY=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
set +u; source /opt/ros/humble/setup.bash; set -u
export PYTHONPATH="$solver_root${PYTHONPATH:+:$PYTHONPATH}"
mkdir -p "$out/smoke-scripts" "$out/evidence/smoke"
cp "$src/src/trilayer_p6_runtime/test/test_service_request.py" "$src/src/trilayer_p6_runtime/test/test_public_session.py" "$out/smoke-scripts/"
cp "$src/src/trilayer_nav2_planner/test/run_real_worker_smoke.py" "$src/src/trilayer_nav2_planner/test/planner_server_smoke.py" "$out/smoke-scripts/"
cd "$out/smoke-scripts"
python3 - "$out/evidence/smoke/import-ownership.json" <<'PY'
import sys,pathlib,json,subprocess
import osqp,qdldl,clarabel,trilayer_p6_runtime,trilayer_planner_service,trilayer_l3_core
mods=[osqp,qdldl,clarabel,trilayer_p6_runtime,trilayer_planner_service,trilayer_l3_core];rows=[]
for m in mods:
 p=pathlib.Path(m.__file__).resolve();assert p.is_relative_to('/opt/ros/humble'),(m.__name__,p)
 owner=subprocess.check_output(['dpkg-query','-S',str(p)],text=True).strip()
 assert 'ros-humble-trilayer-' in owner,(p,owner)
 rows.append({'module':m.__name__,'path':str(p),'owner':owner})
pathlib.Path(sys.argv[1]).write_text(json.dumps(rows,indent=2)+'\n')
PY
for case in straight obstacle; do
 ROS_DOMAIN_ID=179 timeout --signal=INT --kill-after=20s 90s python3 test_service_request.py \
   --case "$case" --solver-root "$solver_root" > "$out/evidence/smoke/p6-$case.log" 2>&1
done
for mode in map semantic; do
 ROS_DOMAIN_ID=179 timeout --signal=INT --kill-after=20s 90s python3 test_public_session.py \
   "$solver_root" "$mode" > "$out/evidence/smoke/p6-session-$mode.log" 2>&1
done
ROS_DOMAIN_ID=185 timeout --signal=INT --kill-after=20s 240s python3 run_real_worker_smoke.py \
 "$solver_root" > "$out/evidence/smoke/nav2-real-worker.log" 2>&1
# Success means installed local .deb code handled actual synthetic P6/Nav2 requests.
# It does not mean physical-robot/J6M validation or official apt publication.
printf 'PASS installed local-deb P6/Nav2 smoke\n' > "$out/evidence/smoke/RESULT.txt"

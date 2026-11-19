#!/usr/bin/env bash
# Run explicitly; preparing this file does not start ROS.
# Usage: bash scripts/validation/legacy_regression.sh [project-root] [new-output] [all|baseline|aws_house]
# Stops on a failed stage, preserves its evidence, and cleans only owned children.
set -eo pipefail
set +m
legacy_project="${1:-/home/willmatic/桌面/semantic_object_search}"
cd -- "$legacy_project"
legacy_project="$PWD"
legacy_output="${2:-$legacy_project/results/development/legacy_20261117}"
legacy_selection="${3:-all}"
case "$legacy_selection" in
    all) legacy_specs=(baseline:67:2 aws_house:68:4) ;;
    baseline) legacy_specs=(baseline:67:2) ;;
    aws_house) legacy_specs=(aws_house:68:4) ;;
    *) echo 'Expected all, baseline or aws_house' >&2; exit 2 ;;
esac
legacy_script="$(readlink -f -- "${BASH_SOURCE[0]}")"
for legacy_spec in "${legacy_specs[@]}"; do
    legacy_scene="${legacy_spec%%:*}"
    [[ ! -e "$legacy_output/$legacy_scene" ]] || {
        echo "Refusing to overwrite evidence: $legacy_output/$legacy_scene" >&2; exit 2;
    }
done
command -v setsid >/dev/null
command -v timeout >/dev/null
python3 - <<'PY'
import os, signal
if not hasattr(os, 'pidfd_open') or not hasattr(signal, 'pidfd_send_signal'):
    raise SystemExit('Linux pidfd support is required for identity-safe cleanup')
PY

cleanup_legacy() {
    local legacy_original_status="$1" legacy_cleanup_status
    trap - EXIT
    trap '' INT TERM
    set +e
    # The token exists BEFORE any child is spawned. It is supplied only to owned
    # children, not exported in this driver. Detached descendants retain it too.
    python3 - "$legacy_token" "$legacy_run/shutdown.json" "$legacy_original_status" <<'PY'
import json, os
from pathlib import Path
import signal, sys, time
token, receipt, exit_code = sys.argv[1:]
marker = b'SEMANTIC_LEGACY_OWNER_TOKEN=' + token.encode()

def info(pid):
    path = Path('/proc')/str(pid)
    try:
        if path.stat().st_uid != os.getuid(): return None
        environment = path.joinpath('environ').read_bytes().split(b'\0')
        if marker not in environment: return None
        fields = path.joinpath('stat').read_text().rsplit(') ', 1)[1].split()
        if fields[0] == 'Z': return None
        role = next((v.split(b'=',1)[1].decode() for v in environment
                     if v.startswith(b'SEMANTIC_LEGACY_ROLE=')), 'other')
        return dict(pid=int(pid), pgid=int(fields[2]), sid=int(fields[3]),
                    start_ticks=int(fields[19]), role=role)
    except (FileNotFoundError, ProcessLookupError, PermissionError): return None

def live():
    return [row for path in Path('/proc').iterdir() if path.name.isdigit()
            if (row := info(path.name)) is not None]

actions, errors = [], []
# Let clients cancel while search/Nav2 still exist; simulation is stopped last.
for phase in ['clients', 'search', 'navigation', 'simulation']:
    def members():
        return [row for row in live() if (row['role'] not in ['search','navigation','simulation']
                if phase == 'clients' else row['role'] == phase)]
    for sig, grace in [(signal.SIGINT, 7.), (signal.SIGTERM, 5.), (signal.SIGKILL, 2.)]:
        rows = members()
        if not rows: break
        for row in rows:
            descriptor = None
            try:
                descriptor = os.pidfd_open(row['pid'])
                current = info(row['pid'])
                if current is not None and current['start_ticks'] == row['start_ticks']:
                    # pidfd pins the exact process even if its numeric PID is reused.
                    # No kill(-PGID), process-name matching, or domain-wide kill.
                    signal.pidfd_send_signal(descriptor, sig)
                    actions.append(dict(process=row, signal=sig.name))
            except ProcessLookupError: pass
            except OSError as error: errors.append(dict(process=row, error=str(error)))
            finally:
                if descriptor is not None: os.close(descriptor)
        deadline = time.monotonic()+grace
        while members() and time.monotonic() < deadline: time.sleep(.2)
remaining = live()
time.sleep(.2)
remaining += live()
Path(receipt).write_text(json.dumps(dict(original_exit_code=int(exit_code),
    method='Owned setsid sessions + inherited random token + verified pidfd signals',
    actions=actions, errors=errors, remaining=remaining, clean=not remaining and not errors), indent=2)+'\n')
raise SystemExit(0 if not remaining and not errors else 1)
PY
    legacy_cleanup_status="$?"
    if (( legacy_cleanup_status != 0 )); then
        echo "Cleanup could not be verified; stopping: $legacy_run/shutdown.json" >&2
        exit 90
    fi
    for legacy_pid in "${legacy_pids[@]}"; do wait "$legacy_pid" 2>/dev/null; done
    set -e
    return "$legacy_original_status"
}

spawn_owned() {
    local legacy_role="$1" legacy_log="$2"
    shift 2
    printf '%q ' "$@" >> "$legacy_run/commands.log"
    printf '\n' >> "$legacy_run/commands.log"
    SEMANTIC_LEGACY_OWNER_TOKEN="$legacy_token" SEMANTIC_LEGACY_ROLE="$legacy_role" \
        setsid --wait "$@" >"$legacy_log" 2>"$legacy_log.stderr.log" &
    legacy_last_pid="$!"
    legacy_pids+=("$legacy_last_pid")
}

run_owned() {
    spawn_owned "$@"
    wait "$legacy_last_pid"
}

record_sources() {
    python3 - "$legacy_project" "$legacy_scene" "$1" "$legacy_script" <<'PY'
import hashlib, json, os, sys
from pathlib import Path
import yaml
root, scene, output, launcher = sys.argv[1:]
root = Path(root)
config = json.loads((root/'src/semantic_search/config/scenes.json').read_text())['scenes'][scene]
map_yaml = root/config['map']
map_image = map_yaml.parent/yaml.safe_load(map_yaml.read_text())['image']
paths = [map_yaml, map_image, root/config['rooms'], root/'src/semantic_search/config/scenes.json',
         root/'src/semantic_search/config/nav2.yaml', root/'src/semantic_search/config/tb3_frames.urdf',
         root/'yolov8n-seg.pt', Path(launcher)]
paths += [root/'scripts'/name for name in ['env.sh','scene_env.sh','simulation.sh','navigation.sh',
          'search.sh','initialize_sim_amcl.py','validate_sensors.py','run_search.py','run_search_matrix.py']]
paths += [root/'src/semantic_search/launch'/name for name in ['fortress.launch.py','search.launch.py']]
paths += [root/'src/semantic_search/semantic_search'/name for name in ['search_node.py','rooms.py',
          'detector.py','sensor_qos.py','semantic_map.py','depth.py','scenes.py','aws_world.py']]
files = [dict(path=str(path.resolve()), sha256=hashlib.sha256(path.read_bytes()).hexdigest()) for path in paths]
Path(output).write_text(json.dumps(dict(scene=scene, files=files,
    environment={key:os.environ.get(key) for key in ['ROS_DOMAIN_ID','ROS_LOCALHOST_ONLY','IGN_PARTITION']},
    boundary='Original fixed chair fixtures, one find/count each; not five repeats or G0'), indent=2)+'\n')
PY
}

mkdir -p -- "$legacy_output"
for legacy_spec in "${legacy_specs[@]}"; do
    IFS=: read -r legacy_scene legacy_domain legacy_expected <<< "$legacy_spec"
    legacy_run="$legacy_output/$legacy_scene"
    mkdir -- "$legacy_run"
    mkdir -- "$legacy_run/parameters"
    cp -- "$legacy_script" "$legacy_run/launcher.sh"
    legacy_token="$(python3 -c 'import secrets; print(secrets.token_hex(24))')"
    legacy_pids=()
    trap 'legacy_exit=$?; cleanup_legacy "$legacy_exit"; exit "$legacy_exit"' EXIT
    trap 'exit 130' INT
    trap 'exit 143' TERM
    export SEMANTIC_ROS_DOMAIN_ID="$legacy_domain" SEMANTIC_USE_SYSTEM_DDS=0
    export IGN_PARTITION="semantic_legacy_20261117_${legacy_scene}_${legacy_domain}_$$"
    source scripts/env.sh
    [[ "$ROS_DOMAIN_ID" == "$legacy_domain" && "$ROS_DOMAIN_ID" != 66 ]]
    record_sources "$legacy_run/sources_before.json"
    run_owned preflight "$legacy_run/domain_preflight.log" \
        timeout --signal=INT --kill-after=5s 15 ros2 node list --no-daemon --spin-time 2
    if [[ -s "$legacy_run/domain_preflight.log" ]]; then
        echo "Domain $legacy_domain already has nodes; refusing to start or stop them." >&2
        exit 3
    fi

    spawn_owned simulation "$legacy_run/simulation.log" \
        bash scripts/simulation.sh scene:="$legacy_scene" gui:=false
    run_owned clock "$legacy_run/clock.txt" \
        timeout --signal=INT --kill-after=5s 90 ros2 topic echo /clock rosgraph_msgs/msg/Clock --once
    run_owned sensors "$legacy_run/sensors.log" \
        timeout --signal=INT --kill-after=5s 45 python3 scripts/validate_sensors.py \
        --seconds 10 --output "$legacy_run/sensors"

    # Original map/nav2.yaml; stage activation without waiting for /rgbd/obstacles.
    spawn_owned navigation "$legacy_run/navigation.log" \
        bash scripts/navigation.sh scene:="$legacy_scene" autostart:=false
    run_owned localization_start "$legacy_run/localization_start.log" \
        timeout --signal=INT --kill-after=5s 90 ros2 service call \
        /lifecycle_manager_localization/manage_nodes nav2_msgs/srv/ManageLifecycleNodes '{command: 0}'
    spawn_owned navigation_start "$legacy_run/navigation_start.log" \
        timeout --signal=INT --kill-after=5s 100 ros2 service call \
        /lifecycle_manager_navigation/manage_nodes nav2_msgs/srv/ManageLifecycleNodes '{command: 0}'
    legacy_navigation_start_pid="$legacy_last_pid"
    run_owned initialize "$legacy_run/initialization.log" \
        timeout --signal=INT --kill-after=5s 90 python3 scripts/initialize_sim_amcl.py
    wait "$legacy_navigation_start_pid"

    # search.sh launches the original CPU detector and scene-specific IoU too.
    spawn_owned search "$legacy_run/search.log" \
        bash scripts/search.sh scene:="$legacy_scene" output_dir:="$legacy_run/tasks"
    run_owned detector_ready "$legacy_run/detector_ready.txt" \
        timeout --signal=INT --kill-after=5s 120 ros2 topic echo /yolo/detections \
        yolo_msgs/msg/DetectionArray --once --field header
    for legacy_node in semantic_search course_detector planner_server controller_server map_server amcl bt_navigator; do
        run_owned parameters "$legacy_run/parameters/$legacy_node.yaml" \
            timeout --signal=INT --kill-after=5s 45 ros2 param dump "/$legacy_node" --no-daemon --spin-time 2
    done
    python3 - "$legacy_run/matrix.json" "$legacy_expected" <<'PY'
import json, sys
from pathlib import Path
Path(sys.argv[1]).write_text(json.dumps(dict(repeats=1, cases=[
    dict(id='find_chair', command='去厨房找椅子', evaluation=dict(status='found', count=1)),
    dict(id='count_chair', command='去厨房统计椅子', evaluation=dict(status='completed', count=int(sys.argv[2])))
]), ensure_ascii=False, indent=2)+'\n')
PY
    run_owned matrix "$legacy_run/matrix.log" \
        python3 scripts/run_search_matrix.py "$legacy_run/matrix.json" \
        --output "$legacy_run/matrix_results" --timeout 480 \
        --first-start cold --subsequent-start warm \
        --startup-note 'Fresh original simulator/Nav2/AMCL/search/detector; observed odometry initialization; no reset between tasks' \
        --config "src/semantic_search/config/rooms_${legacy_scene}.json" \
        --config src/semantic_search/config/nav2.yaml \
        --config src/semantic_search/config/scenes.json --weights yolov8n-seg.pt

    # finish() publishes before writing: allow bounded time for final artifacts.
    python3 - "$legacy_run" <<'PY'
import json, sys, time
from pathlib import Path
import xml.etree.ElementTree as ET
folder=Path(sys.argv[1]); report=json.loads((folder/'matrix_results/report.json').read_text())
deadline=time.monotonic()+10
while True:
    found={}
    try:
        for path in (folder/'tasks').glob('*/result.json'):
            result=json.loads(path.read_text())
            json.loads((path.parent/'observations.json').read_text())
            ET.parse(path.parent/'semantic_map.svg')
            found[result['task_id']]=result
        if all(found.get(row['result']['task_id']) == row['result'] for row in report['runs']): break
    except (OSError, ValueError, ET.ParseError): pass
    if time.monotonic() >= deadline: raise SystemExit('Published task artifacts were not fully persisted')
    time.sleep(.2)
PY
    record_sources "$legacy_run/sources_after.json"
    cleanup_legacy 0
    trap - INT TERM
    echo "Finished $legacy_scene; cleanup verified; evidence: $legacy_run"
done
echo "Selected original-scene regressions passed: $legacy_output"

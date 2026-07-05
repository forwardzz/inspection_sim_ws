import os
import shlex
import signal

from PyQt5.QtCore import QObject, QProcess, pyqtSignal


class RemoteRobotManager(QObject):
    log_line = pyqtSignal(str)
    state_changed = pyqtSignal(str)
    rviz_state_changed = pyqtSignal(str)

    def __init__(
        self,
        host="192.168.43.21",
        user="yy",
        port="22",
        workspace="/home/yy/inspection_sim_ws",
        ros_setup="/opt/ros/jazzy/setup.bash",
        local_workspace="/home/zjy/inspection_sim_ws",
        ros_domain_id="0",
        ros_localhost_only="0",
    ):
        super().__init__()
        self.host = host
        self.user = user
        self.port = str(port)
        self.workspace = workspace
        self.ros_setup = ros_setup
        self.local_workspace = local_workspace
        self.ros_domain_id = str(ros_domain_id)
        self.ros_localhost_only = str(ros_localhost_only)
        self.active_name = "idle"
        self.active_tail = None
        self.rviz_process = None
        self.oneshot_processes = []

    def update_connection(
        self,
        host,
        user,
        port,
        workspace,
        ros_setup,
        local_workspace,
        ros_domain_id="0",
        ros_localhost_only="0",
    ):
        self.host = host
        self.user = user
        self.port = str(port)
        self.workspace = workspace
        self.ros_setup = ros_setup
        self.local_workspace = local_workspace
        self.ros_domain_id = str(ros_domain_id)
        self.ros_localhost_only = str(ros_localhost_only)

    def test_connection(self):
        script = (
            "set -u; "
            "printf '[REMOTE] host=%s user=%s home=%s\\n' \"$(hostname)\" \"$(whoami)\" \"$HOME\"; "
            f"test -d {shlex.quote(self.workspace)} && echo '[OK] workspace exists' || echo '[WARN] workspace missing'; "
            f"test -f {shlex.quote(self.ros_setup)} && echo '[OK] ROS setup exists' || echo '[WARN] ROS setup missing'; "
            "test -d /home/yy/ros2_ws && echo '[OK] /home/yy/ros2_ws still present' || true"
        )
        self.run_remote_once("test_connection", script)

    def sync_workspace(self):
        if not self.local_workspace:
            self.log_line.emit("[ERROR] Local workspace path is empty.")
            return
        source = self.local_workspace.rstrip("/") + "/"
        target = f"{self.user}@{self.host}:{self.workspace.rstrip('/')}/"
        args = [
            "-az",
            "--delete",
            "--exclude=.git/",
            "--exclude=build/",
            "--exclude=install/",
            "--exclude=log/",
            "--exclude=.ros/",
            "--exclude=__pycache__/",
            "--exclude=*.pyc",
            "-e",
            f"ssh -p {shlex.quote(self.port)} -o BatchMode=yes -o ConnectTimeout=8",
            source,
            target,
        ]
        proc = self._make_process("rsync", track_active=False)
        self.oneshot_processes.append(proc)
        self.log_line.emit(f"[RUN] rsync {source} -> {target}")
        proc.start("rsync", args)

    def build_workspace(self):
        command = "colcon build"
        self.run_remote_once("remote_build", self._workspace_command(command, source_install=False))

    def show_navigation_args(self):
        command = "ros2 launch inspection_sim_bringup navigation.launch.py --show-args"
        self.run_remote_once("navigation_args", self._workspace_command(command))

    def list_maps(self):
        maps_dir = os.path.join(self.workspace, "maps")
        command = (
            f"printf '[MAP] maps_dir=%s\\n' {shlex.quote(maps_dir)}; "
            f"find {shlex.quote(maps_dir)} -maxdepth 1 -type f -name '*.yaml' -print 2>/dev/null | sort"
        )
        self.run_remote_once("list_maps", self._workspace_command(command))

    def save_map(self, map_path):
        prefix = os.path.splitext(map_path)[0]
        maps_dir = os.path.dirname(map_path) or os.path.join(self.workspace, "maps")
        command = (
            f"mkdir -p {shlex.quote(maps_dir)} && "
            f"ros2 run nav2_map_server map_saver_cli -f {shlex.quote(prefix)}"
        )
        self.run_remote_once("save_map", self._workspace_command(command))

    def start_thermal(self):
        command = (
            "if ros2 pkg prefix inspection_sim_bringup >/dev/null 2>&1 "
            "&& [ -f \"$(ros2 pkg prefix inspection_sim_bringup)/share/inspection_sim_bringup/launch/sensor_monitor.launch.py\" ]; then "
            "ros2 launch inspection_sim_bringup sensor_monitor.launch.py; "
            "elif ros2 pkg prefix mapping_bringup >/dev/null 2>&1; then "
            "ros2 launch mapping_bringup sensor_monitor.launch.py; "
            "else "
            "echo '[WARN] sensor_monitor.launch.py is not available in this workspace'; "
            "exit 2; "
            "fi"
        )
        self.start_launch("thermal", command, start_local_rviz=False)

    def stop_thermal(self):
        script = """
pkill -TERM -f sensor_monitor.launch.py || true
pkill -TERM -f thermal_camera_node || true
pkill -TERM -f gas_sensor_node || true
echo '[REMOTE] thermal/gas monitor stop requested'
"""
        self.run_remote_once("stop_thermal", script)

    def check_topics(self):
        command = (
            "echo '[REMOTE] topic list'; "
            "ros2 topic list || true; "
            "echo '[REMOTE] active nodes'; "
            "ros2 node list || true"
        )
        self.run_remote_once("topic_check", self._workspace_command(command))

    def publish_stop(self):
        command = (
            "timeout 3 ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "
            "\"{linear: {x: 0.0}, angular: {z: 0.0}}\""
        )
        self.run_remote_once("cmd_vel_stop", self._workspace_command(command))

    def abort_mission(self):
        command = "ros2 service call /abort_mission std_srvs/srv/Trigger {}"
        self.run_remote_once("abort_mission", self._workspace_command(command))

    def status(self):
        script = f"""
set -u
state_dir="$HOME/.inspection_sim_remote"
pid_file="$state_dir/active.pid"
name_file="$state_dir/active.name"
printf '[REMOTE] target=%s@%s:%s\\n' {shlex.quote(self.user)} {shlex.quote(self.host)} {shlex.quote(self.workspace)}
if [ -s "$pid_file" ] && kill -0 "$(cat "$pid_file")" 2>/dev/null; then
  printf '[REMOTE] active=%s pid=%s\\n' "$(cat "$name_file" 2>/dev/null || echo unknown)" "$(cat "$pid_file")"
else
  echo '[REMOTE] active=idle'
fi
test -d {shlex.quote(self.workspace)} && echo '[OK] workspace exists' || echo '[WARN] workspace missing'
test -f {shlex.quote(os.path.join(self.workspace, "install", "setup.bash"))} && echo '[OK] install/setup.bash exists' || echo '[WARN] install/setup.bash missing'
"""
        self.run_remote_once("remote_status", script)

    def start_mapping(
        self,
        use_rviz=False,
        headless=True,
        use_sim_time=False,
        sensor_source="hardware",
        lidar_serial_port="",
        lidar_baudrate="115200",
        imu_serial_port="",
        start_local_rviz=True,
        local_rviz_config="",
    ):
        command = (
            "ros2 launch inspection_sim_bringup mapping.launch.py "
            f"use_rviz:={str(use_rviz).lower()} "
            f"headless:={str(headless).lower()} "
            f"use_sim_time:={str(use_sim_time).lower()} "
            f"sensor_source:={shlex.quote(sensor_source)} "
            f"serial_port:={shlex.quote(lidar_serial_port)} "
            f"serial_baudrate:={shlex.quote(lidar_baudrate)} "
            f"imu_serial_port:={shlex.quote(imu_serial_port)}"
        )
        self.start_launch(
            "mapping",
            command,
            start_local_rviz=start_local_rviz,
            local_rviz_config=local_rviz_config,
        )

    def start_navigation(
        self,
        map_path,
        use_rviz=False,
        headless=True,
        use_sim_time=False,
        sensor_source="hardware",
        lidar_serial_port="",
        lidar_baudrate="115200",
        imu_serial_port="",
        start_local_rviz=True,
        local_rviz_config="",
    ):
        command = (
            "ros2 launch inspection_sim_bringup navigation.launch.py "
            f"map:={shlex.quote(map_path)} "
            f"use_rviz:={str(use_rviz).lower()} "
            f"headless:={str(headless).lower()} "
            f"use_sim_time:={str(use_sim_time).lower()} "
            f"sensor_source:={shlex.quote(sensor_source)} "
            f"serial_port:={shlex.quote(lidar_serial_port)} "
            f"serial_baudrate:={shlex.quote(lidar_baudrate)} "
            f"imu_serial_port:={shlex.quote(imu_serial_port)}"
        )
        self.start_launch(
            "navigation",
            command,
            start_local_rviz=start_local_rviz,
            local_rviz_config=local_rviz_config,
        )

    def start_launch(self, name, command, start_local_rviz=False, local_rviz_config=""):
        if self.is_running():
            self.log_line.emit(f"[WARN] Stop {self.active_name} before starting {name}.")
            return

        wrapped = self._workspace_command(command)
        script = f"""
set -eu
state_dir="$HOME/.inspection_sim_remote"
mkdir -p "$state_dir"
pid_file="$state_dir/active.pid"
name_file="$state_dir/active.name"
log_file="$state_dir/active.log"
if [ -s "$pid_file" ] && kill -0 "$(cat "$pid_file")" 2>/dev/null; then
  printf '[WARN] remote launch already running: %s pid=%s\\n' "$(cat "$name_file" 2>/dev/null || echo unknown)" "$(cat "$pid_file")"
  exit 3
fi
rm -f "$pid_file"
: > "$log_file"
printf '%s\\n' {shlex.quote(name)} > "$name_file"
setsid bash -lc {shlex.quote(wrapped)} >> "$log_file" 2>&1 &
pid=$!
printf '%s\\n' "$pid" > "$pid_file"
printf '[REMOTE] started %s pid=%s\\n' {shlex.quote(name)} "$pid"
printf '[REMOTE] log %s\\n' "$log_file"
"""
        self.run_remote_once(
            f"start_{name}",
            script,
            after_success=lambda: self._after_remote_start(
                name,
                start_local_rviz,
                local_rviz_config,
            ),
        )

    def stop_launch(self):
        script = """
set -u
state_dir="$HOME/.inspection_sim_remote"
pid_file="$state_dir/active.pid"
name_file="$state_dir/active.name"
if [ -s "$pid_file" ] && kill -0 "$(cat "$pid_file")" 2>/dev/null; then
  pid="$(cat "$pid_file")"
  name="$(cat "$name_file" 2>/dev/null || echo unknown)"
  printf '[REMOTE] stopping %s pid=%s\\n' "$name" "$pid"
  kill -INT -- "-$pid" 2>/dev/null || kill -INT "$pid" 2>/dev/null || true
  for _ in 1 2 3 4 5 6 7 8 9 10; do
    kill -0 "$pid" 2>/dev/null || break
    sleep 0.3
  done
  if kill -0 "$pid" 2>/dev/null; then
    kill -TERM -- "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true
    sleep 1
  fi
  if kill -0 "$pid" 2>/dev/null; then
    kill -KILL -- "-$pid" 2>/dev/null || kill -KILL "$pid" 2>/dev/null || true
  fi
  rm -f "$pid_file" "$name_file"
  echo '[REMOTE] stopped'
else
  echo '[REMOTE] no active remote launch'
  rm -f "$pid_file" "$name_file"
fi
"""
        self.run_remote_once("stop_launch", script, after_success=self._after_remote_stop)

    def run_remote_once(self, name, script, after_success=None):
        proc = self._make_process(name, track_active=False, after_success=after_success)
        self.oneshot_processes.append(proc)
        self.log_line.emit(f"[RUN] ssh {self.user}@{self.host} {name}")
        proc.start("ssh", self._ssh_args(script))

    def is_running(self):
        return (
            self.active_tail is not None
            and self.active_tail.state() != QProcess.NotRunning
        ) or self.is_local_rviz_running()

    def shutdown(self):
        self._stop_tail()
        self.stop_local_rviz()
        for proc in list(self.oneshot_processes):
            if proc.state() != QProcess.NotRunning:
                proc.terminate()
                proc.waitForFinished(1000)
                if proc.state() != QProcess.NotRunning:
                    proc.kill()

    def _workspace_command(self, command, source_install=True):
        setup_path = os.path.join(self.workspace, "install", "setup.bash")
        setup_install = f"source {shlex.quote(setup_path)} && " if source_install else ""
        return (
            f"cd {shlex.quote(self.workspace)} && "
            f"export ROS_DOMAIN_ID={shlex.quote(self.ros_domain_id)} && "
            f"export ROS_LOCALHOST_ONLY={shlex.quote(self.ros_localhost_only)} && "
            f"source {shlex.quote(self.ros_setup)} && "
            f"{setup_install}"
            f"{command}"
        )

    def start_local_rviz(self, rviz_config):
        if not rviz_config:
            self.log_line.emit("[WARN] Local RViz config path is empty.")
            return False
        if not os.path.exists(rviz_config):
            self.log_line.emit(f"[WARN] Local RViz config does not exist: {rviz_config}")
            return False
        if self.is_local_rviz_running():
            self.log_line.emit("[RVIZ] local RViz is already running")
            return True

        local_setup = os.path.join(self.local_workspace, "install", "setup.bash")
        command = (
            f"export ROS_DOMAIN_ID={shlex.quote(self.ros_domain_id)} && "
            f"export ROS_LOCALHOST_ONLY={shlex.quote(self.ros_localhost_only)} && "
            f"source {shlex.quote(self.ros_setup)} && "
            f"source {shlex.quote(local_setup)} && "
            f"rviz2 -d {shlex.quote(rviz_config)}"
        )
        proc = self._make_process("local_rviz", track_active=False)
        self.rviz_process = proc
        self.rviz_state_changed.emit("running")
        self.log_line.emit(f"[RVIZ] start local RViz: {rviz_config}")
        proc.start("setsid", ["bash", "-lc", command])
        return True

    def stop_local_rviz(self):
        if self.rviz_process is None:
            self.rviz_state_changed.emit("idle")
            return
        if self.rviz_process.state() != QProcess.NotRunning:
            pid = int(self.rviz_process.processId())
            self.log_line.emit("[RVIZ] stop local RViz")
            try:
                os.killpg(pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            self.rviz_process.waitForFinished(1500)
            if self.rviz_process.state() != QProcess.NotRunning:
                try:
                    os.killpg(pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                self.rviz_process.waitForFinished(500)
        self.rviz_process = None
        self.rviz_state_changed.emit("idle")

    def is_local_rviz_running(self):
        return (
            self.rviz_process is not None
            and self.rviz_process.state() != QProcess.NotRunning
        )

    def _ssh_args(self, script):
        return [
            "-o",
            "BatchMode=yes",
            "-o",
            "ConnectTimeout=8",
            "-p",
            self.port,
            f"{self.user}@{self.host}",
            f"bash -lc {shlex.quote(script)}",
        ]

    def _start_tail(self, name):
        self._stop_tail()
        script = 'tail -n +1 -F "$HOME/.inspection_sim_remote/active.log"'
        proc = self._make_process(f"tail_{name}", track_active=True)
        self.active_tail = proc
        self.active_name = name
        self.state_changed.emit(name)
        self.log_line.emit(f"[TAIL] remote {name} log")
        proc.start("ssh", self._ssh_args(script))

    def _after_remote_start(self, name, start_local_rviz, local_rviz_config):
        self._start_tail(name)
        if start_local_rviz:
            self.start_local_rviz(local_rviz_config)

    def _after_remote_stop(self):
        self._stop_tail()
        self.stop_local_rviz()

    def _stop_tail(self):
        if self.active_tail is None:
            self.active_name = "idle"
            self.state_changed.emit("idle")
            return
        if self.active_tail.state() != QProcess.NotRunning:
            pid = int(self.active_tail.processId())
            try:
                os.kill(pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            self.active_tail.waitForFinished(1000)
            if self.active_tail.state() != QProcess.NotRunning:
                self.active_tail.kill()
        self.active_tail = None
        self.active_name = "idle"
        self.state_changed.emit("idle")

    def _make_process(self, name, track_active, after_success=None):
        proc = QProcess(self)
        proc.setProcessChannelMode(QProcess.MergedChannels)
        proc.readyReadStandardOutput.connect(lambda p=proc: self._read_output(p))
        proc.finished.connect(
            lambda code, status, p=proc, n=name, t=track_active, cb=after_success: self._finished(
                p, n, t, cb, code
            )
        )
        return proc

    def _read_output(self, proc):
        data = bytes(proc.readAllStandardOutput()).decode(errors="replace")
        for line in data.splitlines():
            self.log_line.emit(line)

    def _finished(self, proc, name, track_active, after_success, code):
        self.log_line.emit(f"[EXIT] {name} -> {code}")
        if proc in self.oneshot_processes:
            self.oneshot_processes.remove(proc)
        if track_active and proc is self.active_tail:
            self.active_tail = None
            self.active_name = "idle"
            self.state_changed.emit("idle")
        if proc is self.rviz_process:
            self.rviz_process = None
            self.rviz_state_changed.emit("idle")
        if code == 0 and after_success is not None:
            after_success()

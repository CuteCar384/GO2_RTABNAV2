#!/usr/bin/env bash
# Source the GO2 SLAM stack with workspace verification.
# Usage: source scripts/source_go2.bash
# Override: GO2_WS=~/GO2_RTABNAV2/ws

_GO2_WS="${GO2_WS:-${HOME}/GO2_RTABNAV2/ws}"
_GO2_WS="$(cd "${_GO2_WS}" && pwd)"
_GO2_EXPECTED="${_GO2_WS}/install/go2_slam_nav"

# Prefer system ROS binaries over ~/.local shims.
export PATH="/usr/bin:/bin:/opt/ros/jazzy/bin:${PATH}"

_fail() {
  echo "source_go2.bash: $*" >&2
  if [[ "${BASH_SOURCE[0]}" != "${0}" ]]; then
    return 1
  fi
  exit 1
}

_detect_robot_iface() {
  if [[ -n "${GO2_ROBOT_IFACE:-}" ]]; then
    echo "${GO2_ROBOT_IFACE}"
    return 0
  fi
  local iface
  iface="$(ip -4 -br addr 2>/dev/null | awk '$3 ~ /^192\.168\.123\./ {print $1; exit}')"
  if [[ -n "${iface}" ]]; then
    echo "${iface}"
    return 0
  fi
  iface="$(ip -br link 2>/dev/null | awk '/^enx/ && $2=="UP" {print $1; exit}')"
  if [[ -n "${iface}" ]]; then
    echo "${iface}"
    return 0
  fi
  return 1
}

_setup_cyclonedds() {
  local iface="$1"
  export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
  export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"${iface}\" priority=\"default\" multicast=\"default\" /></Interfaces></General></Domain></CycloneDDS>"
}

_strip_prefix_path() {
  local var_name="$1"
  local -a drop=("${@:2}")
  local current="" part keep
  eval "current=\"\${${var_name}:-}\""
  [[ -z "${current}" ]] && return 0
  local IFS=':'
  local -a out=()
  for part in ${current}; do
    keep=1
    for d in "${drop[@]}"; do
      if [[ "${part}" == "${d}"* ]]; then
        keep=0
        break
      fi
    done
    [[ "${keep}" -eq 1 ]] && out+=("${part}")
  done
  if ((${#out[@]} > 0)); then
    local IFS=':'
    eval "export ${var_name}=\"${out[*]}\""
  else
    unset "${var_name}"
  fi
}

set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source "${HOME}/unitree_ros2/cyclonedds_ws/install/setup.bash"
# Remove stale go2_slam_nav_ws overlay; keep ROS Jazzy + unitree in the path.
_strip_prefix_path CMAKE_PREFIX_PATH "${HOME}/go2_slam_nav_ws"
_strip_prefix_path AMENT_PREFIX_PATH "${HOME}/go2_slam_nav_ws"
_strip_prefix_path COLCON_PREFIX_PATH "${HOME}/go2_slam_nav_ws"
# shellcheck disable=SC1091
source "${_GO2_WS}/install/setup.bash"
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
_iface="$(_detect_robot_iface 2>/dev/null)" || _iface=""
if [[ -n "${_iface}" ]]; then
  if ip -br link show "${_iface}" 2>/dev/null | grep -q '\<UP\>'; then
    _setup_cyclonedds "${_iface}"
    echo "==> DDS interface: ${_iface}  ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
  else
    echo "source_go2.bash: warning: robot interface ${_iface} is not UP" >&2
  fi
else
  echo "source_go2.bash: warning: no 192.168.123.x / enx* interface — GO2 topics may be missing" >&2
  echo "source_go2.bash: connect USB Ethernet or set GO2_ROBOT_IFACE=enxXXXXXXXX" >&2
fi

_pkg_prefix="$(ros2 pkg prefix go2_slam_nav 2>/dev/null)" || _pkg_prefix=""
if [[ -z "${_pkg_prefix}" ]]; then
  _fail "ros2 pkg prefix go2_slam_nav failed — is ROS Jazzy sourced?"
fi
if [[ "${_pkg_prefix}" != "${_GO2_EXPECTED}" ]]; then
  _fail "go2_slam_nav resolves to '${_pkg_prefix}', expected '${_GO2_EXPECTED}' (check ~/go2_slam_nav_ws overlay)"
fi

if [[ ! -x "${_GO2_EXPECTED}/lib/go2_slam_nav/sensor_stamp_relay" ]]; then
  _fail "sensor_stamp_relay missing — rebuild: cd ${_GO2_WS} && colcon build --packages-select go2_slam_nav --symlink-install"
fi

_rtabmap_prefix="$(ros2 pkg prefix rtabmap_slam 2>/dev/null)" || _rtabmap_prefix=""
if [[ -z "${_rtabmap_prefix}" ]]; then
  _fail "rtabmap_slam not found — run: sudo apt install ros-jazzy-rtabmap-ros"
fi

echo "==> GO2 workspace ready: ${_GO2_WS}"
echo "==> go2_slam_nav: ${_pkg_prefix}"
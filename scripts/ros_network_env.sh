#!/usr/bin/env bash

# Source file này, không chạy bằng một subshell:
#   source scripts/ros_network_env.sh [peer_ipv4]

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  echo "ERROR: Hãy source script để biến môi trường có hiệu lực:" >&2
  echo "  source scripts/ros_network_env.sh [peer_ipv4]" >&2
  exit 2
fi

_rc_project_root="$(
  cd "$(dirname "${BASH_SOURCE[0]}")/.." >/dev/null 2>&1 && pwd
)"
_rc_peer_ip="${1:-${RC_DDS_PEER_IP:-}}"

if ! ip link show wlan0 >/dev/null 2>&1; then
  echo "ERROR: Không tìm thấy interface wlan0 trên máy này." >&2
  unset _rc_project_root _rc_peer_ip
  return 1
fi

_rc_local_ip="$(
  ip -4 -o addr show dev wlan0 scope global |
    awk 'NR == 1 {split($4, address, "/"); print address[1]}'
)"
if [[ -z "${_rc_local_ip}" ]]; then
  echo "ERROR: wlan0 chưa có địa chỉ IPv4." >&2
  unset _rc_project_root _rc_peer_ip _rc_local_ip
  return 1
fi

unset ROS_IP
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID="${RC_ROS_DOMAIN_ID:-${ROS_DOMAIN_ID:-0}}"

if [[ -n "${_rc_peer_ip}" ]]; then
  if [[ ! "${_rc_peer_ip}" =~ ^([0-9]{1,3}\.){3}[0-9]{1,3}$ ]]; then
    echo "ERROR: peer phải là IPv4, nhận được '${_rc_peer_ip}'." >&2
    unset _rc_project_root _rc_peer_ip _rc_local_ip
    return 1
  fi
  export RC_DDS_PEER_IP="${_rc_peer_ip}"
  export CYCLONEDDS_URI="file://${_rc_project_root}/config/dds/cyclonedds_pi_peer.xml"
  _rc_mode="unicast peer ${RC_DDS_PEER_IP} + multicast discovery"
else
  unset RC_DDS_PEER_IP
  export CYCLONEDDS_URI="file://${_rc_project_root}/config/dds/cyclonedds_pi_wifi.xml"
  _rc_mode="multicast discovery"
fi

echo "ROS 2 network ready:"
echo "  local IPv4       : ${_rc_local_ip} (wlan0)"
echo "  ROS_DOMAIN_ID    : ${ROS_DOMAIN_ID}"
echo "  RMW_IMPLEMENTATION: ${RMW_IMPLEMENTATION}"
echo "  discovery        : ${_rc_mode}"
echo "  CYCLONEDDS_URI   : ${CYCLONEDDS_URI}"
echo "Lưu ý: process ROS 2 đã chạy trước lệnh source phải được khởi động lại riêng."

unset _rc_project_root _rc_peer_ip _rc_local_ip _rc_mode

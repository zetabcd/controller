#!/usr/bin/env bash

set +e
set +u

SETUP_FILES=(
  "$HOME/nokov_ws/install/setup.bash"
  "$HOME/px4Msg/install/setup.bash"
)

usage() {
  cat <<'USAGE'
Usage:
  ./script/start_onboard_terminals.sh

Opens one gnome-terminal window with three tabs:
  1. MicroXRCEAgent udp4 -p 8888
  2. ros2 launch vrpn_client_ros sample.launch.py
  3. ros2 launch vrpn_client_ros px4_bridge.launch.py

Each tab sources:
  ~/nokov_ws/install/setup.bash
  ~/px4Msg/install/setup.bash
USAGE
}

source_env() {
  local setup_file

  for setup_file in "${SETUP_FILES[@]}"; do
    if [[ ! -f "$setup_file" ]]; then
      echo "Missing setup file: $setup_file" >&2
      return 1
    fi

    # ROS/colcon setup files reference optional variables such as COLCON_TRACE.
    set +e
    set +u

    # shellcheck source=/dev/null
    source "$setup_file"
  done
}

hold_terminal() {
  local status="$1"

  echo
  echo "Command exited with status: $status"
  echo "Type 'exit' or close this tab when finished."
  exec "${SHELL:-/bin/bash}" -i
}

run_command() {
  local title="$1"
  shift

  echo "[$title]"
  printf 'Source: %s\n' "${SETUP_FILES[@]}"
  echo "Run: $*"
  echo

  source_env
  local status="$?"
  set +e

  if [[ "$status" -eq 0 ]]; then
    "$@"
    status="$?"
  fi

  if [[ "${ONBOARD_WRAPPER_HOLDS:-0}" == "1" ]]; then
    return "$status"
  fi

  hold_terminal "$status"
}

shell_join() {
  local quoted=()
  local arg
  local quoted_arg

  for arg in "$@"; do
    printf -v quoted_arg '%q' "$arg"
    quoted+=("$quoted_arg")
  done

  local IFS=' '
  printf '%s' "${quoted[*]}"
}

terminal_shell_command() {
  local mode="$1"
  local script_path="$2"
  local command

  command="$(shell_join "$script_path" "$mode")"
  printf '%s' \
    "$command; status=\$?; echo; echo \"Command exited with status: \$status\"; echo \"Type 'exit' or close this terminal when finished.\"; exec bash -i"
}

open_tab() {
  local mode="$1"
  local title="$2"
  local script_path="$3"

  gnome-terminal --tab \
    --title="$title" \
    --working-directory="$PWD" \
    -- env ONBOARD_WRAPPER_HOLDS=1 bash --noprofile --norc -i -c \
    "$(terminal_shell_command "$mode" "$script_path")"
}

spawn_tabs_and_run_agent() {
  local script_path
  script_path="$(readlink -f "$0")"

  open_tab "--vrpn-sample" "vrpn sample" "$script_path"
  sleep 0.2
  open_tab "--px4-bridge" "px4 bridge" "$script_path"

  run_command "MicroXRCEAgent" MicroXRCEAgent udp4 -p 8888
}

open_terminals() {
  if ! command -v gnome-terminal >/dev/null 2>&1; then
    echo "gnome-terminal is required but was not found." >&2
    exit 1
  fi

  if [[ -z "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]]; then
    echo "No graphical display was detected; cannot open gnome-terminal." >&2
    exit 1
  fi

  local script_path
  script_path="$(readlink -f "$0")"

  gnome-terminal --window \
    --title="MicroXRCEAgent" \
    --working-directory="$PWD" \
    -- env ONBOARD_WRAPPER_HOLDS=1 bash --noprofile --norc -i -c \
    "$(terminal_shell_command "--spawn-tabs" "$script_path")"
}

case "${1:-}" in
  "" | --open)
    open_terminals
    ;;
  --agent)
    run_command "MicroXRCEAgent" MicroXRCEAgent udp4 -p 8888
    ;;
  --spawn-tabs)
    spawn_tabs_and_run_agent
    ;;
  --vrpn-sample)
    run_command "vrpn sample" ros2 launch vrpn_client_ros sample.launch.py
    ;;
  --px4-bridge)
    run_command "px4 bridge" ros2 launch vrpn_client_ros px4_bridge.launch.py
    ;;
  -h | --help)
    usage
    ;;
  *)
    usage >&2
    exit 2
    ;;
esac

#!/usr/bin/env bash

set +e
set +u

CONTROLLER_PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
SETUP_FILE="$CONTROLLER_PROJECT_ROOT/install/setup.bash"

usage() {
  cat <<'USAGE'
Usage:
  ./script/start_px4ctrl_terminals.sh

Opens one gnome-terminal window with two tabs:
  1. ros2 launch px4ctrl run_ctrl.launch.py
  2. ros2 launch px4ctrl realflight_trajectory_visualizer.launch.py

Each tab sources:
  <project root>/install/setup.bash
USAGE
}

source_env() {
  if [[ ! -f "$SETUP_FILE" ]]; then
    echo "Missing setup file: $SETUP_FILE" >&2
    return 1
  fi

  set +e
  set +u

  # shellcheck source=/dev/null
  source "$SETUP_FILE"
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
  echo "Source: $SETUP_FILE"
  echo "Run: $*"
  echo

  source_env
  local status="$?"
  set +e

  if [[ "$status" -eq 0 ]]; then
    "$@"
    status="$?"
  fi

  if [[ "${PX4CTRL_WRAPPER_HOLDS:-0}" == "1" ]]; then
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
    -- env PX4CTRL_WRAPPER_HOLDS=1 bash --noprofile --norc -i -c \
    "$(terminal_shell_command "$mode" "$script_path")"
}

spawn_tabs_and_run_ctrl() {
  local script_path
  script_path="$(readlink -f "$0")"

  open_tab "--trajectory-visualizer" "trajectory visualizer" "$script_path"

  run_command "px4ctrl run_ctrl" ros2 launch px4ctrl run_ctrl.launch.py
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
    --title="px4ctrl run_ctrl" \
    --working-directory="$PWD" \
    -- env PX4CTRL_WRAPPER_HOLDS=1 bash --noprofile --norc -i -c \
    "$(terminal_shell_command "--spawn-tabs" "$script_path")"
}

case "${1:-}" in
  "" | --open)
    open_terminals
    ;;
  --spawn-tabs)
    spawn_tabs_and_run_ctrl
    ;;
  --run-ctrl)
    run_command "px4ctrl run_ctrl" ros2 launch px4ctrl run_ctrl.launch.py
    ;;
  --trajectory-visualizer)
    run_command "trajectory visualizer" ros2 launch px4ctrl realflight_trajectory_visualizer.launch.py
    ;;
  -h | --help)
    usage
    ;;
  *)
    usage >&2
    exit 2
    ;;
esac

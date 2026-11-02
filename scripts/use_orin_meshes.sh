#!/usr/bin/env bash
set -euo pipefail

# Run on the laptop after: gio mount sftp://ubuntu@192.168.0.162
mount_root="/run/user/$(id -u)/gvfs"
remote_robot=''
shopt -s nullglob
for mounted_host in "$mount_root"/sftp:*192.168.0.162*; do
  candidate="$mounted_host/home/ubuntu/docker_dir/engineai_ros2_workspace/src/simulation/mujoco/assets/resource/robot"
  if [[ -f "$candidate/pm_v2/meshes/LINK_ANKLE_PITCH_L.STL" ]]; then
    remote_robot="$candidate"
    break
  fi
done

if [[ -z "$remote_robot" ]]; then
  echo 'Orin mesh directory is not accessible. Mount sftp://ubuntu@192.168.0.162 first.' >&2
  exit 1
fi

resource_prefix="$(ros2 pkg prefix resource)"
resource_share="$resource_prefix/share/resource"
mkdir -p "$resource_share"

if [[ -L "$resource_share/robot" ]]; then
  if [[ "$(readlink "$resource_share/robot")" == "$remote_robot" ]]; then
    echo "Meshes already linked to $remote_robot"
    exit 0
  fi
  echo "Existing resource/robot link points elsewhere: $resource_share/robot" >&2
  exit 1
fi

if [[ -e "$resource_share/robot" ]]; then
  backup="$resource_share/robot.local-copy"
  if [[ -e "$backup" ]]; then
    echo "Backup path already exists: $backup" >&2
    exit 1
  fi
  mv "$resource_share/robot" "$backup"
  echo "Moved the previous laptop mesh copy to $backup"
fi

ln -s "$remote_robot" "$resource_share/robot"
echo "RViz will read robot meshes from $remote_robot"

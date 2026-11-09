#!/usr/bin/env bash
set -eo pipefail
semantic_fetch_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
semantic_asset_dir="$semantic_fetch_root/third_party/aws_small_house"
semantic_asset_sha=ff9631ca6d1db9c1ba656498151464b5ab74aafe
if [[ -e "$semantic_asset_dir" ]]; then
    if [[ -d "$semantic_asset_dir/.git" ]] && \
       [[ "$(git -C "$semantic_asset_dir" rev-parse HEAD)" == "$semantic_asset_sha" ]]; then
        echo "AWS assets already present at pinned commit $semantic_asset_sha"
        exit 0
    fi
    echo "Existing AWS asset directory has another version; inspect it before replacing." >&2
    exit 1
fi
git clone --depth 1 --branch ros2 \
    https://github.com/aws-robotics/aws-robomaker-small-house-world.git "$semantic_asset_dir"
if [[ "$(git -C "$semantic_asset_dir" rev-parse HEAD)" != "$semantic_asset_sha" ]]; then
    git -C "$semantic_asset_dir" fetch --depth 1 origin "$semantic_asset_sha"
    git -C "$semantic_asset_dir" checkout --detach "$semantic_asset_sha"
fi

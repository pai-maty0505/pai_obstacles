#!/usr/bin/env bash
# Gazebo の画面を一定間隔で撮影し、指定したディレクトリに移す。
# 使い方：scripts/gz_screenshots.sh <保存先ディレクトリ> [間隔（秒、既定 3）]
# Ctrl+C か kill で止める。

source "$(dirname "${BASH_SOURCE[0]}")/sim_env.sh"
source_ros

OUT_DIR="$1"
INTERVAL="${2:-3}"
PICTURES="${HOME}/.gz/gui/pictures"
mkdir -p "${OUT_DIR}" "${PICTURES}"

move_new_pictures() {
    find "${PICTURES}" -maxdepth 1 -name '*.png' -newer "${OUT_DIR}/.start" -exec mv {} "${OUT_DIR}/" \;
}
trap 'sleep 1; move_new_pictures; exit 0' INT TERM

touch "${OUT_DIR}/.start"
while true; do
    gz service -s /gui/screenshot --reqtype gz.msgs.StringMsg --reptype gz.msgs.Boolean \
        --timeout 2000 --req 'data: ""' >/dev/null 2>&1
    sleep "${INTERVAL}"
    move_new_pictures
done

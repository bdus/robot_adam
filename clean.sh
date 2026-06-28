# rm -rf build install log
for dir in build/* install/*; do
    # 1. [ -n "$dir" ] ── 必须非空！如果变量为空，直接拦截。
    # 2. [ -d "$dir" ] ── 必须是存在的目录！如果目录不存在，直接拦截。
    # 3. 名字校验     ── 必须不是 livox_ros_driver2。
    if [ -n "$dir" ] && [ -d "$dir" ] && [ "$(basename "$dir")" != "livox_ros_driver2" ]; then
        rm -rf "$dir"
    fi
done

[ -d "log" ] && rm -rf log/*
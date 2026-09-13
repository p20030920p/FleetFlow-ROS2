#!/bin/bash
# 清掉残留的 Gazebo 与仿真节点。
#
# 什么时候需要它：上一次运行是被 Ctrl-C 之外的方式结束的（终端关掉、kill -9、
# 崩溃），Gazebo 的**服务端会作为孤儿进程留下来**。之后再开界面时，GUI 可能连到
# 那个旧服务端上，表现就是"窗口打开了，但里面什么都不渲染"。
#
# 为什么不能简单 `pkill -x gz`：
#   * 这个发行版里 `gz sim` 是 Ruby 包装脚本，进程名是 `ruby`，`pkill -x gz` 匹配不到；
#   * 包装脚本退出后，子进程的参数是 `gz sim server` / `gz sim gui`，
#     不再含 `-s -r`，只按 `-s -r` 匹配同样会漏。
#
# 用法：bash tools/gz_reset.sh
#
# 注意：不要把这个脚本的内容粘到命令行里执行 —— 下面的 pkill -f 会匹配到
# 调用者自己的命令行并把它杀掉。

set +e
for name in robot_controller task_scheduler factory_manager traffic_manager \
            metrics dashboard parameter_bridge capture_views run_recorder; do
    pkill -x "$name" 2>/dev/null
done

echo "清理 Gazebo（包装脚本 / 服务端 / 界面）…"
for pat in "gz sim" "gz-sim" "sim server" "sim gui" "sim -s -r"; do
    pkill -f "$pat" 2>/dev/null
done
sleep 2
for pat in "gz sim" "sim server" "sim gui"; do
    pkill -9 -f "$pat" 2>/dev/null
done
sleep 1

left=$(ps -eo args --no-headers | grep -c "[g]z sim")
nodes=$(ps -eo comm --no-headers | grep -cE 'robot_controlle|task_scheduler|factory_manager|traffic_manager|^metrics$|^dashboard$')
echo "剩余 Gazebo 进程: $left  剩余仿真节点: $nodes"
[ "$left" -eq 0 ] && echo "干净了，可以重新 launch。"

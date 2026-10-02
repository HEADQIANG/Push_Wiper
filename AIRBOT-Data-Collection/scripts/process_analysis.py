""""""

import argparse
import os
import time

import psutil


def prime_cpu_counters():
    """预热所有进程的 CPU 统计，避免首次读取出现 0.0。"""
    for proc in psutil.process_iter():
        try:
            proc.cpu_percent(interval=None)
        except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
            continue


def build_process_snapshot():
    """采样当前系统内的进程信息，返回以 PID 为键的快照。"""
    snapshot = {}
    for proc in psutil.process_iter(attrs=["pid", "name", "cmdline"]):
        try:
            info = proc.info
            cmdline_list = info.get("cmdline") or []
            cmdline = " ".join(cmdline_list)
            snapshot[proc.pid] = {
                "name": info.get("name") or "",
                "cmdline": cmdline,
                "cpu": proc.cpu_percent(interval=None),
                "mem": proc.memory_percent(),
            }
        except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
            continue
    return snapshot


def find_root_process(
    snapshot: dict, cmd_fragment: str, preferred_pid=None, exclude_pids=None
):
    """在快照中查找命令行包含片段的进程，优先使用上次命中的 PID。"""
    if not cmd_fragment:
        return None

    exclude_pids = exclude_pids or set()

    def pick_process(pid: int):
        if pid in exclude_pids:
            return None
        info = snapshot.get(pid)
        if not info:
            return None
        if cmd_fragment not in info.get("cmdline", ""):
            return None
        try:
            proc = psutil.Process(pid)
            # 确保进程仍在运行且非僵尸，避免持有陈旧句柄
            if proc.is_running() and proc.status() != psutil.STATUS_ZOMBIE:
                return proc
        except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
            return None
        return None

    if preferred_pid:
        proc = pick_process(preferred_pid)
        if proc:
            return proc

    for pid in snapshot.keys():
        proc = pick_process(pid)
        if proc:
            return proc
    return None


def collect_process_tree(
    proc: psutil.Process,
    snapshot: dict,
    indent: str = "",
    lines: list = None,
    skip_pids=None,
):
    """递归收集进程树信息到列表中，每层子进程按进程名排序。"""
    if lines is None:
        lines = []
    if skip_pids is None:
        skip_pids = set()

    try:
        info = snapshot.get(proc.pid)
        name = (info and info.get("name")) or proc.name()
        cpu = info and info.get("cpu")
        mem = info and info.get("mem")

        if cpu is None:
            cpu = proc.cpu_percent(interval=None)
        if mem is None:
            mem = proc.memory_percent()

        line = f"{indent}Name:{name} PID:{proc.pid} CPU:{cpu:.1f}% MEM:{mem:.1f}%"
        lines.append(line)
        skip_pids.add(proc.pid)

        children = proc.children(recursive=False)
        sorted_children = sorted(
            children,
            key=lambda p: (
                snapshot.get(p.pid, {}).get("name") or safe_process_name(p)
            ).lower(),
        )

        for child in sorted_children:
            collect_process_tree(child, snapshot, indent + "  ", lines, skip_pids)

    except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
        pass

    return lines


def safe_process_name(proc: psutil.Process):
    """安全获取进程名称。"""
    try:
        return proc.name()
    except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
        return "Unknown"


def collect_top_process_lines(
    snapshot: dict, fragment: str, limit: int, skip_pids=None
):
    """生成按照 CPU 占用排序的额外进程信息行。"""
    if limit <= 0:
        return []

    skip_pids = skip_pids or set()
    entries = []
    for pid, info in snapshot.items():
        if pid in skip_pids:
            continue
        cmdline = info.get("cmdline", "")
        if fragment and fragment in cmdline:
            continue
        entries.append(
            (
                info.get("cpu", 0.0),
                info.get("mem", 0.0),
                info.get("name", ""),
                pid,
            )
        )

    entries.sort(key=lambda item: (item[0], item[1]), reverse=True)
    top_entries = entries[:limit]

    lines = []
    for index, (cpu, mem, name, pid) in enumerate(top_entries, start=1):
        display_name = name or "Unknown"
        lines.append(
            f"{index}. Name:{display_name} PID:{pid} CPU:{cpu:.1f}% MEM:{mem:.1f}%"
        )

    return lines


def parse_args():
    parser = argparse.ArgumentParser(description="实时监控目标进程及系统负载")
    parser.add_argument(
        "command_fragment",
        help="用于匹配目标进程命令行的片段",
    )
    parser.add_argument(
        "--interval",
        "-i",
        type=float,
        default=1.0,
        help="刷新间隔，单位秒",
    )
    parser.add_argument(
        "--top",
        "-n",
        type=positive_int,
        default=5,
        help="额外展示的其他进程数量（按 CPU 排序）",
    )
    return parser.parse_args()


def positive_int(value: str) -> int:
    try:
        result = int(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("请输入整数") from exc
    if result < 0:
        raise argparse.ArgumentTypeError("数量需为非负整数")
    return result


def clear_screen():
    os.system("cls" if os.name == "nt" else "clear")


def main():
    args = parse_args()
    fragment = args.command_fragment
    interval = args.interval
    top_n = args.top
    tracked_pid = None
    self_pid = os.getpid()

    prime_cpu_counters()
    time.sleep(0.1)

    try:
        while True:
            snapshot = build_process_snapshot()
            root_proc = find_root_process(
                snapshot,
                fragment,
                preferred_pid=tracked_pid,
                exclude_pids={self_pid},
            )

            if root_proc:
                tracked_pid = root_proc.pid
            else:
                tracked_pid = None

            # 构建完整输出内容
            output_lines = []
            skip_pids = set()
            if not root_proc:
                output_lines.append(f'未找到包含 "{fragment}" 的进程')
            else:
                output_lines.append(f"进程树 (起点: {fragment}, PID={root_proc.pid})")
                collect_process_tree(
                    root_proc,
                    snapshot,
                    lines=output_lines,
                    skip_pids=skip_pids,
                )

            top_lines = collect_top_process_lines(
                snapshot,
                fragment,
                top_n,
                skip_pids,
            )
            if top_lines:
                output_lines.append("")
                output_lines.append(
                    f'其他进程 CPU TOP {len(top_lines)} (排除包含 "{fragment}" 的进程)'
                )
                output_lines.extend(top_lines)

            # 清屏后一次性输出所有内容
            clear_screen()
            print("\n".join(output_lines))

            time.sleep(interval)

    except KeyboardInterrupt:
        print("\n退出实时监控")


if __name__ == "__main__":
    main()

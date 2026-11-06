#!/usr/bin/env python3
"""分析手柄两次姿态之间的相对旋转，不连接机械臂。"""

import math


def quaternion_from_rpy(roll, pitch, yaw):
    """RPY（弧度）转四元数，返回 qx,qy,qz,qw。"""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)

    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def conjugate(q):
    return (-q[0], -q[1], -q[2], q[3])


def read_triplet(prompt):
    while True:
        try:
            values = [
                float(x.strip())
                for x in input(prompt).split(",")
            ]
            if len(values) != 3 or not all(
                math.isfinite(x) for x in values
            ):
                raise ValueError
            return values
        except ValueError:
            print("请输入三个有限数值，以英文逗号分隔。")


def main():
    print("=== 手柄相对旋转分析（只做数学计算）===")
    print("请输入预览日志中的 RPY，单位为弧度。")
    print()

    ref = read_triplet(
        "参考姿态 RPY（roll,pitch,yaw）："
    )
    cur = read_triplet(
        "移动后姿态 RPY（roll,pitch,yaw）："
    )

    q_ref = quaternion_from_rpy(*ref)
    q_cur = quaternion_from_rpy(*cur)

    # 在机器人/参考坐标系中计算相对旋转
    dq = multiply(q_cur, conjugate(q_ref))
    norm = math.sqrt(sum(x * x for x in dq))
    dq = tuple(x / norm for x in dq)

    # 选择最短旋转表示
    if dq[3] < 0:
        dq = tuple(-x for x in dq)

    w = max(-1.0, min(1.0, dq[3]))
    angle = 2 * math.acos(w)
    s = math.sqrt(max(0.0, 1.0 - w * w))

    print("\n========== 分析结果 ==========")
    print(f"相对旋转角：{math.degrees(angle):.3f}°")

    if s < 1e-8:
        print("旋转轴：角度接近零，无法可靠确定方向")
    else:
        axis = [dq[i] / s for i in range(3)]
        print(
            "旋转轴 XYZ："
            + ", ".join(f"{x:+.4f}" for x in axis)
        )
        print(
            "旋转轴绝对分量："
            + ", ".join(f"{abs(x):.4f}" for x in axis)
        )

    print("\n注意：结果依赖输入 RPY 的定义和坐标系。")
    print("请勿仅凭此结果直接启动机械臂实机控制。")


if __name__ == "__main__":
    main()

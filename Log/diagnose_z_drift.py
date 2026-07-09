#!/usr/bin/env python3
"""
FAST_LIO2 Z轴漂移诊断工具
用法: python3 diagnose_z_drift.py [log_dir]
默认读取当前目录下的 mat_out.txt, mat_pre.txt, pos_log.txt, imu.txt
"""
import numpy as np
import sys, os

def load_file(path):
    if not os.path.exists(path):
        print(f"  [缺失] {path}")
        return None
    try:
        data = np.loadtxt(path)
        if data.ndim == 1:
            data = data.reshape(1, -1)
        if data.size == 0:
            print(f"  [空文件] {path}")
            return None
        return data
    except Exception as e:
        print(f"  [读取失败] {path}: {e}")
        return None

def diagnose(work_dir="."):
    os.chdir(work_dir)
    print("=" * 60)
    print("FAST_LIO2 Z轴漂移诊断报告")
    print("=" * 60)

    # ============ 1. 加载数据 ============
    mat_out = load_file("mat_out.txt")   # 列: time, euler(3), pos(3), extR(3), extT(3), vel(3), bg(3), ba(3), grav(3), pts_num
    mat_pre = load_file("mat_pre.txt")   # 列: time, euler(3), pos(3), extR(3), extT(3), vel(3), bg(3), ba(3), grav(3)
    pos_log = load_file("pos_log.txt")   # 列: time, angle(3), pos(3), omega(3), vel(3), acc(3), bg(3), ba(3), grav(3)
    imu_log = load_file("imu.txt")       # 列: time, gyr(3), acc(3)
    time_csv = load_file("fast_lio_time_log.csv")

    if mat_out is None and pos_log is None:
        print("\n错误: 找不到诊断日志文件!")
        print("请确保配置中 runtime_pos_log_enable: true")
        return

    # 选择可用数据源
    data = pos_log if pos_log is not None else mat_out
    n_frames = len(data)

    time      = data[:, 0]
    pos       = data[:, 4:7]   # x, y, z
    if pos_log is not None:
        vel   = data[:, 10:13]
        bg    = data[:, 16:19]
        ba    = data[:, 19:22]
        grav  = data[:, 22:25]
    else:
        vel   = data[:, 14:17]
        bg    = data[:, 17:20]
        ba    = data[:, 20:23]
        grav  = data[:, 23:26]

    # ============ 2. Z轴漂移特征分析 ============
    print(f"\n[1] 基本信息")
    print(f"  总帧数: {n_frames}")
    print(f"  总时长: {time[-1]:.1f}s")
    print(f"  Z起始:  {pos[0,2]:.3f}m")
    print(f"  Z终止:  {pos[-1,2]:.3f}m")
    print(f"  Z总漂移: {pos[-1,2] - pos[0,2]:.3f}m")
    print(f"  Z最大值: {pos[:,2].max():.3f}m")
    print(f"  Z最小值: {pos[:,2].min():.3f}m")

    # Z漂移速度
    z_drift_total = pos[-1,2] - pos[0,2]
    z_drift_rate = z_drift_total / max(time[-1], 1.0)
    print(f"  Z漂移速率: {z_drift_rate:.4f} m/s")

    # 检测Z突变（>0.1m/帧的跳变）
    z_diff = np.diff(pos[:,2])
    z_jumps = np.where(np.abs(z_diff) > 0.1)[0]
    if len(z_jumps) > 0:
        print(f"\n  ⚠️ 检测到 {len(z_jumps)} 次Z轴跳变 (>0.1m/帧):")
        for j in z_jumps[:5]:
            print(f"    帧 {j}: Z {pos[j,2]:.3f} -> {pos[j+1,2]:.3f} (跳变 {z_diff[j]:.3f}m)")

    # Z轴单调性
    z_monotonic_up = np.sum(z_diff > 0.01)
    z_monotonic_down = np.sum(z_diff < -0.01)
    print(f"  Z上升帧: {z_monotonic_up}, Z下降帧: {z_monotonic_down}")
    if z_monotonic_up > n_frames * 0.6:
        print(f"  ⚠️ Z轴持续单调上升 → IMU积分发散, 雷达约束不足")

    # ============ 3. 有效特征点分析 ============
    print(f"\n[2] 雷达约束强度")
    if mat_out is not None and mat_out.shape[1] >= 26:
        pts_num = mat_out[:, 25]  # feats_undistort->points.size()
    else:
        pts_num = np.ones(n_frames) * -1

    if pts_num[0] > 0:
        print(f"  平均每帧去畸变点数: {pts_num.mean():.0f}")
        print(f"  最小点数: {pts_num.min():.0f}")
        print(f"  最大点数: {pts_num.max():.0f}")

        low_pts_frames = np.sum(pts_num < 500)
        if low_pts_frames > n_frames * 0.05:
            print(f"  ⚠️  {low_pts_frames} 帧点数 < 500 ({100*low_pts_frames/n_frames:.1f}%) → 雷达约束弱")
        elif pts_num.mean() < 1000:
            print(f"  ⚠️  平均点数偏低 ({pts_num.mean():.0f}) → 雷达约束可能不够")
        else:
            print(f"  ✅ 点云密度正常")
    else:
        print(f"  (mat_out.txt 无点云数量列, 请用 runtime_pos_log_enable: true)")

    # 从mat_out获取有效特征数 (effct_feat_num 未直接输出, 但可通过残差均值推断)
    if mat_out is not None and mat_out.shape[1] >= 26:
        residual_mean = np.array([np.mean(mat_out[i, 1:4]) for i in range(len(mat_out))])  # 用euler近似
        print(f"  残差均值范围: {residual_mean.min():.4f} ~ {residual_mean.max():.4f}")

    # ============ 4. IMU数据质量分析 ============
    print(f"\n[3] IMU数据质量")
    if imu_log is not None:
        imu_time  = imu_log[:, 0]
        imu_gyr   = imu_log[:, 1:4]
        imu_acc   = imu_log[:, 4:7]

        imu_rate = len(imu_log) / max(imu_time[-1] - imu_time[0], 1.0)
        print(f"  IMU消息数: {len(imu_log)}")
        print(f"  IMU有效频率: {imu_rate:.0f} Hz")

        # 检测IMU时间间隔异常
        imu_dt = np.diff(imu_time)
        imu_dt_normal = imu_dt[(imu_dt > 0.001) & (imu_dt < 0.1)]
        if len(imu_dt_normal) > 0:
            print(f"  IMU正常间隔: {imu_dt_normal.mean()*1000:.1f}ms ± {imu_dt_normal.std()*1000:.1f}ms")
        imu_gaps = np.where(imu_dt > 0.05)[0]
        if len(imu_gaps) > 0:
            print(f"  ⚠️ 检测到 {len(imu_gaps)} 处IMU时间间隙 >50ms (可能丢包)")

        # Z轴加速度统计
        acc_z = imu_acc[:, 2]
        print(f"  Z加速度均值: {acc_z.mean():.3f} m/s² (期望 ~ -9.8)")
        print(f"  Z加速度标准差: {acc_z.std():.3f}")
        acc_z_spikes = np.sum(np.abs(acc_z - acc_z.mean()) > 3 * acc_z.std())
        if acc_z_spikes > len(imu_log) * 0.02:
            print(f"  ⚠️  Z加速度异常值: {acc_z_spikes} 个 ({100*acc_z_spikes/len(imu_log):.1f}%) → IMU颠簸/振动严重")

        # 陀螺仪统计
        gyr_norm = np.linalg.norm(imu_gyr, axis=1)
        print(f"  陀螺角速度范数均值: {np.rad2deg(gyr_norm.mean()):.1f} °/s")
        print(f"  陀螺角速度范数最大: {np.rad2deg(gyr_norm.max()):.1f} °/s")
    else:
        print(f"  imu.txt 未生成 (可能IMU初始化未完成?)")

    # ============ 5. 加速度bias漂移分析 ============
    print(f"\n[4] IMU Bias估计")
    ba_z = ba[:, 2]
    bg_norm = np.linalg.norm(bg, axis=1)
    print(f"  Z加速度bias: 均值={ba_z.mean():.6f}, 范围=[{ba_z.min():.6f}, {ba_z.max():.6f}]")
    print(f"  陀螺bias范数: 均值={np.rad2deg(bg_norm.mean()):.4f}°/s, 最大={np.rad2deg(bg_norm.max()):.4f}°/s")

    if np.abs(ba_z[-1] - ba_z[0]) > 0.01:
        print(f"  ⚠️  Z加速度bias漂移 {ba_z[-1]-ba_z[0]:.6f} → IMU零偏估计不稳定")

    # ============ 6. 重力估计分析 ============
    print(f"\n[5] 重力估计")
    grav_norm = np.linalg.norm(grav, axis=1)
    print(f"  重力范数: 均值={grav_norm.mean():.2f} (期望 ~9.8)")
    grav_std = grav_norm.std()
    if grav_std > 1.0:
        print(f"  ⚠️  重力估计波动大 (std={grav_std:.2f}) → 姿态估计可能不稳定")

    # ============ 7. 预测 vs 更新对比 ============
    print(f"\n[6] 预测-更新一致性 (IMU预测 vs LiDAR修正)")
    if mat_pre is not None and mat_out is not None:
        pre_pos = mat_pre[:, 4:7]
        out_pos = mat_out[:, 4:7]
        update_diff = out_pos - pre_pos
        z_correction = update_diff[:, 2]
        print(f"  Z轴LiDAR修正量: 均值={z_correction.mean():.4f}m, 最大={np.abs(z_correction).max():.4f}m")
        small_corrections = np.sum(np.abs(z_correction) < 0.01)
        print(f"  Z修正 < 1cm 的帧: {small_corrections}/{n_frames} ({100*small_corrections/n_frames:.1f}%)")
        if small_corrections > n_frames * 0.5:
            print(f"  ⚠️  过半帧Z修正量极小 → 雷达对Z轴约束不足 (场景缺竖直特征)")

    # ============ 8. 时序分析 ============
    print(f"\n[7] 处理时序")
    if time_csv is not None and len(time_csv) > 1:
        # CSV: time_stamp, total_time, scan_point_size, incremental_time, search_time, ...
        process_times = time_csv[:, 1]  # total_time
        scan_points   = time_csv[:, 2]
        print(f"  平均处理时间: {process_times.mean()*1000:.1f}ms")
        print(f"  最大处理时间: {process_times.max()*1000:.1f}ms")
        slow_frames = np.sum(process_times > 0.1)
        if slow_frames > 0:
            print(f"  ⚠️  {slow_frames} 帧处理超100ms → 可能跟不上实时")

    # ============ 9. 综合诊断 ============
    print(f"\n{'='*60}")
    print("综合诊断结论")
    print("="*60)

    issues = []

    # 场景问题
    if 'small_corrections' in dir() and small_corrections > n_frames * 0.5:
        issues.append("【场景几何】Z轴LiDAR修正量极小 → 场景可能缺少竖直结构(平地/长廊/开阔地),雷达无法观测Z轴误差")
    if 'pts_num' in dir() and pts_num[0] > 0 and pts_num.mean() < 800:
        issues.append("【点云稀疏】平均每帧点数不足800 → 特征约束弱,可尝试减小 point_filter_num")

    # IMU问题
    if imu_log is not None:
        if 'acc_z_spikes' in dir() and acc_z_spikes > len(imu_log) * 0.02:
            issues.append("【IMU颠簸】Z加速度异常值过多 → 机器人颠簸导致IMU数据差,增大 acc_cov 降低对IMU的信任")
        if 'imu_gaps' in dir() and len(imu_gaps) > 10:
            issues.append("【IMU丢包】IMU时间间隙过多 → 时间戳可能有跳变,检查rosbag录制质量")
    if np.abs(ba_z[-1] - ba_z[0]) > 0.01:
        issues.append("【Bias漂移】加速度bias不稳定 → 可能IMU预热不足或温度变化大")

    # 漂移特征
    if z_monotonic_up > n_frames * 0.6:
        issues.append("【单调发散】Z轴持续上升无修正 → IMU积分发散且雷达无力拉回,考虑增大 acc_cov/gyr_cov")

    if not issues:
        issues.append("✅ 未发现明显异常。若仍有漂移,请检查: 1)外参标定 2)timestamp_unit设置 3)scan_rate匹配")

    for i, issue in enumerate(issues):
        print(f"  {i+1}. {issue}")

    print(f"\n💡 如果漂移是突然发生的(非渐进),重点检查Z轴跳变时刻的IMU数据和点云数量。")
    print(f"💡 运行: python3 plot.py 可绘制姿态/位置/bias曲线,直观定位异常时刻。")

if __name__ == "__main__":
    work_dir = sys.argv[1] if len(sys.argv) > 1 else "."
    diagnose(work_dir)

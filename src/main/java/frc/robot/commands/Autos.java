// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.SuperstructureSubsystem;

/** 教学用开环自动。位置与航向注释均是理想目标，不是传感器测量。 */
public final class Autos {
  private Autos() {}

  public static Command teachingAuto(
      DriveSubsystem drive, SuperstructureSubsystem superstructure) {
    Runnable stopAll = () -> {
      drive.stop();
      superstructure.stop();
    };
    Runnable shoot = () -> {
      // 射球期间也周期性停止底盘，持续喂 DifferentialDrive 的 MotorSafety。
      drive.stop();
      superstructure.shoot();
    };

    // 固定蓝方坐标：+X 离开蓝方联盟墙，+Y 向左，逆时针为正；起始航向为 0°。
    // runOnce 声明两个 requirement，整个顺序组合全程占用两个 subsystem。
    return Commands.sequence(
            Commands.runOnce(stopAll, drive, superstructure),
            timedDrive(drive, AutoConstants.kDriveOutput, AutoConstants.kDriveOutput,
                AutoConstants.kDrive2MetersSeconds).withName("DriveForward2Meters"),
            timedDrive(drive, -AutoConstants.kTurnOutput, AutoConstants.kTurnOutput,
                AutoConstants.kTurnCcw90Seconds).withName("TurnCcw90Degrees"),
            timedDrive(drive, AutoConstants.kDriveOutput, AutoConstants.kDriveOutput,
                AutoConstants.kDrive4MetersSeconds).withName("DrivePositiveY4Meters"),
            // A 相对起点为 (2, 4)，航向为 90°。等待期间仍逐周期写入零输出。
            timedDrive(drive, 0.0, 0.0, AutoConstants.kWaitAtASeconds).withName("WaitAtA"),
            timedDrive(drive, AutoConstants.kTurnOutput, -AutoConstants.kTurnOutput,
                AutoConstants.kTurnCw180Seconds).withName("TurnCw180Degrees"),
            // 从 A 朝 -Y 前进，顺时针四分之一圆到 (-1, 1)，航向 -180°。
            // 终点比起点更靠近蓝墙 1 m，起点需留足车身及路径边界余量。
            timedDrive(drive, AutoConstants.kArcOuterOutput, AutoConstants.kArcInnerOutput,
                AutoConstants.kArcCw90Seconds).withName("ArcCw90Degrees"),
            timedDrive(drive, AutoConstants.kTurnOutput, -AutoConstants.kTurnOutput,
                AutoConstants.kTurnCw135Seconds).withName("TurnCw135Degrees"),
            // -180° - 135° = -315°，等价于 45°。保持底盘停止，射球 5 s。
            new FunctionalCommand(shoot, shoot, interrupted -> stopAll.run(),
                () -> false, drive, superstructure)
                .withTimeout(AutoConstants.kShootSeconds).withName("Shoot5Seconds"))
        .finallyDo(stopAll)
        .withName("TeachingOpenLoopAuto");
  }

  private static Command timedDrive(
      DriveSubsystem drive, double leftOutput, double rightOutput, double durationSeconds) {
    Runnable output = () -> drive.tankDriveOpenLoop(leftOutput, rightOutput);
    // initialize 立即输出，execute 持续喂 MotorSafety；超时或取消都会调用 end 停车。
    return new FunctionalCommand(output, output, interrupted -> drive.stop(),
        () -> false, drive).withTimeout(durationSeconds);
  }
}

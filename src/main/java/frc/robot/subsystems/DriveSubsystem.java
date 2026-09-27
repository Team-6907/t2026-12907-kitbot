// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

/** 管理底盘硬件和驱动能力；不读取手柄，也不决定自动动作顺序。 */
public class DriveSubsystem extends SubsystemBase {
  private final WPI_TalonSRX m_leftMaster =
      new WPI_TalonSRX(DriveConstants.kLeftMasterCanId);
  private final WPI_VictorSPX m_leftFollower =
      new WPI_VictorSPX(DriveConstants.kLeftFollowerCanId);
  private final WPI_TalonSRX m_rightMaster =
      new WPI_TalonSRX(DriveConstants.kRightMasterCanId);
  private final WPI_VictorSPX m_rightFollower =
      new WPI_VictorSPX(DriveConstants.kRightFollowerCanId);
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMaster, m_rightMaster);

  public DriveSubsystem() {
    m_leftMaster.configFactoryDefault();
    m_leftFollower.configFactoryDefault();
    m_rightMaster.configFactoryDefault();
    m_rightFollower.configFactoryDefault();

    m_leftFollower.follow(m_leftMaster);
    m_rightFollower.follow(m_rightMaster);

    // 保持原来的右侧反向配置，补偿左右电机的镜像安装。
    m_rightMaster.setInverted(true);
    m_rightFollower.setInverted(true);

    m_leftMaster.setNeutralMode(NeutralMode.Brake);
    m_leftFollower.setNeutralMode(NeutralMode.Brake);
    m_rightMaster.setNeutralMode(NeutralMode.Brake);
    m_rightFollower.setNeutralMode(NeutralMode.Brake);
  }

  /** 保持原有遥控死区及输入平方；forward 向前为正，rotation 逆时针为正。 */
  public void arcadeDrive(double forward, double rotation) {
    m_drive.setDeadband(DifferentialDrive.kDefaultDeadband);
    m_drive.arcadeDrive(forward, rotation);
  }

  /** 左右输出向前为正；关闭死区和输入平方，保留圆弧的左右输出比例。 */
  public void tankDriveOpenLoop(double leftOutput, double rightOutput) {
    m_drive.setDeadband(0.0);
    m_drive.tankDrive(leftOutput, rightOutput, false);
  }

  public void stop() {
    m_drive.stopMotor();
    m_drive.setDeadband(DifferentialDrive.kDefaultDeadband);
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import java.util.function.DoubleSupplier;

/** 底盘默认命令：持续读取遥控输入，自动动作占用底盘时由 scheduler 中断。 */
public class TeleopDriveCommand extends Command {
  private final DriveSubsystem m_drive;
  private final DoubleSupplier m_forwardSupplier;
  private final DoubleSupplier m_rotationSupplier;

  public TeleopDriveCommand(
      DriveSubsystem drive, DoubleSupplier forwardSupplier, DoubleSupplier rotationSupplier) {
    m_drive = drive;
    m_forwardSupplier = forwardSupplier;
    m_rotationSupplier = rotationSupplier;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    m_drive.stop();
  }

  @Override
  public void execute() {
    // 默认命令也可能在自动完成后或 test 中被调度，这些模式下不能响应摇杆。
    if (DriverStation.isTeleopEnabled()) {
      m_drive.arcadeDrive(m_forwardSupplier.getAsDouble(), m_rotationSupplier.getAsDouble());
    } else {
      m_drive.stop();
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_drive.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}

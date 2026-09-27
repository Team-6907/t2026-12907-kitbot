// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.TeleopDriveCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.SuperstructureSubsystem;

/** 创建 subsystem，定义默认命令、手柄绑定和自动入口。 */
public class RobotContainer {
  private final DriveSubsystem m_drive = new DriveSubsystem();
  private final SuperstructureSubsystem m_superstructure = new SuperstructureSubsystem();
  private final CommandXboxController m_controller =
      new CommandXboxController(OperatorConstants.kControllerPort);

  public RobotContainer() {
    m_drive.setDefaultCommand(
        new TeleopDriveCommand(
            m_drive, () -> -m_controller.getLeftY(), () -> -m_controller.getRightX()));
    m_superstructure.setDefaultCommand(
        Commands.run(m_superstructure::stop, m_superstructure).withName("IdleSuperstructure"));
    configureBindings();
  }

  private void configureBindings() {
    Trigger teleopEnabled = new Trigger(DriverStation::isTeleopEnabled);
    Trigger intakeButton = m_controller.x();

    intakeButton.and(teleopEnabled)
        .whileTrue(
            Commands.runEnd(m_superstructure::intake, m_superstructure::stop, m_superstructure)
                .withName("Intake"));
    // X 优先于 A；松开 X 而 A 仍按住时，条件重新变为 true，恢复射球。
    m_controller.a()
        .and(intakeButton.negate())
        .and(teleopEnabled)
        .whileTrue(
            Commands.runEnd(m_superstructure::shoot, m_superstructure::stop, m_superstructure)
                .withName("Shoot"));
  }

  public Command getAutonomousCommand() {
    // 每次创建独立组合，避免将已经组合的 command 重复添加到其他组合中。
    return Autos.teachingAuto(m_drive, m_superstructure);
  }

  /** 仅供模式切换时在取消命令之后调用；正常输出通过拥有 requirement 的 command 控制。 */
  public void stopAll() {
    m_drive.stop();
    m_superstructure.stop();
  }
}

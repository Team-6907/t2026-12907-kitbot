// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/** 负责机器人生命周期；硬件能力、手柄绑定和动作分别放在 subsystem、container 和 command 中。 */
public class Robot extends TimedRobot {
  private final RobotContainer m_robotContainer = new RobotContainer();

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {
    cancelCommandsAndStop();
    m_robotContainer.getAutonomousCommand().schedule();
  }

  @Override
  public void teleopInit() {
    cancelCommandsAndStop();
  }

  @Override
  public void disabledInit() {
    cancelCommandsAndStop();
  }

  @Override
  public void testInit() {
    cancelCommandsAndStop();
  }

  private void cancelCommandsAndStop() {
    // 先执行命令的 end() 清理，再显式停止硬件，避免模式切换时保留旧输出。
    CommandScheduler.getInstance().cancelAll();
    m_robotContainer.stopAll();
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SuperstructureConstants;

/** shooter 与 feeder 共同构成吸球/射球机构，由同一个 requirement 防止输出竞争。 */
public class SuperstructureSubsystem extends SubsystemBase {
  private final TalonFX m_feeder = new TalonFX(SuperstructureConstants.kFeederCanId);
  private final TalonFX m_shooter = new TalonFX(SuperstructureConstants.kShooterCanId);

  private final VelocityVoltage m_feederVelocityRequest = new VelocityVoltage(0.0);
  private final VelocityVoltage m_shooterVelocityRequest = new VelocityVoltage(0.0);
  private final VoltageOut m_feederLaunchRequest =
      new VoltageOut(SuperstructureConstants.kLaunchFeederVoltage);
  private final VoltageOut m_shooterLaunchRequest =
      new VoltageOut(SuperstructureConstants.kLaunchShooterVoltage);
  private final NeutralOut m_stopRequest = new NeutralOut();

  public SuperstructureSubsystem() {
    TalonFXConfiguration feederConfig = new TalonFXConfiguration();
    feederConfig.MotorOutput.Inverted = SuperstructureConstants.kFeederInverted;
    feederConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    feederConfig.Slot0.kS = SuperstructureConstants.kFeederKs;
    feederConfig.Slot0.kV = SuperstructureConstants.kFeederKv;
    feederConfig.Slot0.kP = SuperstructureConstants.kFeederKp;
    feederConfig.Slot0.kI = SuperstructureConstants.kFeederKi;
    feederConfig.Slot0.kD = SuperstructureConstants.kFeederKd;

    TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
    shooterConfig.MotorOutput.Inverted = SuperstructureConstants.kShooterInverted;
    shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    shooterConfig.Slot0.kS = SuperstructureConstants.kShooterKs;
    shooterConfig.Slot0.kV = SuperstructureConstants.kShooterKv;
    shooterConfig.Slot0.kP = SuperstructureConstants.kShooterKp;
    shooterConfig.Slot0.kI = SuperstructureConstants.kShooterKi;
    shooterConfig.Slot0.kD = SuperstructureConstants.kShooterKd;

    m_feeder.getConfigurator().apply(feederConfig);
    m_shooter.getConfigurator().apply(shooterConfig);
  }

  /** 保持原 X 按钮的速度闭环目标，单位为 rotations per second。 */
  public void intake() {
    m_shooter.setControl(
        m_shooterVelocityRequest.withVelocity(SuperstructureConstants.kIntakeShooterVelocityRps));
    m_feeder.setControl(
        m_feederVelocityRequest.withVelocity(SuperstructureConstants.kIntakeFeederVelocityRps));
  }

  /** 遥控 A 按钮与自动射球共用原来的开环电压设置。 */
  public void shoot() {
    m_shooter.setControl(m_shooterLaunchRequest);
    m_feeder.setControl(m_feederLaunchRequest);
  }

  public void stop() {
    m_shooter.setControl(m_stopRequest);
    m_feeder.setControl(m_stopRequest);
  }
}

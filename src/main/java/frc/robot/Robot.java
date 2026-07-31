// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  // 手柄在 Driver Station 里的端口号。通常第一个手柄是 0。
  private static final int kControllerPort = 0;

  // 下面这些是 CAN 总线上各个电机控制器的设备 ID，用来让代码找到对应硬件。
  // master 是每侧底盘的主电机控制器，follower 会跟随同侧 master 输出。
  // TODO: 确认每个 CAN ID 和真实机器人接线、Phoenix Tuner 里的配置一致。
  private static final int kLeftMasterCanId = 1;
  private static final int kLeftFollowerCanId = 2;
  private static final int kRightMasterCanId = 3;
  private static final int kRightFollowerCanId = 4;
  private static final int kFeederCanId = 5;
  private static final int kShooterCanId = 6;
  private static final int kIntakeCanId = 7;

  // feeder 和 shooter 的电机方向。Clockwise/CounterClockwise 表示正输出时传感器方向。
  private static final InvertedValue kFeederInverted = InvertedValue.Clockwise_Positive;
  private static final InvertedValue kShooterInverted = InvertedValue.CounterClockwise_Positive;

  // feeder 和 shooter 按键触发时的目标速度，单位是 rotations per second。
  // TODO: 装上真实机构和 game piece 后调节这两个目标转速。
  private static final double kFeederVelocityRps = 40.0;
  private static final double kShooterVelocityRps = 80.0;
  private static final double kIntakeVelocityRps = 60.0;

  // feeder 的速度闭环参数。kS/kV 是前馈，kP/kI/kD 是 PID 反馈。
  // TODO: 确认传感器单位和方向后，重新调节 feeder 的前馈和 PID 参数。
  private static final double kFeederKs = 0.20;
  private static final double kFeederKv = 0.12;
  private static final double kFeederKp = 0.15;
  private static final double kFeederKi = 0.0;
  private static final double kFeederKd = 0.0;

  // shooter 的速度闭环参数。shooter 转速稳定性通常会直接影响射出效果。
  // TODO: 安装 shooter 轮子后，重新调节 shooter 的前馈和 PID 参数。
  private static final double kShooterKs = 0.20;
  private static final double kShooterKv = 0.12;
  private static final double kShooterKp = 0.15;
  private static final double kShooterKi = 0.0;
  private static final double kShooterKd = 0.0;

  // intake 的速度闭环参数。
  // TODO: 安装 intake 后，重新调节 intake 的前馈和 PID 参数。
  private static final double kIntakeKs = 0.20;
  private static final double kIntakeKv = 0.12;
  private static final double kIntakeKp = 0.15;
  private static final double kIntakeKi = 0.0;
  private static final double kIntakeKd = 0.0;

  // 底盘电机对象。TalonSRX/VictorSPX 使用 Phoenix 5 的 WPI 封装，可以直接给 DifferentialDrive 用。
  private final WPI_TalonSRX m_leftMaster = new WPI_TalonSRX(kLeftMasterCanId);
  private final WPI_VictorSPX m_leftFollower = new WPI_VictorSPX(kLeftFollowerCanId);
  private final WPI_TalonSRX m_rightMaster = new WPI_TalonSRX(kRightMasterCanId);
  private final WPI_VictorSPX m_rightFollower = new WPI_VictorSPX(kRightFollowerCanId);

  // feeder 和 shooter 电机对象。这里用 Phoenix 6 的 TalonFX 控制速度闭环。
  private final TalonFX m_feeder = new TalonFX(kFeederCanId);
  private final TalonFX m_shooter = new TalonFX(kShooterCanId);
  private final TalonFX m_intake = new TalonFX(kIntakeCanId);

  // Phoenix 6 控制请求对象：VelocityVoltage 表示速度闭环，NeutralOut 表示停止输出。
  private final VelocityVoltage m_feederVelocityRequest = new VelocityVoltage(0.0);
  private final VelocityVoltage m_shooterVelocityRequest = new VelocityVoltage(0.0);
  private final VelocityVoltage m_intakeVelocityRequest = new VelocityVoltage(0.0);
  private final NeutralOut m_stopRequest = new NeutralOut();

  // DifferentialDrive 负责把 forward/rotation 转换成左右两侧底盘输出。
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMaster, m_rightMaster);

  // XboxController 负责读取手柄摇杆和按键。
  private final XboxController m_controller = new XboxController(kControllerPort);

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    // 构造函数只在机器人程序启动时运行一次，适合做硬件初始化和参数下发。

    // 先把 Phoenix 5 底盘控制器恢复默认配置，减少旧配置影响当前代码。
    m_leftMaster.configFactoryDefault();
    m_leftFollower.configFactoryDefault();
    m_rightMaster.configFactoryDefault();
    m_rightFollower.configFactoryDefault();

    // 创建 feeder 的 TalonFX 配置，包括方向、刹车模式和速度闭环参数。
    TalonFXConfiguration feederConfig = new TalonFXConfiguration();
    feederConfig.MotorOutput.Inverted = kFeederInverted;
    feederConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    feederConfig.Slot0.kS = kFeederKs;
    feederConfig.Slot0.kV = kFeederKv;
    feederConfig.Slot0.kP = kFeederKp;
    feederConfig.Slot0.kI = kFeederKi;
    feederConfig.Slot0.kD = kFeederKd;

    // 创建 shooter 的 TalonFX 配置。shooter 使用 Coast，停止输出后可以自然滑行。
    TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
    shooterConfig.MotorOutput.Inverted = kShooterInverted;
    shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    shooterConfig.Slot0.kS = kShooterKs;
    shooterConfig.Slot0.kV = kShooterKv;
    shooterConfig.Slot0.kP = kShooterKp;
    shooterConfig.Slot0.kI = kShooterKi;
    shooterConfig.Slot0.kD = kShooterKd;

    // 把上面创建的配置真正写入 TalonFX 控制器。
    m_feeder.getConfigurator().apply(feederConfig);
    m_shooter.getConfigurator().apply(shooterConfig);

    // 创建 intake 的 TalonFX 配置。
    TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
    intakeConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    intakeConfig.Slot0.kS = kIntakeKs;
    intakeConfig.Slot0.kV = kIntakeKv;
    intakeConfig.Slot0.kP = kIntakeKp;
    intakeConfig.Slot0.kI = kIntakeKi;
    intakeConfig.Slot0.kD = kIntakeKd;

    // 把 intake 配置写入 TalonFX 控制器。
    m_intake.getConfigurator().apply(intakeConfig);

    // 设置底盘 follower，让每侧副电机自动跟随同侧主电机。
    m_leftFollower.follow(m_leftMaster);
    m_rightFollower.follow(m_rightMaster);

    // 反转右侧底盘，因为差速底盘左右电机通常镜像安装。
    m_rightMaster.setInverted(true);
    m_rightFollower.setInverted(true);

    // 底盘设置为 Brake，松开摇杆时更快停下，也更不容易被推动。
    m_leftMaster.setNeutralMode(NeutralMode.Brake);
    m_leftFollower.setNeutralMode(NeutralMode.Brake);
    m_rightMaster.setNeutralMode(NeutralMode.Brake);
    m_rightFollower.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    // teleopPeriodic 在遥控阶段每 20ms 左右运行一次，用来持续读取手柄并控制机器人。

    // 读取左摇杆 Y 轴作为前后速度，读取右摇杆 X 轴作为转向速度。
    // 这里加负号是为了让“摇杆向上”对应机器人前进。
    double forward = -m_controller.getLeftY();
    double rotation = -m_controller.getRightX();

    // arcadeDrive 用一个前后量和一个旋转量控制差速底盘。
    m_drive.arcadeDrive(forward, rotation);

    // 按住 B 键时 feeder 按目标速度运行，松开 B 键时 feeder 停止。
    if (m_controller.getBButton()) {
      m_feeder.setControl(m_feederVelocityRequest.withVelocity(kFeederVelocityRps));
    } else {
      m_feeder.setControl(m_stopRequest);
    }

    // 按住 A 键时 shooter 按目标速度运行，松开 A 键时 shooter 停止。
    if (m_controller.getAButton()) {
      m_shooter.setControl(m_shooterVelocityRequest.withVelocity(kShooterVelocityRps));
    } else {
      m_shooter.setControl(m_stopRequest);
    }

    // 按住 X 键时 intake 按目标速度运行，松开 X 键时 intake 停止。
    if (m_controller.getXButton()) {
      m_intake.setControl(m_intakeVelocityRequest.withVelocity(kIntakeVelocityRps));
    } else {
      m_intake.setControl(m_stopRequest);
    }
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
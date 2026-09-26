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
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
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

  // feeder 和 shooter 的电机方向。Clockwise/CounterClockwise 表示正输出时传感器方向。
  private static final InvertedValue kFeederInverted = InvertedValue.CounterClockwise_Positive;
  private static final InvertedValue kShooterInverted = InvertedValue.Clockwise_Positive;

  // feeder, shooter 和 intake 按键触发时的目标速度，单位是 rotations per second。
  private static final double kLaunchShooterVoltage = 11.0;  //TODO: Tune
  private static final double kLaunchFeederVoltage = 9.0;  //TODO: Tune
  private static final double kLaunchShooterVelocityRps = 50.0;  //TODO: Tune
  private static final double kLaunchFeederVelocityRps = 30.0;  //TODO: Tune
  private static final double kFeederVelocityRps = 40.0;
  private static final double kShooterTowerVelocityRps = 20.0;
  private static final double kIntakeShooterVelocityRps = 10.0;  //TODO: Tune
  private static final double kIntakeFeederVelocityRps = -10.0;  //TODO: Tune

  // 教学自动全程开环：输出是 [-1, 1] 的比例，不是 m/s，也不读取距离或航向反馈。
  private static final double kAutoDriveOutput = 0.35;
  private static final double kAutoTurnOutput = 0.30;
  private static final double kAutoArcOuterOutput = 0.35;

  // TODO: 以下移动时长只是未标定的初始估计，不能保证达到名称中的距离或角度。
  // 调整输出后必须重新标定时长；地面、电池电压、负载和轮胎打滑也会改变结果。
  // 各段初值合计 18.1 s，不含周期切换误差；标定后需重新核对自动阶段时间预算。
  private static final double kAutoDrive2MetersSeconds = 1.4;
  private static final double kAutoTurnCcw90Seconds = 0.6;
  private static final double kAutoDrive4MetersSeconds = 2.8;
  private static final double kAutoTurnCw180Seconds = 1.2;
  private static final double kAutoArcCw90Seconds = 3.2;
  private static final double kAutoTurnCw135Seconds = 0.9;
  private static final double kAutoWaitAtASeconds = 3.0;
  private static final double kAutoShootSeconds = 5.0;

  // 圆弧半径以底盘中心为参考；24.75 in 换算为 0.62865 m。
  private static final double kAutoTrackWidthMeters = 24.75 * 0.0254;
  private static final double kAutoArcRadiusMeters = 3.0;
  // 顺时针前进时左轮在外侧。理想轮速比约 0.8103；输出比仍需实车标定。
  private static final double kAutoArcInnerToOuterRatio =
      (kAutoArcRadiusMeters - kAutoTrackWidthMeters / 2.0)
          / (kAutoArcRadiusMeters + kAutoTrackWidthMeters / 2.0);
  private static final double kAutoArcInnerOutput =
      kAutoArcOuterOutput * kAutoArcInnerToOuterRatio;

  // 蓝方固定坐标：+X 离开蓝方联盟墙，+Y 为蓝方驾驶站视角的左侧，逆时针为正。
  // 下面的位置均为相对起点的理想位移，不是测得的场地位置。
  private enum AutoStep {
    DRIVE_FORWARD_2M(kAutoDrive2MetersSeconds, kAutoDriveOutput, kAutoDriveOutput, false),
    TURN_CCW_90(kAutoTurnCcw90Seconds, -kAutoTurnOutput, kAutoTurnOutput, false),
    DRIVE_POSITIVE_Y_4M(kAutoDrive4MetersSeconds, kAutoDriveOutput, kAutoDriveOutput, false),
    // A = (2, 4)，理想航向 90°。
    WAIT_AT_A(kAutoWaitAtASeconds, 0.0, 0.0, false),
    TURN_CW_180(kAutoTurnCw180Seconds, kAutoTurnOutput, -kAutoTurnOutput, false),
    // 从 A 朝 -Y 前进并顺时针走四分之一圆：到 (-1, 1)，理想航向 -180°。
    // 终点比起点更靠近蓝方联盟墙 1 m；摆放起点时需留出车身及路径的边界余量。
    ARC_CW_90(kAutoArcCw90Seconds, kAutoArcOuterOutput, kAutoArcInnerOutput, false),
    TURN_CW_135(kAutoTurnCw135Seconds, kAutoTurnOutput, -kAutoTurnOutput, false),
    // -180° - 135° = -315°，等价于 45°。射球时底盘停止。
    SHOOT(kAutoShootSeconds, 0.0, 0.0, true),
    DONE(0.0, 0.0, 0.0, false);

    final double durationSeconds;
    final double leftOutput;
    final double rightOutput;
    final boolean shoot;

    AutoStep(double durationSeconds, double leftOutput, double rightOutput, boolean shoot) {
      this.durationSeconds = durationSeconds;
      this.leftOutput = leftOutput;
      this.rightOutput = rightOutput;
      this.shoot = shoot;
    }
  }

  private static final AutoStep[] kAutoSequence = AutoStep.values();

  // feeder 的速度闭环参数。kS/kV 是前馈，kP/kI/kD 是 PID 反馈。
  // TODO: 确认传感器单位和方向后，重新调节 feeder 的前馈和 PID 参数。
  private static final double kFeederKs = 0.20;
  private static final double kFeederKv = 0.12;
  private static final double kFeederKp = 0.15;
  private static final double kFeederKi = 0.0;
  private static final double kFeederKd = 0.0;

  // shooter 的速度闭环参数。shooter 转速稳定性通常会直接影响射出效果。
  // TODO: 安装 shooter 轮子后，重新调节 shooter 的前馈和 PID 参数。\[]
  private static final double kShooterKs = 0.20;
  private static final double kShooterKv = 0.5;
  private static final double kShooterKp = 0.7;
  private static final double kShooterKi = 0.0;
  private static final double kShooterKd = 0.0;

  // 底盘电机对象。TalonSRX/VictorSPX 使用 Phoenix 5 的 WPI 封装，可以直接给 DifferentialDrive 用。
  private final WPI_TalonSRX m_leftMaster = new WPI_TalonSRX(kLeftMasterCanId);
  private final WPI_VictorSPX m_leftFollower = new WPI_VictorSPX(kLeftFollowerCanId);
  private final WPI_TalonSRX m_rightMaster = new WPI_TalonSRX(kRightMasterCanId);
  private final WPI_VictorSPX m_rightFollower = new WPI_VictorSPX(kRightFollowerCanId);

  // feeder 和 shooter 电机对象。这里用 Phoenix 6 的 TalonFX 控制速度闭环。
  private final TalonFX m_feeder = new TalonFX(kFeederCanId);
  private final TalonFX m_shooter = new TalonFX(kShooterCanId);

  // Phoenix 6 控制请求对象：VelocityVoltage 表示速度闭环，NeutralOut 表示停止输出。
  private final VelocityVoltage m_feederVelocityRequest = new VelocityVoltage(0.0);
  private final VelocityVoltage m_shooterVelocityRequest = new VelocityVoltage(0.0);
  private final NeutralOut m_stopRequest = new NeutralOut();

  // DifferentialDrive 负责把 forward/rotation 转换成左右两侧底盘输出。
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMaster, m_rightMaster);

  // XboxController 负责读取手柄摇杆和按键。
  private final XboxController m_controller = new XboxController(kControllerPort);

  private final Timer m_autoStepTimer = new Timer();
  private AutoStep m_autoStep = AutoStep.DONE;
  // 自动射球沿用遥控 A 按钮的电压，复用请求，避免在周期循环中创建对象。
  private final VoltageOut m_autoShooterRequest = new VoltageOut(kLaunchShooterVoltage);
  private final VoltageOut m_autoFeederRequest = new VoltageOut(kLaunchFeederVoltage);

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
  public void autonomousInit() {
    stopAutonomous();
    // 保持圆弧左右输出比例；退出自动时恢复默认死区，不影响遥控手感。
    m_drive.setDeadband(0.0);
    m_autoStep = AutoStep.DRIVE_FORWARD_2M;
    m_autoStepTimer.restart();
  }

  @Override
  public void autonomousPeriodic() {
    if (m_autoStep != AutoStep.DONE
        && m_autoStepTimer.hasElapsed(m_autoStep.durationSeconds)) {
      m_autoStep = kAutoSequence[m_autoStep.ordinal() + 1];
      // 每段从真正切换输出的时刻重新计时，不补跑或跳过因循环延迟错过的动作。
      m_autoStepTimer.restart();
    }

    if (m_autoStep == AutoStep.DONE) {
      stopAutonomous();
      return;
    }

    // 每周期更新输出以喂 MotorSafety；false 关闭输入平方，保留圆弧输出比例。
    // 左负右正为逆时针原地转，左正右负为顺时针原地转。
    m_drive.tankDrive(m_autoStep.leftOutput, m_autoStep.rightOutput, false);
    if (m_autoStep.shoot) {
      m_shooter.setControl(m_autoShooterRequest);
      m_feeder.setControl(m_autoFeederRequest);
    } else {
      m_shooter.setControl(m_stopRequest);
      m_feeder.setControl(m_stopRequest);
    }
  }

  private void stopAutonomous() {
    m_autoStep = AutoStep.DONE;
    m_autoStepTimer.stop();
    m_drive.stopMotor();
    m_drive.setDeadband(DifferentialDrive.kDefaultDeadband);
    m_shooter.setControl(m_stopRequest);
    m_feeder.setControl(m_stopRequest);
  }

  @Override
  public void teleopInit() {
    stopAutonomous();
  }

  @Override
  public void teleopPeriodic() {
    // teleopPeriodic 在遥控阶段每 20ms 左右运行一次，用来持续读取手柄并控制机器人。

    // 读取左摇杆 Y 轴作为前后速度，读取右摇杆 X 轴作为转向速度。
    // 这里加负号是为了让“摇杆向上”对应机器人前进。
    double forward = -m_controller.getLeftY();
    double rotation = -m_controller.getRightX();

    // arcadeDrive 用一个前后量和一个旋转量控制差速底盘。
    m_drive.arcadeDrive(forward, rotation);

    if (m_controller.getXButton()) {
      m_shooter.setControl(m_shooterVelocityRequest.withVelocity(kIntakeShooterVelocityRps));
      m_feeder.setControl(m_feederVelocityRequest.withVelocity(kIntakeFeederVelocityRps));
    }
    else if (m_controller.getAButton()) {
      m_shooter.setControl(new VoltageOut(kLaunchShooterVoltage));
      m_feeder.setControl(new VoltageOut(kLaunchFeederVoltage));
    }
    else {
      m_shooter.setControl(m_stopRequest);
      m_feeder.setControl(m_stopRequest);
    }
  }

  @Override
  public void disabledInit() {
    stopAutonomous();
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {
    stopAutonomous();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}

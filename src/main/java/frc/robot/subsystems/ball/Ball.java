package frc.robot.subsystems.ball;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

import static com.revrobotics.PersistMode.kPersistParameters;
import static com.revrobotics.ResetMode.kResetSafeParameters;
import static com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kCoast;
import static frc.robot.subsystems.ball.BallConstants.*;

public class Ball extends SubsystemBase {
  private final static Ball instance = new Ball();
  public static Ball getInstance() {
    return instance;
  }

  private final SparkMax beltMotor = new SparkMax(23, SparkLowLevel.MotorType.kBrushless);
  private final SparkMax indexerMotor = new SparkMax(22, SparkLowLevel.MotorType.kBrushless);

  private final SparkFlex intakeMotor = new SparkFlex(25, SparkLowLevel.MotorType.kBrushless);
  private final SparkClosedLoopController intakeController;
  private final SparkMax pivotMotor = new SparkMax(26, SparkLowLevel.MotorType.kBrushless);
  private final SparkClosedLoopController pivotController;

  private final SparkFlex shooterMotor = new SparkFlex(32, SparkLowLevel.MotorType.kBrushless);
  private final SparkFlex shooterMotor2 = new SparkFlex(33, SparkLowLevel.MotorType.kBrushless);
  private final RelativeEncoder shooterEncoder;
  private final SparkClosedLoopController shooterController;

  private Ball() {
    SparkFlexConfig baseShooterConfig = new SparkFlexConfig();
    baseShooterConfig.voltageCompensation(12.0);
    baseShooterConfig.smartCurrentLimit(40).idleMode(kCoast);
    baseShooterConfig.closedLoop.pid(0.002, 0, 0, ClosedLoopSlot.kSlot0);
    baseShooterConfig.closedLoop.feedForward.kV(0.00022, ClosedLoopSlot.kSlot0);
    baseShooterConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    baseShooterConfig.closedLoop.maxMotion.maxAcceleration(12000).allowedProfileError(1000);
    baseShooterConfig.openLoopRampRate(0.5);
    baseShooterConfig.closedLoopRampRate(0.5);
    baseShooterConfig.encoder.velocityConversionFactor(1);
    baseShooterConfig
        .encoder
        .uvwAverageDepth(2)
        .quadratureAverageDepth(2)
        .uvwMeasurementPeriod(10)
        .quadratureMeasurementPeriod(10);
    SparkFlexConfig flywheel1Config = new SparkFlexConfig();
    flywheel1Config.apply(baseShooterConfig).inverted(true);
    flywheel1Config.signals.appliedOutputPeriodMs(5);
    shooterMotor.configure(flywheel1Config, kResetSafeParameters, kPersistParameters);
    shooterEncoder = shooterMotor.getEncoder();
    shooterController = shooterMotor.getClosedLoopController();

    SparkFlexConfig flywheel2Config = new SparkFlexConfig();
    flywheel2Config.apply(baseShooterConfig).inverted(false);
    flywheel2Config.follow(32, true);
    shooterMotor2.configure(flywheel2Config, kResetSafeParameters, kPersistParameters);

    SparkMaxConfig beltConfig = new SparkMaxConfig();
    beltConfig.smartCurrentLimit(40).idleMode(SparkBaseConfig.IdleMode.kBrake).inverted(false);
    beltConfig.openLoopRampRate(.2);
    beltMotor.configure(beltConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig indexerConfig = new SparkMaxConfig();
    indexerConfig.smartCurrentLimit(40).idleMode(SparkBaseConfig.IdleMode.kBrake).inverted(false);
    indexerConfig.inverted(true);
    indexerConfig.openLoopRampRate(.2);
    indexerMotor.configure(
        indexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkFlexConfig intakeConfig = new SparkFlexConfig();
    intakeConfig.smartCurrentLimit(60).idleMode(SparkBaseConfig.IdleMode.kBrake).inverted(false);

    intakeConfig.closedLoop.pid(overBumperP, overBumperI, overBumperD, ClosedLoopSlot.kSlot0);
    intakeConfig.closedLoop.feedForward.kV(overBumperV, ClosedLoopSlot.kSlot0);
    intakeConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    intakeConfig.closedLoopRampRate(.2);
    intakeConfig.openLoopRampRate(.2);
    intakeMotor.configure(
        intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    intakeController = intakeMotor.getClosedLoopController();

    SparkMaxConfig pivotConfig = new SparkMaxConfig();
    pivotConfig.smartCurrentLimit(40).idleMode(SparkBaseConfig.IdleMode.kCoast).inverted(false);
    pivotConfig.absoluteEncoder.inverted(false);
    pivotConfig.inverted(true);
    pivotConfig.closedLoop.pid(
        overBumperPivotP, overBumperPivotI, overBumperPivotD, ClosedLoopSlot.kSlot0);
    pivotConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
    pivotConfig.closedLoop.outputRange(-.5, .5, ClosedLoopSlot.kSlot0);
    pivotConfig
        .closedLoop
        .maxMotion
        .allowedProfileError(.5, ClosedLoopSlot.kSlot0)
        .cruiseVelocity(.2, ClosedLoopSlot.kSlot0)
        .maxAcceleration(1, ClosedLoopSlot.kSlot0);
    pivotConfig.closedLoop.feedForward.kCos(.3, ClosedLoopSlot.kSlot0);

    pivotConfig.closedLoop.pid(
        overBumperPivotP, overBumperPivotI, overBumperPivotD, ClosedLoopSlot.kSlot1);
    pivotConfig.closedLoop.outputRange(-.5, .5, ClosedLoopSlot.kSlot1);
    pivotConfig
        .closedLoop
        .maxMotion
        .allowedProfileError(.5, ClosedLoopSlot.kSlot1)
        .cruiseVelocity(.05, ClosedLoopSlot.kSlot1)
        .maxAcceleration(1, ClosedLoopSlot.kSlot1);
    pivotConfig.closedLoop.feedForward.kCos(.3, ClosedLoopSlot.kSlot1);

    pivotMotor.configure(
        pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    pivotController = pivotMotor.getClosedLoopController();

  }

  private double targetFlywheelSpeed = 0;

  public void runFlywheel(double speed) {
    targetFlywheelSpeed = speed;
    shooterController.setSetpoint(targetFlywheelSpeed, SparkBase.ControlType.kVelocity, ClosedLoopSlot.kSlot0);
  }

  public void stopFlywheel() {
    targetFlywheelSpeed = 0;
    shooterMotor.set(0);
  }

  public boolean isFlywheelAtSpeed() {
    return Math.abs(shooterEncoder.getVelocity() - targetFlywheelSpeed) < Math.max(100, targetFlywheelSpeed * 0.1);
  }

  // shooting/intaking is positive, outtaking is negative
  public void runIndexerAndBelt(double speed) {
    speed = MathUtil.clamp(speed, -1, 1);
    beltMotor.set(speed);
    indexerMotor.set(speed);
//    System.out.println("running indexer and belt at " + speed);
  }

  public void runIntake(double speed) {
    SmartDashboard.putNumber("intake setpoint", speed);
//    intakeController.setSetpoint(speed, SparkBase.ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    intakeMotor.set(speed/intakeSpeed);
  }

  public void setIntakePosition(double position) {
    pivotController.setSetpoint(position, SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  public void setIntakePositionSlowly(double position) {
    pivotController.setSetpoint(position, SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot1);
  }

  public void stopIntake() {
    intakeMotor.set(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("intake speed", intakeMotor.getEncoder().getVelocity());
  }
}

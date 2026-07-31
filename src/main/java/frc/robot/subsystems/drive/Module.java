package frc.robot.subsystems.drive;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.testmode.SparkTestMotor;
import frc.robot.testmode.TestMotor;

import java.util.List;

import static frc.robot.subsystems.drive.DriveConstants.*;

public class Module {
  private final SparkFlex driveMotor;
  private final SparkClosedLoopController driveController;
  private final RelativeEncoder driveEncoder;

  private final SparkMax turnMotor;
  private final SparkClosedLoopController turnController;
  private final AbsoluteEncoder turnEncoder;

  private final double zeroRotation;

  public Module(int moduleId) {
    zeroRotation = switch (moduleId) {
      case 0 -> frontLeftZeroRotation;
      case 1 -> frontRightZeroRotation;
      case 2 -> backLeftZeroRotation;
      case 3 -> backRightZeroRotation;
      default -> 0;
    };
    int driveMotorId = switch (moduleId) {
      case 0 -> frontLeftDriveCanId;
      case 1 -> frontRightDriveCanId;
      case 2 -> backLeftDriveCanId;
      case 3 -> backRightDriveCanId;
      default -> 0;
    };
    int turnMotorId = switch (moduleId) {
      case 0 -> frontLeftTurnCanId;
      case 1 -> frontRightTurnCanId;
      case 2 -> backLeftTurnCanId;
      case 3 -> backRightTurnCanId;
      default -> 0;
    };

    driveMotor = new SparkFlex(driveMotorId, SparkLowLevel.MotorType.kBrushless);
    turnMotor = new SparkMax(turnMotorId, SparkMax.MotorType.kBrushless);

    SparkFlexConfig driveConfig = new SparkFlexConfig();
    driveConfig
        .idleMode(SparkBaseConfig.IdleMode.kBrake)
        .smartCurrentLimit(driveMotorCurrentLimit)
        .voltageCompensation(12.0);
    driveConfig
        .encoder
        .positionConversionFactor(driveEncoderPositionFactor)
        .velocityConversionFactor(driveEncoderVelocityFactor)
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);
    driveConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(driveKp, 0.0, driveKd);
    driveConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderVelocityAlwaysOn(true);

    SparkMaxConfig turnConfig = new SparkMaxConfig();
    turnConfig
        .inverted(turnInverted)
        .idleMode(SparkBaseConfig.IdleMode.kBrake)
        .smartCurrentLimit(turnMotorCurrentLimit)
        .voltageCompensation(12.0);
    turnConfig
        .absoluteEncoder
        .inverted(turnEncoderInverted)
        .positionConversionFactor(turnEncoderPositionFactor)
        .velocityConversionFactor(turnEncoderVelocityFactor)
        .averageDepth(2);
    turnConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(turnPIDMinInput, turnPIDMaxInput)
        .pid(turnKp, 0.0, turnKd);
    turnConfig
        .signals
        .absoluteEncoderPositionAlwaysOn(true)
        .absoluteEncoderVelocityAlwaysOn(true);

    driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    turnMotor.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    driveController = driveMotor.getClosedLoopController();
    driveEncoder = driveMotor.getEncoder();
    driveEncoder.setPosition(0.0);

    turnController = turnMotor.getClosedLoopController();
    turnEncoder = turnMotor.getAbsoluteEncoder();
  }

  public void setDriveVelocity(double velocityRadPerSec) {
    double ffVolts = driveKs * Math.signum(velocityRadPerSec) + driveKv * velocityRadPerSec;
    driveController.setSetpoint(
        velocityRadPerSec,
        SparkBase.ControlType.kVelocity,
        ClosedLoopSlot.kSlot0,
        ffVolts,
        SparkClosedLoopController.ArbFFUnits.kVoltage);
  }

  public double getDriveVelocity() {
    return driveEncoder.getVelocity();
  }

  public double getDrivePosition() {
    return driveEncoder.getPosition();
  }

  public void setTurnPosition(double rotation) {
    double setpoint =
        MathUtil.inputModulus(
            rotation + zeroRotation, turnPIDMinInput, turnPIDMaxInput);
    turnController.setSetpoint(setpoint, SparkBase.ControlType.kPosition);
  }

  public double getTurnPosition() {
    return turnEncoder.getPosition() - zeroRotation;
  }

  public void runSetpoint(SwerveModuleState state) {
    // Optimize velocity setpoint
    state.optimize(new Rotation2d(getTurnPosition()));
    state.cosineScale(new Rotation2d(getTurnPosition()));

    // Apply setpoints
    setDriveVelocity(state.speedMetersPerSecond / wheelRadiusMeters);
    setTurnPosition(state.angle.getRadians());
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getDrivePosition() * wheelRadiusMeters, new Rotation2d(getTurnPosition()));
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocity() * wheelRadiusMeters, new Rotation2d(getTurnPosition()));
  }

  public List<TestMotor> getTestMotors(String moduleName) {
    return List.of(
        new SparkTestMotor(moduleName + " drive", driveMotor),
        new SparkTestMotor(moduleName + " steer", turnMotor));
  }
}

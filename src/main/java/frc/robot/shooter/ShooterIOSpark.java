package frc.robot.shooter;

import static com.revrobotics.PersistMode.*;
import static com.revrobotics.ResetMode.*;
import static com.revrobotics.spark.SparkLowLevel.MotorType.*;
import static com.revrobotics.spark.config.SparkBaseConfig.IdleMode.*;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Servo;

public class ShooterIOSpark implements ShooterIO {
  private final Servo linearActuator;
  private final SparkFlex panMotor;
  private final AbsoluteEncoder panEncoder;
  private final SparkClosedLoopController panController;
  private final SparkFlex flywheelMotor;
  private final RelativeEncoder flywheelEncoder;
  private final SparkClosedLoopController flywheelController;
  private final SparkFlex flywheelMotor2;
  private final RelativeEncoder flywheelEncoder2;
  private final SparkClosedLoopController flywheelController2;
  private final int panMotorCanId = 31;
  private final int flywheelMotorCanId = 32;
  private final int flywheelMotor2CanId = 33;
  private final int linearActuatorServoPwmId = 0;
  private final double linearActuatorMinExtension = 15;
  private final double linearActuatorLengthMm = 100;
  private final double linearActuatorSpeedMmPerSec = 32;
  private double linearActuatorSetpoint = 0;

  public ShooterIOSpark() {
    linearActuator = new Servo(linearActuatorServoPwmId);
    linearActuator.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);

    SparkFlexConfig panConfig = new SparkFlexConfig();
    panConfig.smartCurrentLimit(40).idleMode(kBrake).inverted(false);
    panConfig.closedLoop.pid(0, 0, 0, ClosedLoopSlot.kSlot0);
    panMotor = new SparkFlex(panMotorCanId, kBrushless);
    panMotor.configure(panConfig, kResetSafeParameters, kPersistParameters);
    panEncoder = panMotor.getAbsoluteEncoder();
    panController = panMotor.getClosedLoopController();

    SparkFlexConfig flywheelConfig = new SparkFlexConfig();
    flywheelConfig.smartCurrentLimit(40).idleMode(kCoast).inverted(true);
    flywheelConfig.closedLoop.pid(0.0002, 0, 0, ClosedLoopSlot.kSlot0);
    flywheelConfig.closedLoop.feedForward.kV(.0016);
    flywheelConfig.openLoopRampRate(0.2);
    flywheelConfig.closedLoopRampRate(0.2);
    flywheelMotor = new SparkFlex(flywheelMotorCanId, kBrushless);
    flywheelMotor.configure(flywheelConfig, kResetSafeParameters, kPersistParameters);
    flywheelEncoder = flywheelMotor.getEncoder();
    flywheelController = flywheelMotor.getClosedLoopController();

    SparkFlexConfig flywheel2Config = new SparkFlexConfig();
    flywheel2Config.smartCurrentLimit(40).idleMode(kCoast).inverted(false);
    flywheel2Config.closedLoop.pid(0.0002, 0, 0, ClosedLoopSlot.kSlot0);
    flywheel2Config.closedLoop.feedForward.kV(.0016);
    flywheel2Config.openLoopRampRate(0.2);
    flywheel2Config.closedLoopRampRate(0.2);
    flywheelMotor2 = new SparkFlex(flywheelMotor2CanId, kBrushless);
    flywheelMotor2.configure(flywheel2Config, kResetSafeParameters, kPersistParameters);
    flywheelEncoder2 = flywheelMotor2.getEncoder();
    flywheelController2 = flywheelMotor2.getClosedLoopController();
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    double flywheelVelocity1 = flywheelEncoder.getVelocity();
    double flywheelVelocity2 = flywheelEncoder2.getVelocity();
    if (!MathUtil.isNear(flywheelVelocity1, flywheelVelocity2, 250)) {
      System.out.println(
          "[WARNING] Flywheel velocity mismatch: "
              + flywheelVelocity1
              + " vs "
              + flywheelVelocity2);
    }
    inputs.flywheelVelocityRotationsPerMinute = (flywheelVelocity1 + flywheelVelocity2) / 2;
    //        inputs.turretPitchRadians = tiltMotor.getAbsoluteEncoder().getPosition();
    inputs.turretYawRadians = panEncoder.getPosition();
    inputs.linearActuatorSetpointMm = ((linearActuatorSetpoint + 1) / 2) * linearActuatorLengthMm;
  }

  @Override
  public void setFlywheelVelocity(double speedRotationsPerMinute) {
    flywheelController.setSetpoint(
        (9d / 13) * speedRotationsPerMinute, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    flywheelController2.setSetpoint(
        (9d / 13) * speedRotationsPerMinute, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
  }

  @Override
  public void setTurretAngle(double yawRadians, double pitchRadians) {
    setTurretYaw(yawRadians);
    setTurretPitch(pitchRadians);
  }

  @Override
  public void setTurretPitch(double pitchRadians) {}

  @Override
  public void setTurretYaw(double yawRadians) {
    panController.setSetpoint(yawRadians, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  @Override
  public void setTurretPitchOpenLoop(double voltage) {
    double setpoint = MathUtil.clamp(voltage / 12, -1, 1);
    double setpointSpeed = setpoint / 100;
    double newSetpoint =
        MathUtil.clamp(
            linearActuatorSetpoint + setpointSpeed,
            -1 + (2 * linearActuatorMinExtension / linearActuatorLengthMm),
            1);
    linearActuatorSetpoint = newSetpoint;
    linearActuator.setSpeed(newSetpoint);
  }

  @Override
  public void setTurretYawOpenLoop(double voltage) {
    panMotor.setVoltage(voltage);
  }

  @Override
  public void setFlywheelOpenLoop(double voltage) {
    flywheelMotor.setVoltage(voltage);
    flywheelMotor2.setVoltage(voltage);
  }
}

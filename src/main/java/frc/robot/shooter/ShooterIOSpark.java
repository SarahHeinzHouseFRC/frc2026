package frc.robot.shooter;

import static com.revrobotics.PersistMode.*;
import static com.revrobotics.ResetMode.*;
import static com.revrobotics.spark.SparkLowLevel.MotorType.*;
import static com.revrobotics.spark.config.SparkBaseConfig.IdleMode.*;
import static frc.robot.shooter.ShooterConstants.*;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class ShooterIOSpark implements ShooterIO {
  private final Servo linearActuator;
  private final Servo linearActuator2;

  private double turretYawSetpointRadians;

  private final SparkMax panMotor;
  private AbsoluteEncoder panEncoder26;
  private AbsoluteEncoder panEncoder28 = null;
  private final RelativeEncoder panEncoderRelative;
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
  private final int linearActuatorServoPwmId2 = 1;

  private final double linearActuatorMinExtension =
      switch (Robot.VERSION) {
        case V1 -> 15;
        case V2 -> 0;
      };
  private final double linearActuatorLengthMm = 100;
  private final double linearActuatorSpeedMmPerSec = 32;
  private double linearActuatorSetpoint = 0;
  private double linearActuatorEstimatedPosition = 0;

  private final LoggedNetworkNumber tunableP;
  private final LoggedNetworkNumber tunableI;
  private final LoggedNetworkNumber tunableD;
  private final LoggedNetworkNumber tunableV;
  private final LoggedNetworkBoolean flywheel1On;
  private final LoggedNetworkBoolean flywheel2On;
  private final PIDController flywheelPID = new PIDController(flywheelP, flywheelI, flywheelD);

  private final LoggedNetworkNumber yawTunableP;
  private final LoggedNetworkNumber yawTunableI;
  private final LoggedNetworkNumber yawTunableD;
  private final PIDController yawPID = new PIDController(yawP, yawI, yawD);

  public ShooterIOSpark() {
    linearActuator = new Servo(linearActuatorServoPwmId);
    linearActuator.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);

    linearActuator2 = new Servo(linearActuatorServoPwmId2);
    linearActuator2.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);

    SparkMaxConfig panConfig = new SparkMaxConfig();
    panConfig.smartCurrentLimit(10).idleMode(kCoast).inverted(true);
    panConfig.encoder.positionConversionFactor(2 * Math.PI / yawReduction);
    //    panConfig.softLimit.forwardSoftLimit(yawMax);
    //    panConfig.softLimit.reverseSoftLimit(yawMin);
    //    panConfig.softLimit.forwardSoftLimitEnabled(true);
    //    panConfig.softLimit.reverseSoftLimitEnabled(true);
    panConfig.absoluteEncoder.positionConversionFactor(2 * Math.PI);
    panConfig.absoluteEncoder.velocityConversionFactor(2 * Math.PI / 60);
    panConfig.absoluteEncoder.inverted(true);
    panConfig.closedLoop.pid(yawP, yawI, yawD, ClosedLoopSlot.kSlot0);
    panConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    panConfig.closedLoop.outputRange(-maxYawVolts / 12, maxYawVolts / 12, ClosedLoopSlot.kSlot0);
    panConfig.closedLoopRampRate(0.2);
    panMotor = new SparkMax(panMotorCanId, kBrushless);
    panMotor.configure(panConfig, kResetSafeParameters, kPersistParameters);
    panEncoderRelative = panMotor.getEncoder();
    panController = panMotor.getClosedLoopController();

    SparkFlexConfig baseFlywheelConfig = new SparkFlexConfig();
    baseFlywheelConfig.voltageCompensation(12.0);
    baseFlywheelConfig.smartCurrentLimit(60).idleMode(kCoast);
    baseFlywheelConfig.closedLoop.pid(flywheelP, flywheelI, flywheelD, ClosedLoopSlot.kSlot0);
    baseFlywheelConfig.closedLoop.feedForward.kV(flywheelV, ClosedLoopSlot.kSlot0);
    baseFlywheelConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    baseFlywheelConfig.closedLoop.maxMotion.maxAcceleration(12000).allowedProfileError(1000);
    baseFlywheelConfig.openLoopRampRate(0.2);
    baseFlywheelConfig.closedLoopRampRate(0.2);
    baseFlywheelConfig.encoder.velocityConversionFactor(motorToFlywheel);
    baseFlywheelConfig
        .encoder
        .uvwAverageDepth(2)
        .quadratureAverageDepth(2)
        .uvwMeasurementPeriod(10)
        .quadratureMeasurementPeriod(10);
    SparkFlexConfig flywheel1Config = new SparkFlexConfig();
    flywheel1Config.apply(baseFlywheelConfig).inverted(true);
    flywheelMotor = new SparkFlex(flywheelMotorCanId, kBrushless);
    flywheel1Config.signals.appliedOutputPeriodMs(5);
    flywheelMotor.configure(flywheel1Config, kResetSafeParameters, kPersistParameters);
    flywheelEncoder = flywheelMotor.getEncoder();
    flywheelController = flywheelMotor.getClosedLoopController();

    SparkFlexConfig flywheel2Config = new SparkFlexConfig();
    flywheel2Config.apply(baseFlywheelConfig).inverted(false);
    flywheel2Config.follow(flywheelMotorCanId, true);
    flywheelMotor2 = new SparkFlex(flywheelMotor2CanId, kBrushless);
    flywheelMotor2.configure(flywheel2Config, kResetSafeParameters, kPersistParameters);
    flywheelEncoder2 = flywheelMotor2.getEncoder();
    flywheelController2 = flywheelMotor2.getClosedLoopController();

    if (tuningMode) {
      tunableP = new LoggedNetworkNumber("/Tuning/Shooter/P", flywheelP);
      tunableI = new LoggedNetworkNumber("/Tuning/Shooter/I", flywheelI);
      tunableD = new LoggedNetworkNumber("/Tuning/Shooter/D", flywheelD);
      tunableV = new LoggedNetworkNumber("/Tuning/Shooter/V", flywheelV);
      yawTunableP = new LoggedNetworkNumber("/Tuning/Shooter/Yaw/P", yawP);
      yawTunableI = new LoggedNetworkNumber("/Tuning/Shooter/Yaw/I", yawI);
      yawTunableD = new LoggedNetworkNumber("/Tuning/Shooter/Yaw/D", yawD);
      flywheel1On = new LoggedNetworkBoolean("/Tuning/Shooter/flywheel1On", true);
      flywheel2On = new LoggedNetworkBoolean("/Tuning/Shooter/flywheel2On", true);
    } else {
      tunableP = null;
      tunableI = null;
      tunableD = null;
      tunableV = null;
      yawTunableP = null;
      yawTunableI = null;
      yawTunableD = null;
      flywheel2On = null;
      flywheel1On = null;
    }

    switch (Robot.VERSION) {
      case V1:
        panEncoder26 = null;
        panEncoder28 = null;
        break;
      case V2:
        panEncoder26 = panMotor.getAbsoluteEncoder();
        panEncoder28 = null;
        break;
      default:
        throw new IllegalStateException("Invalid robot version");
    }
  }

  @Override
  public void zeroYaw(double position) {
    panEncoderRelative.setPosition(position);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    if (tuningMode) {
      tunableP.periodic();
      tunableI.periodic();
      tunableD.periodic();
      tunableV.periodic();
      yawTunableP.periodic();
      yawTunableI.periodic();
      yawTunableD.periodic();
      flywheel1On.periodic();
      flywheel2On.periodic();
      yawPID.setPID(yawTunableP.get() * 12, yawTunableI.get() * 12, yawTunableD.get() * 12);
      flywheelPID.setPID(tunableP.get() * 12, tunableI.get() * 12, tunableD.get() * 12);
    }
    double flywheelVelocity1 = motorToFlywheel * flywheelEncoder.getVelocity();
    double flywheelVelocity2 = motorToFlywheel * flywheelEncoder2.getVelocity();
    if (!MathUtil.isNear(flywheelVelocity1, flywheelVelocity2, 250)) {
      System.out.println(
          "[WARNING] Flywheel velocity mismatch: "
              + flywheelVelocity1
              + " vs "
              + flywheelVelocity2);
    }
    inputs.flywheelVelocityRotationsPerMinute = (flywheelVelocity1 + flywheelVelocity2) / 2;
    inputs.turretPitchRadians = ShotCalculators.pitchFromLinearActuator(linearActuatorSetpoint);
    inputs.turretYawRadians = getYaw();
    inputs.timestamp = Timer.getFPGATimestamp();
    inputs.linearActuatorSetpointMm = ((linearActuatorSetpoint + 1) / 2) * linearActuatorLengthMm;
    inputs.turretYawSetpointRadians = turretYawSetpointRadians;

    inputs.position26Radians = 2 * Math.PI - panEncoder26.getPosition();
    inputs.velocity26RadiansPerSecond = -panEncoder26.getVelocity();
    inputs.position28Radians = 2 * Math.PI - panEncoder28.getPosition();
    inputs.velocity28RadiansPerSecond = -panEncoder28.getVelocity();

    inputs.encoder26Connected = panEncoder26 != null;
    inputs.encoder28Connected = panEncoder28 != null;
  }

  private double getYaw() {
    return panEncoderRelative.getPosition();
  }

  @Override
  public void setFlywheelVelocity(double speedRotationsPerMinute) {
    double v = flywheelV;
    if (tuningMode) {
      v = tunableV.get();
    }
    v *= 12;

    //    flywheelMotor.set(1);
    //    flywheelController.setSetpoint(
    //        speedRotationsPerMinute,
    //        SparkBase.ControlType.kMAXMotionVelocityControl,
    //        ClosedLoopSlot.kSlot0,
    //        v * speedRotationsPerMinute,
    //        SparkClosedLoopController.ArbFFUnits.kVoltage);

    flywheelController.setSetpoint(
        speedRotationsPerMinute,
        SparkBase.ControlType.kVelocity,
        ClosedLoopSlot.kSlot0);
    //    double flywheelVelocity1 = motorToFlywheel * flywheelEncoder.getVelocity();
    //    double flywheelVelocity2 = motorToFlywheel * flywheelEncoder2.getVelocity();
    //    double flywheelVelocity = (flywheelVelocity1 + flywheelVelocity2) / 2;
    //    double output =
    //        flywheelPID.calculate(flywheelVelocity, speedRotationsPerMinute)
    //            + v * speedRotationsPerMinute;
    //    flywheelMotor2.setVoltage(tuningMode && !flywheel2On.get() ? 0 : output);
    //    flywheelMotor.setVoltage(tuningMode && !flywheel1On.get() ? 0 : output);

  }

  @Override
  public void setLinearActuatorPosition(double extensionMm) {
    double clamped =
        MathUtil.clamp(extensionMm, linearActuatorMinExtension, linearActuatorLengthMm);
    double newSetpoint = (clamped / linearActuatorLengthMm) * 2 - 1;
    linearActuatorSetpoint = newSetpoint;
    linearActuator.setSpeed(newSetpoint);
    linearActuator2.setSpeed(newSetpoint);
  }

  @Override
  public void setTurretYaw(double yawRadians) {
    double setpoint = MathUtil.clamp(yawRadians, yawMin, yawMax);
    turretYawSetpointRadians = setpoint;
    if (tuningMode) {
      panMotor.setVoltage(
          MathUtil.clamp(yawPID.calculate(getYaw(), setpoint), -maxYawVolts, maxYawVolts));
    } else {
      panController.setSetpoint(setpoint, SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }
    //    setTurretYawOpenLoop(0.0);
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
    linearActuator2.setSpeed(newSetpoint);
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

  @Override
  public void set28TurretAngleSupplier(AbsoluteEncoder enc) {
    panEncoder28 = enc;
  }
}

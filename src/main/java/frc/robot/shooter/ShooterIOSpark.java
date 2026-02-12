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
import frc.robot.ContinuousAbsoluteEncoder;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class ShooterIOSpark implements ShooterIO {
  private final Servo linearActuator;
  private final SparkMax panMotor;
  private final AbsoluteEncoder panEncoder;
  //  private final SparkClosedLoopController panController;
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

  private final ContinuousAbsoluteEncoder yawEncoderContinuous;

  private LoggedNetworkNumber tunableP;
  private LoggedNetworkNumber tunableI;
  private LoggedNetworkNumber tunableD;
  private LoggedNetworkNumber tunableV;
  private final PIDController flywheelPID = new PIDController(flywheelP, flywheelI, flywheelD);

  private LoggedNetworkNumber yawTunableP;
  private LoggedNetworkNumber yawTunableI;
  private LoggedNetworkNumber yawTunableD;
  private final PIDController yawPID = new PIDController(yawP, yawI, yawD);

  private SparkFlexConfig flywheelConfig;
  private SparkFlexConfig flywheel2Config;

  public ShooterIOSpark() {
    linearActuator = new Servo(linearActuatorServoPwmId);
    linearActuator.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);

    SparkMaxConfig panConfig = new SparkMaxConfig();
    panConfig.smartCurrentLimit(20).idleMode(kCoast).inverted(true);

    //        panConfig.encoder.inverted(true);
    panConfig.encoder.positionConversionFactor(2 * Math.PI / yawReduction);
    panMotor = new SparkMax(panMotorCanId, kBrushless);
    panMotor.configure(panConfig, kResetSafeParameters, kPersistParameters);
    panEncoder = panMotor.getAbsoluteEncoder();
    yawEncoderContinuous = new ContinuousAbsoluteEncoder();

    flywheelConfig = new SparkFlexConfig();
    flywheelConfig.smartCurrentLimit(40).idleMode(kCoast).inverted(true);
    flywheelConfig.closedLoop.pid(flywheelP, flywheelI, flywheelD, ClosedLoopSlot.kSlot0);
    flywheelConfig.closedLoop.feedForward.kV(flywheelV, ClosedLoopSlot.kSlot0);
    flywheelConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    flywheelConfig.openLoopRampRate(0.2);
    flywheelConfig.closedLoopRampRate(0.2);
    flywheelMotor = new SparkFlex(flywheelMotorCanId, kBrushless);
    flywheelMotor.configure(flywheelConfig, kResetSafeParameters, kPersistParameters);
    flywheelEncoder = flywheelMotor.getEncoder();
    flywheelController = flywheelMotor.getClosedLoopController();

    flywheel2Config = new SparkFlexConfig();
    flywheel2Config.smartCurrentLimit(40).idleMode(kCoast).inverted(false);
    flywheel2Config.closedLoop.pid(flywheelP, flywheelI, flywheelD, ClosedLoopSlot.kSlot0);
    flywheel2Config.closedLoop.feedForward.kV(flywheelV, ClosedLoopSlot.kSlot0);
    flywheel2Config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    flywheel2Config.openLoopRampRate(0.2);
    flywheel2Config.closedLoopRampRate(0.2);
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
    }
  }

  @Override
  public void zeroYaw() {
    if (panEncoder.getPosition() > .5) {
      yawEncoderContinuous.setAccumulator(-1);
    } else {
      yawEncoderContinuous.setAccumulator(0);
    }
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
      yawPID.setPID(yawTunableP.get(), yawTunableI.get(), yawTunableD.get());
    }
    double flywheelVelocity1 = (13d / 9) * flywheelEncoder.getVelocity();
    double flywheelVelocity2 = (13d / 9) * flywheelEncoder2.getVelocity();
    if (!MathUtil.isNear(flywheelVelocity1, flywheelVelocity2, 250)) {
      System.out.println(
          "[WARNING] Flywheel velocity mismatch: "
              + flywheelVelocity1
              + " vs "
              + flywheelVelocity2);
    }
    yawEncoderContinuous.update(panEncoder.getPosition());
    inputs.flywheelVelocityRotationsPerMinute = (flywheelVelocity1 + flywheelVelocity2) / 2;
    //        inputs.turretPitchRadians = tiltMotor.getAbsoluteEncoder().getPosition();
    inputs.turretYawRadians = yawEncoderContinuous.getPosition() * (28d / 200d) * (2 * Math.PI);
    inputs.timestamp = Timer.getFPGATimestamp();
    inputs.linearActuatorSetpointMm = ((linearActuatorSetpoint + 1) / 2) * linearActuatorLengthMm;
    //    inputs.turretYawRadians = panController.getSetpoint();
    //    if (flywheelController.isAtSetpoint()) {
    //      System.out.println("1 at setpoint");
    //    }
    //    if (flywheelController2.isAtSetpoint()) {
    //      System.out.println("2 at setpoint");
    //    }
  }

  @Override
  public void setFlywheelVelocity(double speedRotationsPerMinute) {
    double v = flywheelV;
    if (tuningMode) {
      v = tunableV.get();
      flywheelPID.setPID(tunableP.get() * 12, tunableI.get() * 12, tunableD.get() * 12);
    }
    //    else {
    //      flywheelController.setSetpoint(
    //              (9d / 13) * speedRotationsPerMinute, ControlType.kVelocity,
    // ClosedLoopSlot.kSlot0);
    //      flywheelController2.setSetpoint(
    //              (9d / 13) * speedRotationsPerMinute, ControlType.kVelocity,
    // ClosedLoopSlot.kSlot0);
    //    }
    double flywheelVelocity1 = (13d / 9) * flywheelEncoder.getVelocity();
    double flywheelVelocity2 = (13d / 9) * flywheelEncoder2.getVelocity();
    double flywheelVelocity = (flywheelVelocity1 + flywheelVelocity2) / 2;

    double output = flywheelPID.calculate(flywheelVelocity, speedRotationsPerMinute);
    output += v * speedRotationsPerMinute;
    flywheelMotor.setVoltage(output);
    flywheelMotor2.setVoltage(output);
  }

  //  @Override
  //  public void setTurretAngle(double yawRadians, double pitchRadians) {
  //    setTurretYaw(yawRadians);
  //    setTurretPitch(pitchRadians);
  //  }

  //  @Override
  //  public void setTurretPitch(double pitchRadians) {}

  @Override
  public void setLinearActuatorPosition(double extensionMm) {
    double clamped =
        MathUtil.clamp(extensionMm, linearActuatorMinExtension, linearActuatorLengthMm);
    double newSetpoint = (clamped / linearActuatorLengthMm) * 2 - 1;
    linearActuatorSetpoint = newSetpoint;
    linearActuator.setSpeed(newSetpoint);
  }

  @Override
  public void setTurretYaw(double yawRadians) {
    panMotor.setVoltage(
        MathUtil.clamp(
            yawPID.calculate(
                yawEncoderContinuous.getPosition(),
                MathUtil.clamp(yawRadians * 1.1441647597, -1, 1)),
            -2,
            2));
    //    panController.setSetpoint(yawRadians, ControlType.kPosition, ClosedLoopSlot.kSlot0);
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

package frc.robot.overbumper;

import static com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless;
import static frc.robot.overbumper.OverBumperConstants.*;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Robot;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class OverBumperIOSpark implements OverBumperIO {
  private final SparkBase obiMotor =
      switch (Robot.VERSION) {
        case V1 -> new SparkMax(overBumperMotorCanId, kBrushless);
        case V2 -> new SparkFlex(overBumperMotorCanId, kBrushless);
      };
  private final SparkClosedLoopController obiMotorController = obiMotor.getClosedLoopController();
  private final RelativeEncoder obiMotorEncoder = obiMotor.getEncoder();

  private final SparkMax obiPivotMotor = new SparkMax(overBumperPivotMotorCanId, kBrushless);
  private final SparkClosedLoopController obiPivotController =
      obiPivotMotor.getClosedLoopController();
  private final AbsoluteEncoder obiPivotEncoder = obiPivotMotor.getAbsoluteEncoder();

  private final PIDController obiPID = new PIDController(overBumperP, overBumperI, overBumperD);

  private final LoggedNetworkNumber tunableP;
  private final LoggedNetworkNumber tunableI;
  private final LoggedNetworkNumber tunableD;
  private final LoggedNetworkNumber tunableV;

  public OverBumperIOSpark() {
    SparkBaseConfig obiMotorConfig =
        switch (Robot.VERSION) {
          case V1 -> new SparkMaxConfig();
          case V2 -> new SparkFlexConfig();
        };
    obiMotorConfig.smartCurrentLimit(40).idleMode(SparkBaseConfig.IdleMode.kBrake).inverted(true);
    if (Robot.VERSION == Robot.RobotVersion.V2) {
      obiMotorConfig.smartCurrentLimit(60);
    }
    obiMotorConfig.closedLoop.pid(overBumperP, overBumperI, overBumperD, ClosedLoopSlot.kSlot0);
    obiMotorConfig.closedLoop.feedForward.kV(overBumperV, ClosedLoopSlot.kSlot0);
    obiMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    obiMotorConfig.closedLoopRampRate(.2);
    obiMotorConfig.openLoopRampRate(.2);
    obiMotor.configure(
        obiMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig pivconfig = new SparkMaxConfig();
    pivconfig.smartCurrentLimit(40).idleMode(SparkBaseConfig.IdleMode.kBrake).inverted(false);
    pivconfig.closedLoop.pid(
        overBumperPivotP, overBumperPivotI, overBumperPivotD, ClosedLoopSlot.kSlot0);
    pivconfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
    if (Robot.VERSION == Robot.RobotVersion.V2) {
      pivconfig.closedLoop.outputRange(-.3, .3, ClosedLoopSlot.kSlot0);
    }
    // NOT working??? todo
    //    pivconfig.closedLoop.feedForward.kCos(.3, ClosedLoopSlot.kSlot0);
    pivconfig.closedLoop.positionWrappingEnabled(true);
    pivconfig.closedLoop.positionWrappingInputRange(-.5, .5);
    obiPivotMotor.configure(
        pivconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    if (tuningMode) {
      tunableP = new LoggedNetworkNumber("/Tuning/OBI/P", overBumperP);
      tunableI = new LoggedNetworkNumber("/Tuning/OBI/I", overBumperI);
      tunableD = new LoggedNetworkNumber("/Tuning/OBI/D", overBumperD);
      tunableV = new LoggedNetworkNumber("/Tuning/OBI/V", overBumperV);
    } else {
      tunableP = null;
      tunableI = null;
      tunableD = null;
      tunableV = null;
    }
  }

  @Override
  public void updateInputs(OverBumperIOInputs inputs) {
    if (tuningMode) {
      tunableP.periodic();
      tunableI.periodic();
      tunableD.periodic();
      tunableV.periodic();
      obiPID.setPID(tunableP.get(), tunableI.get(), tunableD.get());
    }
    inputs.obiPosition = obiPivotEncoder.getPosition();
    inputs.obiSpeed = obiMotorEncoder.getVelocity();
  }

  @Override
  public void setOBIClosedLoop(double speed) {
    if (Robot.VERSION != Robot.RobotVersion.V2) {
      obiMotorController.setSetpoint(speed, SparkBase.ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    } else {
      double v = overBumperV;
      if (tuningMode) {
        v = tunableV.get();
      }
      double vel = obiMotorEncoder.getVelocity();
      setOBIOpenLoop(obiPID.calculate(vel, speed) + v * speed);
    }
  }

  @Override
  public void setOBIOpenLoop(double voltage) {
    obiMotor.setVoltage(voltage);
  }

  @Override
  public void setOBIPivotMotorOpenLoop(double voltage) {
    obiPivotMotor.setVoltage(voltage);
  }

  @Override
  public void setOBIPivotMotorClosedLoop(double position) {
    double clampedPosition = MathUtil.clamp(position, -.5, .5);
    double theta = clampedPosition * 2 * Math.PI;
    double cos = Math.cos(theta);
    double arbFF = .3 * cos;
    obiPivotController.setSetpoint(
        clampedPosition,
        SparkBase.ControlType.kPosition,
        ClosedLoopSlot.kSlot0,
        arbFF,
        SparkClosedLoopController.ArbFFUnits.kVoltage);
  }
}

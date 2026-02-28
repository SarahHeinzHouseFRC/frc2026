package frc.robot.intake;

import static com.revrobotics.spark.SparkLowLevel.MotorType.*;
import static frc.robot.intake.IntakeConstants.*;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.shooter.Shooter;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class IntakeIOSpark implements IntakeIO {
  private final SparkMax beltMotor = new SparkMax(beltMotorCanId, kBrushless);
  private final SparkMax intakeMotor = new SparkMax(intakeMotorCanId, kBrushless);
  private final SparkMax beltStarMotor = new SparkMax(beltStarMotorCanId, kBrushless);
  private final SparkBase obiMotor =
      switch (Robot.VERSION) {
        case V1 -> new SparkMax(overBumperMotorCanId, kBrushless);
        case V2 -> new SparkFlex(overBumperMotorCanId, kBrushless);
      };
  private final SparkClosedLoopController obiMotorController;
  private final SparkMax obiPivotMotor = new SparkMax(overBumperPivotMotorCanId, kBrushless);
  private final SparkClosedLoopController obiPivotController;
  private final AbsoluteEncoder obiPivotEncoder;
  private final RelativeEncoder obiMotorEncoder;
  private final SparkMax agitatorMotor =
      switch (Robot.VERSION) {
        case V1 -> null;
        case V2 -> new SparkMax(agitatorMotorCanId, kBrushless);
      };

  private final LoggedNetworkNumber tunableP;
  private final LoggedNetworkNumber tunableI;
  private final LoggedNetworkNumber tunableD;
  private final LoggedNetworkNumber tunableV;
  private final PIDController obiPID = new PIDController(overBumperP, overBumperI, overBumperD);
  private double obiAppliedOupt = 0.0;

  public IntakeIOSpark() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.smartCurrentLimit(40).idleMode(IdleMode.kBrake).inverted(false);
    SparkMaxConfig invertedConfig = new SparkMaxConfig();
    invertedConfig.smartCurrentLimit(40).idleMode(IdleMode.kBrake).inverted(true);
    beltMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    intakeMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig beltStarConfig = new SparkMaxConfig();
    beltStarConfig.smartCurrentLimit(40).idleMode(IdleMode.kBrake).inverted(true);
    if (Robot.VERSION == Robot.RobotVersion.V2) {
      beltStarConfig.inverted(false);
    }
    beltStarConfig.absoluteEncoder.positionConversionFactor(2 * Math.PI);
    beltStarConfig.absoluteEncoder.inverted(true);
    beltStarMotor.configure(
        beltStarConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkBaseConfig obiMotorConfig =
        switch (Robot.VERSION) {
          case V1 -> new SparkMaxConfig();
          case V2 -> new SparkFlexConfig();
        };
    obiMotorConfig.smartCurrentLimit(40).idleMode(IdleMode.kBrake).inverted(true);
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
    obiMotorController = obiMotor.getClosedLoopController();
    obiMotorEncoder = obiMotor.getEncoder();

    SparkMaxConfig pivconfig = new SparkMaxConfig();
    pivconfig.smartCurrentLimit(40).idleMode(IdleMode.kBrake).inverted(false);
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
    obiPivotController = obiPivotMotor.getClosedLoopController();
    obiPivotEncoder = obiPivotMotor.getAbsoluteEncoder();

    if (agitatorMotor != null) {
      SparkMaxConfig agitatorConfig = new SparkMaxConfig();
      agitatorConfig.smartCurrentLimit(20).idleMode(IdleMode.kBrake).inverted(true);
      agitatorMotor.configure(
          agitatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    if (Robot.VERSION == Robot.RobotVersion.V2) {
      Shooter.getInstance().set28TurretAngleSupplier(beltStarMotor.getAbsoluteEncoder());
    }

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
  public void updateInputs(IntakeIOInputs inputs) {
    if (tuningMode) {
      tunableP.periodic();
      tunableI.periodic();
      tunableD.periodic();
      tunableV.periodic();
      obiPID.setPID(tunableP.get(), tunableI.get(), tunableD.get());
    }
    inputs.obiPosition = obiPivotEncoder.getPosition();
    inputs.obiSpeed = obiMotorEncoder.getVelocity();
    inputs.obiAppliedOput = obiAppliedOupt;
    SmartDashboard.putNumber("obi I accum", obiMotorController.getIAccum());
    SmartDashboard.putNumber("obi I accum max", 1.5 / overBumperI);
  }

  @Override
  public void setBeltOpenLoop(double voltage) {
    beltMotor.setVoltage(voltage);
  }

  @Override
  public void setIntakeOpenLoop(double voltage) {
    intakeMotor.setVoltage(voltage);
  }

  @Override
  public void setBeltStarOpenLoop(double voltage) {
    beltStarMotor.setVoltage(voltage);
  }

  @Override
  public void setOBIClosedLoop(double speed) {
    obiMotorController.setSetpoint(speed, SparkBase.ControlType.kVelocity, ClosedLoopSlot.kSlot0);
  }

  @Override
  public void setOBIOpenLoop(double voltage) {
    obiAppliedOupt = voltage;
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

  @Override
  public void setAgitatorOpenLoop(double voltage) {
    if (agitatorMotor == null) return;
    agitatorMotor.setVoltage(voltage);
  }

  @Override
  public void setOBIClosedLoopWithJamDetect(double speed) {
    if (Robot.VERSION != Robot.RobotVersion.V2) {
      setOBIClosedLoop(speed);
      return;
    }
    double vel = obiMotorEncoder.getVelocity();
    if (Math.abs(vel) < Math.abs(Math.min(60, speed * .03))) {
      // full send
      setOBIOpenLoop(12 * Math.signum(speed));
    } else {
      double v = overBumperV;
      if (tuningMode) {
        v = tunableV.get();
      }
      setOBIOpenLoop(obiPID.calculate(vel, speed) + v * speed);
    }
  }
}

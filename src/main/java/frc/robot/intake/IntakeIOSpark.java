package frc.robot.intake;

import static com.revrobotics.spark.SparkLowLevel.MotorType.*;
import static frc.robot.intake.IntakeConstants.*;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;

public class IntakeIOSpark implements IntakeIO {
  private final SparkMax beltMotor = new SparkMax(beltMotorCanId, kBrushless);
  private final SparkMax intakeMotor = new SparkMax(intakeMotorCanId, kBrushless);
  private final SparkMax beltStarMotor = new SparkMax(beltStarMotorCanId, kBrushless);
  private final SparkMax obiMotor = new SparkMax(overBumperMotorCanId, kBrushless);
  private final SparkClosedLoopController obiMotorController;
  private final SparkMax obiPivotMotor = new SparkMax(overBumperPivotMotorCanId, kBrushless);
  private final SparkClosedLoopController obiPivotController;
  private final AbsoluteEncoder obiPivotEncoder;
  private final RelativeEncoder obiMotorEncoder;

  public IntakeIOSpark() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.smartCurrentLimit(40).idleMode(IdleMode.kBrake).inverted(false);
    SparkMaxConfig invertedConfig = new SparkMaxConfig();
    invertedConfig.smartCurrentLimit(40).idleMode(IdleMode.kBrake).inverted(true);
    beltMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    intakeMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    beltStarMotor.configure(
        invertedConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    SparkMaxConfig obiMotorConfig = new SparkMaxConfig();
    obiMotorConfig.apply(invertedConfig);
    obiMotorConfig.closedLoop.pid(0.00013, 0, 0);
    obiMotorConfig.closedLoop.feedForward.kV(0.000195);
    obiMotorConfig.closedLoopRampRate(.2);
    obiMotorConfig.openLoopRampRate(.2);
    obiMotor.configure(
        obiMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    obiMotorController = obiMotor.getClosedLoopController();
    obiMotorEncoder = obiMotor.getEncoder();
    SparkMaxConfig pivconfig = new SparkMaxConfig();
    pivconfig.smartCurrentLimit(40).idleMode(IdleMode.kBrake).inverted(false);
    pivconfig.closedLoop.pid(6, 0, 0, ClosedLoopSlot.kSlot0);
    pivconfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
    // NOT working??? todo
    //    pivconfig.closedLoop.feedForward.kCos(.3, ClosedLoopSlot.kSlot0);
    pivconfig.closedLoop.positionWrappingEnabled(true);
    pivconfig.closedLoop.positionWrappingInputRange(-.5, .5);
    obiPivotMotor.configure(
        pivconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    obiPivotController = obiPivotMotor.getClosedLoopController();
    obiPivotEncoder = obiPivotMotor.getAbsoluteEncoder();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.obiPosition = obiPivotEncoder.getPosition();
    inputs.obiSpeed = obiMotorEncoder.getVelocity();
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
    obiPivotController.setSetpoint(
        clampedPosition,
        SparkBase.ControlType.kPosition,
        ClosedLoopSlot.kSlot0,
        .3 * cos,
        SparkClosedLoopController.ArbFFUnits.kVoltage);
  }
}

package frc.robot.subsystems.intake;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.SharpSubsystem;

import static frc.robot.subsystems.intake.IntakeConstants.*;

public class IntakeSubsystem extends SharpSubsystem {
  private static final IntakeSubsystem instance = new IntakeSubsystem();
  public static IntakeSubsystem getInstance() {
    return instance;
  }

  private final SparkMax pivotMotor = new SparkMax(26, SparkLowLevel.MotorType.kBrushless);
  private final SparkClosedLoopController pivotController;
  private final SparkMax pivotMotor2 = new SparkMax(27, SparkLowLevel.MotorType.kBrushless);
  private final SparkClosedLoopController pivotController2;

  private IntakeSubsystem() {

    SparkMaxConfig pivotConfig = new SparkMaxConfig();
    pivotConfig.smartCurrentLimit(40).idleMode(SparkBaseConfig.IdleMode.kCoast);
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
        .cruiseVelocity(.02, ClosedLoopSlot.kSlot1)
        .maxAcceleration(1, ClosedLoopSlot.kSlot1);
    pivotConfig.closedLoop.feedForward.kCos(.3, ClosedLoopSlot.kSlot1);

    pivotMotor.configure(
        pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    pivotController = pivotMotor.getClosedLoopController();

    SparkMaxConfig pivotConfig2 = new SparkMaxConfig();
    pivotConfig2.apply(pivotConfig);
    pivotConfig2.absoluteEncoder.inverted(true);
    pivotConfig2.inverted(true);

    pivotMotor2.configure(pivotConfig2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    pivotController2 = pivotMotor2.getClosedLoopController();
  }

  public void setPosition(double position) {
    pivotController.setSetpoint(position, SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0);
    pivotController2.setSetpoint(position, SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  public void setPositionSlowly(double position) {
    pivotController.setSetpoint(position, SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot1);
    pivotController2.setSetpoint(position, SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot1);
  }

  public void deploy() {
    setPosition(presetEngaged);
  }

  public void retract() {
    setPositionSlowly(presetStowed);
  }

  public Command shakeCommand() {
    double extraWaitTimeAtStart = 1.0;
    double waitTime = 1.0;

    return Commands.sequence(
        new WaitCommand(extraWaitTimeAtStart),
        Commands.repeatingSequence(
            new WaitCommand(waitTime),
            new StowIntake(this),
            new WaitCommand(waitTime),
            new DeployIntake(this)
        )
    );
  }
}

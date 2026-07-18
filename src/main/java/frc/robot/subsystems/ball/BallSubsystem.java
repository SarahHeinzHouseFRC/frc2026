package frc.robot.subsystems.ball;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.SharpSubsystem;

public class BallSubsystem extends SharpSubsystem {
  private final static BallSubsystem instance = new BallSubsystem();
  public static BallSubsystem getInstance() {
    return instance;
  }

  private final SparkMax beltMotor = new SparkMax(23, SparkLowLevel.MotorType.kBrushless);
  private final SparkMax indexerMotor = new SparkMax(22, SparkLowLevel.MotorType.kBrushless);

  private final SparkFlex intakeMotor = new SparkFlex(25, SparkLowLevel.MotorType.kBrushless);

  private Debouncer indexerJamDebouncer = new Debouncer(0.25);
  private boolean indexerJammed = false;

  private Debouncer intakeJamDebouncer = new Debouncer(0.25);
  private boolean intakeJammed = false;

  private BallSubsystem() {
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
    intakeConfig.closedLoopRampRate(.2);
    intakeConfig.openLoopRampRate(.2);
    intakeMotor.configure(
        intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  // + to shoot, - to not do that
  public void runIndexer(double speed) {
    indexerMotor.set(speed);
  }

  // + to shoot or intake, - to not do those things
  public void runBelt(double speed) {
    beltMotor.set(speed);
  }

  // idk this one should be pretty obvious (+ intake, - out)
  public void runIntake(double speed) {
    intakeMotor.set(speed);
  }

  public void runInputs(BallInputs inputs) {
    runIntake(inputs.intakeSpeed());
    runBelt(inputs.beltSpeed());
    runIndexer(inputs.indexerSpeed());
  }

  public void stop() {
    runIntake(0);
    runBelt(0);
    runIndexer(0);
  }

  public boolean isIndexerJammed() {
    return indexerJammed;
  }

  public boolean isIntakeJammed() {
    return intakeJammed;
  }

  @Override
  public void periodic() {
    indexerJammed = indexerJamDebouncer.calculate(
        indexerMotor.getAppliedOutput() > .1 && indexerMotor.getEncoder().getVelocity() < 60
    );

    if (indexerJammed) {
      System.out.println("(debug) indexer jammed!");
    }

    intakeJammed = intakeJamDebouncer.calculate(
        Math.abs(intakeMotor.getAppliedOutput()) > .1 && intakeMotor.getEncoder().getVelocity() * Math.signum(intakeMotor.getAppliedOutput()) < 60
    );
    SmartDashboard.putNumber("intake speed", intakeMotor.getEncoder().getVelocity());
  }
}

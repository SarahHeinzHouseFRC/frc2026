package frc.robot.intake;

import static com.revrobotics.spark.SparkLowLevel.MotorType.*;
import static frc.robot.intake.IntakeConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Robot;
import frc.robot.shooter.Shooter;

public class IntakeIOSpark implements IntakeIO {
  private final SparkMax beltMotor = new SparkMax(beltMotorCanId, kBrushless);
  private final SparkMax intakeMotor = new SparkMax(intakeMotorCanId, kBrushless);
  private final SparkMax indexerMotor = new SparkMax(indexerMotorCanId, kBrushless);

  public IntakeIOSpark() {
    SparkMaxConfig beltConfig = new SparkMaxConfig();
    beltConfig.smartCurrentLimit(40).idleMode(IdleMode.kBrake).inverted(false);
    beltMotor.configure(beltConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    SparkMaxConfig intakeConfig = new SparkMaxConfig();
    intakeConfig.smartCurrentLimit(40)
      .idleMode(IdleMode.kBrake)
      .inverted(false)
      .openLoopRampRate(0.2);    //suggest: between 0.1 - 0.4
    intakeMotor.configure(
        intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig indexerConfig = new SparkMaxConfig();
    indexerConfig.smartCurrentLimit(40).idleMode(IdleMode.kBrake).inverted(false);
    if (Robot.VERSION == Robot.RobotVersion.V2) {
      indexerConfig.inverted(true);
    }
    indexerConfig.absoluteEncoder.positionConversionFactor(2 * Math.PI);
    indexerConfig.absoluteEncoder.velocityConversionFactor(2 * Math.PI / 60);
    indexerConfig.absoluteEncoder.inverted(true);
    indexerMotor.configure(
        indexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    if (Robot.VERSION == Robot.RobotVersion.V2) {
      Shooter.getInstance().set28TurretAngleSupplier(indexerMotor.getAbsoluteEncoder());
    }
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {}

  @Override
  public void setBeltOpenLoop(double voltage) {
    beltMotor.setVoltage(voltage);
  }

  @Override
  public void setIntakeOpenLoop(double voltage) {
    intakeMotor.setVoltage(voltage);
  }

  @Override
  public void setIndexerOpenLoop(double voltage) {
    indexerMotor.setVoltage(voltage);
  }
}

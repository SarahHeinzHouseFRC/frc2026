package frc.robot.climber;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ClimberIOSpark implements ClimberIO {
  private SparkMax climberMotor = new SparkMax(40, SparkLowLevel.MotorType.kBrushless);

  public ClimberIOSpark() {
    SparkMaxConfig climberConfig = new SparkMaxConfig();
    climberConfig.smartCurrentLimit(40).idleMode(SparkBaseConfig.IdleMode.kBrake);
    climberMotor.configure(
        climberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void runClimberOpenLoop(double voltage) {
    if (voltage > 12) {
      voltage = 12;
    } else if (voltage < -12) {
      voltage = -12;
    }
    climberMotor.setVoltage(voltage);
  }
}

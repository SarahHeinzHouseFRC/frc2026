package frc.robot.flywheel;

import frc.robot.commands.CommandScheduler;
import frc.robot.commands.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Flywheel extends SubsystemBase {
  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

  public Flywheel(CommandScheduler scheduler, FlywheelIO io) {
    super(scheduler);
    this.io = io;
  }

  public void setVelocity(double speedRotationsPerSecond) {
    io.setVelocity(speedRotationsPerSecond);
  }

  public void stop() {
    io.setVelocity(0.0);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Flywheel", inputs);
  }
}

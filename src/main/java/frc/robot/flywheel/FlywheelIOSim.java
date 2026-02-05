package frc.robot.flywheel;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class FlywheelIOSim implements FlywheelIO {
  private final FlywheelSim flywheelSim;

  private double appliedVoltage = 0.0;
  private final PIDController pid;
  private final double dt = 0.02;

  public FlywheelIOSim() {
    DCMotor gearbox = DCMotor.getNEO(1);
    LinearSystem<N1, N1, N1> plant = LinearSystemId.createFlywheelSystem(gearbox, 0.025, 1.0);
    flywheelSim = new FlywheelSim(plant, gearbox, 0.0);

    pid = new PIDController(0.1, 0.0, 0.0);
    pid.setTolerance(1.0);
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {

    double currentVelocity = flywheelSim.getAngularVelocityRPM() / 60.0;

    appliedVoltage = pid.calculate(currentVelocity);
    appliedVoltage = Math.max(-12.0, Math.min(12.0, appliedVoltage));

    flywheelSim.setInputVoltage(appliedVoltage);
    flywheelSim.update(dt);

    inputs.velocityRotationsPerSecond = currentVelocity;
  }

  @Override
  public void setVelocity(double speedRotationsPerSecond) {
    pid.setSetpoint(speedRotationsPerSecond);
  }
}

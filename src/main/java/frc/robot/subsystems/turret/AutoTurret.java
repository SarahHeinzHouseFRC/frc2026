package frc.robot.subsystems.turret;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ShotCalculator;

public class AutoTurret extends Command {
  private final TurretSubsystem turret;
  private final ShotCalculator shotCalculator = ShotCalculator.getInstance();
  public AutoTurret(TurretSubsystem turret) {
    this.turret = turret;
    addRequirements(turret);
  }

  @Override
  public void execute() {
    turret.setPanSetpoint(shotCalculator.getShotAngle());
    turret.setLinearActuator(shotCalculator.getShotParams().linearActuatorExtensionMillimeters());
  }

  @Override
  public void end(boolean interrupted) {
    turret.setPanMotor(0);
  }
}

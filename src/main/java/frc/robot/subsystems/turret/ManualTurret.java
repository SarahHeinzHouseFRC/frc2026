package frc.robot.subsystems.turret;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

public class ManualTurret extends Command {
  private final TurretSubsystem turret;
  private final XboxController controller;
  public ManualTurret(TurretSubsystem turret, XboxController controller) {
    this.turret = turret;
    this.controller = controller;
    addRequirements(TurretSubsystem.getInstance());
  }

  @Override
  public void execute() {
    int pov = controller.getPOV();
    boolean right = pov == 45 || pov == 90 || pov == 135;
    boolean down = pov == 135 || pov == 180 || pov == 225;
    boolean left = pov == 225 || pov == 270 || pov == 315;
    boolean up = pov == 315 || pov == 360 || pov == 0 || pov == 45;

    if (left) {
      turret.setPanMotor(.2);
    } else if (right) {
      turret.setPanMotor(-.2);
    } else {
      turret.setPanMotor(0);
    }

    if (down) {
      turret.alterLinearActuator(-0.5);
    } else if (up) {
      turret.alterLinearActuator(0.5);
    }
  }

  @Override
  public void end(boolean interrupted) {
    turret.setPanMotor(0);
  }
}

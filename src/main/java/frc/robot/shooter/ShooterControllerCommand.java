package frc.robot.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants;
import frc.robot.vision.Vision;
import org.littletonrobotics.junction.Logger;

public class ShooterControllerCommand extends Command {
  enum ShooterMode {
    AUTO,
    MANUAL
  }
  private ShooterMode mode = ShooterMode.AUTO;

  private final XboxController controller;
  private final Shooter shooter;

  private double shooterSpeedSetpoint = 6000;

  public ShooterControllerCommand(XboxController controller, Shooter shooter) {
    this.shooter = shooter;
    this.controller = controller;
    addRequirements(shooter);
  }

  @Override
  public void execute() {
    handleRecalibration();
    handleModeSwitch();
    if (!shooter.isLimpMode()) {
      switch (mode) {
        case MANUAL -> executeManual();
        case AUTO -> executeAuto();
      }
    } else {
      executeManual();
    }
  }

  public void handleRecalibration() {
    shooter.handleRecalibrationRequest(() -> controller.getLeftBumperButton() && controller.getRightBumperButton());
  }

  public void handleModeSwitch() {
    if (controller.getXButton()) {
      mode = ShooterMode.MANUAL;
    } else if (controller.getAButton()) {
      mode = ShooterMode.AUTO;
    }
  }

  public void executeManual() {
    int pov = controller.getPOV();
    boolean right = pov == 45 || pov == 90 || pov == 135;
    boolean down = pov == 135 || pov == 180 || pov == 225;
    boolean left = pov == 225 || pov == 270 || pov == 315;
    boolean up = pov == 315 || pov == 360 || pov == 0 || pov == 45;

    if (up) {
      shooter.setTurretPitchOpenLoop(12);
    }
    if (down) {
      shooter.setTurretPitchOpenLoop(-12);
    }
    if (left) {
      shooter.setTurretYawOpenLoop(1d);
    }
    if (right) {
      shooter.setTurretYawOpenLoop(-1d);
    }

    if (!left && !right) {
      shooter.setTurretYawOpenLoop(0d);
    }
    shooterSpeedSetpoint =
        MathUtil.clamp(
            shooterSpeedSetpoint - 50 * MathUtil.applyDeadband(controller.getRightY(), .1),
            0,
            6000);
    Logger.recordOutput("/Shooter/directSpeedSetpoint", shooterSpeedSetpoint);
    if (controller.getRightBumperButton()) {
      shooter.setFlywheelVelocityRpm(shooterSpeedSetpoint);
    } else {
      shooter.stopFlywheel();
    }
    shooter.autoAimDry(shooter.getShotTarget());
  }

  public void executeAuto() {
    if (!Vision.getInstance().isVisionInit()) {
      shooter.scanForTarget();
      shooter.autoAimDry(shooter.getShotTarget());
    } else {
      boolean flywheel = controller.getRightBumperButton() || controller.getRightTriggerAxis() > .1;
      shooter.autoAim(shooter.getShotTarget(), flywheel);
    }
  }
}

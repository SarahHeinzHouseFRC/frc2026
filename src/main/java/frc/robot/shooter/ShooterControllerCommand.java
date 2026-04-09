package frc.robot.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.drive.Drive;
import frc.robot.vision.Vision;
import org.littletonrobotics.junction.Logger;

public class ShooterControllerCommand extends Command {
  enum ShooterMode {
    AUTO,
    MANUAL,
    SEMIMANUAL,
  }

  private static final double OPEN_LOOP_TURRET_SPEED = 2.0;

  private ShooterMode mode = ShooterMode.AUTO;

  private boolean scanForTargetEnabled = true;

  private final XboxController controller;
  private final Shooter shooter;

  private double shooterSpeedSetpoint = 3235;

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
        case SEMIMANUAL -> executeSemimanual();
      }
    } else {
      executeManual();
    }
  }

  public void executeSemimanual() {
    int pov = controller.getPOV();
    boolean right = pov == 45 || pov == 90 || pov == 135;
    boolean left = pov == 225 || pov == 270 || pov == 315;

    if (left) {
      shooter.setTurretYawOpenLoop(OPEN_LOOP_TURRET_SPEED);
    } else if (right) {
      shooter.setTurretYawOpenLoop(-OPEN_LOOP_TURRET_SPEED);
    } else {
      shooter.setTurretYawOpenLoop(0d);
    }

    boolean flywheel = controller.getRightBumperButton() || controller.getRightTriggerAxis() > .1;
    shooter.autoAimYawOff(
        shooter.getShotTarget(),
        Drive.getInstance().getPose(),
        Drive.getInstance().getChassisSpeeds(),
        flywheel,
        false);
  }

  public void handleRecalibration() {
    if (controller.getLeftBumperButton() && controller.getRightBumperButton()) {
      shooter.forceZeroYaw();
    }
  }

  public void handleModeSwitch() {
    if (controller.getXButton()) {
      mode = ShooterMode.MANUAL;
      shooterSpeedSetpoint = 3235;
      shooter.setLinearMm(0);
      scanForTargetEnabled = false;
    } else if (controller.getAButton()) {
      mode = ShooterMode.AUTO;
      scanForTargetEnabled = false;
    } else if (controller.getBButton()) {
      mode = ShooterMode.SEMIMANUAL;
      scanForTargetEnabled = false;
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
      shooter.setTurretYawOpenLoop(OPEN_LOOP_TURRET_SPEED);
    }
    if (right) {
      shooter.setTurretYawOpenLoop(-OPEN_LOOP_TURRET_SPEED);
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
      controller.setRumble(GenericHID.RumbleType.kBothRumble, 1d);
      //        shooter.setFlywheelOpenLoop(12d);
    } else {
      shooter.stopFlywheel();
      controller.setRumble(GenericHID.RumbleType.kBothRumble, 0d);
    }
    shooter.autoAimDry(shooter.getShotTarget());
  }

  public void executeAuto() {
    if (!Vision.getInstance().isVisionInit()) {
      if (scanForTargetEnabled) {
        shooter.scanForTarget();
      }
      shooter.autoAimDry(shooter.getShotTarget());
    } else {
      boolean flywheel = controller.getRightBumperButton() || controller.getRightTriggerAxis() > .1;
      if (controller.getRightBumperButton()) {
        controller.setRumble(GenericHID.RumbleType.kBothRumble, 1d);
      } else {
        controller.setRumble(GenericHID.RumbleType.kBothRumble, 0d);
      }
      shooter.autoAim(shooter.getShotTarget(), flywheel);
    }
  }
}

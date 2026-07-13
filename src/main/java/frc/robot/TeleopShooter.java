package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.ball.BallSubsystem;
import frc.robot.subsystems.ball.TeleopBallControl;
import frc.robot.subsystems.intake.DeployIntake;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.StowIntake;
import frc.robot.subsystems.launcher.LauncherSubsystem;
import frc.robot.subsystems.turret.AutoTurret;
import frc.robot.subsystems.turret.ManualTurret;
import frc.robot.subsystems.turret.TurretSubsystem;

import java.util.function.DoubleSupplier;

public class TeleopShooter {
  private static final TeleopShooter instance = new TeleopShooter();

  public static TeleopShooter getInstance() {
    return instance;
  }

  private final TurretSubsystem turret = TurretSubsystem.getInstance();
  private final BallSubsystem ball = BallSubsystem.getInstance();
  private final IntakeSubsystem intake = IntakeSubsystem.getInstance();
  private final LauncherSubsystem launcher = LauncherSubsystem.getInstance();
  private final ShotCalculator shotCalculator = ShotCalculator.getInstance();

  private XboxController controller;

  private final SendableChooser<ShooterMode> shooterModeChooser = new SendableChooser<>();

  private double manualFlywheelSpeed = 6000;

  private double autoShooterOffset = 0; // + makes it overshoot

  public enum ShooterMode {
    TURRET_AUTO, DRIVE_AUTO, MANUAL
  }

  private TeleopShooter() {
    configureChooser();
  }

  private void configureChooser() {
    shooterModeChooser.addOption("Turret-based Aim", ShooterMode.TURRET_AUTO);
    shooterModeChooser.addOption("Drivetrain-based Aim", ShooterMode.DRIVE_AUTO);
    shooterModeChooser.setDefaultOption("Manual Control", ShooterMode.MANUAL);
    SmartDashboard.putData("Shooter Mode", shooterModeChooser);
  }

  private boolean stowOnLeftBumperRelease = false;

  private void configureBindings() {
    DoubleSupplier intakeRequestSupplier = () -> controller.getLeftTriggerAxis() * (controller.getLeftBumperButton() ? -1 : 1);
    DoubleSupplier shootRequestSupplier = () -> controller.getRightTriggerAxis();
    TeleopBallControl teleopBallControl = new TeleopBallControl(ball, intakeRequestSupplier, shootRequestSupplier);
    Trigger wantsTeleopBallControl = new Trigger(() -> controller.getLeftTriggerAxis() > .1 || controller.getRightTriggerAxis() > .1);
    wantsTeleopBallControl.whileTrue(teleopBallControl);

    Trigger wantsLauncher = new Trigger(() -> controller.getRightTriggerAxis() > .1 || controller.getRightBumperButton());
    wantsLauncher.whileTrue(launcher.launcherSpeedCommand(() -> switch (getShooterMode()) {
      case TURRET_AUTO, DRIVE_AUTO -> shotCalculator.getShotParams().flywheelVelocityRotationsPerMinute();
      case MANUAL -> manualFlywheelSpeed;
    }));


    Trigger leftBumper = new Trigger(() -> controller.getLeftBumperButton());
    Trigger leftTriggerActive = new Trigger(() -> controller.getLeftTriggerAxis() > 0.1);

    // when the bumper is pressed, reset the flag allowing us to stow if possible
    leftBumper.onTrue(Commands.runOnce(() -> stowOnLeftBumperRelease = true));

    // when the trigger is active, set the flag to false because the user does not intend to stow,
    // they intend to outtake
    leftTriggerActive
        .whileTrue(Commands.run(() -> stowOnLeftBumperRelease = false));

    // when the bumper is lifted, stow the intake only if the flag is still true,
    // and set the flag to false until next press.
    leftBumper.onFalse(
        new StowIntake(intake)
            .alongWith(Commands.runOnce(() -> stowOnLeftBumperRelease = false))
            .onlyIf(() -> stowOnLeftBumperRelease));

    leftTriggerActive.onTrue(new DeployIntake(intake));

    Trigger doAutoTurret = new Trigger(() -> getShooterMode() == ShooterMode.TURRET_AUTO);
    Trigger doManualTurret = new Trigger(() -> getShooterMode() == ShooterMode.MANUAL || getShooterMode() == ShooterMode.DRIVE_AUTO);
    doAutoTurret.whileTrue(new AutoTurret(turret));
    doManualTurret.whileTrue(new ManualTurret(turret, controller));
  }

  public void periodic() {
    if (getShooterMode() == ShooterMode.MANUAL) {
      if (controller.getYButton()) {
        manualFlywheelSpeed += 10;
      }
      if (controller.getAButton()) {
        manualFlywheelSpeed -= 10;
      }
      manualFlywheelSpeed = Math.min(Math.max(0, manualFlywheelSpeed), 6000);
    } else if (getShooterMode() == ShooterMode.DRIVE_AUTO || getShooterMode() == ShooterMode.TURRET_AUTO) {
      if (controller.getYButton()) {
        autoShooterOffset += .01;
      } else if (controller.getAButton()) {
        autoShooterOffset -= .01;
      }
      autoShooterOffset = Math.min(Math.max(-1, autoShooterOffset), 1);
    }

    SmartDashboard.putNumber("Manual Flywheel Speed", manualFlywheelSpeed);
    SmartDashboard.putNumber("Auto Shooter Offset", autoShooterOffset);
    shotCalculator.setOffset(autoShooterOffset);
  }

  public ShooterMode getShooterMode() {
    return shooterModeChooser.getSelected();
  }

  public void setXboxController(XboxController controller) {
    if (this.controller != null) {
      throw new IllegalStateException("Xbox controller already configured");
    }
    this.controller = controller;
    configureBindings();
  }
}

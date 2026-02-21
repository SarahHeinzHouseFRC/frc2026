package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.auto.AutoRebuilt2026;
import frc.robot.commands.Command;
import frc.robot.drive.ControllerDriveCommand;
import frc.robot.drive.Drive;
import frc.robot.intake.Intake;
import frc.robot.intake.IntakeControllerCommand;
import frc.robot.shooter.Shooter;

/**
 * Central command wiring for teleop bindings and autonomous command selection.
 *
 * <p>Autonomous routine used here: {@link AutoRebuilt2026}.
 */
public class RobotContainer {
  private final Drive drive;
  private final Shooter shooter;
  private final Intake intake;

  private final GenericController driverController;
  private final XboxController driverXboxController;
  private final XboxController operatorXboxController;

  public RobotContainer(
      Drive drive,
      Shooter shooter,
      Intake intake,
      GenericController driverController,
      XboxController driverXboxController,
      XboxController operatorXboxController) {
    this.drive = drive;
    this.shooter = shooter;
    this.intake = intake;
    this.driverController = driverController;
    this.driverXboxController = driverXboxController;
    this.operatorXboxController = operatorXboxController;

    configureDefaultCommands();
    configureBindings();
  }

  private void configureDefaultCommands() {
    drive.setDefaultCommand(new ControllerDriveCommand(driverController, drive));
    intake.setDefaultCommand(
        new IntakeControllerCommand(driverXboxController, operatorXboxController, intake));
  }

  private void configureBindings() {}

  /** Returns the command to run during autonomous. */
  public Command getAutonomousCommand() {
    return new AutoRebuilt2026(drive, shooter, intake);
  }
}

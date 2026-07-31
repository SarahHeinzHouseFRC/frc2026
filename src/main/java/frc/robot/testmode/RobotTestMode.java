package frc.robot.testmode;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ball.BallSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.launcher.LauncherSubsystem;
import frc.robot.subsystems.vision.Vision;

/** Runs each subsystem test in the required order. */
public final class RobotTestMode extends SequentialCommandGroup {
  private final Drive drive;
  private final LauncherSubsystem launcher;
  private final IntakeSubsystem intake;
  private final BallSubsystem ball;
  private final Vision vision;
  private final SubsystemTestCommand[] subsystemTests;
  private final Alert overallAlert =
      new Alert(
          SubsystemTestCommand.ALERT_GROUP,
          "[TESTMODE] All subsystem tests finished",
          Alert.AlertType.kInfo);

  public RobotTestMode(
      Drive drive, LauncherSubsystem launcher, IntakeSubsystem intake, BallSubsystem ball, Vision vision) {
    this.drive = drive;
    this.launcher = launcher;
    this.intake = intake;
    this.ball = ball;
    this.vision = vision;

    subsystemTests =
        new SubsystemTestCommand[] {
          new DriveTestCommand(drive),
          new LauncherTestCommand(launcher),
          new IntakeTestCommand(intake),
          new BallTestCommand(ball)
        };

    Command sequentialTests =
        Commands.sequence(
            subsystemTests[0],
            subsystemTests[1],
            subsystemTests[2],
            subsystemTests[3]);

    Command backgroundTests =
        Commands.parallel(
            new ContinuousTestCommand(
                "VISION CONNECTION",
                vision::areCamerasConnected),
            new ContinuousTestCommand(
                "BATTERY VOLTAGE",
                () -> RobotController.getBatteryVoltage() >= 12.0));

    addCommands(
        Commands.runOnce(this::beginTestMode),
        Commands.deadline(sequentialTests, backgroundTests),
        Commands.runOnce(this::finishTestMode));
  }

  private void beginTestMode() {
    stopAll();
    overallAlert.set(false);
    for (SubsystemTestCommand test : subsystemTests) {
      test.clearResultAlerts();
    }
  }

  private void finishTestMode() {
    stopAll();
    overallAlert.setText("[TESTMODE] All subsystem tests finished");
    overallAlert.set(true);
  }

  private void stopAll() {
    drive.runVelocity(new ChassisSpeeds());
    launcher.stopFlywheel();
    intake.stop();
    ball.stop();
  }
}

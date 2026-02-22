package frc.robot.auto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.FieldConstants;
import frc.robot.commands.Command;
import frc.robot.commands.Commands;
import frc.robot.commands.FunctionalCommand;
import frc.robot.commands.SequentialCommandGroup;
import frc.robot.drive.Drive;
import frc.robot.intake.Intake;
import frc.robot.shooter.Shooter;
import org.littletonrobotics.junction.Logger;

/**
 * Autonomous routine for the 2026 game REBUILT.
 *
 * <p>This routine assumes a swerve drivetrain with odometry. It drives out of the community, shoots
 * into the active hub, optionally moves to a second position if time remains, and ends at a known
 * heading.
 */
public class AutoRebuilt2026 extends SequentialCommandGroup {
  // This implementation uses PID + timed/pose drive as a fallback auto path.
  // If PathPlanner is added later, replace driveToPoseCommand(...) calls with loaded paths.

  // Robot starts on the line, facing downfield.
  private static final Pose2d kStartPose = new Pose2d(1.25, 4.05, Rotation2d.kZero);

  // First position outside the community.
  private static final Pose2d kCrossedLinePose = new Pose2d(6.10, 4.05, Rotation2d.kZero);

  // Optional second scoring position.
  private static final Pose2d kSecondShotPose = new Pose2d(7.20, 3.35, Rotation2d.kZero);

  // End auto at a deterministic heading for teleop handoff.
  private static final Rotation2d kFinalHeading = Rotation2d.fromDegrees(180.0);

  private static final double kShooterRpm = 1.0;
  private static final double kFeedSeconds = 1.6;
  private static final double kSpinupTimeoutSeconds = 1.4;

  // If less than this remains in auto, skip the second scoring move.
  private static final double kMinTimeLeftForSecondMoveSeconds = 3.0;

  public AutoRebuilt2026(Drive drive, Shooter shooter, Intake intake) {
    addCommands(
        Commands.print("[AutoRebuilt2026] Starting auto routine"),

        // Initialize odometry and mechanisms to safe states.
        Commands.runOnce(
            () -> {
              boolean pathPlannerAvailable =
                  isClassAvailable("com.pathplanner.lib.auto.AutoBuilder");
              drive.setPose(kStartPose);
              drive.stop();
              shooter.setFlywheelVelocityRpm(0.0);
              intake.setIntakeOpenLoop(0.0);
              intake.setBeltOpenLoop(0.0);
              intake.setBeltStarOpenLoop(0.0);
              SmartDashboard.putBoolean(
                  "AutoRebuilt2026/PathPlannerAvailable", pathPlannerAvailable);
              Logger.recordOutput("AutoRebuilt2026/StartPose", kStartPose);
              Logger.recordOutput("AutoRebuilt2026/PathPlannerAvailable", pathPlannerAvailable);
            },
            drive,
            shooter,
            intake),

        // Move from starting line out of the community while holding heading.
        driveToPoseCommand(drive, kCrossedLinePose, 2.8).withName("DriveOutOfCommunity"),

        // Aim and fire after crossing out.
        shootAtHubCommand(drive, shooter, intake).withName("ShootAtHub"),

        // Optional second move if enough autonomous time remains.
        Commands.either(
                Commands.sequence(
                    Commands.print("[AutoRebuilt2026] Running optional second scoring move"),
                    driveToPoseCommand(drive, kSecondShotPose, 2.2).withName("DriveToSecondShot"),
                    shootAtHubCommand(drive, shooter, intake).withName("SecondShotAtHub")),
                Commands.print("[AutoRebuilt2026] Skipping optional second scoring move"),
                () -> Timer.getMatchTime() > kMinTimeLeftForSecondMoveSeconds)
            .withName("OptionalSecondScore"),

        // End auto facing a known heading.
        turnToHeadingCommand(drive, kFinalHeading, 1.6).withName("FaceKnownHeading"),
        Commands.runOnce(
            () -> {
              drive.stop();
              shooter.setFlywheelVelocityRpm(0.0);
              intake.setIntakeOpenLoop(0.0);
              intake.setBeltOpenLoop(0.0);
              intake.setBeltStarOpenLoop(0.0);
              Logger.recordOutput("AutoRebuilt2026/FinalPose", drive.getPose());
            },
            drive,
            shooter,
            intake),
        Commands.print("[AutoRebuilt2026] Auto routine complete"));
  }

  private static Command driveToPoseCommand(Drive drive, Pose2d goalPose, double timeoutSeconds) {
    PIDController xController = new PIDController(1.6, 0.0, 0.05);
    PIDController yController = new PIDController(1.6, 0.0, 0.05);
    PIDController thetaController = new PIDController(4.0, 0.0, 0.1);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    return new FunctionalCommand(
            () -> {
              xController.reset();
              yController.reset();
              thetaController.reset();
              Logger.recordOutput("AutoRebuilt2026/DriveToPoseGoal", goalPose);
            },
            () -> {
              Pose2d current = drive.getPose();

              double vx =
                  MathUtil.clamp(xController.calculate(current.getX(), goalPose.getX()), -2.3, 2.3);
              double vy =
                  MathUtil.clamp(yController.calculate(current.getY(), goalPose.getY()), -2.3, 2.3);
              double omega =
                  MathUtil.clamp(
                      thetaController.calculate(
                          current.getRotation().getRadians(), goalPose.getRotation().getRadians()),
                      -3.0,
                      3.0);

              ChassisSpeeds fieldRelative = new ChassisSpeeds(vx, vy, omega);
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelative, drive.getRotation()));

              Logger.recordOutput("AutoRebuilt2026/DriveToPoseCurrent", current);
              SmartDashboard.putNumber(
                  "AutoRebuilt2026/PoseErrorX", goalPose.getX() - current.getX());
              SmartDashboard.putNumber(
                  "AutoRebuilt2026/PoseErrorY", goalPose.getY() - current.getY());
            },
            interrupted -> drive.stop(),
            () -> {
              Pose2d current = drive.getPose();
              double translationError =
                  current.getTranslation().getDistance(goalPose.getTranslation());
              double headingError =
                  Math.abs(
                      MathUtil.angleModulus(
                          current.getRotation().minus(goalPose.getRotation()).getRadians()));
              return translationError < 0.20 && headingError < Math.toRadians(7.0);
            },
            drive)
        .withTimeout(timeoutSeconds);
  }

  private static Command shootAtHubCommand(Drive drive, Shooter shooter, Intake intake) {
    PIDController aimController = new PIDController(5.0, 0.0, 0.2);
    aimController.enableContinuousInput(-Math.PI, Math.PI);

    Command aimAndSpinUp =
        Commands.run(
                () -> {
                  Pose2d pose = drive.getPose();
                  Translation2d toHub =
                      FieldConstants.HUB.toTranslation2d().minus(pose.getTranslation());

                  Rotation2d desired = toHub.getAngle();
                  double omega =
                      MathUtil.clamp(
                          aimController.calculate(
                              pose.getRotation().getRadians(), desired.getRadians()),
                          -2.5,
                          2.5);

                  drive.runVelocity(
                      ChassisSpeeds.fromFieldRelativeSpeeds(0.0, 0.0, omega, drive.getRotation()));
                  shooter.pointAtHub();
                  shooter.setFlywheelVelocityRpm(kShooterRpm);

                  Logger.recordOutput("AutoRebuilt2026/AimDesiredHeadingDeg", desired.getDegrees());
                  Logger.recordOutput("AutoRebuilt2026/ShooterTargetRpm", kShooterRpm);
                },
                drive,
                shooter)
            .until(
                () -> {
                  Pose2d pose = drive.getPose();
                  Rotation2d desired =
                      FieldConstants.HUB.toTranslation2d().minus(pose.getTranslation()).getAngle();
                  double headingError =
                      Math.abs(
                          MathUtil.angleModulus(desired.minus(pose.getRotation()).getRadians()));
                  return headingError < Math.toRadians(3.0)
                      && shooter.isFlywheelAtSpeed(kShooterRpm, 200.0);
                })
            .withTimeout(kSpinupTimeoutSeconds)
            .finallyDo(
                () -> {
                  drive.stop();
                  Logger.recordOutput("AutoRebuilt2026/ShooterReady", true);
                });

    Command feed =
        Commands.deadline(
                Commands.waitSeconds(kFeedSeconds),
                Commands.run(
                    () -> {
                      intake.setIntakeOpenLoop(0.50);
                      intake.setBeltOpenLoop(0.85);
                      intake.setBeltStarOpenLoop(0.85);
                      shooter.setFlywheelVelocityRpm(kShooterRpm);
                    },
                    intake,
                    shooter))
            .finallyDo(
                () -> {
                  intake.setIntakeOpenLoop(0.0);
                  intake.setBeltOpenLoop(0.0);
                  intake.setBeltStarOpenLoop(0.0);
                  shooter.setFlywheelVelocityRpm(0.0);
                  Logger.recordOutput("AutoRebuilt2026/FedBalls", true);
                });

    return Commands.sequence(aimAndSpinUp, feed);
  }

  private static Command turnToHeadingCommand(
      Drive drive, Rotation2d heading, double timeoutSeconds) {
    PIDController thetaController = new PIDController(5.0, 0.0, 0.2);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    return Commands.run(
            () -> {
              double omega =
                  MathUtil.clamp(
                      thetaController.calculate(
                          drive.getRotation().getRadians(), heading.getRadians()),
                      -2.4,
                      2.4);
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(0.0, 0.0, omega, drive.getRotation()));
              Logger.recordOutput("AutoRebuilt2026/FinalHeadingTargetDeg", heading.getDegrees());
            },
            drive)
        .until(
            () ->
                Math.abs(MathUtil.angleModulus(drive.getRotation().minus(heading).getRadians()))
                    < Math.toRadians(2.0))
        .withTimeout(timeoutSeconds)
        .finallyDo(drive::stop);
  }

  private static boolean isClassAvailable(String className) {
    try {
      Class.forName(className);
      return true;
    } catch (ClassNotFoundException ex) {
      return false;
    }
  }
}

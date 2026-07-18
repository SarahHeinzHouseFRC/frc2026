package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.ShotCalculator;
import frc.robot.subsystems.ball.BallSubsystem;
import frc.robot.subsystems.ball.BallConstants;
import frc.robot.subsystems.ball.Intake;
import frc.robot.subsystems.ball.Shoot;
import frc.robot.subsystems.drive.BetterSmoothMoveCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveAutoShootCommand;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.launcher.LauncherSubsystem;
import frc.robot.subsystems.turret.AutoTurret;
import frc.robot.subsystems.turret.TurretSubsystem;

public class Autos {
  public static Command preloads() {
    return Commands.parallel(
        new Shoot(BallSubsystem.getInstance(), LauncherSubsystem.getInstance(), () -> ShotCalculator.getInstance().getShotParams().flywheelVelocityRotationsPerMinute()),
        new AutoTurret(TurretSubsystem.getInstance())
    ).withTimeout(20.0);
  }

  public static Command sweep(boolean isLeft) {
    double aLimit = 7;
    double vLimit = 4;
    return Commands.sequence(
        // move under the trench towards mid field in a straight line on the x axis
        new BetterSmoothMoveCommand(new Pose2d(6.5, .55, Rotation2d.kZero), isLeft)
            .withAccelerationLimit(aLimit + 2)
            .withVelocityLimit(vLimit + 1)
            .withTimeout(5.0),
        // rotate ourselves such that the intake is pointed towards the balls.
        // also move away from the wall while rotating so we don't break stuff.
        new BetterSmoothMoveCommand(new Pose2d(7.5, 1.0, Rotation2d.kCW_Pi_2), isLeft)
            .withAccelerationLimit(aLimit + 2)
            .withVelocityLimit(vLimit + 1)
            .withPositionTolerance(1)
            .withTimeout(5.0),
        // race means this composition finishes when either subcommand finishes
        Commands.race(
            // move toward true mid field along the y axis
            new BetterSmoothMoveCommand(new Pose2d(7.5, 3.0, Rotation2d.kCW_Pi_2), isLeft)
                .withAccelerationLimit(aLimit)
                .withVelocityLimit(.6)
                .withTimeout(4.0),
            // also run our overbumper intake while we do this
            new Intake(BallSubsystem.getInstance(), IntakeSubsystem.getInstance(), () -> 1.0)),
        // move back along the y axis so we are aligned x-wise to the trench
        new BetterSmoothMoveCommand(new Pose2d(7.5, .55, Rotation2d.kZero), isLeft)
            .withAccelerationLimit(aLimit)
            .withVelocityLimit(vLimit)
            .withTimeout(5.0),
        // drive through the trench so that we can legally score
        new BetterSmoothMoveCommand(new Pose2d(3.0, .55, Rotation2d.kZero), isLeft)
            .withAccelerationLimit(aLimit + 1)
            .withVelocityLimit(vLimit + 1)
            .withTimeout(5.0),
        // in parallel...
        Commands.parallel(
            new Shoot(BallSubsystem.getInstance(), LauncherSubsystem.getInstance(), () -> ShotCalculator.getInstance().getShotParams().flywheelVelocityRotationsPerMinute()) // also autoaim and shoot at the same time
                .withTimeout(7.0),
            new DriveAutoShootCommand(Drive.getInstance()),
            IntakeSubsystem.getInstance().shakeCommand()
        ),
        new BetterSmoothMoveCommand(new Pose2d(6.5, .55, Rotation2d.kZero), isLeft)
            .withAccelerationLimit(aLimit + 2)
            .withVelocityLimit(vLimit + 1)
            .withTimeout(5.0),
        // rotate ourselves such that the intake is pointed towards the balls.
        // also move away from the wall while rotating so we don't break stuff.
        new BetterSmoothMoveCommand(new Pose2d(7.5, 3.0, Rotation2d.kCW_Pi_2), isLeft)
            .withAccelerationLimit(aLimit + 1)
            .withVelocityLimit(vLimit + 1)
            .withPositionTolerance(1)
            .withTimeout(5.0),
        // race means this composition finishes when either subcommand finishes
        Commands.race(
            // move toward true mid field along the y axis
            new BetterSmoothMoveCommand(new Pose2d(7.5, 5.0, Rotation2d.kCW_Pi_2), isLeft)
                .withAccelerationLimit(aLimit)
                .withVelocityLimit(.6)
                .withTimeout(5.0),
            // also run our overbumper intake while we do this
            new Intake(BallSubsystem.getInstance(), IntakeSubsystem.getInstance(), () -> 1.0)),
        // move back along the y axis so we are aligned x-wise to the trench
        new BetterSmoothMoveCommand(new Pose2d(7.5, .55, Rotation2d.kZero), isLeft)
            .withAccelerationLimit(aLimit)
            .withVelocityLimit(vLimit)
            .withTimeout(5.0),
        // drive through the trench so that we can legally score
        new BetterSmoothMoveCommand(new Pose2d(3.0, .55, Rotation2d.kZero), isLeft)
            .withAccelerationLimit(aLimit + 1)
            .withVelocityLimit(vLimit + 1)
            .withTimeout(5.0),
        // in parallel...
        Commands.parallel(
            new Shoot(BallSubsystem.getInstance(), LauncherSubsystem.getInstance(), () -> ShotCalculator.getInstance().getShotParams().flywheelVelocityRotationsPerMinute()) // also autoaim and shoot at the same time
                .withTimeout(7.0),
            new DriveAutoShootCommand(Drive.getInstance()),
            IntakeSubsystem.getInstance().shakeCommand()
        ))
    .withTimeout(20.0);
  }

}

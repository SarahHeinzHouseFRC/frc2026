package frc.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.drive.BetterSmoothMoveCommand;
import frc.robot.drive.SmoothMoveCommand;
import frc.robot.intake.Intake;
import frc.robot.intake.IntakeAutoCommand;
import frc.robot.overbumper.OverBumper;
import frc.robot.overbumper.ShakeCommand;
import frc.robot.shooter.Shooter;

public class AutoWeekZero {
  public static Command autoV1() {
    return Commands.sequence(
            Commands.parallel(
                    Commands.sequence(
                        Commands.waitSeconds(2), new IntakeAutoCommand(Intake.getInstance())),
                    new ShakeCommand(OverBumper.getInstance()),
                    Shooter.getInstance().autoAimCommandAuto())
                .withDeadline(Commands.waitSeconds(5)),
            Commands.parallel(
                    Commands.sequence(
                        new SmoothMoveCommand(new Pose2d(.60, .65, Rotation2d.kZero))
                            .withAccelerationLimit(2)
                            .withVelocityLimit(2),
                        new SmoothMoveCommand(new Pose2d(.20, .65, Rotation2d.kZero))
                            .withAccelerationLimit(1)
                            .withVelocityLimit(.5)
                            .withDeadline(Commands.waitSeconds(4.0))),
                    Shooter.getInstance().autoAimCommandAutoDryish())
                .withDeadline(Commands.waitSeconds(10.0)),
            Commands.parallel(
                    new IntakeAutoCommand(Intake.getInstance()),
                    new ShakeCommand(OverBumper.getInstance()),
                    Shooter.getInstance().autoAimCommandAuto())
                .withDeadline(Commands.waitSeconds(15)))
        .withDeadline(Commands.waitSeconds(20));
  }

  public static Command depot() {
    return Commands.sequence(
            Commands.parallel(
                    Commands.sequence(
                        Commands.waitSeconds(2), new IntakeAutoCommand(Intake.getInstance())),
                    new ShakeCommand(OverBumper.getInstance()),
                    Shooter.getInstance().autoAimCommandAuto())
                .withDeadline(Commands.waitSeconds(5)),
            Commands.parallel(
                    new SmoothMoveCommand(new Pose2d(1.5, 6.0, Rotation2d.kZero))
                        .withAccelerationLimit(2)
                        .withVelocityLimit(2),
                    Shooter.getInstance().autoAimCommandAutoDryish())
                .withDeadline(Commands.waitSeconds(2.5)),
            Commands.parallel(
                    new SmoothMoveCommand(new Pose2d(.50, 6.0, Rotation2d.kZero))
                        .withAccelerationLimit(3)
                        .withVelocityLimit(1.5),
                    Shooter.getInstance().autoAimCommandAuto(),
                    OverBumper.getInstance().intakeCommand(),
                    new IntakeAutoCommand(Intake.getInstance()))
                .withDeadline(Commands.waitSeconds(5)),
            Commands.parallel(
                    new SmoothMoveCommand(new Pose2d(.75, 6.0, Rotation2d.kZero))
                        .withAccelerationLimit(2)
                        .withVelocityLimit(1),
                    new IntakeAutoCommand(Intake.getInstance()),
                    new ShakeCommand(OverBumper.getInstance()),
                    Shooter.getInstance().autoAimCommandAuto())
                .withDeadline(Commands.waitSeconds(15)))
        .withDeadline(Commands.waitSeconds(20));
  }

  public static Command center() {
    return Commands.parallel(
            Commands.sequence(Commands.waitSeconds(2), new IntakeAutoCommand(Intake.getInstance())),
            new ShakeCommand(OverBumper.getInstance()),
            Shooter.getInstance().autoAimCommandAuto())
        .withDeadline(Commands.waitSeconds(20));
  }

  public static Command rightSweep() {
    double aLimit = 7;
    double vLimit = 4;
    return Commands.sequence(
        Commands.parallel(
                Commands.sequence(
                    Commands.waitSeconds(2), new IntakeAutoCommand(Intake.getInstance())),
                new ShakeCommand(OverBumper.getInstance()),
                Shooter.getInstance().autoAimCommandAuto())
            .withTimeout(5.0),
//        new BetterSmoothMoveCommand(new Pose2d(3.0, .55, Rotation2d.kZero))
//            .withAccelerationLimit(aLimit)
//            .withVelocityLimit(vLimit)
//            .withDeadline(Commands.waitSeconds(5)),
        new BetterSmoothMoveCommand(new Pose2d(7.5, .55, Rotation2d.kZero))
            .withAccelerationLimit(aLimit)
            .withVelocityLimit(vLimit)
            .withTimeout(5.0),
        new BetterSmoothMoveCommand(new Pose2d(7.5, 1.0, Rotation2d.kCW_Pi_2))
            .withAccelerationLimit(aLimit)
            .withVelocityLimit(vLimit)
            .withPositionTolerance(1)
            .withTimeout(5.0),
        Commands.race(
            new BetterSmoothMoveCommand(new Pose2d(7.5, 3.0, Rotation2d.kCW_Pi_2))
                .withAccelerationLimit(aLimit)
                .withVelocityLimit(.65)
                .withTimeout(5.0),
            OverBumper.getInstance().intakeCommand(3500)),
        new BetterSmoothMoveCommand(new Pose2d(7.5, .55, Rotation2d.kZero))
            .withAccelerationLimit(aLimit)
            .withVelocityLimit(vLimit)
            .withTimeout(5.0),
        new BetterSmoothMoveCommand(new Pose2d(2.0, .55, Rotation2d.kZero))
            .withAccelerationLimit(aLimit)
            .withVelocityLimit(vLimit)
            .withTimeout(5.0),
        Commands.parallel(
                new IntakeAutoCommand(Intake.getInstance()),
                new ShakeCommand(OverBumper.getInstance()),
                Shooter.getInstance().autoAimCommandAuto())
            .withTimeout(5.0)).withDeadline(Commands.waitSeconds(20.0));
  }
}

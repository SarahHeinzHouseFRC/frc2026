package frc.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
            new SmoothMoveCommand(new Pose2d(.75, .65, Rotation2d.kZero))
            .withAccelerationLimit(2)
            .withVelocityLimit(1),
            Shooter.getInstance().autoAimCommandAutoDryish()
        ).withDeadline(Commands.waitSeconds(5)),
        Commands.parallel(
                new IntakeAutoCommand(Intake.getInstance()),
                new ShakeCommand(OverBumper.getInstance()),
                Shooter.getInstance().autoAimCommandAuto())
            .withDeadline(Commands.waitSeconds(10))
    ).withDeadline(Commands.waitSeconds(20));
  }
}

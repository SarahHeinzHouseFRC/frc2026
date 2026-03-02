package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.intake.Intake;
import frc.robot.intake.IntakeAutoCommand;
import frc.robot.shooter.Shooter;

public class AutoWeekZero {
  public static Command autoV1() {
    return Commands.sequence(
        Commands.parallel(
                Commands.sequence(
                    Commands.waitSeconds(2), new IntakeAutoCommand(Intake.getInstance())),
                Shooter.getInstance().autoAimCommandAuto())
            .withDeadline(Commands.waitSeconds(10))
        //        new SmoothMoveCommand(new Pose2d(1.160, 4.448,
        // Rotation2d.kZero)).withAccelerationLimit(2).withVelocityLimit(2);
        );
  }
}

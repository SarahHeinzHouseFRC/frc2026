package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.drive.SmoothMoveCommand;
import frc.robot.intake.Intake;
import frc.robot.intake.IntakeAutoCommand;
import frc.robot.shooter.Shooter;

public class AutoWeekZero {
  public static Command autoV1() {
    return Commands.sequence(
        Commands.parallel(new IntakeAutoCommand(Intake.getInstance()), Shooter.getInstance().autoAimCommandAuto()).withDeadline(Commands.waitSeconds(10))
        //X: 1.160m, Y: 4.448m,
    );
  }
}

package frc.robot.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class IntakeAutoCommand extends Command {
  private final Intake intake;
  private double startTime = 0;



  public IntakeAutoCommand(Intake intake) {
    addRequirements(intake);
    this.intake = intake;
  }

  @Override
  public void initialize() {
    startTime = Timer.getTimestamp();
  }

  @Override
  public void execute() {
    double running = (Timer.getTimestamp() - startTime) > 2.0 ? 1 : 0;


    intake.setIntakeOpenLoop(running * -1);
    intake.setBeltStarOpenLoop(running * -1);
    intake.setBeltOpenLoop(running * 1);
    intake.setAgitatorOpenLoop(running * .5);
    //    System.out.println("intake: " + intakeMotor + ", index: " + indexerMotor + ", belt:" +
    // beltMotor);

      intake.setOBIOpenLoop(0);


    intake.editObiSetpoint(0);
  }
}

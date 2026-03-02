package frc.robot.overbumper;

import static frc.robot.overbumper.OverBumperConstants.*;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class OverBumperControllerCommand extends Command {
  private final OverBumper overBumper;
  private final XboxController driverController;

  public OverBumperControllerCommand(XboxController driver, OverBumper overBumper) {
    this.overBumper = overBumper;
    this.driverController = driver;
    addRequirements(overBumper);
  }

  @Override
  public void execute() {
    if (driverController.getLeftTriggerAxis() > .1) {
      switch (Robot.VERSION) {
        case V1:
          overBumper.setOBIClosedLoop(driverController.getLeftTriggerAxis() * 1600);
          break;
        case V2:
          overBumper.setOBIClosedLoop(
              driverController.getLeftTriggerAxis()
                  * 2250
                  * (driverController.getLeftBumperButton() ? -1 : 1));
          break;
      }
    } else {
      overBumper.setOBIOpenLoop(0);
    }

    if (driverController.getPOV() == 0) overBumper.setObiSetpoint(presetStowed);
    if (driverController.getPOV() == 90) overBumper.editObiSetpoint(-changeSpeed);
    if (driverController.getPOV() == 180) overBumper.setObiSetpoint(presetEngaged);
    if (driverController.getPOV() == 270) overBumper.editObiSetpoint(changeSpeed);
  }
}

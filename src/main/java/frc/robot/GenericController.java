package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;

public class GenericController {
  private GenericHID controller;

  public GenericController(GenericHID controller) {
    this.controller = controller;
  }

  public boolean readDigital(int input) {
    if (controller instanceof XboxController) {
      return switch (input) {
        case 0 -> ((XboxController) controller).getLeftBumperButton();
        case 1 -> ((XboxController) controller).getRightBumperButton();
        case 2 -> ((XboxController) controller).getYButton();
        case 3 -> ((XboxController) controller).getXButton();
        case 4 -> ((XboxController) controller).getBButton();
        case 5 -> ((XboxController) controller).getAButton();
        case 6 -> ((XboxController) controller).getLeftBumperButtonPressed();
        case 7 -> ((XboxController) controller).getRightBumperButtonPressed();
        case 8 -> ((XboxController) controller).getYButtonPressed();
        case 9 -> ((XboxController) controller).getXButtonPressed();
        case 10 -> ((XboxController) controller).getBButtonPressed();
        case 11 -> ((XboxController) controller).getAButtonPressed();
        default -> throw new IllegalStateException(
            "Invalid input " + input + " for XboxController");
      };
    } else {
      throw new IllegalStateException("Controller has no valid mapping");
    }
  }

  public double readAnalog(int input) {
    if (controller instanceof XboxController) {
      return switch (input) {
        case 0 -> ((XboxController) controller).getLeftTriggerAxis();
        case 1 -> ((XboxController) controller).getRightTriggerAxis();
        case 2 -> ((XboxController) controller).getLeftX();
        case 3 -> ((XboxController) controller).getLeftY();
        case 4 -> ((XboxController) controller).getRightX();
        case 5 -> ((XboxController) controller).getRightY();
        case 6 -> ((XboxController) controller).getPOV();
        default -> throw new IllegalStateException(
            "Invalid input " + input + " for XboxController");
      };
    } else {
      throw new IllegalStateException("Controller has no valid mapping");
    }
  }
}

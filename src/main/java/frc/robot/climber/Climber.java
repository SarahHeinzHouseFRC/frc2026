package frc.robot.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import java.util.function.DoubleSupplier;

public class Climber extends SubsystemBase {
  private ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
  private ClimberIO io;

  private static Climber instance = null;

  public static void init() {
    if (instance != null) {
      throw new IllegalStateException("climber already init");
    }
    instance = new Climber();
  }

  public static Climber getInstance() {
    if (instance == null) {
      throw new IllegalStateException("climber is null");
    }
    return instance;
  }

  private Climber() {
    if (Robot.currentMode == Robot.Mode.REAL) {
      io = new ClimberIOSpark();
    }
  }

  public static Command climbCommand(DoubleSupplier supplier) {
    return Commands.run(
        () -> {
          instance.io.runClimberOpenLoop(supplier.getAsDouble() * 12);
        },
        instance);
  }
}

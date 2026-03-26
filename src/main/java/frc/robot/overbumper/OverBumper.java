package frc.robot.overbumper;

import static frc.robot.overbumper.OverBumperConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class OverBumper extends SubsystemBase {
  @AutoLogOutput private double obiSetpoint = presetEngaged;

  private final OverBumperIO io =
      switch (Robot.currentMode) {
        case REAL -> new OverBumperIOSpark();
        default -> new OverBumperIO() {};
      };
  private final OverBumperIOInputsAutoLogged inputs = new OverBumperIOInputsAutoLogged();

  private static OverBumper instance = null;

  public static void init() {
    if (instance != null) {
      throw new IllegalStateException("OverBumper already initialized.");
    }
    instance = new OverBumper();
  }

  public double getPosition() {
    return MathUtil.inputModulus(inputs.obiPosition, -.5, .5);
  }

  public boolean isDeployedish() {
    return MathUtil.isNear(getPosition(), 0, .15);
  }

  public static OverBumper getInstance() {
    if (instance == null) {
      throw new IllegalStateException("OverBumper not initialized.");
    }
    return instance;
  }

  private OverBumper() {}

  public void setOBIClosedLoop(double rpm) {
    if (Robot.VERSION != Robot.RobotVersion.V2) {
      io.setOBIClosedLoop(rpm);

    } else {
      double vel = inputs.obiSpeed;
      if (Math.abs(vel) < Math.abs(Math.min(60, rpm * .03))) {
        // full send
        io.setOBIOpenLoop(12 * Math.signum(rpm));
      } else {
        io.setOBIClosedLoop(rpm);
      }
    }
  }

  public void setOBIOpenLoop(double speed) {
    io.setOBIOpenLoop(speed * 12);
  }

  public void editObiSetpoint(double change) {
    obiSetpoint += change;
    io.setOBIPivotMotorClosedLoop(obiSetpoint);
  }

  public void setObiSetpoint(double setpoint) {
    obiSetpoint = setpoint;
    io.setOBIPivotMotorClosedLoop(obiSetpoint);
  }

  public double getObiSetpoint() {
    return obiSetpoint;
  }

  public void stopPivot() {
    io.setOBIPivotMotorOpenLoop(0);
  }

  public void stopIntake() {
    io.setOBIOpenLoop(0);
  }

  public Command intakeCommand() {
    return Commands.run(
        () -> {
          setObiSetpoint(presetEngaged);
          setOBIClosedLoop(2250);
        },
        this);
  }

  public Command intakeCommand(double speed) {
    return Commands.run(
        () -> {
          setObiSetpoint(presetEngaged);
          setOBIClosedLoop(speed);
        },
        this);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("OverBumper", inputs);
  }
}

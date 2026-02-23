package frc.robot.intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  @AutoLogOutput private double obiSetpoint = 0.0;

  public static Intake instance = null;

  public static void init() {
    if (instance != null) {
      throw new IllegalStateException("Intake instance already initialized.");
    }
    instance = new Intake();
  }

  public static Intake getInstance() {
    if (instance == null) {
      throw new IllegalStateException("Intake instance not initialized.");
    }
    return instance;
  }

  private Intake() {
    instance = this;
    io =
        switch (Robot.currentMode) {
          case REAL -> new IntakeIOSpark();
          default -> new IntakeIO() {};
        };
  }

  public void setBeltOpenLoop(double speed) {
    io.setBeltOpenLoop(speed * 12.0);
  }

  public void setIntakeOpenLoop(double speed) {
    io.setIntakeOpenLoop(speed * 12.0);
    SmartDashboard.putNumber("intake speed", speed);
  }

  public void setBeltStarOpenLoop(double speed) {
    io.setBeltStarOpenLoop(speed * 12.0);
  }

  public void setOBIClosedLoop(double rpm) {
    io.setOBIClosedLoop(rpm);
  }

  public void setOBIOpenLoop(double speed) {
    io.setOBIOpenLoop(speed);
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

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  public void setAgitatorOpenLoop(double speed) {
    io.setAgitatorOpenLoop(speed * 12.0);
  }
}

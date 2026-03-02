package frc.robot.intake;

import static frc.robot.intake.IntakeConstants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

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

  public void intakeAndShoot() {
    io.setIntakeOpenLoop(-12.0);
    io.setIndexerOpenLoop(6.0);
    io.setBeltOpenLoop(-12.0);
    io.setAgitatorOpenLoop(0.0);
  }

  public void intake() {
    io.setIntakeOpenLoop(-12.0);
    io.setIndexerOpenLoop(-12.0);
    io.setBeltOpenLoop(-12.0);
    io.setAgitatorOpenLoop(-12.0);
  }

  public void shoot() {
    io.setIntakeOpenLoop(0.0);
    io.setIndexerOpenLoop(12.0);
    io.setBeltOpenLoop(-12.0);
    io.setAgitatorOpenLoop(-12.0);
  }

  // "Pulling out is positive" - Max Choset
  public void outtake() {
    io.setIntakeOpenLoop(12.0);
    io.setIndexerOpenLoop(12.0);
    io.setBeltOpenLoop(12.0);
    io.setAgitatorOpenLoop(12.0);
  }

  // only to be used when one is certain that no balls will be caught in intake
  public void autoShoot() {
    io.setIntakeOpenLoop(-1);
    io.setIndexerOpenLoop(-1);
    io.setBeltOpenLoop(1);
    io.setAgitatorOpenLoop(.5);
  }

  public void stop() {
    io.setIntakeOpenLoop(0.0);
    io.setIndexerOpenLoop(0.0);
    io.setBeltOpenLoop(0.0);
    io.setAgitatorOpenLoop(0.0);
  }

  public void setBeltOpenLoop(double speed) {
    io.setBeltOpenLoop(speed * 12.0);
  }

  public void setIntakeOpenLoop(double speed) {
    io.setIntakeOpenLoop(speed * 12.0);
  }

  public void setIndexerOpenLoop(double speed) {
    io.setIndexerOpenLoop(speed * 12.0);
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

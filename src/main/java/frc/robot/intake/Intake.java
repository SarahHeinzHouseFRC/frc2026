package frc.robot.intake;

import static frc.robot.intake.IntakeConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public static Intake instance = null;

  public final MotorOscillator beltOscillator = new MotorOscillator();

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
    beltOscillator.setAmplitude(4);
    beltOscillator.setCenter(8);
    beltOscillator.setPeriod(.75);
  }

  public void oscillateBelt(double speed) {
    if (!beltOscillator.isOn()) beltOscillator.on();
    io.setBeltOpenLoop(beltOscillator.getValue() * speed);
  }

  public void intakeAndShoot() {
    intakeAndShoot(1.0);
  }

  public void intakeAndShoot(double speed) {
    speed = MathUtil.clamp(speed, 0, 1);
    io.setIntakeOpenLoop(-12.0 * speed);
    io.setIndexerOpenLoop(6.0 * speed);
    oscillateBelt(-speed);
  }

  public void intake() {
    intake(1.0);
  }

  public void intake(double speed) {
    speed = MathUtil.clamp(speed, 0, 1);
    io.setIntakeOpenLoop(-12.0 * speed);
    io.setIndexerOpenLoop(-12.0 * speed);
    oscillateBelt(-speed);
  }

  public void shoot() {
    shoot(1.0);
  }

  public void shoot(double speed) {
    speed = MathUtil.clamp(speed, 0, 1);
    io.setIntakeOpenLoop(-12.0 * speed);
    io.setIndexerOpenLoop(9.6 * speed);
    oscillateBelt(speed);
  }

  public void outtake() {
    outtake(1.0);
  }

  // "Pulling out is positive" - Max Choset
  public void outtake(double speed) {
    speed = MathUtil.clamp(speed, 0, 1);
    io.setIntakeOpenLoop(12.0 * speed);
    io.setIndexerOpenLoop(12.0 * speed);
    oscillateBelt(speed);
  }

  // only to be used when one is certain that no balls will be caught in intake
  public void autoShoot() {
    io.setIntakeOpenLoop(-12.0);
    io.setIndexerOpenLoop(12.0);
    oscillateBelt(1);
  }

  public void stop() {
    io.setIntakeOpenLoop(0.0);
    io.setIndexerOpenLoop(0.0);
    io.setBeltOpenLoop(0.0);
    beltOscillator.off();
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
}

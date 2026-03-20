package frc.robot.intake;

import edu.wpi.first.wpilibj.Timer;

public class MotorOscillator {
  private boolean on = false;
  private double startTime = Timer.getTimestamp();
  private double period = 1.0;
  private double amplitude = 1.0;
  private double center = 0.0;

  public void setPeriod(double periodSeconds) {
    period = periodSeconds;
  }

  public void setAmplitude(double amplitude) {
    this.amplitude = amplitude;
  }

  public void setCenter(double center) {
    this.center = center;
  }

  public boolean isOn() {
    return on;
  }

  public void on() {
    on = true;
    startTime = Timer.getTimestamp();
  }

  public double getValue() {
    if (on) {
      return center
          + amplitude * Math.sin(2 * Math.PI * (Timer.getTimestamp() - startTime) / period);
    }
    return 0.0;
  }

  public void off() {
    on = false;
  }
}

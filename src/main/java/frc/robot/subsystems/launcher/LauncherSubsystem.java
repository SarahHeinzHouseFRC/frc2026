package frc.robot.subsystems.launcher;

import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.SharpSubsystem;
import frc.robot.utils.SparkUtils;

import java.util.function.DoubleSupplier;

import static com.revrobotics.PersistMode.kPersistParameters;
import static com.revrobotics.ResetMode.kResetSafeParameters;
import static com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kCoast;

public class LauncherSubsystem extends SharpSubsystem {
  private final static LauncherSubsystem instance = new LauncherSubsystem();
  public static LauncherSubsystem getInstance() {
    return instance;
  }

  private double setpoint = 0;

  private final SparkFlex flywheel1 = new SparkFlex(32, SparkLowLevel.MotorType.kBrushless);
  private final SparkFlex flywheel2 = new SparkFlex(33, SparkLowLevel.MotorType.kBrushless);
  private final RelativeEncoder flywheel1Encoder = flywheel1.getEncoder();
  private final RelativeEncoder flywheel2Encoder = flywheel2.getEncoder();
  private final SparkClosedLoopController flywheelController = flywheel1.getClosedLoopController();

  private LauncherSubsystem() {
    SparkFlexConfig baseFlywheelConfig = new SparkFlexConfig();
    baseFlywheelConfig.voltageCompensation(12.0);
    baseFlywheelConfig.smartCurrentLimit(40).idleMode(kCoast);
    baseFlywheelConfig.closedLoop.pid(0.002, 0, 0, ClosedLoopSlot.kSlot0);
    baseFlywheelConfig.closedLoop.feedForward.kV(0.00022, ClosedLoopSlot.kSlot0);
    baseFlywheelConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    baseFlywheelConfig.closedLoop.maxMotion.maxAcceleration(12000).allowedProfileError(1000);
    baseFlywheelConfig.openLoopRampRate(0.5);
    baseFlywheelConfig.closedLoopRampRate(0.5);
    baseFlywheelConfig.encoder.velocityConversionFactor(1);
    baseFlywheelConfig
        .encoder
        .uvwAverageDepth(2)
        .quadratureAverageDepth(2)
        .uvwMeasurementPeriod(10)
        .quadratureMeasurementPeriod(10);
    SparkFlexConfig flywheel1Config = new SparkFlexConfig();
    flywheel1Config.apply(baseFlywheelConfig).inverted(true);
    flywheel1Config.signals.appliedOutputPeriodMs(5);

    REVLibError flywheel1error = SparkUtils.tryUntilOk(() -> flywheel1.configure(flywheel1Config, kResetSafeParameters, kPersistParameters));
    if (flywheel1error != REVLibError.kOk) {
      new Alert("[LAUNCHER] Error configuring flywheel1: " + flywheel1error, Alert.AlertType.kError).set(true);
    }

    SparkFlexConfig flywheel2Config = new SparkFlexConfig();
    flywheel2Config.apply(baseFlywheelConfig).inverted(false);
    flywheel2Config.follow(32, true);

    REVLibError flywheel2error = SparkUtils.tryUntilOk(() -> flywheel2.configure(flywheel2Config, kResetSafeParameters, kPersistParameters));
    if (flywheel2error != REVLibError.kOk) {
      new Alert("[LAUNCHER] Error configuring flywheel2: " + flywheel2error, Alert.AlertType.kError).set(true);
    }

    setDefaultCommand(new RunCommand(this::stopFlywheel, this));
  }

  public void setFlywheelSetpoint(double setpoint) {
    if (setpoint <= 0) { stopFlywheel(); return; }
    this.setpoint = Math.min(setpoint, 6000);
    flywheelController.setSetpoint(this.setpoint, SparkBase.ControlType.kVelocity, ClosedLoopSlot.kSlot0);
  }

  public double getFlywheelVelocity() {
    return flywheel1Encoder.getVelocity();
  }

  public void stopFlywheel() {
    setpoint = 0;
    flywheel1.set(0);
  }

  public boolean isFlywheelAtSpeed() {
    return Math.abs(getFlywheelVelocity() - setpoint) < Math.max(100, setpoint * 0.05) && setpoint != 0;
  }

  public Command launcherSpeedCommand(DoubleSupplier speedSupplier) {
    return new FunctionalCommand(
        () -> {},
        () -> setFlywheelSetpoint(speedSupplier.getAsDouble()),
        interrupted -> stopFlywheel(),
        () -> false,
        this
    );
  }
}

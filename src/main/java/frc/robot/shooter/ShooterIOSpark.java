package frc.robot.shooter;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import static com.revrobotics.PersistMode.*;
import static com.revrobotics.ResetMode.*;
import static com.revrobotics.spark.SparkLowLevel.MotorType.*;
import static com.revrobotics.spark.config.SparkBaseConfig.IdleMode.*;

public class ShooterIOSpark implements ShooterIO {
    public SparkMax panMotor;
    public SparkMax tiltMotor;
    public SparkMax flywheelMotor;
    public ShooterIOSpark() {
        SparkMaxConfig panConfig = new SparkMaxConfig();
        panConfig
                .smartCurrentLimit(40)
                .idleMode(kBrake)
                .inverted(false);
        panConfig.closedLoop.pid(0, 0, 0, ClosedLoopSlot.kSlot0);
        panMotor = new SparkMax(-1, kBrushed);
        panMotor.configure(panConfig, kResetSafeParameters, kPersistParameters);

        SparkMaxConfig tiltConfig = new SparkMaxConfig();
        tiltConfig
                .smartCurrentLimit(40)
                .idleMode(kBrake)
                .inverted(false);
        tiltConfig.closedLoop.pid(0, 0, 0, ClosedLoopSlot.kSlot0);
        tiltMotor = new SparkMax(-1, kBrushed);
        tiltMotor.configure(tiltConfig, kResetSafeParameters, kPersistParameters);

        SparkMaxConfig flywheelConfig = new SparkMaxConfig();
        flywheelConfig
                .smartCurrentLimit(40)
                .idleMode(kCoast)
                .inverted(false);
        flywheelConfig.closedLoop.pid(0, 0, 0, ClosedLoopSlot.kSlot0);
        flywheelMotor = new SparkMax(-1, kBrushed);
        flywheelMotor.configure(tiltConfig, kResetSafeParameters, kPersistParameters);
    }

    public void updateInputs(ShooterIOInputs inputs) {
        inputs.flywheelVelocityRotationsPerSecond = flywheelMotor.getEncoder().getVelocity();
        inputs.turretPitchRadians = tiltMotor.getAbsoluteEncoder().getPosition();
        inputs.turretYawRadians = panMotor.getAbsoluteEncoder().getPosition();
    }
    public void setFlywheelVelocity(double speedRotationsPerSecond) {
        flywheelMotor.getClosedLoopController().setSetpoint(speedRotationsPerSecond, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    }
    public void setTurretAngle(double yawRadians, double pitchRadians) {
        setTurretYaw(yawRadians);
        setTurretPitch(pitchRadians);
    }

    public void setTurretPitch(double pitchRadians) {
        tiltMotor.getClosedLoopController().setSetpoint(pitchRadians, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }
    public void setTurretYaw(double yawRadians) {
        panMotor.getClosedLoopController().setSetpoint(yawRadians, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }
    public void setTurretPitchOpenLoop(double voltage) {
        tiltMotor.setVoltage(voltage);
    }
    public void setTurretYawOpenLoop(double voltage) {
        panMotor.setVoltage(voltage);
    }
    public void setFlywheelOpenLoop(double voltage) {
        flywheelMotor.setVoltage(voltage);
    }
}

package frc.robot.shooter;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;

import static com.revrobotics.PersistMode.*;
import static com.revrobotics.ResetMode.*;
import static com.revrobotics.spark.SparkLowLevel.MotorType.*;
import static com.revrobotics.spark.config.SparkBaseConfig.IdleMode.*;

public class ShooterIOSpark implements ShooterIO {
    public SparkFlex panMotor;
    public SparkFlex tiltMotor;
    public SparkFlex flywheelMotor;
    private final int panMotorCanId = -1;
    private final int tiltMotorCanId = 32;
    private final int flywheelMotorCanId = 30;
    public ShooterIOSpark() {
        SparkFlexConfig panConfig = new SparkFlexConfig();
        panConfig
                .smartCurrentLimit(40)
                .idleMode(kBrake)
                .inverted(false);
        panConfig.closedLoop.pid(0, 0, 0, ClosedLoopSlot.kSlot0);
        panMotor = new SparkFlex(panMotorCanId, kBrushless);
        panMotor.configure(panConfig, kResetSafeParameters, kPersistParameters);

        SparkFlexConfig tiltConfig = new SparkFlexConfig();
        tiltConfig
                .smartCurrentLimit(40)
                .idleMode(kBrake)
                .inverted(false);
        tiltConfig.closedLoop.pid(0, 0, 0, ClosedLoopSlot.kSlot0);
        tiltMotor = new SparkFlex(tiltMotorCanId, kBrushless);
        tiltMotor.configure(tiltConfig, kResetSafeParameters, kPersistParameters);

        SparkFlexConfig flywheelConfig = new SparkFlexConfig();
        flywheelConfig
                .smartCurrentLimit(40)
                .idleMode(kCoast)
                .inverted(false);
        flywheelConfig.closedLoop.pid(0, 0, 0, ClosedLoopSlot.kSlot0);
        flywheelMotor = new SparkFlex(flywheelMotorCanId, kBrushless);
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

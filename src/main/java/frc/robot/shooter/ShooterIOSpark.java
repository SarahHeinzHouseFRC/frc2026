package frc.robot.shooter;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static com.revrobotics.PersistMode.*;
import static com.revrobotics.ResetMode.*;
import static com.revrobotics.spark.SparkLowLevel.MotorType.*;
import static com.revrobotics.spark.config.SparkBaseConfig.IdleMode.*;

public class ShooterIOSpark implements ShooterIO {
    private final Servo linearActuator;
    private final SparkFlex panMotor;
    private final SparkFlex flywheelMotor;
    private final SparkFlex flywheelMotor2;
    private final int panMotorCanId = 31;
    private final int flywheelMotorCanId = 32;
    private final int flywheelMotor2CanId = 33;
    private final int linearActuatorServoPwmId = 0;
    private final int linearActuatorLengthMm = 100;
    private final int linearActuatorSpeedMmPerSec = 32;
    private double linearActuatorSetpoint = 0;
    public ShooterIOSpark() {
        linearActuator = new Servo(linearActuatorServoPwmId);
        linearActuator.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1075);

        SparkFlexConfig panConfig = new SparkFlexConfig();
        panConfig
                .smartCurrentLimit(40)
                .idleMode(kBrake)
                .inverted(false);
        panConfig.closedLoop.pid(0, 0, 0, ClosedLoopSlot.kSlot0);
        panMotor = new SparkFlex(panMotorCanId, kBrushless);
        panMotor.configure(panConfig, kResetSafeParameters, kPersistParameters);

        // this motor is not connected right now :(
//        SparkFlexConfig tiltConfig = new SparkFlexConfig();
//        tiltConfig
//                .smartCurrentLimit(40)
//                .idleMode(kBrake)
//                .inverted(false);
//        tiltConfig.closedLoop.pid(1, 0, 0.2, ClosedLoopSlot.kSlot0);
//        tiltMotor = new SparkFlex(tiltMotorCanId, kBrushless);
//        tiltMotor.configure(tiltConfig, kResetSafeParameters, kPersistParameters);

//        tiltMotor.getEncoder().setPosition(0);

        SparkFlexConfig flywheelConfig = new SparkFlexConfig();
        flywheelConfig
                .smartCurrentLimit(40)
                .idleMode(kCoast)
                .inverted(true);
        flywheelConfig.closedLoop.pid(0, 0, 0, ClosedLoopSlot.kSlot0);
        flywheelConfig.openLoopRampRate(0.1);
        flywheelConfig.closedLoopRampRate(0.1);
        flywheelMotor = new SparkFlex(flywheelMotorCanId, kBrushless);
        flywheelMotor.configure(flywheelConfig, kResetSafeParameters, kPersistParameters);

        SparkFlexConfig flywheel2Config = new SparkFlexConfig();
        flywheel2Config
                .smartCurrentLimit(40)
                .idleMode(kCoast)
                .inverted(false);
        flywheel2Config.closedLoop.pid(0, 0, 0, ClosedLoopSlot.kSlot0);
        flywheel2Config.openLoopRampRate(0.1);
        flywheel2Config.closedLoopRampRate(0.1);
        flywheelMotor2 = new SparkFlex(flywheelMotor2CanId, kBrushless);
        flywheelMotor2.configure(flywheel2Config, kResetSafeParameters, kPersistParameters);
    }

    public void updateInputs(ShooterIOInputs inputs) {
        inputs.flywheelVelocityRotationsPerSecond = flywheelMotor.getEncoder().getVelocity();
//        inputs.turretPitchRadians = tiltMotor.getAbsoluteEncoder().getPosition();
        inputs.turretYawRadians = panMotor.getAbsoluteEncoder().getPosition();
//        SmartDashboard.putNumber("linear Actuator Setpoint", linearActuatorSetpoint);
    }
    public void setFlywheelVelocity(double speedRotationsPerSecond) {
        flywheelMotor.getClosedLoopController().setSetpoint(speedRotationsPerSecond, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
        flywheelMotor2.getClosedLoopController().setSetpoint(speedRotationsPerSecond, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    }
    public void setTurretAngle(double yawRadians, double pitchRadians) {
        setTurretYaw(yawRadians);
        setTurretPitch(pitchRadians);
    }

    public void setTurretPitch(double pitchRadians) {
        final double r = 6.25; // inches
        final double offsetX = 9.5; // inches
        final double offsetY = 2.75; // inches
        double distanceIn = Math.sqrt(
            Math.pow((r * Math.cos(pitchRadians) + offsetX), 2) +
            Math.pow((r * Math.sin(pitchRadians) + offsetY), 2)
        );
        double distanceMm = distanceIn * 25.4;

        final double minMm = 10;
        final double maxMm = 100;

        double distance = MathUtil.clamp(distanceMm, minMm, maxMm);
        double setpoint = -0.8 + ((distance - minMm) / (maxMm - minMm)) * 1.8;
        double setpointSpeed = setpoint / 100;
        double newSetpoint = MathUtil.clamp(linearActuatorSetpoint + setpointSpeed, -1, 1);
        linearActuatorSetpoint = newSetpoint;
        linearActuator.setSpeed(newSetpoint);
    }
    public void setTurretYaw(double yawRadians) {
        panMotor.getClosedLoopController().setSetpoint(yawRadians, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }
    public void setTurretPitchOpenLoop(double voltage) {
        double setpoint = MathUtil.clamp(voltage/12, -0.8, 1);
        double setpointSpeed = setpoint / 100;
        double newSetpoint = MathUtil.clamp(linearActuatorSetpoint + setpointSpeed, -1, 1);
        linearActuatorSetpoint = newSetpoint;
        linearActuator.setSpeed(newSetpoint);
    }
    public void setTurretYawOpenLoop(double voltage) {
        panMotor.setVoltage(voltage);
    }
    public void setFlywheelOpenLoop(double voltage) {
        flywheelMotor.setVoltage(voltage);
        flywheelMotor2.setVoltage(voltage);
    }

    public double getTurretPitch() {
//        return tiltMotor.getEncoder().getPosition() * 2 * Math.PI;
        return 0;
    }
}

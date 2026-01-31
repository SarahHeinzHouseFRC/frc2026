package frc.robot.intake;

import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import static com.revrobotics.spark.SparkLowLevel.MotorType.*;
import static frc.robot.intake.IntakeConstants.*;

import com.revrobotics.ResetMode;
import edu.wpi.first.math.MathUtil;

public class IntakeIOSpark implements IntakeIO {
    private SparkMax beltMotor = new SparkMax(beltMotorCanId, kBrushless);
    private SparkMax intakeMotor = new SparkMax(intakeMotorCanId, kBrushless);
    private SparkMax intakeAngleMotor = new SparkMax(intakeAngleMotorCanId, kBrushless);
    private SparkMax beltStarMotor = new SparkMax(beltStarMotorCanId, kBrushless);

    public IntakeIOSpark() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit(40).idleMode(IdleMode.kBrake).inverted(false);
        SparkMaxConfig invertedConfig = new SparkMaxConfig();
        invertedConfig.smartCurrentLimit(40).idleMode(IdleMode.kBrake).inverted(true);
        beltMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        intakeMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        intakeAngleMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        beltStarMotor.configure(invertedConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    @Override
    public void setBeltOpenLoop(double voltage) {
        beltMotor.setVoltage(voltage);
    }

    @Override
    public void setIntakeOpenLoop(double voltage) {
        intakeMotor.setVoltage(voltage);
    }

    @Override
    public void setBeltStarOpenLoop(double voltage) {
        beltStarMotor.setVoltage(voltage);
    }

    @Override
    public void setIntakeAngle(double angleRadians) {
        IntakeIO.super.setIntakeAngle(angleRadians);
        // TODO
    }

    @Override
    public void setIntakeAngleOpenLoop(double voltage) {
        IntakeIO.super.setIntakeAngleOpenLoop(voltage);
        intakeAngleMotor.set(MathUtil.clamp(voltage, 0d, 12d));
    }
}

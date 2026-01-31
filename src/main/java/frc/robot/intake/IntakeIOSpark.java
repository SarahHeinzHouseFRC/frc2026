package frc.robot.intake;

import com.revrobotics.PersistMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import static com.revrobotics.spark.SparkLowLevel.MotorType.*;
import static frc.robot.intake.IntakeConstants.*;

import com.revrobotics.ResetMode;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class IntakeIOSpark implements IntakeIO {
    private SparkMax beltMotor = new SparkMax(beltMotorCanId, kBrushless);
    private SparkMax intakeMotor = new SparkMax(intakeMotorCanId, kBrushless);
    private SparkMax beltStarMotor = new SparkMax(beltStarMotorCanId, kBrushless);
    private SparkMax OBIMotor = new SparkMax(25, kBrushless);
    private SparkMax OBIPivotMotor = new SparkMax(26, kBrushless);

    public IntakeIOSpark() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit(40).idleMode(IdleMode.kBrake).inverted(false);
        SparkMaxConfig invertedConfig = new SparkMaxConfig();
        invertedConfig.smartCurrentLimit(40).idleMode(IdleMode.kBrake).inverted(true);
        beltMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        intakeMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        beltStarMotor.configure(invertedConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        OBIMotor.configure(invertedConfig,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
        SparkMaxConfig pivconfig = new SparkMaxConfig();
        pivconfig.smartCurrentLimit(40).idleMode(IdleMode.kBrake).inverted(false);
        pivconfig.closedLoop.pid(6, 0, 0, ClosedLoopSlot.kSlot0);
        pivconfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        OBIPivotMotor.configure(pivconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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
    public void setOBIOpenLoop(double voltage) {
        OBIMotor.setVoltage(voltage);
    }

    @Override
    public void setOBIPivotMotorOpenLoop(double voltage) {
        OBIPivotMotor.setVoltage(voltage);
    }

    @Override
    public void setOBIPivotMotorClosedLoop(double position) {
        OBIPivotMotor.getClosedLoopController().setSetpoint(position, SparkBase.ControlType.kPosition);
    }
}

package frc.robot.intake;


import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import frc.robot.commands.CommandScheduler;
import frc.robot.commands.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    private final SparkMax intakeMotor;

    public IntakeSubsystem(CommandScheduler scheduler) {
        super(scheduler);
        intakeMotor = new SparkMax(0, SparkLowLevel.MotorType.kBrushless);
    }

    public void startIntake() {
        intakeMotor.set(1);
    }

    public void stopIntake() {
        intakeMotor.set(0);
    }
}


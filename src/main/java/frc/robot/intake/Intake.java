package frc.robot.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase

public class Intake extends SubsystemBase {
    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged;

    public static Intake instance;

    public Intake(CommandScheduler scheduler) {
        super(scheduler);
        instance = this;
        io = switch (Robot.currentMode) {
            case SIM -> new IntakeIOSim();
            case REAL -> new intakeIOSpark();
            default -> new IntakeIO() {};
        }
    }

    @SDMXAnalogInputEventHandler(-1)
    public static void setBeltOpenLoop(double speed) {
        Intake.instance.io.setBeltOpenLoop(speek * 12.0);
    }

    @SDMXAnalogInputEventHandler(-1)
    public static void seIntakeOpenLoop(double speed) {
        Intake.instance.io.setIntakeOpenLoop(speek * 12.0);
    }

    @SDMXAnalogInputEventHandler(-1)
    public void setBeltStartOpenLoop(double speed) {
        Intake.instance.io.setBeltStartOpenLoop(speek * 12.0);
    }
}

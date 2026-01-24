// package frc.robot.intake;

// import frc.robot.Robot;
// import frc.robot.SDMXAnalogInputEventHandler;
// import frc.robot.commands.CommandScheduler;
// import frc.robot.commands.SubsystemBase;

// public class Intake extends SubsystemBase {
//     private final IntakeIO io;
//     private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

//     public static Intake instance;

//     public Intake(CommandScheduler scheduler) {
//         super(scheduler);
//         instance = this;
//         io = switch (Robot.currentMode) {
//             case SIM -> new IntakeIOSim();
//             case REAL -> new IntakeIOSpark();
//             default -> new IntakeIO() {
//                 @Override
//                 public void setBeltStarOpenLoop(double voltage) {

//                 }
//             };
//         }
//     }

//     @SDMXAnalogInputEventHandler(-1)
//     public static void setBeltOpenLoop(double speed) {
//         Intake.instance.io.setBeltOpenLoop(speed * 12.0);
//     }

//     @SDMXAnalogInputEventHandler(-1)
//     public static void seIntakeOpenLoop(double speed) {
//         Intake.instance.io.setIntakeOpenLoop(speed * 12.0);
//     }

//     @SDMXAnalogInputEventHandler(-1)
//     public void setBeltStartOpenLoop(double speed) {
//         Intake.instance.io.setBeltStartOpenLoop(speed * 12.0);
//     }
// }

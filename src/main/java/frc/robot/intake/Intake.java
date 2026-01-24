 package frc.robot.intake;

 import edu.wpi.first.wpilibj.XboxController;
 import frc.robot.Robot;
 import frc.robot.SDMXAnalogInputEventHandler;
 import frc.robot.commands.Command;
 import frc.robot.commands.CommandScheduler;
 import frc.robot.commands.Commands;
 import frc.robot.commands.SubsystemBase;
 import org.littletonrobotics.junction.Logger;

 public class Intake extends SubsystemBase {
     private final IntakeIO io;
     private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

     public static Intake instance;
     private XboxController controller;

     public Intake(XboxController controller, CommandScheduler scheduler) {
         super(scheduler);
         instance = this;
         this.controller = controller;
         io = switch (Robot.currentMode) {
//             case SIM -> new IntakeIOSim();
             case REAL -> new IntakeIOSpark();
             default -> new IntakeIO() {
                 @Override
                 public void setBeltStarOpenLoop(double voltage) {

                 }
             };
         };
         setDefaultCommand(defaultCommand());
     }

     public void setBeltOpenLoop(double speed) {
         io.setBeltOpenLoop(speed * 12.0);
     }

     public void setIntakeOpenLoop(double speed) {
         io.setIntakeOpenLoop(speed * 12.0);
     }

     public void setBeltStarOpenLoop(double speed) {
         io.setBeltStarOpenLoop(speed * 12.0);
     }

     @Override
     public void periodic() {
         io.updateInputs();
         Logger.processInputs("Intake", inputs);
     }

     public void intakePeriodic() {
         setIntakeOpenLoop(
                 controller.getRightTriggerAxis() * (controller.getRightBumperButton() ? 1 : -1));
         setBeltOpenLoop(controller.getLeftTriggerAxis() * (controller.getLeftBumperButton() ? 1 : -1));

         double beltStarInput = 0.0;
         if (controller.getBButton()) {
             beltStarInput += 1.0;
         }
         if (controller.getYButton()) {
             beltStarInput -= 1.0;
         }
         setBeltStarOpenLoop(beltStarInput);
     }

     private Command defaultCommand() {
         return Commands.run(this::intakePeriodic, this);
     }
 }

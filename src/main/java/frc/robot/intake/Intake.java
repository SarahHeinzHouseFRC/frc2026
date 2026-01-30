 package frc.robot.intake;

 import edu.wpi.first.wpilibj.XboxController;
 import frc.robot.GenericController;
 import frc.robot.Robot;
 import frc.robot.commands.Command;
 import frc.robot.commands.CommandScheduler;
 import frc.robot.commands.Commands;
 import frc.robot.commands.SubsystemBase;
 import org.littletonrobotics.junction.Logger;

 public class Intake extends SubsystemBase {
     private final IntakeIO io;
     private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

     public static Intake instance;
     private GenericController controller;

     public Intake(GenericController controller, CommandScheduler scheduler) {
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
                 controller.readAnalog(1) * (controller.readDigital(1) ? 1 : -1));
         setBeltOpenLoop(controller.readAnalog(0) * (controller.readDigital(0) ? 1 : -1));

         double beltStarInput = 0.0;
         if (controller.readDigital(4)) {
             beltStarInput += 1.0;
         }
         if (controller.readDigital(2)) {
             beltStarInput -= 1.0;
         }
         setBeltStarOpenLoop(beltStarInput);
     }

     private Command defaultCommand() {
         return Commands.run(this::intakePeriodic, this);
     }
 }

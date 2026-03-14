package frc.robot.sid_vision;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.drive.BallChaserCommand;
import frc.robot.drive.Drive;
import frc.robot.intake.Intake;
import frc.robot.intake.IntakeFrontIntakeCommand;
import java.util.function.DoubleSupplier;

public class AutoIntake extends SubsystemBase {
  private final NetworkTableInstance inst;
  private final NetworkTable table;
  private final XboxController controller;
  private final BooleanEntry definitely;
  private final BooleanEntry drive;
  private final DoubleEntry steerL;
  private final DoubleEntry steerR;
  private final IntakeFrontIntakeCommand intakeFrontIntakeCommand =
      new IntakeFrontIntakeCommand(Intake.getInstance());
  private DoubleSupplier steer;
  private double steerNum;
  private final BallChaserCommand ballChaserCommand;
  private final CommandScheduler commandScheduler = CommandScheduler.getInstance();

  public AutoIntake(XboxController controller) {
    inst = NetworkTableInstance.getDefault();
    table = inst.getTable("BallVision");
    this.controller = controller;
    definitely = table.getBooleanTopic("/boolean/definitely").getEntry(false);
    drive = table.getBooleanTopic("/boolean/drive").getEntry(false);
    steerL = table.getDoubleTopic("/number/steer_l").getEntry(0d);
    steerR = table.getDoubleTopic("/number/steer_r").getEntry(0d);
    steer = () -> steerNum;
    ballChaserCommand = new BallChaserCommand(Drive.getInstance(), steer);
  }

  @Override
  public void periodic() {
    boolean shouldScheduleIntake = controller.getXButton() && definitely.getAsBoolean();
    if (shouldScheduleIntake && !commandScheduler.isScheduled(intakeFrontIntakeCommand)) {
      commandScheduler.schedule(intakeFrontIntakeCommand);
    } else if (!shouldScheduleIntake && commandScheduler.isScheduled(intakeFrontIntakeCommand)) {
      commandScheduler.cancel(intakeFrontIntakeCommand);
    }

    boolean shouldScheduleDrive = controller.getXButton() /* && drive.getAsBoolean()*/;
    //        if (drive.getAsBoolean()) {
    //            steerNum = 0d;
    //        } else if (steerL.getAsBoolean()) {
    //            steerNum = -0.5d;
    //        } else if (steerR.getAsBoolean()) {
    //            steerNum = 0.5d;
    //        } else {
    //            steerNum = 0d;
    //        }
    steerNum = (steerR.getAsDouble() - steerL.getAsDouble()) / 100d;
    if (shouldScheduleDrive && !commandScheduler.isScheduled(ballChaserCommand)) {
      commandScheduler.schedule(ballChaserCommand);
    } else if (!shouldScheduleDrive && commandScheduler.isScheduled(ballChaserCommand)) {
      commandScheduler.cancel(ballChaserCommand);
    }
  }
}

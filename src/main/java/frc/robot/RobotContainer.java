// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.Command;
import frc.robot.flywheel.Flywheel;
import frc.robot.flywheel.FlywheelIO;
import frc.robot.flywheel.FlywheelIOSim;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private Flywheel flywheel;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final XboxController driverController = new XboxController(0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureSubsystems();
    // Configure the trigger bindings
    configureBindings();
  }

  private void configureSubsystems() {
    switch (Robot.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
//        TODO
//        flywheel = new Flywheel(new FlywheelIOSpark());
        flywheel = null;
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        flywheel = new Flywheel(new FlywheelIOSim());
        break;

      default:
        // Replayed robot, disable IO implementations
        flywheel = new Flywheel(new FlywheelIO() {});
        break;
    }
  }


  private void configureBindings() {

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
//    return Autos.exampleAuto(m_exampleSubsystem);
    return null;
  }
}

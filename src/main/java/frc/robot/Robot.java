// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.Command;
import frc.robot.commands.CommandScheduler;
import frc.robot.shooter.Shooter;
import frc.robot.simulator.Simulator;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import java.lang.reflect.InvocationTargetException;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */


public class Robot extends LoggedRobot {
  private Command autonomousCommand;
  private static CommandScheduler commandScheduler;
  public static Simulator simulator;

  public Shooter shooter;

  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static XboxController driverController = new XboxController(0);
//    public static final SDMXController sdmxController = new SDMXController(new GenericHID(1));
    public static final SDMXController sdmxController = new SDMXController(driverController);

  StructArrayTopic<Translation3d> ballPositionsTopic = NetworkTableInstance.getDefault().getStructArrayTopic("/SHARP/ballPositions", Translation3d.struct);
  private final StructArrayPublisher<Translation3d> ballPositionsPublisher = ballPositionsTopic.publish();

  private final StructPublisher<Pose3d> robotPositionPublisher = NetworkTableInstance.getDefault().getStructTopic("/SHARP/robotPosition", Pose3d.struct).publish();


  public enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    Simulator.init();
    simulator = Simulator.getInstance();
    commandScheduler = new CommandScheduler();
    shooter = new Shooter(commandScheduler);
    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    Logger.recordMetadata(
            "GitDirty",
            switch (BuildConstants.DIRTY) {
              case 0 -> "All changes committed";
              case 1 -> "Uncommitted changes";
              default -> "Unknown";
            });

    // Set up data receivers & replay source
    switch (currentMode) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // Start AdvantageKit logger
    Logger.start();

    configureSubsystems();
    configureBindings();
  }

  private void configureBindings() {

  }

  private void configureSubsystems() {
    switch (Robot.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        break;

      default:
        // Replayed robot, disable IO implementations
        break;
    }
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    commandScheduler.run();

    // temporary
    if (driverController.getAButtonPressed()) {
//      simulator.getBallSim().shootBall(2, 2, 0, 2.9, 2.3, 8);
      simulator.shootBallFromRobot(3.14/2 - .1775, 0, 7);
    }
    ballPositionsPublisher.set(simulator.getBallPositions());
    robotPositionPublisher.set(simulator.getTruePosition());
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
    sdmxController.registerEventHandlers();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
      try {
          sdmxController.periodic();
      } catch (InvocationTargetException | IllegalAccessException e) {
          throw new RuntimeException(e);
      }
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    commandScheduler.cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}

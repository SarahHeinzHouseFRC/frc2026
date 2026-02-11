// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.CommandScheduler;
import frc.robot.drive.ControllerDriveCommand;
import frc.robot.drive.Drive;
import frc.robot.intake.Intake;
import frc.robot.intake.IntakeControllerCommand;
import frc.robot.math.Transformation;
import frc.robot.math.Vector3d;
import frc.robot.shooter.Shooter;
import frc.robot.shooter.ShooterCurveFit;
import frc.robot.shooter.ShooterMath;
import frc.robot.simulator.Simulator;
import frc.robot.vision.Vision;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.urcl.URCL;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends LoggedRobot {
  private static CommandScheduler commandScheduler;
  public static Simulator simulator;

  public Shooter shooter;
  public Drive drive;
  public Intake intake;
  public Vision vision;

  public GetPose poseGetter;

  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static XboxController xboxDriverController = new XboxController(0);
  public static GenericController driverController = new GenericController(xboxDriverController);
  public static XboxController operatorController = new XboxController(1);
  public static GenericController operatorGenericController =
      new GenericController(operatorController);
  //    public static final SDMXController sdmxController = new SDMXController(new GenericHID(1));
  //    public static final SDMXController sdmxController = new SDMXController(driverController);

  StructArrayTopic<Translation3d> ballPositionsTopic =
      NetworkTableInstance.getDefault()
          .getStructArrayTopic("/SHARP/ballPositions", Translation3d.struct);
  private final StructArrayPublisher<Translation3d> ballPositionsPublisher =
      ballPositionsTopic.publish();

  private final StructPublisher<Pose3d> robotPositionPublisher =
      NetworkTableInstance.getDefault()
          .getStructTopic("/SHARP/robotPosition", Pose3d.struct)
          .publish();

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
    if (currentMode == Mode.SIM) {
      Simulator.init();
      simulator = Simulator.getInstance();
    }
    //    poseGetter = new GetPose("10.32.60.200:50001");
    commandScheduler = new CommandScheduler();
    Shooter.init(operatorController, commandScheduler);
    shooter = Shooter.getInstance();
    Drive.init(commandScheduler);
    drive = Drive.getInstance();
    vision = new Vision(commandScheduler);
    drive.setDefaultCommand(new ControllerDriveCommand(driverController, drive));
    intake = new Intake(commandScheduler);
    intake.setDefaultCommand(
        new IntakeControllerCommand(xboxDriverController, operatorController, intake));
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
        // maybe this is causing loop overruns so changed back to USB
        Logger.addDataReceiver(new WPILOGWriter("/U/logs"));
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

    // Start URCL logger
    Logger.registerURCL(URCL.startExternal());
    // Start AdvantageKit logger
    Logger.start();

    configureSubsystems();
    configureBindings();
  }

  private void configureBindings() {}

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
    //    poseGetter.writePose();
    //    System.out.println(poseGetter.readPose());
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    commandScheduler.run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {}

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    //    if (autonomousCommand != null) {
    //      autonomousCommand.cancel();
    //    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

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

  // private Pose3d robotPoseSimTestingDontUse = moveRobotToRandomPositionTestingDontUse();
  // private int i = 0;
  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
    // if (false) {
    // temporary
    // i++;
    // if (i % 1 == 0) {
    // // System.out.println("here");
    //   robotPoseSimTestingDontUse = moveRobotToRandomPositionTestingDontUse();

    //     // Vector from A to B
    //     Translation2d delta =
    // FieldConstants.HUB.toTranslation2d().minus(robotPoseSimTestingDontUse.getTranslation().toTranslation2d());

    //     // Angle of that vector
    //     double yaw = new Rotation2d(delta.getX(), delta.getY()).getRadians();

    //     double x = delta.getNorm();

    //     double speed = ShooterCurveFit.calculateY(x);
    //     double pitch = ShooterCurveFit.calculateZ(x);
    //     simulator.shootBallFromPosition(robotPoseSimTestingDontUse, pitch, yaw, speed);
    //   }
    //   ballPositionsPublisher.set(simulator.getBallPositions());
    //   robotPositionPublisher.set(robotPoseSimTestingDontUse);
    // }

    Translation3d target = FieldConstants.HUB;

    Pose3d robotPose = null;

    Vector3d robotVelocity = null;

    ShooterMath calc =
        new ShooterMath(
            (Vector3d) target.toVector(), (Transformation) robotPose, robotVelocity, Math.PI / 6);
    Translation2d delta =
        FieldConstants.HUB.toTranslation2d().minus(robotPose.getTranslation().toTranslation2d());

    double yaw = new Rotation2d(delta.getX(), delta.getY()).getRadians();

    double x = delta.getNorm();

    double speed = ShooterCurveFit.calculateY(x);
    double pitch = ShooterCurveFit.calculateZ(x);

    Vector3d out = calc.solve(new double[] {speed, yaw, pitch});

    simulator.shootBallFromPosition(robotPose, out.z(), out.y(), out.x());

    ballPositionsPublisher.set(simulator.getBallPositions());
    robotPositionPublisher.set(robotPose);
  }

  //  private Pose3d moveRobotToRandomPositionTestingDontUse() {
  //    return new Pose3d(new Translation3d(Math.random() * 4.6, Math.random() * 8.1, 0),
  // Rotation3d.kZero);
  //  }
}

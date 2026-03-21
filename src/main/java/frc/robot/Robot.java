// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.auto.AutoWeekZero;
import frc.robot.climber.Climber;
import frc.robot.drive.ControllerDriveCommand;
import frc.robot.drive.Drive;
import frc.robot.intake.Intake;
import frc.robot.intake.IntakeControllerCommand;
import frc.robot.math.Matrix4d;
import frc.robot.math.Transformation;
import frc.robot.math.Vector3d;
import frc.robot.overbumper.OverBumper;
import frc.robot.overbumper.OverBumperControllerCommand;
import frc.robot.shooter.Shooter;
import frc.robot.shooter.ShooterControllerCommand;
import frc.robot.shooter.ShooterCurveFit;
import frc.robot.shooter.ShooterMath;
import frc.robot.sid_vision.AutoIntake;
import frc.robot.simulator.Simulator;
import frc.robot.vision.Vision;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
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
  public enum RobotVersion {
    V1,
    V2
  }

  public static final RobotVersion VERSION = RobotVersion.V2;
  private final CommandScheduler commandScheduler = CommandScheduler.getInstance();
  private Command autonomousCommand;

  public static final double loopFrequency = 50.0; // Hz

  public Shooter shooter;
  public Drive drive;
  public Intake intake;
  public Vision vision;
  public Climber climber;
  public OverBumper overBumper;

  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public XboxController driverController;
  public XboxController operatorController;

  public Simulator simulator = null;
  private StructArrayPublisher<Translation3d> ballPositionsPublisherSim = null;
  private StructPublisher<Pose3d> robotPositionPublisherSim = null;

  private AutoIntake autoIntake;

  private LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("Auto");

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
    super(1 / loopFrequency);

    if (currentMode == Mode.SIM) {
      setupSim();
    }

    setupAdvantagekit();

    configureControllers();

    configureSubsystems();
    configureBindings();

    configureAutoChooser();
  }

  private void configureAutoChooser() {
    autoChooser.addDefaultOption("right side", AutoWeekZero.autoV1());
    autoChooser.addOption("left side", AutoWeekZero.depot());
    autoChooser.addOption("right sweep", AutoWeekZero.rightSweep());
    autoChooser.addOption("center", AutoWeekZero.center());
  }

  private void setupSim() {
    Simulator.init();
    simulator = Simulator.getInstance();
    ballPositionsPublisherSim =
        NetworkTableInstance.getDefault()
            .getStructArrayTopic("/SHARP/ballPositions", Translation3d.struct)
            .publish();
    robotPositionPublisherSim =
        NetworkTableInstance.getDefault()
            .getStructTopic("/SHARP/robotPosition", Pose3d.struct)
            .publish();
  }

  private void setupAdvantagekit() {
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
  }

  private void configureControllers() {
    driverController = new XboxController(0);
    operatorController = new XboxController(1);
  }

  private void configureSubsystems() {
    Shooter.init(operatorController);
    shooter = Shooter.getInstance();
    Drive.init();
    drive = Drive.getInstance();
    Vision.init();
    vision = Vision.getInstance();
    Intake.init();
    intake = Intake.getInstance();
    Climber.init();
    climber = Climber.getInstance();
    OverBumper.init();
    overBumper = OverBumper.getInstance();
    autoIntake = new AutoIntake(driverController);
  }

  private void configureBindings() {
    drive.setDefaultCommand(new ControllerDriveCommand(driverController, drive));
    intake.setDefaultCommand(
        new IntakeControllerCommand(
            driverController,
            operatorController,
            () ->
                (driverController.getLeftTriggerAxis() < .1)
                    && OverBumper.getInstance().isDeployedish(),
            intake));
    climber.setDefaultCommand(
        Climber.climbCommand(
            () ->
                (driverController.getAButton() ? 1 : 0)
                    + (driverController.getYButton() ? -1 : 0)));
    overBumper.setDefaultCommand(new OverBumperControllerCommand(driverController, overBumper));
    shooter.setDefaultCommand(new ShooterControllerCommand(operatorController, shooter));
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
    autoChooser.periodic();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    autonomousCommand = autoChooser.get();
    commandScheduler.schedule(autonomousCommand);
    //    autonomousCommand = robotContainer.getAutonomousCommand();
    //    if (autonomousCommand != null) {
    //      commandScheduler.schedule(autonomousCommand);
    //    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // Stop autonomous command on teleop transition.
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
      autonomousCommand = null;
    }
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
  public void simulationPeriodic() throws ClassCastException {
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

    Pose3d robotPose = new Pose3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0));

    Vector3d robotVelocity = new Vector3d(0, 0, 0);

    if (robotPose == null || robotVelocity == null) {
      return;
    }

    ShooterMath calc =
        new ShooterMath(
            new Vector3d(target.toVector()),
            new Transformation(new Matrix4d(robotPose.toMatrix())),
            robotVelocity,
            Math.PI / 6);
    Translation2d delta =
        FieldConstants.HUB.toTranslation2d().minus(robotPose.getTranslation().toTranslation2d());

    double yaw = new Rotation2d(delta.getX(), delta.getY()).getRadians();

    double x = delta.getNorm();

    double speed = ShooterCurveFit.calculateY(x) * 0.75;
    double pitch = ShooterCurveFit.calculateZ(x);

    Vector3d out = calc.solve(new double[] {speed, yaw, pitch});

    System.out.println(out);

    simulator.shootBallFromPosition(robotPose, pitch, yaw, speed);
    simulator.shootBallFromPosition(robotPose, out.z(), out.y(), out.x());

    ballPositionsPublisherSim.set(simulator.getBallPositions());
    robotPositionPublisherSim.set(robotPose);
  }

  // private Pose3d moveRobotToRandomPositionTestingDontUse() {
  //   return new Pose3d(new Translation3d(Math.random() * 4.6, Math.random() * 8.1, 0),
  // Rotation3d.kZero);
  // }

  // private Vector3d makeRandomVelocity() {
  //   return new Vector3d(Math.random() * 4.5, Math.random() * 4.5, 0);
  // }
}

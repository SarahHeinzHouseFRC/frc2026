// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Autos;
import frc.robot.subsystems.SharpSubsystem;
import frc.robot.subsystems.ball.*;
import frc.robot.subsystems.drive.ControllerDriveCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.StowIntake;
import frc.robot.subsystems.launcher.LauncherSubsystem;
import frc.robot.subsystems.turret.AutoTurret;
import frc.robot.subsystems.turret.ManualTurret;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.subsystems.vision.Vision;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.function.DoubleSupplier;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final Drive drive = Drive.getInstance();
  private final TurretSubsystem turret = TurretSubsystem.getInstance();
  private final BallSubsystem ball = BallSubsystem.getInstance();
  private final IntakeSubsystem intake = IntakeSubsystem.getInstance();
  private final LauncherSubsystem launcher = LauncherSubsystem.getInstance();
  private final Vision vision = Vision.getInstance();

  private final ShotCalculator shotCalculator = ShotCalculator.getInstance();
  private final XboxController controller = new XboxController(0);

  private final TeleopShooter teleopShooter = TeleopShooter.getInstance();

  private final List<SharpSubsystem> subsystems = new ArrayList<>();

  private final DoublePublisher distanceToHubPublisher = NetworkTableInstance.getDefault().getDoubleTopic("/SHARP/Shooter/distanceToHub").publish();

  private static final RobotContainer instance = new RobotContainer();

  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  public static RobotContainer getInstance() {
    return instance;
  }

  private boolean isBeforeFirstEnable = true;

  public boolean isBeforeFirstEnable() {
    return isBeforeFirstEnable;
  }

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  private RobotContainer() {
    teleopShooter.setXboxController(controller);
    // Configure the trigger bindings
    configureAutoChooser();
    configureBindings();
  }

  public void registerSubsystem(SharpSubsystem subsystem) {
    if (subsystems.contains(subsystem)) {
      throw new IllegalArgumentException(
          "Subsystem " + subsystem.getName() + " is already registered!");
    }
    subsystems.add(subsystem);
  }

  public List<SharpSubsystem> getSubsystems() {
    return Collections.unmodifiableList(subsystems);
  }

  private void configureAutoChooser() {
    autoChooser.setDefaultOption("preloads", Autos.preloads());
    autoChooser.addOption("sweep right", Autos.sweep(false));
    autoChooser.addOption("sweep left", Autos.sweep(true));
    SmartDashboard.putData("Auto choices", autoChooser);
  }

  public void periodic() {
    if (isBeforeFirstEnable && DriverStation.isEnabled()) {
      isBeforeFirstEnable = false;
    }

    shotCalculator.update(drive.getPose(), drive.getChassisSpeeds());

    Transform2d robotToShooter = new Transform2d(.12, 0, Rotation2d.kZero);
    distanceToHubPublisher.set(Drive.getInstance().getPose().transformBy(robotToShooter).getTranslation().getDistance(FieldConstants.HUB.toTranslation2d()));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    drive.setDefaultCommand(new ControllerDriveCommand(controller, drive));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
//    return Autos.exampleAuto(m_exampleSubsystem);
    return autoChooser.getSelected();
  }
}

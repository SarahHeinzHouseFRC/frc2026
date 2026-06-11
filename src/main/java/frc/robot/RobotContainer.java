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
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.OperatorConstants;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.ball.Ball;
import frc.robot.subsystems.ball.Intake;
import frc.robot.subsystems.ball.IntakeAndShoot;
import frc.robot.subsystems.ball.Shoot;
import frc.robot.subsystems.drive.ControllerDriveCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.turret.AutoTurret;
import frc.robot.subsystems.turret.ManualTurret;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.vision.Vision;

import java.util.function.DoubleSupplier;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final Drive drive = Drive.getInstance();
  private final Turret turret = Turret.getInstance();
  private final Ball ball = Ball.getInstance();
  private final Vision vision = Vision.getInstance();
  private final ShotCalculator shotCalculator = ShotCalculator.getInstance();
  private final XboxController controller = new XboxController(0);

  private final AutoTurret autoTurret = new AutoTurret(Turret.getInstance());
  private final ManualTurret manualTurret = new ManualTurret(Turret.getInstance(), controller);

  private final DoublePublisher distanceToHubPublisher = NetworkTableInstance.getDefault().getDoubleTopic("/SHARP/Shooter/distanceToHub").publish();

  private static final RobotContainer instance = new RobotContainer();

  private boolean shooterIsAuto = true;

  private double shooterManualSetpoint = 6000;

  public static RobotContainer getInstance() {
    return instance;
  }

  private boolean isBeforeFirstEnable = true;

  public boolean isShooterAuto() {
    return shooterIsAuto;
  }

  public boolean isBeforeFirstEnable() {
    return isBeforeFirstEnable;
  }

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  private RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    switchToAutoShoot();
  }

  public void periodic() {
    if (isBeforeFirstEnable && DriverStation.isEnabled()) {
      isBeforeFirstEnable = false;
    }

    shotCalculator.update(drive.getPose(), drive.getChassisSpeeds());

    if (controller.getBButtonPressed()) {
      switchToAutoShoot();
    }

    if (controller.getXButtonPressed()) {
      switchToManualShoot();
    }

    if (shooterIsAuto) {
      handleAutoShooterAdjustment();
    } else {
      handleManualShooterAdjustment();
    }

    Transform2d robotToShooter = new Transform2d(.12, 0, Rotation2d.kZero);
    distanceToHubPublisher.set(Drive.getInstance().getPose().transformBy(robotToShooter).getTranslation().getDistance(FieldConstants.HUB.toTranslation2d()));
  }

  public void handleAutoShooterAdjustment() {}

  public void handleManualShooterAdjustment() {
    if (controller.getYButton()) {
      shooterManualSetpoint += 10;
    } else if (controller.getAButton()) {
      shooterManualSetpoint -= 10;
    }
  }

  public void switchToAutoShoot() {
    CommandScheduler.getInstance().schedule(autoTurret);
    shooterIsAuto = true;
  }

  public void switchToManualShoot() {
    CommandScheduler.getInstance().schedule(manualTurret);
    shooterIsAuto = false;
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

    Trigger controllerIntakeTrigger = new Trigger(() -> controller.getLeftTriggerAxis() > 0.1);
    Trigger controllerShootTrigger = new Trigger(() -> controller.getRightTriggerAxis() > 0.1);
    Trigger shouldRunShoot = controllerShootTrigger.and(controllerIntakeTrigger.negate());
    Trigger shouldRunIntake = controllerIntakeTrigger.and(controllerShootTrigger.negate());
    Trigger shouldRunShootAndIntake = controllerIntakeTrigger.and(controllerShootTrigger);

    DoubleSupplier shooterSpeed = () -> shooterIsAuto ? shotCalculator.getShotParams().flywheelVelocityRotationsPerMinute() : shooterManualSetpoint;

    shouldRunIntake.whileTrue(new Intake(ball, () -> controller.getLeftTriggerAxis() * (controller.getLeftBumperButton() ? -1 : 1)));

    shouldRunShoot.whileTrue(new Shoot(ball, shooterSpeed));

    shouldRunShootAndIntake.whileTrue(new IntakeAndShoot(ball, controller::getLeftTriggerAxis, shooterSpeed));
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

  public ShotCalculator getShotCalculator() {
    return shotCalculator;
  }
}

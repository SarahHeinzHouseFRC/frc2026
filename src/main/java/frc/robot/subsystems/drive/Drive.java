package frc.robot.subsystems.drive;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.SharpSubsystem;

import static frc.robot.subsystems.drive.DriveConstants.*;

public class Drive extends SharpSubsystem {
  private final Module[] modules = new Module[4];
  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(moduleTranslations);
  private double gyroAngle = 0;
  private SwerveModulePosition[] modulePositions = // For delta tracking
      new SwerveModulePosition[] {
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition()
      };

  private SwerveModuleState[] states = new SwerveModuleState[] {
      new SwerveModuleState(), new SwerveModuleState(),
      new SwerveModuleState(), new SwerveModuleState()
  };
  private final SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(kinematics, new Rotation2d(gyroAngle), modulePositions, Pose2d.kZero);

  private final Pigeon2 pigeon = new Pigeon2(pigeonCanId);
  private final StatusSignal<Angle> yaw = pigeon.getYaw();

  private final StructArrayPublisher<SwerveModuleState> unoptimizedSwerveSetpointPublisher = NetworkTableInstance.getDefault().getStructArrayTopic("/SHARP/Drive/UnoptmizedSwerveSetpoints", SwerveModuleState.struct).publish();
  private final StructArrayPublisher<SwerveModuleState> swerveSetpointPublisher = NetworkTableInstance.getDefault().getStructArrayTopic("/SHARP/Drive/SwerveSetpoints", SwerveModuleState.struct).publish();
  private final StructArrayPublisher<SwerveModuleState> swerveStatePublisher = NetworkTableInstance.getDefault().getStructArrayTopic("/SHARP/Drive/SwerveStates", SwerveModuleState.struct).publish();
  private final StructPublisher<Pose2d> posePublisher = NetworkTableInstance.getDefault().getStructTopic("/SHARP/Drive/Pose", Pose2d.struct).publish();
  private final StructPublisher<ChassisSpeeds> chassisSpeedsPublisher = NetworkTableInstance.getDefault().getStructTopic("/SHARP/Drive/chassisspeeds", ChassisSpeeds.struct).publish();

  private static final Drive instance = new Drive();

  public static Drive getInstance() {
    return instance;
  }

  private Drive() {
    modules[0] = new Module(0);
    modules[1] = new Module(1);
    modules[2] = new Module(2);
    modules[3] = new Module(3);
    pigeon.getConfigurator().apply(new Pigeon2Configuration());
    pigeon.getConfigurator().setYaw(0.0);
    yaw.setUpdateFrequency(50);
  }

  @Override
  public void periodic() {
    for (int i = 0; i < 4; i++) {
      modulePositions[i] = modules[i].getPosition();
    }

    gyroAngle = yaw.refresh().getValueAsDouble() * Math.PI / 180; // of course degrees is the default unit...

    poseEstimator.update(new Rotation2d(gyroAngle), modulePositions);

    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }

    swerveStatePublisher.set(states);

    posePublisher.set(poseEstimator.getEstimatedPosition());

    SmartDashboard.putNumber("gyroAngle", gyroAngle);
  }

  public void runVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, .02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, maxSpeedMetersPerSec);

    // Log unoptimized setpoints
    unoptimizedSwerveSetpointPublisher.set(setpointStates);

    // Send setpoints to modules
    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(setpointStates[i]);
    }

    // Log optimized setpoints (runSetpoint mutates each state)
    swerveSetpointPublisher.set(setpointStates);
  }

  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(Rotation2d.fromRadians(gyroAngle), modulePositions, pose);
  }

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    poseEstimator.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }
}

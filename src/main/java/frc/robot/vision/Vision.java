package frc.robot.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import frc.robot.commands.CommandScheduler;
import frc.robot.commands.SubsystemBase;
import frc.robot.drive.Drive;
import frc.robot.shooter.Shooter;

import java.io.IOException;

import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private Transform3d turretCamTransform =
      new Transform3d(.18, 0, .5, new Rotation3d(0, -Math.PI / 6, 0));
  private CameraIO turretCam =
      new PhotonCameraIO("camera-4", turretCamTransform);
  private CameraIOInputsAutoLogged turretCamInputs = new CameraIOInputsAutoLogged();

  // VisionSystem handles updates from coprocessors and calls drive.addVisionMeasurement directly.
  private AdvVisionServerIO2 visionSystem = new AdvVisionServerIO2();

  public Vision(CommandScheduler commandScheduler) {
    super(commandScheduler);

    System.out.println("Starting vision server");
    try {
      visionSystem.start();
    } catch (IOException e) {
      e.printStackTrace();
      throw new RuntimeException("Cannot start vision server", e);
    }
  }

  @Override
  public void periodic() {
    turretCam.updateInputs(turretCamInputs);
    Logger.processInputs("Vision/Turret Cam", turretCamInputs);

    Drive drive = Drive.getInstance();
    Shooter shooter = Shooter.getInstance();
    for (CameraIO.PoseObservation obs : turretCamInputs.results) {
      double stddev = 0.9;
      drive.addVisionMeasurement(
          obs.pose()
              .toPose2d()
              .plus(
                  (new Transform2d(.12, 0, new Rotation2d(shooter.getYawAtTime(obs.timestamp())))
                      .inverse()))
              .plus(new Transform2d(0d, 0d, Rotation2d.kPi)),
          obs.timestamp(),
          VecBuilder.fill(stddev, stddev, stddev));
    }
  }
}

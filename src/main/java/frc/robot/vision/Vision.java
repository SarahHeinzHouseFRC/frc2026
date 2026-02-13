package frc.robot.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import frc.robot.commands.CommandScheduler;
import frc.robot.commands.SubsystemBase;
import frc.robot.drive.Drive;
import frc.robot.shooter.Shooter;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private Transform3d turretCamTransform =
      new Transform3d(.18, 0, .5, new Rotation3d(0, -Math.PI / 6, 0));
  // private CameraIO turretCam =
  //     new PhotonCameraIO("camera-4", turretCamTransform); // TODO change to RobotVisionIO
  private CameraIO turretCam =
      new AdvVisionServerIO(turretCamTransform, "camera-2");
  private CameraIOInputsAutoLogged turretCamInputs = new CameraIOInputsAutoLogged();

  public Vision(CommandScheduler commandScheduler) {
    super(commandScheduler);
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

package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.PnpResult;

public class PhotonCameraIO implements CameraIO {
  private PhotonCamera camera;
  private String cameraName;
  private Transform3d cameraToRobot;

  private StructPublisher<Pose3d> publisher;

  public PhotonCameraIO(String cameraName) {
    this(cameraName, Transform3d.kZero);
  }

  public PhotonCameraIO(String cameraName, Transform3d robotToCamera) {
    this.cameraToRobot = robotToCamera.inverse();
    this.cameraName = cameraName;
    camera = new PhotonCamera(cameraName);
    publisher =
        NetworkTableInstance.getDefault()
            .getStructTopic("/SHARP/Vision" + cameraName, Pose3d.struct)
            .publish();
  }

  @Override
  public void updateInputs(CameraIOInputs inputs) {
    List<PoseObservation> results = new ArrayList<>();
    List<PhotonPipelineResult> photonResults = camera.getAllUnreadResults();

    Pose3d latestCameraPose = null;
    double latestTimestamp = 0;

    for (PhotonPipelineResult photonResult : photonResults) {
      Optional<MultiTargetPNPResult> multiTagResult = photonResult.getMultiTagResult();
      if (multiTagResult.isPresent()) {
        PnpResult pnpResult = multiTagResult.get().estimatedPose;
        double targetDistance = 0;
        int targetCount = 0;
        for (PhotonTrackedTarget target : photonResult.targets) {
          if (target.fiducialId < 0) continue;
          targetCount++;
          targetDistance +=
              target.bestCameraToTarget.getTranslation().getDistance(Translation3d.kZero);
        }
        targetDistance /= targetCount;
        Pose3d fieldToCamera = Pose3d.kZero.plus(pnpResult.best);
        results.add(
            new PoseObservation(
                photonResult.getTimestampSeconds(),
                fieldToCamera.plus(cameraToRobot),
                pnpResult.ambiguity,
                pnpResult.bestReprojErr,
                targetCount,
                targetDistance));
        if (photonResult.getTimestampSeconds() > latestTimestamp) {
          latestTimestamp = photonResult.getTimestampSeconds();
          latestCameraPose = fieldToCamera.plus(pnpResult.best);
        }
      } else if (!photonResult.targets.isEmpty()) {
        if (photonResult.targets.size() != 1) {
          System.out.println(
              "[WARNING] if there is no multitag there should be at most 1 tag present!");
        } else {
          PhotonTrackedTarget target = photonResult.targets.get(0);
          Optional<Pose3d> tagPose =
              VisionConstants.aprilTagFieldLayout.getTagPose(target.fiducialId);
          if (tagPose.isPresent()) {
            Pose3d fieldToCamera = tagPose.get().plus(target.bestCameraToTarget.inverse());
            Pose3d fieldToRobot = fieldToCamera.plus(cameraToRobot);
            results.add(
                new PoseObservation(
                    photonResult.getTimestampSeconds(),
                    fieldToRobot,
                    target.poseAmbiguity,
                    -1,
                    1,
                    target.bestCameraToTarget.getTranslation().getDistance(Translation3d.kZero)));
            if (photonResult.getTimestampSeconds() > latestTimestamp) {
              latestTimestamp = photonResult.getTimestampSeconds();
              latestCameraPose = fieldToCamera;
            }
          } else {
            //            System.out.println("[WARNING] tag pose not present!");
          }
        }
      }
    }
    inputs.results = results.toArray(new PoseObservation[0]);
    if (latestCameraPose != null) {
      publisher.set(latestCameraPose);
    } else {
      //      publisher.set(Pose3d.kZero);
    }
  }
}

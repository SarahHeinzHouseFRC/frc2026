package frc.robot.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import frc.robot.protos.DataHandling.GetRobotPositionRequest;
import frc.robot.protos.DataHandling.GetRobotPositionResponse;
import frc.robot.protos.VisionSystemGrpc;
import io.grpc.Channel;
import io.grpc.ManagedChannelBuilder;

// DO NOT USE
public class AdvVisionServerIO implements CameraIO {
  // public static final String ADV_VISION_SERVER = "10.32.60.200:50001";
  public static final String ADV_VISION_SERVER = "127.0.0.1:50001";

  // Ignored because it is ZERO for now
  private Transform3d cameraToRobot;
  private VisionSystemGrpc.VisionSystemBlockingStub visionSystem;

  private StructPublisher<Pose3d> publisher;

  public AdvVisionServerIO(String cameraName) {
    this(Transform3d.kZero, cameraName);
  }

  public AdvVisionServerIO(Transform3d robotToCamera, String cameraName) {
    this.cameraToRobot = robotToCamera.inverse();
    publisher =
        NetworkTableInstance.getDefault()
            .getStructTopic("/SHARP/Vision" + cameraName, Pose3d.struct)
            .publish();

    Channel channel = ManagedChannelBuilder.forTarget(ADV_VISION_SERVER).usePlaintext().build();
    visionSystem = VisionSystemGrpc.newBlockingStub(channel);
  }

  @Override
  public void updateInputs(CameraIOInputs inputs) {
    GetRobotPositionRequest req = GetRobotPositionRequest.newBuilder().build();
    GetRobotPositionResponse resp;
    try {
      resp = visionSystem.getRobotPosition(req);
    } catch (Exception e) {
      e.printStackTrace();
      publisher.set(Pose3d.kZero);
      inputs.results = new PoseObservation[0];
      return;
    }

    Nat<N4> rows = N4.instance;
    Nat<N4> cols = N4.instance;

    Double[] data = resp.getMatList().toArray(new Double[0]);

    double[] primData = new double[data.length];

    for (int i = 0; i < primData.length; ++i) {
      primData[i] = data[i];
    }

    // inPose is robot-to-world, but robot-to-camera is identity during test,
    // therefore, we can consider it camera-to-world
    Matrix<N4, N4> inMat = new Matrix<N4, N4>(rows, cols, primData).inv();
    inMat.set(3, 0, 0);
    inMat.set(3, 1, 0);
    inMat.set(3, 2, 0);
    inMat.set(3, 3, 1);
    System.out.println(inMat);
    Pose3d fieldToCamera = new Pose3d(inMat);

    inputs.results =
        new PoseObservation[] {
          new PoseObservation(
              System.currentTimeMillis(), fieldToCamera.plus(cameraToRobot), 0, 0, 1, 0)
        };

    publisher.set(fieldToCamera /*?*/);
  }
}

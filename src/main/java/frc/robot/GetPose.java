package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N4;
import frc.robot.protos.DataHandling.GetRobotPositionRequest;
import frc.robot.protos.DataHandling.GetRobotPositionResponse;
import frc.robot.protos.VisionSystemGrpc;
import io.grpc.Channel;
import io.grpc.ManagedChannelBuilder;
import java.util.concurrent.locks.ReentrantLock;

public class GetPose {

  private final ReentrantLock mutex;
  private Pose3d pose;

  private VisionSystemGrpc.VisionSystemBlockingStub visionSystem;

  public GetPose(String visionServer) {
    mutex = new ReentrantLock();
    pose = null;

    Channel channel = ManagedChannelBuilder.forTarget(visionServer).usePlaintext().build();
    visionSystem = VisionSystemGrpc.newBlockingStub(channel);
  }

  public void writePose() {

    GetRobotPositionRequest req = GetRobotPositionRequest.newBuilder().build();
    GetRobotPositionResponse resp = visionSystem.getRobotPosition(req);

    Nat<N4> rows = N4.instance;
    Nat<N4> cols = N4.instance;

    Double[] data = resp.getMatList().toArray(new Double[0]);

    double[] primData = new double[data.length];

    for (int i = 0; i < primData.length; ++i) {
      primData[i] = data[i];
    }

    Matrix<N4, N4> inPose = new Matrix<N4, N4>(rows, cols, primData);

    Pose3d nPose = new Pose3d(inPose);

    mutex.lock();
    try {
      pose = nPose;
    } finally {
      mutex.unlock();
    }
  }

  public Pose3d readPose() {
    mutex.lock();
    try {
      return pose;
    } finally {
      mutex.unlock();
    }
  }
}

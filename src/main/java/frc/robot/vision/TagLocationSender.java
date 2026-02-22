package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.commands.CommandScheduler;
import frc.robot.commands.SubsystemBase;
import frc.robot.protos.DataHandling.SetTagLocationsRequest;
import frc.robot.protos.DataHandling.TagLocation;
import frc.robot.protos.VisionSystemGrpc;
import io.grpc.Channel;
import io.grpc.ManagedChannelBuilder;
import java.util.Optional;

// DO NOT USE
public class TagLocationSender extends SubsystemBase {
  private int cnt;
  private VisionSystemGrpc.VisionSystemBlockingStub visionSystem;

  public TagLocationSender(CommandScheduler commandScheduler) {
    super(commandScheduler);
    cnt = 99;

    Channel channel =
        ManagedChannelBuilder.forTarget(AdvVisionServerIO.ADV_VISION_SERVER).usePlaintext().build();
    visionSystem = VisionSystemGrpc.newBlockingStub(channel);
  }

  public void periodic() {
    if (cnt++ >= 100) {
      SetTagLocationsRequest.Builder reqBuilder = SetTagLocationsRequest.newBuilder();
      int[] tags = new int[] {25, 26};
      for (int i = 0; i < tags.length; ++i) {
        TagLocation.Builder tagLocation = reqBuilder.addTagLocationsBuilder();
        tagLocation.setTagId(i);
        Optional<Pose3d> pos = VisionConstants.aprilTagFieldLayout.getTagPose(tags[i]);

        if (!pos.isPresent()) {
          System.out.println("Tag " + i + " does not exist");
          continue;
        }

        Translation3d t = pos.get().getTranslation();

        tagLocation.addMat(0);
        tagLocation.addMat(0);
        tagLocation.addMat(-1);
        tagLocation.addMat(t.getX());
        tagLocation.addMat(1);
        tagLocation.addMat(0);
        tagLocation.addMat(0);
        tagLocation.addMat(t.getY());
        tagLocation.addMat(0);
        tagLocation.addMat(-1);
        tagLocation.addMat(0);
        tagLocation.addMat(t.getZ());
      }
      try {
        visionSystem.setTagLocations(reqBuilder.build());
      } catch (Exception e) {
        e.printStackTrace();
      }
      cnt = 0;
    }
  }
}

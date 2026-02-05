package frc.robot.simulator;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.FieldConstants;

public class DriveSim {
  private final DriveSimModule[] modules = new DriveSimModule[4];
  private final Pose2d truePosition =
      new Pose2d(
          new Translation2d(FieldConstants.HUB.getX() - 1.2, FieldConstants.HUB.getY()),
          Rotation2d.kZero);
  private final Twist3d trueVelocity = new Twist3d();
  private SwerveModuleState[] moduleStates = new SwerveModuleState[4];

  protected DriveSim() {}

  protected void periodic(double dt) {
    //        Vector2d[] forces = new Vector2d[4];
    //        for (int i = 0; i < 4; i++) {
    //            forces[i] = modules[i].simulate();
    //        }
  }

  protected Pose2d getTruePosition() {
    return new Pose2d(
        new Translation2d(FieldConstants.HUB.getX() - 1.2, FieldConstants.HUB.getY()),
        Rotation2d.kZero);
  }

  protected void setModuleStates(SwerveModuleState[] states) {
    moduleStates = states;
  }
}

package frc.robot.simulator;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist3d;
import frc.robot.FieldConstants;

public class DriveSim {
    DriveSimModule frontLeft = new DriveSimModule(DriveSimModuleType.FrontLeft);
    DriveSimModule frontRight = new DriveSimModule(DriveSimModuleType.FrontRight);
    DriveSimModule backLeft = new DriveSimModule(DriveSimModuleType.BackLeft);
    DriveSimModule backRight = new DriveSimModule(DriveSimModuleType.BackRight);

    Pose3d truePosition =  new Pose3d(new Translation3d(FieldConstants.HUB.getX() - 1.2, FieldConstants.HUB.getY(), 0), Rotation3d.kZero);
    Twist3d trueVelocity = new Twist3d();
    protected DriveSim() {

    }
    protected void periodic(double dt) {

    }
    protected Pose3d getTruePosition() {
        return new Pose3d(new Translation3d(FieldConstants.HUB.getX() - 1.2, FieldConstants.HUB.getY(), 0), Rotation3d.kZero);
    }
}

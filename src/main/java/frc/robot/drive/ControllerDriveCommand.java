package frc.robot.drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Utils;
import frc.robot.commands.Command;

public class ControllerDriveCommand extends Command {
    private final Drive drive;
    private final XboxController controller;
    public ControllerDriveCommand(XboxController controller, Drive drive) {
        this.drive = drive;
        this.controller = controller;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        double vx = Utils.scaleAxis(Utils.deadband(controller.getLeftY(), .1), 2);
        double vy = Utils.scaleAxis(Utils.deadband(controller.getLeftX(), .1), 2);
        double omega = Utils.scaleAxis(Utils.deadband(controller.getRightX(), .1), 2);
        ChassisSpeeds speeds = new ChassisSpeeds(vx, vy, omega);
        drive.runVelocity(speeds);
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }
}

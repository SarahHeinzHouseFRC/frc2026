package frc.robot.drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
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
        double vx = controller.getLeftY() * controller.getLeftY() * controller.getLeftY();
        double vy = controller.getLeftX() * controller.getLeftX() * controller.getLeftX();
        double omega = controller.getRightX() * controller.getRightX() * controller.getRightX();
        ChassisSpeeds speeds = new ChassisSpeeds(vx, vy, omega);
        drive.runVelocity(speeds);
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }
}

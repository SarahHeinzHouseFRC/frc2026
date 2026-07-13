package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public abstract class SharpSubsystem extends SubsystemBase {
  public SharpSubsystem() {
    super();
    RobotContainer.getInstance().registerSubsystem(this);
  }

  public void execute() {}
}

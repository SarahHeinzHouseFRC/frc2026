package frc.robot.shooter;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants;

public class TargetAimCommand extends Command {
  private final Translation2d target = FieldConstants.HUB.toTranslation2d();

  public void setTarget(Translation2d target) {}

  public TargetAimCommand() {}
}

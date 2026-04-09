package frc.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.climber.Climber;
import frc.robot.drive.BetterSmoothMoveCommand;
import frc.robot.drive.SmoothMoveCommand;
import frc.robot.intake.Intake;
import frc.robot.intake.IntakeAutoCommand;
import frc.robot.overbumper.OverBumper;
import frc.robot.overbumper.ShakeCommand;
import frc.robot.shooter.Shooter;

import java.util.function.DoubleSupplier;

public class AutoWeekZero {
  public static Command autoV1() {
    return Commands.sequence(
            Commands.parallel(
                    Commands.sequence(
                        Commands.waitSeconds(2), new IntakeAutoCommand(Intake.getInstance())),
                    new ShakeCommand(OverBumper.getInstance()),
                    Shooter.getInstance().autoAimCommandAuto())
                .withDeadline(Commands.waitSeconds(5)),
            Commands.parallel(
                    Commands.sequence(
                        new SmoothMoveCommand(new Pose2d(.60, .65, Rotation2d.kZero))
                            .withAccelerationLimit(2)
                            .withVelocityLimit(2),
                        new SmoothMoveCommand(new Pose2d(.20, .65, Rotation2d.kZero))
                            .withAccelerationLimit(1)
                            .withVelocityLimit(.5)
                            .withDeadline(Commands.waitSeconds(4.0))),
                    Shooter.getInstance().autoAimCommandAutoDryish())
                .withDeadline(Commands.waitSeconds(10.0)),
            Commands.parallel(
                    new IntakeAutoCommand(Intake.getInstance()),
                    new ShakeCommand(OverBumper.getInstance()),
                    Shooter.getInstance().autoAimCommandAuto())
                .withDeadline(Commands.waitSeconds(15)))
        .withDeadline(Commands.waitSeconds(20));
  }

  public static Command depot() {
    return Commands.sequence(
            Commands.parallel(
                    Commands.sequence(
                        Commands.waitSeconds(2), new IntakeAutoCommand(Intake.getInstance())),
                    new ShakeCommand(OverBumper.getInstance()),
                    Shooter.getInstance().autoAimCommandAuto())
                .withDeadline(Commands.waitSeconds(5)),
            Commands.parallel(
                    new SmoothMoveCommand(new Pose2d(1.5, 6.0, Rotation2d.kZero))
                        .withAccelerationLimit(2)
                        .withVelocityLimit(2),
                    Shooter.getInstance().autoAimCommandAutoDryish())
                .withDeadline(Commands.waitSeconds(2.5)),
            Commands.parallel(
                    new SmoothMoveCommand(new Pose2d(.50, 6.0, Rotation2d.kZero))
                        .withAccelerationLimit(3)
                        .withVelocityLimit(1.5),
                    Shooter.getInstance().autoAimCommandAuto(),
                    OverBumper.getInstance().intakeCommand(),
                    new IntakeAutoCommand(Intake.getInstance()))
                .withDeadline(Commands.waitSeconds(5)),
            Commands.parallel(
                    new SmoothMoveCommand(new Pose2d(.75, 6.0, Rotation2d.kZero))
                        .withAccelerationLimit(2)
                        .withVelocityLimit(1),
                    new IntakeAutoCommand(Intake.getInstance()),
                    new ShakeCommand(OverBumper.getInstance()),
                    Shooter.getInstance().autoAimCommandAuto())
                .withDeadline(Commands.waitSeconds(15)))
        .withDeadline(Commands.waitSeconds(20));
  }

  public static Command center() {
    return Commands.parallel(
            Commands.sequence(Commands.waitSeconds(2), new IntakeAutoCommand(Intake.getInstance())),
            new ShakeCommand(OverBumper.getInstance()),
            Shooter.getInstance().autoAimCommandAuto())
        .withDeadline(Commands.waitSeconds(20));
  }

  public static Command rightSweep() {
    return sweep(false);
  }

  public static Command leftSweep() {
    return sweep(true);
  }

  public static Command sweep(boolean isLeft) {
    double aLimit = 7;
    double vLimit = 4;
    return Commands.sequence(
//        Commands.parallel(
//                Commands.sequence(
//                    Commands.waitSeconds(2), new IntakeAutoCommand(Intake.getInstance())),
//                new ShakeCommand(OverBumper.getInstance()),
//                Shooter.getInstance().autoAimCommandAuto())
//            .withTimeout(5.0),
//        new BetterSmoothMoveCommand(new Pose2d(3.0, .55, Rotation2d.kZero))
//            .withAccelerationLimit(aLimit)
//            .withVelocityLimit(vLimit)
//            .withDeadline(Commands.waitSeconds(5)),

            // move under the trench towards mid field in a straight line on the x axis
        new BetterSmoothMoveCommand(new Pose2d(6.5, .55, Rotation2d.kZero), isLeft)
            .withAccelerationLimit(aLimit + 2)
            .withVelocityLimit(vLimit + 1)
            .withTimeout(5.0),
        // rotate ourselves such that the intake is pointed towards the balls.
            // also move away from the wall while rotating so we don't break stuff.
        new BetterSmoothMoveCommand(new Pose2d(7.5, 1.0, Rotation2d.kCW_Pi_2), isLeft)
            .withAccelerationLimit(aLimit + 2)
            .withVelocityLimit(vLimit + 1)
            .withPositionTolerance(1)
            .withTimeout(5.0),
        // race means this composition finishes when either subcommand finishes
        Commands.race(
                // move toward true mid field along the y axis
            new BetterSmoothMoveCommand(new Pose2d(7.5, 3.0, Rotation2d.kCW_Pi_2), isLeft)
                .withAccelerationLimit(aLimit)
                .withVelocityLimit(.6)
                .withTimeout(4.0),
            // also run our overbumper intake while we do this
            OverBumper.getInstance().intakeCommand(2000)),
        // move back along the y axis so we are aligned x-wise to the trench
        new BetterSmoothMoveCommand(new Pose2d(7.5, .55, Rotation2d.kZero), isLeft)
            .withAccelerationLimit(aLimit)
            .withVelocityLimit(vLimit)
            .withTimeout(5.0),
        // drive through the trench so that we can legally score
        new BetterSmoothMoveCommand(new Pose2d(3.0, .55, Rotation2d.kZero), isLeft)
            .withAccelerationLimit(aLimit + 1)
            .withVelocityLimit(vLimit + 1)
            .withTimeout(5.0),
        // in parallel...
        Commands.parallel(
                new IntakeAutoCommand(Intake.getInstance()), // run the "intake," which pushes balls tot he shooter
                new ShakeCommand(OverBumper.getInstance()), // shake the overbumper to dislodge balls
                Shooter.getInstance().autoAimCommandAuto()) // also autoaim and shoot at the same time
            .withTimeout(7.0),


      new BetterSmoothMoveCommand(new Pose2d(6.5, .55, Rotation2d.kZero), isLeft)
              .withAccelerationLimit(aLimit + 2)
              .withVelocityLimit(vLimit + 1)
              .withTimeout(5.0),
              // rotate ourselves such that the intake is pointed towards the balls.
              // also move away from the wall while rotating so we don't break stuff.
              new BetterSmoothMoveCommand(new Pose2d(7.5, 3.0, Rotation2d.kCW_Pi_2), isLeft)
                      .withAccelerationLimit(aLimit + 1)
                      .withVelocityLimit(vLimit + 1)
                      .withPositionTolerance(1)
                      .withTimeout(5.0),
              // race means this composition finishes when either subcommand finishes
              Commands.race(
                      // move toward true mid field along the y axis
                      new BetterSmoothMoveCommand(new Pose2d(7.5, 5.0, Rotation2d.kCW_Pi_2), isLeft)
                              .withAccelerationLimit(aLimit)
                              .withVelocityLimit(.6)
                              .withTimeout(5.0),
                      // also run our overbumper intake while we do this
                      OverBumper.getInstance().intakeCommand(2000)),
              // move back along the y axis so we are aligned x-wise to the trench
              new BetterSmoothMoveCommand(new Pose2d(7.5, .55, Rotation2d.kZero), isLeft)
                      .withAccelerationLimit(aLimit)
                      .withVelocityLimit(vLimit)
                      .withTimeout(5.0),
              // drive through the trench so that we can legally score
              new BetterSmoothMoveCommand(new Pose2d(3.0, .55, Rotation2d.kZero), isLeft)
                      .withAccelerationLimit(aLimit + 1)
                      .withVelocityLimit(vLimit + 1)
                      .withTimeout(5.0),
              // in parallel...
              Commands.parallel(
                              new IntakeAutoCommand(Intake.getInstance()), // run the "intake," which pushes balls tot he shooter
                              new ShakeCommand(OverBumper.getInstance()), // shake the overbumper to dislodge balls
                              Shooter.getInstance().autoAimCommandAuto()) // also autoaim and shoot at the same time
                      .withTimeout(10.0)
    ).withTimeout(20.0);


  }

    public static Command climbLeft() {
        double aLimit = 7;
        double vLimit = 4;
        return Commands.sequence(
//        Commands.parallel(
//                Commands.sequence(
//                    Commands.waitSeconds(2), new IntakeAutoCommand(Intake.getInstance())),
//                new ShakeCommand(OverBumper.getInstance()),
//                Shooter.getInstance().autoAimCommandAuto())
//            .withTimeout(5.0),
//        new BetterSmoothMoveCommand(new Pose2d(3.0, .55, Rotation2d.kZero))
//            .withAccelerationLimit(aLimit)
//            .withVelocityLimit(vLimit)
//            .withDeadline(Commands.waitSeconds(5)),

                // move to approach position to climber
                new BetterSmoothMoveCommand(new Pose2d(1.054d, 4.469d, Rotation2d.fromDegrees(0.11d)), false)
                        .withAccelerationLimit(aLimit)
                        .withVelocityLimit(vLimit)
                        .withTimeout(6.0),

                // approach tower
                new BetterSmoothMoveCommand(new Pose2d(1.054d, 4.469d, Rotation2d.fromDegrees(0.11d)), false)
                        .withAccelerationLimit(1)
                        .withVelocityLimit(0.4)
                        .withPositionTolerance(0.02)
                        .withTimeout(14.0)
        ).withTimeout(20.0);


    }

  public static Command climbRight() {
    double aLimit = 7;
    double vLimit = 4;
    return Commands.sequence(
//        Commands.parallel(
//                Commands.sequence(
//                    Commands.waitSeconds(2), new IntakeAutoCommand(Intake.getInstance())),
//                new ShakeCommand(OverBumper.getInstance()),
//                Shooter.getInstance().autoAimCommandAuto())
//            .withTimeout(5.0),
//        new BetterSmoothMoveCommand(new Pose2d(3.0, .55, Rotation2d.kZero))
//            .withAccelerationLimit(aLimit)
//            .withVelocityLimit(vLimit)
//            .withDeadline(Commands.waitSeconds(5)),

           Commands.parallel(
                   // move under the trench towards mid field in a straight line on the x axis
                   new BetterSmoothMoveCommand(new Pose2d(6.5, .55, Rotation2d.kZero), false)
                           .withAccelerationLimit(aLimit + 2)
                           .withVelocityLimit(vLimit + 1)
                           .withTimeout(5.0),
                   OverBumper.getInstance().intakeCommand(2000).withTimeout(0.1d)
           ),
            // rotate ourselves such that the intake is pointed towards the balls.
            // also move away from the wall while rotating so we don't break stuff.
            new BetterSmoothMoveCommand(new Pose2d(7.5, 1.0, Rotation2d.kCW_Pi_2), false)
                    .withAccelerationLimit(aLimit + 2)
                    .withVelocityLimit(vLimit + 1)
                    .withPositionTolerance(1)
                    .withTimeout(5.0),
            // race means this composition finishes when either subcommand finishes
            Commands.race(
                    // move toward true mid field along the y axis
                    new BetterSmoothMoveCommand(new Pose2d(7.5, 3.0, Rotation2d.kCW_Pi_2), false)
                            .withAccelerationLimit(aLimit)
                            .withVelocityLimit(.67) // SIX SEVEN HAHAHAHAHA
                            .withTimeout(3.5),
                    // also run our overbumper intake while we do this
                    OverBumper.getInstance().intakeCommand(2000)),
            // move back along the y axis so we are aligned x-wise to the trench
            new BetterSmoothMoveCommand(new Pose2d(7.5, .55, Rotation2d.kZero), false)
                    .withAccelerationLimit(aLimit)
                    .withVelocityLimit(vLimit)
                    .withTimeout(5.0),
            // drive through the trench so that we can legally score
            new BetterSmoothMoveCommand(new Pose2d(3.0, .55, Rotation2d.kZero), false)
                    .withAccelerationLimit(aLimit + 1)
                    .withVelocityLimit(vLimit + 1)
                    .withTimeout(5.0),
            // in parallel...


            Commands.deadline(
                    // rotate before moving and shooting to improve accuracy
                    new BetterSmoothMoveCommand(new Pose2d(1.087d, 2.5d, Rotation2d.fromDegrees(-178.1d)), false)
                            .withAccelerationLimit(aLimit)
                            .withVelocityLimit(vLimit)
                            .withTimeout(0.5d),


                    Shooter.getInstance().autoAimCommandAutoDryish()
            ),


            Commands.parallel(
                    Commands.sequence(
                            Climber.climbCommand(() -> -1d).withTimeout(3.8d),
                            Climber.climbCommand(() -> 0d).withTimeout(0.5d)
                    ),
                            Commands.sequence(


                                    Commands.deadline(

                                            Commands.parallel(
                                                            new IntakeAutoCommand(Intake.getInstance()), // run the "intake," which pushes balls tot he shooter
                                                            new ShakeCommand(OverBumper.getInstance()), // shake the overbumper to dislodge balls
                                                            Shooter.getInstance().autoAimCommandAuto()) // also autoaim and shoot at the same time
                                                    .withTimeout(4.0),

                                            // move to approach position to climb
                                            new BetterSmoothMoveCommand(new Pose2d(1.087d, 2.5d, Rotation2d.fromDegrees(-178.1d)), false)
                                                    .withAccelerationLimit(aLimit)
                                                    .withVelocityLimit(0.4)
                                                    .withTimeout(10.0)
                                    ),

                                    // move to approach position to climb
                                    new BetterSmoothMoveCommand(new Pose2d(1.017d, 2.9d, Rotation2d.fromDegrees(179.67d)), false)
                                            .withAccelerationLimit(aLimit)
                                            .withVelocityLimit(vLimit + 2)
                                            .withTimeout(10.0)
                            )
            ),

          Commands.parallel(
                  Commands.parallel(
                                  new IntakeAutoCommand(Intake.getInstance()), // run the "intake," which pushes balls tot he shooter
                                  new ShakeCommand(OverBumper.getInstance()), // shake the overbumper to dislodge balls
                                  Shooter.getInstance().autoAimCommandAuto()) // also autoaim and shoot at the same time
                          .withTimeout(6.0),

                  Commands.sequence(
                          // approach tower
                          new BetterSmoothMoveCommand(new Pose2d(1.067d, 2.960d, Rotation2d.fromDegrees(179.67d)), false)
                                  .withAccelerationLimit(1)
                                  .withVelocityLimit(0.5)
                                  .withPositionTolerance(0.08)
                                  .withTimeout(5.0),

                          Climber.climbCommand(() -> 1d).withTimeout(3.5d)
                  )
          )

    ).withTimeout(20d);


  }
}

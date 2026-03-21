package frc.robot;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.LinkedHashMap;
import java.util.Map;

public class AutoRoutines {
  private final RobotState state;
  private final RobotContainer container;
  private final CommandSwerveDrivetrain drivetrain;
  private final FollowPath.Builder pathBuilder;

  private final NetworkTable auto = NetworkTableInstance.getDefault().getTable("Auto");
  private final Map<String, StructPublisher<Pose2d>> poseLoggers = new LinkedHashMap<>();

  private final PIDController translationController =
      new PIDController(
          DriveConstants.Translation.kP,
          DriveConstants.Translation.kI,
          DriveConstants.Translation.kD,
          Constants.cyclePeriodSeconds);

  private final PIDController rotationController =
      new PIDController(
          DriveConstants.Rotation.kP,
          DriveConstants.Rotation.kI,
          DriveConstants.Rotation.kD,
          Constants.cyclePeriodSeconds);

  private final PIDController crossTrackController =
      new PIDController(
          DriveConstants.CrossTrack.kP,
          DriveConstants.CrossTrack.kI,
          DriveConstants.CrossTrack.kD,
          Constants.cyclePeriodSeconds);

  private void logPose(String key, Pose2d pose) {
    var logger = poseLoggers.get(key);
    if (logger == null) {
      logger = auto.getStructTopic(key, Pose2d.struct).publish();
      poseLoggers.put(key, logger);
    }

    logger.accept(pose);
  }

  public AutoRoutines(RobotContainer container, CommandSwerveDrivetrain drivetrain) {
    this.container = container;
    this.drivetrain = drivetrain;
    this.state = RobotState.getInstance();

    // logging
    FollowPath.setPoseLoggingConsumer(pair -> logPose(pair.getFirst(), pair.getSecond()));

    // events
    FollowPath.registerEventTrigger("climb_extend", container.climb.extend().asProxy());
    FollowPath.registerEventTrigger("intake_start", container.intake.enable().asProxy());
    FollowPath.registerEventTrigger("intake_stop", container.intake.disable().asProxy());
    FollowPath.registerEventTrigger(
        "shoot_4", container.shooter.enable().withTimeout(Seconds.of(4)).asProxy());
    FollowPath.registerEventTrigger(
        "shoot_5", container.shooter.enable().withTimeout(Seconds.of(5)).asProxy());

    Path.setDefaultGlobalConstraints(
        new Path.DefaultGlobalConstraints(
            3.0, // maximum velocity (m/s)
            6.0, // maximum acceleration (m/s^2)
            540, // max angular velocity (deg/s)
            1080, // max angular acceleration (deg/s^2)
            DriveConstants.kTranslationTolerance, // translation tolerance (meters)
            DriveConstants.kRotationTolerance, // rotation tolerance (degrees)
            0.2 // handoff radius (meters)
            ));

    this.pathBuilder =
        new FollowPath.Builder(
                drivetrain,
                drivetrain::getPose,
                drivetrain::getRobotSpeeds,
                drivetrain::driveAtRobotRelative,
                translationController,
                rotationController,
                crossTrackController)
            .withDefaultShouldFlip();
  }

  private static final Path test1 = new Path("Test 1");
  private static final Path climb = new Path("Climb");
  private static final Path sweepTrenchL = new Path("Sweep Trench L");

  public Command test() {
    return pathBuilder.build(sweepTrenchL);
  }

  public Command climb() {
    return pathBuilder
        .build(climb)
        .withTimeout(Seconds.of(4.5))
        .andThen(container.climb.engage().asProxy());
  }
}

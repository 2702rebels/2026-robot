package frc.robot;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.Deployment;
import frc.robot.Constants.DriveConstants;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Set;

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

  private Command shoot(Time duration) {
    return container
        .shooter
        .enable()
        .alongWith(container.hopper.enable(container.canShoot))
        .withTimeout(duration)
        .asProxy();
  }

  private Command climbExtend() {
    return container.climb.extend().asProxy();
  }

  private Command climbEngage() {
    return container.climb.engage().asProxy();
  }

  private static final Path climbL = new Path("Climb L");
  private static final Path climbR = new Path("Climb R");
  private static final Path trenchLBump2Runs = new Path("Trench L Bump 2 Runs");
  private static final Path trenchRBump2Runs = new Path("Trench L Bump 2 Runs");

  public AutoRoutines(RobotContainer container, CommandSwerveDrivetrain drivetrain) {
    this.container = container;
    this.drivetrain = drivetrain;
    this.state = RobotState.getInstance();

    // enable logging [development only]
    if (Constants.DeploymentMode == Deployment.kDevelopment) {
      FollowPath.setPoseLoggingConsumer(pair -> logPose(pair.getFirst(), pair.getSecond()));
    }

    // events
    FollowPath.registerEventTrigger("climb_extend", climbExtend());
    FollowPath.registerEventTrigger("climb_engage", climbEngage());
    FollowPath.registerEventTrigger("intake_start", container.intake.enable().asProxy());
    FollowPath.registerEventTrigger("intake_stop", container.intake.disable().asProxy());
    FollowPath.registerEventTrigger("intake_extend", container.intake.extend().asProxy());
    FollowPath.registerEventTrigger(
        "intake_agitate", container.intake.agitate().withTimeout(5).asProxy());
    FollowPath.registerEventTrigger("shoot_4", shoot(Seconds.of(4)));
    FollowPath.registerEventTrigger("shoot_5", shoot(Seconds.of(5)));
    FollowPath.registerEventTrigger("shoot_6", shoot(Seconds.of(6)));

    Path.setDefaultGlobalConstraints(
        new Path.DefaultGlobalConstraints(
            3.5, // maximum velocity (m/s)
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

    trenchRBump2Runs.mirror();
  }

  private static final Path sweepTrenchL = new Path("Sweep Trench L");
  private static final Path sweepTrenchLOverBump = new Path("Sweep Trench L over Bump");
  private static final Path trenchLRArcStation = new Path("Trench LR Arc Station");

  public Command climbLeft() {
    return pathBuilder.build(climbL).withTimeout(Seconds.of(3)).andThen(climbEngage());
  }

  public Command climbRight() {
    return pathBuilder.build(climbR).withTimeout(Seconds.of(3)).andThen(climbEngage());
  }

  public Command trenchLBump2Runs() {
    return pathBuilder.build(trenchLBump2Runs);
  }

  public Command trenchRBump2Runs() {
    return pathBuilder.build(trenchRBump2Runs);
  }

  public Command test1() {
    return pathBuilder.build(sweepTrenchLOverBump);
  }

  public Command test2() {
    return pathBuilder.build(sweepTrenchL).andThen(shoot(Seconds.of(4)));
  }

  public Command test3() {
    return pathBuilder.build(trenchLRArcStation);
  }

  private static Path createTowerPath(Path.PathElement... pathElements) {
    final var constraints =
        new Path.PathConstraints()
            .setMaxVelocityMetersPerSec(new Path.RangedConstraint(1.2, 0, 1))
            .setMaxVelocityMetersPerSec(new Path.RangedConstraint(0.4, 1, Integer.MAX_VALUE));

    return new Path(constraints, pathElements);
  }

  // tower upper (left side)
  private static final Path.Waypoint towerUpperEndpoint =
      new Path.Waypoint(new Translation2d(1.090, 4.530), Rotation2d.fromDegrees(90));

  private static final Path.TranslationTarget towerUpperApproach =
      new Path.TranslationTarget(
          towerUpperEndpoint.translationTarget().translation().minus(new Translation2d(0.2, 0)));

  private static final Path.TranslationTarget towerUpperInitial =
      new Path.TranslationTarget(
          towerUpperApproach.translation().plus(new Translation2d(0.2, 0.8)));

  private static final Path towerUpper =
      createTowerPath(towerUpperInitial, towerUpperApproach, towerUpperEndpoint);

  // tower lower (right side)
  private static final Path.Waypoint towerLowerEndpoint =
      new Path.Waypoint(new Translation2d(1.040, 2.970), Rotation2d.fromDegrees(-90));

  private static final Path.TranslationTarget towerLowerApproach =
      new Path.TranslationTarget(
          towerLowerEndpoint.translationTarget().translation().plus(new Translation2d(0.2, 0)));

  private static final Path.TranslationTarget towerLowerInitial =
      new Path.TranslationTarget(towerLowerApproach.translation().minus(new Translation2d(0, 0.8)));

  private static final Path towerLower =
      createTowerPath(towerLowerInitial, towerLowerApproach, towerLowerEndpoint);

  /** Runs automatic climbing sequence if the robot is within the alliance zone. */
  public Command climb() {
    return Commands.defer(
        () -> {
          // must be relatively close to the tower to initiate the sequence
          final var pose = drivetrain.getPose();
          if (!Field.isInShootingZone(pose.getTranslation())) {
            return Commands.none();
          }

          // select upper/lower (left/right) of the tower to climb
          final var towerY = Field.kTowerCenter.getY();
          final var robotY = pose.getY();

          final var path =
              Field.shouldFlip()
                  ? robotY > Field.flipY(towerY) ? towerLower : towerUpper
                  : robotY > towerY ? towerUpper : towerLower;

          return climbExtend()
              .andThen(pathBuilder.build(path).withTimeout(Seconds.of(3)))
              .andThen(climbEngage());
        },
        Set.of(drivetrain));
  }
}

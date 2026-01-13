package frc.robot;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.lib.TimestampedCircularBuffer;
import frc.robot.lib.VisionState;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.Arrays;
import java.util.Optional;
import java.util.function.Supplier;

public class RobotState {
  private static RobotState instance;

  public static RobotState getInstance() {
    if (instance == null) instance = new RobotState();
    return instance;
  }

  private static final long kVisionCutoff = 60 * 1000; // microseconds

  private static final double kStationarySpeedTolerance = 0.05; // 5 cm/sec
  private static final double kStationaryOmegaTolerance = Units.degreesToRadians(5); // 5 deg/sec
  private static final double kSlowSpeedTolerance = 1; // 1.0 m/sec
  private static final double kSlowOmegaTolerance = Units.degreesToRadians(60); // 60 deg/sec
  private static final double kRobotSlipThreshold = 0.3;
  private static final double kOdometryTrustThreshold = 0.8;
  private static final double kOdometryTrustPenalty = 0.05;

  // std.dev for vision: we do trust vision so set these relatively low
  private static final Matrix<N3, N1> kVisionStdDevs =
      VecBuilder.fill(
          Units.inchesToMeters(0.5), Units.inchesToMeters(0.5), Units.degreesToRadians(5));

  // timestamped log of recently measured (within last second) chassis speeds
  private final TimestampedCircularBuffer<ChassisSpeeds> chassisSpeeds =
      new TimestampedCircularBuffer<>(50);

  // bound drivetrain
  private CommandSwerveDrivetrain drivetrain;

  // external vision state supplier
  private Supplier<VisionState> visionStateSupplier;

  private long visionPoseTimestamp = -1;
  private Pose2d visionPose = Pose2d.kZero;

  private RobotState() {}

  public static enum Intent {
    kNone,
  }

  /** Robot intent */
  @Logged(name = "Intent", importance = Importance.INFO)
  public Intent intent = Intent.kNone;

  /** Odometry trust (0 - no trust, 1 - full trust) */
  @Logged(name = "OdometryTrust", importance = Importance.INFO)
  private double odometryTrust = 0; // no trust initially

  /** Fused robot pose */
  @Logged(name = "RobotPose", importance = Importance.INFO)
  public Pose2d getRobotPose() {
    return drivetrain.getPose();
  }

  /** Vision robot pose */
  @Logged(name = "VisionPose", importance = Importance.INFO)
  public Pose2d getVisionPose() {
    return visionPose;
  }

  /** Age (milliseconds) of the most recent vision pose */
  @Logged(name = "VisionPoseAgeMS", importance = Importance.DEBUG)
  public double getVisionPoseAge() {
    return visionPoseTimestamp > 0
        ? (RobotController.getFPGATime() - visionPoseTimestamp) / 1000.0
        : -1;
  }

  /** Roll angle */
  @Logged(name = "Roll", importance = Importance.INFO)
  public Angle getRoll() {
    return drivetrain.getRoll();
  }

  /** Pitch angle */
  @Logged(name = "Pitch", importance = Importance.INFO)
  public Angle getPitch() {
    return drivetrain.getPitch();
  }

  // spotless:off
  @Logged(name = "Idle", importance = Importance.INFO)
  public final Trigger idle = new Trigger(() -> intent == Intent.kNone);

  @Logged(name = "IsPoseTrusted", importance = Importance.INFO)
  public final Trigger isPoseTrusted = new Trigger(() -> odometryTrust >= kOdometryTrustThreshold);
  // spotless:on

  /** Binds drivetrain. */
  public void bind(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
    this.drivetrain.setVisionMeasurementStdDevs(kVisionStdDevs);
  }

  /** Binds external vision state supplier. */
  public void setVisionStateSupplier(Supplier<VisionState> value) {
    visionStateSupplier = value;
  }

  /** Commands intent of the robot. */
  public Command setIntent(Intent intent) {
    return Commands.runOnce(
            () -> {
              this.intent = intent;
            })
        .withName("SetIntent");
  }

  // maintains most recent module slip ratios (to avoid allocations)
  private final double[] moduleSlipRatios = new double[4];

  /** Resets odometry pose and restores trust. */
  private void resetOdometryPose(Pose2d pose) {
    odometryTrust = 1;
    drivetrain.resetPose(pose);
  }

  /** Sets robot pose to a known pose and restores trust. */
  public void setRobotPose(Pose2d pose) {
    resetOdometryPose(pose);
  }

  /**
   * Runs logic at the beginning of the periodic cycle.
   *
   * <p>Use this to poll external inputs, e.g. operator or vision, before the command scheduler
   * runs.
   */
  public void enterPeriodic() {
    // synchronize all states to a common timestamp representing this update cycle
    final var timestamp = RobotController.getFPGATime();

    boolean isRobotStationary = false;
    boolean isRobotSlow = false;

    // poll odometry
    if (drivetrain != null) {
      final var driveState = drivetrain.getState();
      final var rotation = driveState.Pose.getRotation();
      final var robotSpeeds = driveState.Speeds;
      final var fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(robotSpeeds, rotation);
      chassisSpeeds.enqueue(timestamp, fieldSpeeds);

      Telemetry.computeSlipRatios(driveState, moduleSlipRatios);

      // reduce trust in odometry if slippage is detected
      final var maxSlipRatio = Arrays.stream(moduleSlipRatios).map(Math::abs).max().getAsDouble();
      if (maxSlipRatio > kRobotSlipThreshold) {
        odometryTrust = Math.max(0, odometryTrust - kOdometryTrustPenalty);
      }

      isRobotStationary =
          MathUtil.isNear(0, robotSpeeds.vxMetersPerSecond, kStationarySpeedTolerance)
              && MathUtil.isNear(0, robotSpeeds.vyMetersPerSecond, kStationarySpeedTolerance)
              && MathUtil.isNear(0, robotSpeeds.omegaRadiansPerSecond, kStationaryOmegaTolerance);

      isRobotSlow =
          MathUtil.isNear(0, robotSpeeds.vxMetersPerSecond, kSlowSpeedTolerance)
              && MathUtil.isNear(0, robotSpeeds.vyMetersPerSecond, kSlowSpeedTolerance)
              && MathUtil.isNear(0, robotSpeeds.omegaRadiansPerSecond, kSlowOmegaTolerance);
    }

    // poll vision state
    if (visionStateSupplier != null) {
      final var visionState = visionStateSupplier.get();

      // check if we have received a newer vision pose
      final var newVisionPoseTimestamp = visionState.pose.timestamp();
      if (newVisionPoseTimestamp > visionPoseTimestamp) {
        visionPose = visionState.pose.value();

        // special handling of the very first vision update:
        // if the robot is stationary set odometry pose to the vision pose
        if (visionPoseTimestamp <= 0 && isRobotStationary) {
          resetOdometryPose(visionPose);
        }

        visionPoseTimestamp = newVisionPoseTimestamp;
      }
    }

    // fuse robot pose with vision
    if (visionPoseTimestamp > 0 && visionPoseTimestamp > timestamp - kVisionCutoff) {
      // reset robot pose to vision pose when moving slowly and odometry is not trusted
      if (isRobotSlow && odometryTrust < kOdometryTrustThreshold) {
        resetOdometryPose(visionPose);
      } else {
        drivetrain.addVisionMeasurement(
            visionPose, Utils.fpgaToCurrentTime(visionPoseTimestamp / 1000000.0));
      }
    }
  }

  /**
   * Runs logic at the end of the periodic cycle.
   *
   * <p>Use this to synchronize the state with external modules, after the command scheduler runs.
   */
  public void exitPeriodic() {}

  /** Runs logic at the beginning of the autonomous cycle. */
  public void enterAutonomous() {}

  /** Runs logic at the beginning of the teleop cycle. */
  public void enterTeleop() {}

  /** Returns observed chassis speeds at the specified timestamp. */
  public Optional<ChassisSpeeds> getSpeedsAt(long timestamp) {
    return chassisSpeeds.findAt(timestamp);
  }

  /** Returns observed pose at the specified timestamp. */
  public Optional<Pose2d> getOdometryPoseAt(long timestamp) {
    return drivetrain != null ? drivetrain.getPoseAt(timestamp) : Optional.empty();
  }
}

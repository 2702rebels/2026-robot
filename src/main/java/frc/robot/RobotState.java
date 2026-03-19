package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.TunerConstants;
import frc.robot.lib.GamePhaseTracker.Phase;
import frc.robot.lib.GamePhaseTracker.PhaseInfo;
import frc.robot.lib.ShotCalculator;
import frc.robot.lib.SwerveTelemetry;
import frc.robot.lib.TimestampedCircularBuffer;
import frc.robot.lib.Tunable;
import frc.robot.lib.VisionState;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.Arrays;
import java.util.Optional;
import java.util.function.Supplier;

public class RobotState {
  private static RobotState instance;

  public static synchronized RobotState getInstance() {
    if (instance == null) instance = new RobotState();
    return instance;
  }

  public static enum PassingPoint {
    kDefault,
    kTrenchAuto,
    kTrenchUpper,
    kTrenchLower,
  }

  private static final long kVisionCutoff = 60 * 1000; // microseconds

  private static final double kStationaryVelocityTolerance = 0.1; // 10 cm/sec
  private static final double kStationarySpeedTolerance = 0.05; // 5 cm/sec
  private static final double kStationaryOmegaTolerance = Units.degreesToRadians(5); // 5 deg/sec
  private static final double kSlowSpeedTolerance = 1; // 1.0 m/sec
  private static final double kSlowOmegaTolerance = Units.degreesToRadians(60); // 60 deg/sec
  private static final double kRobotSlipThreshold = 0.3;
  private static final double kOdometryTrustThreshold = 0.8;
  private static final double kOdometryTrustPenalty = 0.05;
  private static final double kLowBatteryVoltage = 11.0;

  // shooting constants
  private static final Distance kMinScoreDistance = Meters.of(0.5);
  private static final Distance kMaxScoreDistance = Meters.of(5);
  private static final LinearVelocity kMaxScoreSpeed = MetersPerSecond.of(4);

  // offset to the center of the turret in meters
  public static final Translation2d kShooterOffsetXY =
      new Translation2d(Meters.convertFrom(-6, Inches), Meters.convertFrom(-6, Inches));

  // offset to the zero of the turret from the robot heading
  public static final Rotation2d kTurretOffset = Rotation2d.fromDegrees(180);

  // std.dev for vision: we do trust vision so set these relatively low
  private static final Matrix<N3, N1> kVisionStdDevs =
      VecBuilder.fill(
          Units.inchesToMeters(0.5), Units.inchesToMeters(0.5), Units.degreesToRadians(10));

  // timestamped log of recently measured (within last second) chassis speeds
  private final TimestampedCircularBuffer<ChassisSpeeds> chassisSpeeds =
      new TimestampedCircularBuffer<>(50);

  // previously recorded chassis speed (field relative)
  private ChassisSpeeds prevChassisSpeed = null;

  // timestamp of previously recorded chassis speed in seconds
  // NOTE: this timestamp is in timebase of the CTRE status signals,
  // which is based on a monotonic continuous time source that does
  // NOT have to match FPGA time on the robot
  private double prevChassisSpeedTimestamp = -1;

  // acceleration estimate (SMA) based on chassis speed delta
  private final LinearFilter accelerationEstimate = LinearFilter.movingAverage(5);

  @Logged(name = "Acceleration", importance = Importance.DEBUG)
  public LinearAcceleration getAcceleration() {
    return MetersPerSecondPerSecond.of(accelerationEstimate.lastValue());
  }

  // bound drivetrain
  private CommandSwerveDrivetrain drivetrain;

  // external vision state supplier
  private Supplier<VisionState> visionStateSupplier;

  // external game phase supplier
  private Supplier<PhaseInfo> phaseInfoSupplier;

  private long visionPoseTimestamp = -1;
  private Pose2d visionPose = Pose2d.kZero;
  private Phase phase;
  private Time phaseElapsedTime = Seconds.zero();
  private Time phaseRemainingTime = Seconds.zero();
  private boolean hubActive;

  private final ShotCalculator shotCalculator;
  private ShotCalculator.ShotParameters shotParameters =
      new ShotCalculator.ShotParameters(
          Translation2d.kZero, Meters.zero(), Seconds.zero(), 0, 0, 0, 0);

  private final MedianFilter shotVelocity = new MedianFilter(5);
  private final MedianFilter shotAngle = new MedianFilter(5);
  private final MedianFilter turretAngle = new MedianFilter(5);

  @Logged(name = "PassingPoint", importance = Importance.CRITICAL)
  private PassingPoint passingPoint = PassingPoint.kDefault;

  @Logged(name = "Shot/Confidence", importance = Importance.CRITICAL)
  private double shotConfidence = 0;

  @Logged(name = "Shot/Velocity", importance = Importance.CRITICAL)
  public double getShotVelocity() {
    return shotVelocity.lastValue();
  }

  @Logged(name = "Shot/Angle", importance = Importance.CRITICAL)
  public double getShotAngle() {
    return shotAngle.lastValue();
  }

  @Logged(name = "Shot/Quality", importance = Importance.CRITICAL)
  public double getShotQuality() {
    return shotParameters.quality();
  }

  @Logged(name = "Shot/Iterations", importance = Importance.DEBUG)
  public int getShotIterations() {
    return shotParameters.iterations();
  }

  @Logged(name = "Shot/Turret", importance = Importance.CRITICAL)
  public Angle getTurretAngle() {
    return Degrees.of(turretAngle.lastValue());
  }

  private RobotState() {
    if (Constants.Settings.ctreLoggingEnabled) {
      SignalLogger.setPath("/u/logs/ctre/");
      SignalLogger.start();
    }

    // initialize shot calculator LUT
    shotCalculator = new ShotCalculator(kShooterOffsetXY);

    shotCalculator.putLUTEntry(Meters.of(5.0), 4150, 25, 1.25);
    shotCalculator.putLUTEntry(Meters.of(4.5), 3900, 20, 1.25);
    shotCalculator.putLUTEntry(Meters.of(4.0), 3600, 20, 1.20);
    shotCalculator.putLUTEntry(Meters.of(3.5), 3450, 20, 1.10);
    shotCalculator.putLUTEntry(Meters.of(3.0), 3150, 20, 1.00);
    shotCalculator.putLUTEntry(Meters.of(2.5), 3050, 15, 1.00);
    shotCalculator.putLUTEntry(Meters.of(2.0), 2900, 15, 0.90);

    // passing distances
    // TODO: 5.5m, 6.0m, 6.5m
    shotCalculator.putLUTEntry(Meters.of(7.0), 5000, 30, 1.5);
  }

  /** Odometry trust (0 - no trust, 1 - full trust) */
  @Logged(name = "OdometryTrust", importance = Importance.CRITICAL)
  private double odometryTrust = 0; // no trust initially

  /** Odometry frequency in Hz */
  @Logged(name = "OdometryFrequency", importance = Importance.DEBUG)
  private double odometryFrequency;

  /** Battery voltage */
  @Logged(name = "BatteryVoltage", importance = Importance.CRITICAL)
  private Voltage batteryVoltage = Volts.zero();

  /** Low battery condition */
  @Logged(name = "IsLowBattery", importance = Importance.CRITICAL)
  public boolean isLowBattery() {
    final var v = batteryVoltage.in(Volts);
    return v > 0 && v < kLowBatteryVoltage;
  }

  /** Fused robot pose */
  @Logged(name = "RobotPose", importance = Importance.CRITICAL)
  public Pose2d getRobotPose() {
    return drivetrain.getPose();
  }

  /** Vision robot pose */
  @Logged(name = "VisionPose", importance = Importance.CRITICAL)
  public Pose2d getVisionPose() {
    return visionPose;
  }

  /** Age (milliseconds) of the most recent vision pose */
  @Logged(name = "VisionPoseAge", importance = Importance.DEBUG)
  public double getVisionPoseAge() {
    return visionPoseTimestamp > 0
        ? (RobotController.getTime() - visionPoseTimestamp) / 1000.0
        : -1;
  }

  /** Recent vision pose indicator */
  @Logged(name = "HasRecentVisionPose", importance = Importance.CRITICAL)
  public boolean hasRecentVisionPose() {
    final var age = getVisionPoseAge();
    return age >= 0 && age < 120;
  }

  /** Roll angle */
  @Logged(name = "Roll", importance = Importance.CRITICAL)
  public Angle getRoll() {
    return drivetrain.getRoll();
  }

  /** Pitch angle */
  @Logged(name = "Pitch", importance = Importance.CRITICAL)
  public Angle getPitch() {
    return drivetrain.getPitch();
  }

  /** Orientation lock */
  @Logged(name = "HasOrientationLock", importance = Importance.CRITICAL)
  public boolean hasOrientationLock() {
    return drivetrain.hasOrientationLock();
  }

  /** Indicator that odometry pose can be trusted */
  @Logged(name = "IsPoseTrusted", importance = Importance.CRITICAL)
  public final Trigger isPoseTrusted = new Trigger(() -> odometryTrust >= kOdometryTrustThreshold);

  /** Integrated swerve telemetry structure */
  @Logged(name = "SwerveTelemetry", importance = Importance.CRITICAL)
  public final SwerveTelemetry telemetry = new SwerveTelemetry();

  /** Indicator that auto win alliance information is missing */
  @Logged(name = "MissingAutoWinAlliance", importance = Importance.CRITICAL)
  public boolean missingAutoWinAlliance;

  /** Current game phase */
  @Logged(name = "Game/Phase", importance = Importance.CRITICAL)
  public Phase getGamePhase() {
    return phase;
  }

  /** Current game phase elapsed time */
  @Logged(name = "Game/PhaseElapsedTime", importance = Importance.CRITICAL)
  public Time getGamePhaseElapsedTime() {
    return phaseElapsedTime;
  }

  /** Current game phase remaining time */
  @Logged(name = "Game/PhaseRemainingTime", importance = Importance.CRITICAL)
  public Time getGamePhaseRemainingTime() {
    return phaseRemainingTime;
  }

  /** Hub active status */
  @Logged(name = "Game/HubActive", importance = Importance.CRITICAL)
  public boolean isHubActive() {
    return hubActive;
  }

  /** Binds drivetrain. */
  public void bind(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
    this.drivetrain.setVisionMeasurementStdDevs(kVisionStdDevs);
  }

  /** Binds external vision state supplier. */
  public void setVisionStateSupplier(Supplier<VisionState> value) {
    visionStateSupplier = value;
  }

  /** Binds external game phase supplier. */
  public void setGamePhaseSupplier(Supplier<PhaseInfo> value) {
    phaseInfoSupplier = value;
  }

  /** Resets odometry pose and restores trust. */
  private void resetOdometryPose(Pose2d pose) {
    odometryTrust = 1;
    drivetrain.resetPose(pose);
  }

  private static final double kShotQualityFactor = 1.0;
  private static final double kOdometryFactor = 1.0;
  private static final double kDistanceFactor = 0.5;
  private static final double kAccelerationFactor = 0.8;

  private static final double kConfidenceFactor =
      kShotQualityFactor + kOdometryFactor + kDistanceFactor + kAccelerationFactor;

  private static final double kMaxAcceleration =
      Constants.DriveConstants.maxAcceleration.in(MetersPerSecondPerSecond);

  private static final double kScoreDistanceRange =
      kMaxScoreDistance.in(Meters) - kMinScoreDistance.in(Meters);

  /**
   * Computes a shot confidence score between 0 and 1 as a weighted geometry mean of multiple
   * factors: shot solution quality, acceleration, vision, and distance from range edges.
   */
  private double computeShotConfidence(ShotCalculator.ShotParameters parameters) {
    var sum = 0.0;

    final var q = parameters.quality();
    if (q <= 0) {
      return 0;
    }

    sum += kShotQualityFactor * Math.log(q);

    if (odometryTrust <= 0) {
      return 0;
    }

    sum += kOdometryFactor * Math.log(odometryTrust);

    // distance factor
    var df = parameters.distance().minus(kMinScoreDistance).in(Meters) / kScoreDistanceRange;
    df = MathUtil.clamp(1.0 - 2.0 * Math.abs(df - 0.5), 0, 1);
    if (df <= 0) {
      return 0;
    }

    sum += kDistanceFactor * Math.log(df);

    // acceleration factor
    final var af =
        MathUtil.clamp(1.0 - Math.abs(accelerationEstimate.lastValue()) / kMaxAcceleration, 0, 1);
    if (af <= 0) {
      return 0;
    }

    sum += kAccelerationFactor * Math.log(af);

    return sum <= 0 ? 0 : MathUtil.clamp(Math.exp(sum / kConfidenceFactor), 0, 1);
  }

  /** Determines turret angle to the target in degrees. */
  private double getTurretAngleTo(Translation2d target) {
    final var pose = getRobotPose();
    final var shooterXY = getShooterFieldPose(pose);

    // turret desired angle and zero in field frame of reference
    final var turretDesired = target.minus(shooterXY).getAngle();
    final var turretZero = pose.getRotation().plus(kTurretOffset);

    return turretDesired.minus(turretZero).getDegrees();
  }

  /** Returns the (X,Y) position of the shoooter in the field frame of reference. */
  public Translation2d getShooterFieldPose(Pose2d pose) {
    final var offsetXY = kShooterOffsetXY.rotateBy(pose.getRotation());
    return pose.getTranslation().plus(offsetXY);
  }

  /** Returns effective shooter parameters. */
  public Optional<Pair<Double, Double>> getShooterParameters() {
    if (shotParameters.quality() < 0.5) return Optional.empty();
    return Optional.of(Pair.of(shotAngle.lastValue(), shotVelocity.lastValue()));
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
    final var timestamp = RobotController.getTime();

    batteryVoltage = RobotController.getMeasureBatteryVoltage();

    boolean isRobotStationary = false;
    boolean isRobotSlow = false;

    // poll odometry
    if (drivetrain != null) {
      final var driveState = drivetrain.getState();
      final var rotation = driveState.Pose.getRotation();
      final var robotSpeeds = driveState.Speeds;
      final var fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(robotSpeeds, rotation);
      chassisSpeeds.enqueue(timestamp, fieldSpeeds);

      // estimate acceleration
      if (prevChassisSpeed != null) {
        final var dt = driveState.Timestamp - prevChassisSpeedTimestamp;
        final var prevSpeed =
            Math.hypot(prevChassisSpeed.vxMetersPerSecond, prevChassisSpeed.vyMetersPerSecond);
        final var currentSpeed =
            Math.hypot(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond);
        accelerationEstimate.calculate((currentSpeed - prevSpeed) / dt);
      }

      prevChassisSpeed = fieldSpeeds;
      prevChassisSpeedTimestamp = driveState.Timestamp;

      // reduce trust in odometry if slippage is detected
      final var maxSlipRatio =
          Arrays.stream(moduleSlipRatiosArray).map(Math::abs).max().getAsDouble();
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

    // sync game phase info
    final var phaseInfo = phaseInfoSupplier.get();
    phase = phaseInfo.phase();
    phaseElapsedTime = Seconds.of(phaseInfo.elapsedTime());
    phaseRemainingTime = Seconds.of(phaseInfo.remainingTime());
    hubActive = phaseInfo.active();

    // determine effective turret angle and shot parameters
    final var pose = getRobotPose();
    final var shotTarget =
        Field.isInShootingZone(pose.getTranslation())
            ? Field.fieldTranslation.apply(Field.kHubCenter)
            : getPassingTarget();

    shotParameters = shotCalculator.calculate(pose, drivetrain.getFieldSpeeds(), shotTarget);
    shotConfidence = computeShotConfidence(shotParameters);
    shotVelocity.calculate(shotParameters.shooterVelocity());
    shotAngle.calculate(shotParameters.shooterAngle());
    turretAngle.calculate(getTurretAngleTo(shotParameters.target()));
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

  private final double[] poseArray = new double[3];
  private final double[] moduleStatesArray = new double[8];
  private final double[] moduleTargetsArray = new double[8];
  private final double[] moduleSlipRatiosArray = new double[4];

  /** Invoked by CTRE odometry thread. */
  public void updateSwerveTelemetry(SwerveDriveState state) {
    telemetry.rotation = state.Pose.getRotation();
    telemetry.currentSpeeds = state.Speeds;
    telemetry.currentStates = state.ModuleStates;
    telemetry.desiredStates = state.ModuleTargets;
    odometryFrequency = 1.0 / state.OdometryPeriod;

    computeSlipRatios(state, moduleSlipRatiosArray);

    // signal logger
    if (Constants.Settings.ctreLoggingEnabled) {
      poseArray[0] = state.Pose.getX();
      poseArray[1] = state.Pose.getY();
      poseArray[2] = state.Pose.getRotation().getDegrees();

      for (var i = 0; i < state.ModuleStates.length; ++i) {
        moduleStatesArray[i * 2 + 0] = state.ModuleStates[i].angle.getRadians();
        moduleStatesArray[i * 2 + 1] = state.ModuleStates[i].speedMetersPerSecond;
        moduleTargetsArray[i * 2 + 0] = state.ModuleTargets[i].angle.getRadians();
        moduleTargetsArray[i * 2 + 1] = state.ModuleTargets[i].speedMetersPerSecond;
      }

      SignalLogger.writeDoubleArray("DriveState/Pose", poseArray);
      SignalLogger.writeDoubleArray("DriveState/ModuleStates", moduleStatesArray);
      SignalLogger.writeDoubleArray("DriveState/ModuleTargets", moduleTargetsArray);
      SignalLogger.writeDoubleArray("DriveState/ModuleSlipRatios", moduleSlipRatiosArray);
      SignalLogger.writeDouble("DriveState/OdometryPeriod", state.OdometryPeriod, "seconds");
    }
  }

  /** Computes slip ratios for each module and populates the array. */
  private static void computeSlipRatios(SwerveDriveState state, double[] slipRatios) {
    // detect slippage by comparing translation velocities of the wheels with IMU velocity
    final var robotVelocity =
        new Translation2d(state.Speeds.vxMetersPerSecond, state.Speeds.vyMetersPerSecond);

    final var robotVelocityNorm = robotVelocity.getNorm();

    for (var i = 0; i < state.ModuleStates.length; ++i) {
      final var moduleVelocity =
          new Translation2d(
              state.ModuleStates[i].speedMetersPerSecond, state.ModuleStates[i].angle);

      final var moduleTranslationVelocity =
          new Translation2d(
              moduleVelocity.getX()
                  + state.Speeds.omegaRadiansPerSecond * TunerConstants.modulePositions[i].getY(),
              moduleVelocity.getY()
                  - state.Speeds.omegaRadiansPerSecond * TunerConstants.modulePositions[i].getX());

      // robot can be stationary, but can still slip if we are trying to push against an obstacle
      if (MathUtil.isNear(0, robotVelocityNorm, kStationaryVelocityTolerance)) {
        slipRatios[i] =
            MathUtil.isNear(0, moduleTranslationVelocity.getNorm(), kStationaryVelocityTolerance)
                ? 0
                : 1;
      } else {
        // technically we should compute projections on the robot IMU velocity vector
        // but all these vectors should be colinear unless the robot is falling apart
        slipRatios[i] =
            (moduleTranslationVelocity.getNorm() - robotVelocityNorm) / robotVelocityNorm;
      }
    }
  }

  /** Sets passing target. */
  public void setPassingTarget(PassingPoint targetPoint) {
    passingPoint = targetPoint;
  }

  /** Returns appropriate passing target based on the selection and robot pose. */
  public Translation2d getPassingTarget() {
    return getPassingTarget(passingPoint);
  }

  /** Returns appropriate passing point based on the selection and robot pose. */
  public Translation2d getPassingTarget(PassingPoint targetPoint) {
    final var pose = getRobotPose();

    // determine what Y side of the field we are on
    var originY = 0.0;
    var signY = 1.0;

    if (pose.getY() > Field.kFieldHalfWidth) {
      originY = Field.kFieldWidth;
      signY = -1.0;
    }

    switch (targetPoint) {
      case kTrenchLower:
        return new Translation2d(
            Field.fieldX.apply(Field.kStartingLineX), Field.fieldY.apply(Field.kTrenchWidth / 2));

      case kTrenchUpper:
        return new Translation2d(
            Field.fieldX.apply(Field.kStartingLineX),
            Field.fieldY.apply(Field.kFieldWidth - Field.kTrenchWidth / 2));

      case kTrenchAuto:
        return new Translation2d(
            Field.fieldX.apply(Field.kStartingLineX), originY + signY * Field.kTrenchWidth / 2);

      default:
        return new Translation2d(
            Field.fieldX.apply(Field.kDefaultPassingPoint.getX()),
            originY + signY * Field.kDefaultPassingPoint.getY());
    }
  }

  /** Determines whether shooter position is within trench zone. */
  public boolean isShooterWithinTrench() {
    return Field.isInTrenchZone(getShooterFieldPose(getRobotPose())).isPresent();
  }

  private static final Time kTrenchZonePredictiveTime = Seconds.of(0.5);

  /** Determines whether robot is within or entering trench zone. */
  public Optional<Translation2d> isRobotNearTrench() {
    return Field.isInTrenchZone(
        getRobotPose(), drivetrain.getFieldSpeeds(), kTrenchZonePredictiveTime);
  }

  @Logged(name = "Tuning/ActualShotDistance", importance = Importance.DEBUG)
  public Distance getActualShotDistance() {
    final var target = Field.fieldTranslation.apply(Field.kHubCenter);
    return Meters.of(getShooterFieldPose(getRobotPose()).minus(target).getNorm());
  }

  @Logged(name = "Tuning/ActualPassDistance", importance = Importance.DEBUG)
  public Distance getActualPasstDistance() {
    final var target = getPassingTarget(PassingPoint.kTrenchUpper);
    return Meters.of(getShooterFieldPose(getRobotPose()).minus(target).getNorm());
  }

  private static final Tunable.DoubleValue tuningShotDistance =
      Tunable.value("Tuning/RequiredShotDistance", 3);

  @Logged(name = "Tuning/ShotPose", importance = Importance.DEBUG)
  public Pose2d getTuningShotPose() {
    final var target = Field.fieldTranslation.apply(Field.kHubCenter);
    final var corner = Field.fieldTranslation.apply(new Translation2d(0, Field.kFieldWidth));

    final var angle = target.minus(corner).getAngle();
    final var position = new Translation2d(tuningShotDistance.get(), 0).rotateBy(angle);

    final var robotRotation = angle.rotateBy(Rotation2d.k180deg);
    final var offsetXY = kShooterOffsetXY.rotateBy(robotRotation);

    return new Pose2d(target.minus(position).minus(offsetXY), robotRotation);
  }
}

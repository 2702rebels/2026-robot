package frc.robot;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Optional;
import java.util.function.Function;

/** Field-related functionality. Blue alliance origin. */
public final class Field {
  /** Robot bumper size (assumes square robot) */
  public static final double kBumperSize = Units.inchesToMeters(34);

  /** Field length (X) */
  public static final double kFieldLength = Units.inchesToMeters(651.2);

  /** Field width (Y) */
  public static final double kFieldWidth = Units.inchesToMeters(317.7);

  /** Field half length (X) */
  public static final double kFieldHalfLength = kFieldLength / 2.0;

  /** Field half width (Y) */
  public static final double kFieldHalfWidth = kFieldWidth / 2.0;

  /** Middle of the starting line */
  public static final double kStartingLineX = 4;

  /** Neutral zone near side */
  public static final double kNeutralZoneNear = kFieldHalfLength - Units.inchesToMeters(120);

  /** Neutral zone far side */
  public static final double kNeutralZoneFar = kFieldHalfLength - Units.inchesToMeters(120);

  /** Middle of the hub center line */
  public static final double kHubLineX = 4.625;

  /** Effective zone for hub shooting, requiring bumpers to be partially within the alliance zone */
  public static final double kShootingZoneX =
      kStartingLineX + kBumperSize / 2 - Units.inchesToMeters(4);

  /** Trench width (approximate) in meters */
  public static final double kTrenchWidth = Units.inchesToMeters(50);

  /** Center of the hub */
  public static final Translation2d kHubCenter = new Translation2d(kHubLineX, kFieldHalfWidth);

  /** Center of the lower trench (Blue) */
  public static final Translation2d kBlueTrenchLowerCenter =
      new Translation2d(kHubLineX, kTrenchWidth / 2);

  /** Center of the upper trench (Blue) */
  public static final Translation2d kBlueTrenchUpperCenter =
      new Translation2d(kHubLineX, kFieldWidth - kTrenchWidth / 2);

  /** Center of the lower trench (Red) */
  public static final Translation2d kRedTrenchLowerCenter =
      new Translation2d(kFieldLength - kHubLineX, kFieldWidth - kTrenchWidth / 2);

  /** Center of the upper trench (Red) */
  public static final Translation2d kRedTrenchUpperCenter =
      new Translation2d(kFieldLength - kHubLineX, kTrenchWidth / 2);

  /** Approximate passing point as measured from the corresponding corner of the field. */
  public static final Translation2d kDefaultPassingPoint = new Translation2d(1, 1);

  private Field() {
    throw new AssertionError();
  }

  public static enum FieldType {
    MIRRORED {
      public double flipX(double x) {
        return kFieldLength - x;
      }

      public double flipY(double y) {
        return y;
      }

      public double flipHeading(double heading) {
        return Math.PI - heading;
      }
    },
    ROTATED {
      public double flipX(double x) {
        return kFieldLength - x;
      }

      public double flipY(double y) {
        return kFieldWidth - y;
      }

      public double flipHeading(double heading) {
        return Math.PI + heading;
      }
    };

    public abstract double flipX(double x);

    public abstract double flipY(double y);

    public abstract double flipHeading(double heading);
  }

  public static final FieldType kType = FieldType.ROTATED;

  /** Determines whether field coordinates should be flipped, aka being on red alliance. */
  public static boolean shouldFlip() {
    return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
  }

  public static double flipX(double x) {
    return kType.flipX(x);
  }

  public static double flipY(double y) {
    return kType.flipY(y);
  }

  public static double flipHeading(double heading) {
    return kType.flipHeading(heading);
  }

  public static Translation2d flip(Translation2d translation) {
    return new Translation2d(flipX(translation.getX()), flipY(translation.getY()));
  }

  public static Rotation2d flip(Rotation2d rotation) {
    return switch (kType) {
      case MIRRORED -> new Rotation2d(-rotation.getCos(), rotation.getSin());
      case ROTATED -> new Rotation2d(-rotation.getCos(), -rotation.getSin());
    };
  }

  public static Pose2d flip(Pose2d pose) {
    return new Pose2d(flip(pose.getTranslation()), flip(pose.getRotation()));
  }

  public static Translation3d flip(Translation3d translation) {
    return new Translation3d(
        flipX(translation.getX()), flipY(translation.getY()), translation.getZ());
  }

  public static Rotation3d flip(Rotation3d rotation) {
    return new Rotation3d(
        rotation.getX(), rotation.getY(), flip(rotation.toRotation2d()).getRadians());
  }

  public static Pose3d flip(Pose3d pose) {
    return new Pose3d(flip(pose.getTranslation()), flip(pose.getRotation()));
  }

  public static Function<Pose2d, Pose2d> fieldPose = (Pose2d v) -> shouldFlip() ? flip(v) : v;
  public static Function<Translation2d, Translation2d> fieldTranslation =
      (Translation2d v) -> shouldFlip() ? flip(v) : v;
  public static Function<Double, Double> fieldX = (Double v) -> shouldFlip() ? flipX(v) : v;
  public static Function<Double, Double> fieldY = (Double v) -> shouldFlip() ? flipY(v) : v;

  /**
   * Determines whether the specified pose falls within the either trench zone on both alliance
   * sides and returns center of the trench position if true.
   */
  public static Optional<Translation2d> isInTrenchZone(Translation2d xy) {
    final var x = xy.getX();
    final var y = xy.getY();

    if (y < kTrenchWidth || y > kFieldWidth - kTrenchWidth) {
      // blue alliance side
      if (x > kStartingLineX && x < kNeutralZoneNear) {
        return y < kTrenchWidth
            ? Optional.of(kBlueTrenchLowerCenter)
            : Optional.of(kBlueTrenchUpperCenter);
      }

      // red alliance side
      if (x > kNeutralZoneFar && x < kFieldLength - kStartingLineX) {
        return y < kTrenchWidth
            ? Optional.of(kRedTrenchUpperCenter)
            : Optional.of(kRedTrenchLowerCenter);
      }
    }

    return Optional.empty();
  }

  /**
   * Extends {@link #isInTrenchZone(Translation2d)} with predictive behavior based on current
   * velocity and time horizon.
   */
  public static Optional<Translation2d> isInTrenchZone(
      Pose2d pose, ChassisSpeeds fieldSpeeds, Time dt) {
    return isInTrenchZone(pose.getTranslation())
        .or(() -> isInTrenchZone(pose.exp(fieldSpeeds.toTwist2d(dt.in(Seconds))).getTranslation()));
  }

  /**
   * Determines whether the specified robot pose's translation falls within the shooting zone aka
   * bumpers partially within the alliance zone
   */
  public static boolean isInShootingZone(Translation2d xy) {
    final var x = xy.getX();
    return shouldFlip() ? x > kFieldLength - kShootingZoneX : x < kShootingZoneX;
  }
}

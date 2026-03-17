package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.function.Function;

/** Field-related functionality. Blue alliance origin. */
public final class Field {
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

  /** Trench width (approximate) in meters */
  public static final double kTrenchWidth = Units.inchesToMeters(50);

  /** Center of the hub */
  public static final Translation2d kHubCenter = new Translation2d(kHubLineX, kFieldHalfWidth);

  /** Center of the lower trench */
  public static final Translation2d kTrenchLowerCenter = new Translation2d(kHubLineX, 1.279 / 2);

  /** Center of the upper trench */
  public static final Translation2d kTrenchUpperCenter =
      new Translation2d(kHubLineX, kFieldWidth - (1.279 / 2));

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

  /**
   * Returns the supplier of pose specified in the blue alliance origin corrected to the current
   * alliance.
   */
  public static Function<Pose2d, Pose2d> createFieldPoseSupplier() {
    return (Pose2d pose) -> {
      return shouldFlip() ? flip(pose) : pose;
    };
  }

  public static Function<Pose2d, Pose2d> fieldPose = createFieldPoseSupplier();

  /**
   * Returns the supplier of translation specified in the blue alliance origin corrected to the
   * current alliance.
   */
  public static Function<Translation2d, Translation2d> createFieldTranslationSupplier() {
    return (Translation2d translation) -> {
      return shouldFlip() ? flip(translation) : translation;
    };
  }

  public static Function<Translation2d, Translation2d> fieldTranslation =
      createFieldTranslationSupplier();

  /**
   * Determines whether the specified pose falls within the either trench zone on both alliance
   * sides.
   */
  public static boolean isInTrenchZone(Translation2d translation) {
    final var t = translation;
    final var x = t.getX();
    final var y = t.getY();

    if (y < kTrenchWidth || y > kFieldWidth - kTrenchWidth) {
      // blue alliance side
      if (x > kStartingLineX && x < kNeutralZoneNear) {
        return true;
      }

      // red alliance side
      if (x > kNeutralZoneFar && x < kFieldLength - kStartingLineX) {
        return true;
      }
    }

    return false;
  }
}

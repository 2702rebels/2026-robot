package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.function.Function;

/** Field-related functionality. Blue alliance origin. */
public final class Field {
  /** Field length (X) */
  public static final Distance kFieldLength = Inches.of(651.2);

  /** Field width (Y) */
  public static final Distance kFieldWidth = Inches.of(317.7);

  /** Field half length (X) */
  public static final Distance kFieldHalfLength = kFieldLength.div(2);

  /** Field half width (Y) */
  public static final Distance kFieldHalfWidth = kFieldWidth.div(2);

  /** Middle of the starting line */
  public static final Distance kStartingLineX = Meters.of(4);

  /** Middle of the hub center line */
  public static final Distance kHubLineX = Meters.of(4.625);

  /** Center of the hub */
  public static final Translation2d kHubCenter =
      new Translation2d(kHubLineX.in(Meters), kFieldHalfWidth.in(Meters));

  /** Center of the lower trench */
  public static final Translation2d kTrenchLowerCenter =
      new Translation2d(kHubLineX.in(Meters), 1.279 / 2);

  /** Center of the upper trench */
  public static final Translation2d kTrenchUpperCenter =
      new Translation2d(kHubLineX.in(Meters), kFieldWidth.in(Meters) - (1.279 / 2));

  private Field() {}

  private static final double fieldLength = kFieldLength.in(Meter);
  private static final double fieldWidth = kFieldWidth.in(Meter);

  public static enum FieldType {
    MIRRORED {
      public double flipX(double x) {
        return fieldLength - x;
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
        return fieldLength - x;
      }

      public double flipY(double y) {
        return fieldWidth - y;
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
}

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.LoggedTalonFX;
import frc.robot.lib.Tunable;

public final class Hopper extends SubsystemBase {
  // MAKE SURE TO UPDATE THESE VALUES AFTER TUNING
  // DO NOT USE THESE VALUES DIRECTLY IN PERIODIC CODE,
  // USE THEM TO INITIALIZE TUNABLE VALUES AND CONTROLLERS
  private static final double __kSpindexerS = 0;
  private static final double __kSpindexerV = 0;
  private static final double __kSpindexerP = 10;
  private static final double __kSpindexerD = 0;

  private static final double __kFeedS = 0;
  private static final double __kFeedV = 0;
  private static final double __kFeedP = 10;
  private static final double __kFeedD = 0;

  private static final double __kSpindexerVelocity = 1000; // RPMs
  private static final double __kFeedVelocity = 1500; // RPMs

  @Logged(name = "Spindexer", importance = Importance.INFO)
  private final LoggedTalonFX spindexerMotor;

  @Logged(name = "Feed", importance = Importance.INFO)
  private final LoggedTalonFX feedMotor;

  private final VelocityTorqueCurrentFOC spindexerControl;
  private final VelocityTorqueCurrentFOC feedControl;

  private boolean disabled;

  private static final double kSpindexerStatorCurrentLimit = 30;
  private static final double kFeedStatorCurrentLimit = 40;

  private static final double kSpindexerConversionFactor = (36 / 14) * (72 / 24);
  private static final double kSpindexerTolerance = 5; // RPMs
  private static final double kFeedConversionFactor = 2;
  private static final double kFeedTolerance = 5; // RPMs

  public Hopper(CANBus canbus) {
    spindexerMotor =
        new LoggedTalonFX(
            Constants.CanivoreID.MOTOR_HOPPER_SPINDEXER,
            canbus,
            (config) -> {
              config.CurrentLimits.StatorCurrentLimit = kSpindexerStatorCurrentLimit;
              config.CurrentLimits.StatorCurrentLimitEnable = true;
              config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
              config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

              config.Slot0.kS = __kSpindexerS;
              config.Slot0.kV = __kSpindexerV;
              config.Slot0.kP = __kSpindexerP;
              config.Slot0.kD = __kSpindexerD;
            });

    feedMotor =
        new LoggedTalonFX(
            Constants.CanivoreID.MOTOR_HOPPER_FEED,
            canbus,
            (config) -> {
              config.CurrentLimits.StatorCurrentLimit = kFeedStatorCurrentLimit;
              config.CurrentLimits.StatorCurrentLimitEnable = true;
              config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
              config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

              config.Slot0.kS = __kFeedS;
              config.Slot0.kV = __kFeedV;
              config.Slot0.kP = __kFeedP;
              config.Slot0.kD = __kFeedD;
            });

    spindexerControl = new VelocityTorqueCurrentFOC(0).withSlot(0);
    feedControl = new VelocityTorqueCurrentFOC(0).withSlot(0);

    // make configuration tunable (must be done after configuration is bound to the motor)
    Tunable.bind("Spindexer", spindexerMotor);
    Tunable.bind("Feed", feedMotor);

    disabled = true;
  }

  @Override
  public void periodic() {
    if (disabled) {
      spindexerMotor.stopMotor();
      feedMotor.stopMotor();
    } else {
      spindexerMotor.setControl(
          spindexerControl.withVelocity(
              RotationsPerSecond.convertFrom(kSpindexerVelocity.get(), RPM)
                  * kSpindexerConversionFactor));

      feedMotor.setControl(
          feedControl.withVelocity(
              RotationsPerSecond.convertFrom(kFeedVelocity.get(), RPM) * kFeedConversionFactor));
    }
  }

  /** Determines whether the subsystem is at its nominal state. */
  @Logged(name = "Nominal", importance = Importance.INFO)
  public boolean isNominal() {
    return MathUtil.isNear(kSpindexerVelocity.get(), getSpindexerVelocity(), kSpindexerTolerance)
        && MathUtil.isNear(kFeedVelocity.get(), getFeedVelocity(), kFeedTolerance);
  }

  @Logged(name = "Velocity/Spindexer", importance = Importance.INFO)
  public double getSpindexerVelocity() {
    return spindexerMotor.getVelocityRPM() / kSpindexerConversionFactor;
  }

  @Logged(name = "Velocity/Feed", importance = Importance.INFO)
  public double getFeedVelocity() {
    return feedMotor.getVelocityRPM() / kFeedConversionFactor;
  }

  /** Disables the subsystem. */
  public Command disable() {
    return runOnce(
            () -> {
              disabled = true;
            })
        .withName("HopperDisable");
  }

  /** Enables the subsystem. */
  public Command enable() {
    return runOnce(
            () -> {
              disabled = false;
            })
        .withName("HopperEnable");
  }

  // spotless:off
  private static Tunable.DoubleValue kSpindexerVelocity = Tunable.value("Spindexer/Velocity", __kSpindexerVelocity);
  private static Tunable.DoubleValue kFeedVelocity = Tunable.value("Feed/Velocity", __kFeedVelocity);
  // spotless:on
}

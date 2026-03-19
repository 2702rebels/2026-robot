package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.EnergyReporter;
import frc.robot.lib.LoggedTalonFX;
import frc.robot.lib.Tunable;

public final class Hopper extends SubsystemBase implements EnergyReporter {
  // MAKE SURE TO UPDATE THESE VALUES AFTER TUNING
  // DO NOT USE THESE VALUES DIRECTLY IN PERIODIC CODE,
  // USE THEM TO INITIALIZE TUNABLE VALUES AND CONTROLLERS
  private static final double __kSpindexerS = 2.2;
  private static final double __kSpindexerV = 0.0025;
  private static final double __kSpindexerP = 5;
  private static final double __kSpindexerD = 0;

  private static final double __kFeedS = 5.3;
  private static final double __kFeedV = 0.07;
  private static final double __kFeedP = 10;
  private static final double __kFeedD = 0;

  private static final double __kSpindexerVelocity = 700; // RPMs
  private static final double __kFeedVelocity = 1500; // RPMs

  @Logged(name = "Spindexer", importance = Importance.CRITICAL)
  private final LoggedTalonFX spindexerMotor;

  @Logged(name = "Feed", importance = Importance.CRITICAL)
  private final LoggedTalonFX feedMotor;

  private final VelocityTorqueCurrentFOC spindexerControl;
  private final VelocityTorqueCurrentFOC feedControl;

  @Logged(name = "Enabled", importance = Importance.CRITICAL)
  private boolean enabled = false;

  private static final double kSpindexerStatorCurrentLimit = 40.0;
  private static final double kFeedStatorCurrentLimit = 80.0;

  private static final double kSpindexerConversionFactor = (36.0 / 14.0) * (72.0 / 24.0);
  private static final double kFeedConversionFactor = 2.0;

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
              config.MotorOutput.ControlTimesyncFreqHz = 100;

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
              config.MotorOutput.ControlTimesyncFreqHz = 100;

              config.Slot0.kS = __kFeedS;
              config.Slot0.kV = __kFeedV;
              config.Slot0.kP = __kFeedP;
              config.Slot0.kD = __kFeedD;
            });

    spindexerControl =
        new VelocityTorqueCurrentFOC(0).withSlot(0).withUseTimesync(true).withUpdateFreqHz(0);

    feedControl =
        new VelocityTorqueCurrentFOC(0).withSlot(0).withUseTimesync(true).withUpdateFreqHz(0);

    // make configuration tunable (must be done after configuration is bound to the motor)
    Tunable.bind("Spindexer", spindexerMotor);
    Tunable.bind("Feed", feedMotor);
  }

  @Override
  public void periodic() {
    if (enabled) {
      spindexerMotor.setControl(
          spindexerControl.withVelocity(
              RotationsPerSecond.convertFrom(kSpindexerVelocity.get(), RPM)
                  * kSpindexerConversionFactor));

      feedMotor.setControl(
          feedControl.withVelocity(
              RotationsPerSecond.convertFrom(kFeedVelocity.get(), RPM) * kFeedConversionFactor));
    } else {
      spindexerMotor.stopMotor();
      feedMotor.stopMotor();
    }
  }

  @Logged(name = "Velocity/Spindexer", importance = Importance.CRITICAL)
  public double getSpindexerVelocity() {
    return spindexerMotor.getMotorVelocity().in(RPM) / kSpindexerConversionFactor;
  }

  @Logged(name = "Velocity/Feed", importance = Importance.CRITICAL)
  public double getFeedVelocity() {
    return feedMotor.getMotorVelocity().in(RPM) / kFeedConversionFactor;
  }

  /** Enables the subsystem. */
  public Command enable() {
    return runEnd(() -> enabled = true, () -> enabled = false).withName("EnableShooter");
  }

  @Override
  public double getTotalSupplyCurrent() {
    return Math.abs(feedMotor.getSupplyCurrentAmps())
        + Math.abs(spindexerMotor.getSupplyCurrentAmps());
  }

  // spotless:off
  private static Tunable.DoubleValue kSpindexerVelocity = Tunable.value("Spindexer/Velocity", __kSpindexerVelocity);
  private static Tunable.DoubleValue kFeedVelocity = Tunable.value("Feed/Velocity", __kFeedVelocity);
  // spotless:on
}

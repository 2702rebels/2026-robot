package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.LoggedTalonFX;
import frc.robot.lib.PhoenixUtil;
import frc.robot.lib.Tunable;

public final class Shooter extends SubsystemBase {
  // MAKE SURE TO UPDATE THESE VALUES AFTER TUNING
  // DO NOT USE THESE VALUES DIRECTLY IN PERIODIC CODE,
  // USE THEM TO INITIALIZE TUNABLE VALUES AND CONTROLLERS
  private static final double __kFlywheelS = 6.85;
  private static final double __kFlywheelV = 0.023;
  private static final double __kFlywheelP = 10;
  private static final double __kFlywheelD = 0;
  private static final double __kHoodG = 0;
  private static final double __kHoodS = 0.5;
  private static final double __kHoodP = 3;
  private static final double __kHoodD = 0.2;

  private static final double __kHoodParkAngle = 10;

  private static final double __kHoodManual = 15;
  private static final double __kFlywheelManual = 4000; // RPMs

  @Logged(name = "Flywheel/Leader", importance = Importance.INFO)
  private final LoggedTalonFX flywheelMotorLeader;

  @Logged(name = "Flywheel/Follower", importance = Importance.INFO)
  private final LoggedTalonFX flywheelMotorFollower;

  @Logged(name = "Hood", importance = Importance.INFO)
  private final LoggedTalonFX hoodMotor;

  @Logged(name = "Setpoint/Flywheel", importance = Importance.INFO)
  private double flywheelSetpoint = 0;

  @Logged(name = "Setpoint/Hood", importance = Importance.INFO)
  private Angle hoodSetpoint = Degrees.zero();

  private final Follower followerControl;
  private final VelocityTorqueCurrentFOC flywheelControl;
  private final PositionTorqueCurrentFOC hoodControl;

  @Logged(name = "Enabled", importance = Importance.INFO)
  private boolean enabled = false;

  // indicates that the subsystem has been initialized by zeroing the position of the hood
  private boolean initialized = false;
  private boolean hoodParked;

  // manual mode used for testing
  private boolean manual;

  private static final double kFlywheelStatorCurrentLimit = 60.0;
  private static final double kHoodStatorCurrentLimit = 20.0;

  private static final double kHoodPositionMin = 0.0; // degrees
  private static final double KHoodPositionMax = 30.0; // degrees
  private static final double kHoodPositionTolerance = 0.5; // degrees
  private static final double kHoodConversionFactor = 108.0;

  private static final double kFlywheelVelocityMax = 6500.0;
  private static final double kFlywheelVelocityTolerance = 50.0; // RPMs
  private static final double kFlywheelConversionFactor = 15.0 / 18.0;

  /** Convert hood angular position to motor position in rotations. */
  private static double toHoodMotorPosition(Angle value) {
    return value.in(Rotations) * kHoodConversionFactor;
  }

  public Shooter(CANBus canbus) {
    flywheelMotorLeader =
        new LoggedTalonFX(
            Constants.CanivoreID.MOTOR_SHOOTER_FLYWHEEL_LEADER,
            canbus,
            (config) -> {
              config.CurrentLimits.StatorCurrentLimit = kFlywheelStatorCurrentLimit;
              config.CurrentLimits.StatorCurrentLimitEnable = true;
              config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
              config.MotorOutput.ControlTimesyncFreqHz = 250;

              config.Slot0.kS = __kFlywheelS;
              config.Slot0.kV = __kFlywheelV;
              config.Slot0.kP = __kFlywheelP;
              config.Slot0.kD = __kFlywheelD;
            });

    flywheelMotorFollower =
        new LoggedTalonFX(
            Constants.CanivoreID.MOTOR_SHOOTER_FLYWHEEL_FOLLOWER,
            canbus,
            (config) -> {
              config.CurrentLimits.StatorCurrentLimit = kFlywheelStatorCurrentLimit;
              config.CurrentLimits.StatorCurrentLimitEnable = true;
              config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
              config.MotorOutput.ControlTimesyncFreqHz = 250;
            });

    hoodMotor =
        new LoggedTalonFX(
            Constants.CanivoreID.MOTOR_SHOOTER_HOOD,
            canbus,
            (config) -> {
              config.CurrentLimits.StatorCurrentLimit = kHoodStatorCurrentLimit;
              config.CurrentLimits.StatorCurrentLimitEnable = true;
              config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
              config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
              config.MotorOutput.ControlTimesyncFreqHz = 100;

              config.Slot0.kG = __kHoodG;
              config.Slot0.kS = __kHoodS;
              config.Slot0.kP = __kHoodP;
              config.Slot0.kD = __kHoodD;
              config.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
            });

    // follow leader's output (inverted)
    followerControl =
        new Follower(flywheelMotorLeader.getDeviceID(), MotorAlignmentValue.Opposed)
            .withUpdateFreqHz(250);

    PhoenixUtil.run(
        "Shooter Follow Leader",
        flywheelMotorFollower.getDeviceID(),
        () -> flywheelMotorFollower.setControl(followerControl));

    flywheelControl =
        new VelocityTorqueCurrentFOC(0).withSlot(0).withUseTimesync(true).withUpdateFreqHz(0);

    hoodControl =
        new PositionTorqueCurrentFOC(toHoodMotorPosition(Degrees.of(KHoodPositionMax)))
            .withSlot(0)
            .withUseTimesync(true)
            .withUpdateFreqHz(0);

    // make configuration tunable (must be done after configuration is bound to the motor)
    Tunable.bind("Flywheel", flywheelMotorLeader);
    Tunable.bind("Hood", hoodMotor);

    manual = true; // FOR TESTING ONLY
  }

  @Override
  public void periodic() {
    // run initialization sequence regardless of subsystem disabled state
    if (!initialized) {
      hoodMotor.set(-0.1);

      // hood is zeroed against a hard stop
      if (hoodMotor.isStalled()) {
        hoodMotor.stopMotor();
        hoodMotor.resetPosition(0);
        initialized = true;
      }
    } else {
      if (enabled) {
        if (manual) {
          flywheelSetpoint = kFlywheelManual.get();
          hoodSetpoint = Degrees.of(kHoodManual.get());
        }

        flywheelMotorLeader.setControl(
            flywheelControl.withVelocity(
                RotationsPerSecond.convertFrom(flywheelSetpoint, RPM) * kFlywheelConversionFactor));

        final var hoodSetpointActual = hoodParked ? Degrees.of(kHoodParkAngle.get()) : hoodSetpoint;

        hoodMotor.setControl(hoodControl.withPosition(toHoodMotorPosition(hoodSetpointActual)));
      } else {
        flywheelMotorLeader.stopMotor();
        hoodMotor.stopMotor();
      }
    }
  }

  private void setAngle(Angle value) {
    hoodSetpoint =
        Degrees.of(MathUtil.clamp(value.in(Degrees), kHoodPositionMin, KHoodPositionMax));
  }

  private void setVelocity(AngularVelocity value) {
    flywheelSetpoint = MathUtil.clamp(value.in(RPM), 0, kFlywheelVelocityMax);
  }

  @Logged(name = "Initialized", importance = Importance.INFO)
  public boolean isInitialized() {
    return initialized;
  }

  @Logged(name = "HoodParked", importance = Importance.INFO)
  public boolean isHoodParked() {
    return hoodParked;
  }

  /** Determines whether the subsystem is at its commanded setpoint. */
  @Logged(name = "AtSetpoint", importance = Importance.INFO)
  public boolean isAtSetpoint() {
    return MathUtil.isNear(hoodSetpoint.in(Degrees), getAngle().in(Degrees), kHoodPositionTolerance)
        && MathUtil.isNear(flywheelSetpoint, getVelocity(), kFlywheelVelocityTolerance);
  }

  @Logged(name = "Angle", importance = Importance.INFO)
  public Angle getAngle() {
    return hoodMotor.getMotorPosition().div(kHoodConversionFactor);
  }

  @Logged(name = "Velocity", importance = Importance.INFO)
  public double getVelocity() {
    return flywheelMotorLeader.getMotorVelocity().in(RPM) / kFlywheelConversionFactor;
  }

  /** Disables the subsystem. */
  public Command disable() {
    return runOnce(
            () -> {
              enabled = false;
            })
        .withName("ShooterDisable");
  }

  /** Enables the subsystem. */
  public Command enable() {
    return runOnce(
            () -> {
              enabled = true;
            })
        .withName("ShooterEnable");
  }

  /** Parks hood for driving under the trench. */
  public Command parkHood() {
    return runOnce(
            () -> {
              this.hoodParked = true;
            })
        .withName("ParkHood");
  }

  /** Releases hood if previously parked. */
  public Command unparkHood() {
    return runOnce(
            () -> {
              this.hoodParked = false;
            })
        .withName("UnparkHood");
  }

  /** Sets shooting parameters: hood angle and flwyheel velocity. */
  public Command setParameters(Angle angle, AngularVelocity speed) {
    return runOnce(
            () -> {
              setAngle(angle);
              setVelocity(speed);
            })
        .withName("SetParameters");
  }

  // spotless:off
  private static Tunable.DoubleValue kHoodParkAngle = Tunable.value("Hood/ParkAngle", __kHoodParkAngle);
  private static Tunable.DoubleValue kHoodManual = Tunable.value("Hood/Manual", __kHoodManual);
  private static Tunable.DoubleValue kFlywheelManual = Tunable.value("Flywheel/Manual", __kFlywheelManual);
  // spotless:on
}

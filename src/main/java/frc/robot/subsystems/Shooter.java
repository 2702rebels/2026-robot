package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.LoggedTalonFX;
import frc.robot.lib.PhoenixUtil;
import frc.robot.lib.Tunable;

public final class Shooter extends SubsystemBase {
  // MAKE SURE TO UPDATE THESE VALUES AFTER TUNING
  // DO NOT USE THESE VALUES DIRECTLY IN PERIODIC CODE,
  // USE THEM TO INITIALIZE TUNABLE VALUES AND CONTROLLERS
  private static final double __kFlywheelS = 0;
  private static final double __kFlywheelV = 0;
  private static final double __kFlywheelP = 0;
  private static final double __kFlywheelD = 0;
  private static final double __kHoodG = 0;
  private static final double __kHoodS = 0;
  private static final double __kHoodP = 0;
  private static final double __kHoodD = 0;

  private static final double __kHoodParkAngle = 85;

  @Logged(name = "Flywheel/Leader", importance = Importance.INFO)
  private final LoggedTalonFX flywheelMotorLeader;

  @Logged(name = "Flywheel/Follower", importance = Importance.INFO)
  private final LoggedTalonFX flywheelMotorFollower;

  @Logged(name = "Hood", importance = Importance.INFO)
  private final LoggedTalonFX hoodMotor;

  @Logged(name = "Setpoint/Flywheel", importance = Importance.INFO)
  private double flywheelSetpoint;

  @Logged(name = "Setpoint/Hood", importance = Importance.INFO)
  private double hoodSetpoint;

  private final Follower followerControl;
  private final VelocityTorqueCurrentFOC flywheelControl;
  private final PositionTorqueCurrentFOC hoodControl;

  // indicates that the subsystem has been initialized by zeroing the position of the hood
  private boolean initialized;
  private boolean disabled;
  private boolean hoodParked;

  private static final double kFlywheelStatorCurrentLimit = 60;
  private static final double kHoodStatorCurrentLimit = 30;

  private static final double kHoodPositionMin = 40; // degrees
  private static final double KHoodPositionMax = 85; // degrees
  private static final double kHoodPositionTolerance = 0.5; // degrees
  private static final double kHoodConversionFactor = 1;
  private static final double kFlywheelVelocityTolerance = 3; // RPMs

  public Shooter(CANBus canbus) {
    flywheelMotorLeader =
        new LoggedTalonFX(
            Constants.CanivoreID.MOTOR_SHOOTER_FLYWHEEL_LEADER,
            canbus,
            (config) -> {
              config.CurrentLimits.StatorCurrentLimit = kFlywheelStatorCurrentLimit;
              config.CurrentLimits.StatorCurrentLimitEnable = true;
              config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

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
            });

    hoodMotor =
        new LoggedTalonFX(
            Constants.CanivoreID.MOTOR_SHOOTER_HOOD,
            canbus,
            (config) -> {
              config.CurrentLimits.StatorCurrentLimit = kHoodStatorCurrentLimit;
              config.CurrentLimits.StatorCurrentLimitEnable = true;
              config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

              config.Slot0.kG = __kHoodG;
              config.Slot0.kS = __kHoodS;
              config.Slot0.kP = __kHoodP;
              config.Slot0.kD = __kHoodD;
            });

    // follow leader's output (inverted)
    followerControl = new Follower(flywheelMotorLeader.getDeviceID(), MotorAlignmentValue.Opposed);
    PhoenixUtil.run(
        "Shooter Follow Leader",
        flywheelMotorFollower.getDeviceID(),
        () -> flywheelMotorFollower.setControl(followerControl));

    flywheelControl = new VelocityTorqueCurrentFOC(0).withSlot(0);
    hoodControl =
        new PositionTorqueCurrentFOC(KHoodPositionMax * kHoodConversionFactor).withSlot(0);

    // make configuration tunable (must be done after configuration is bound to the motor)
    Tunable.bind("Flywheel", flywheelMotorLeader);
    Tunable.bind("Hood", hoodMotor);

    reset();
  }

  @Override
  public void periodic() {
    if (disabled) {
      flywheelMotorLeader.stopMotor();
      hoodMotor.stopMotor();
    } else if (initialized) {
      flywheelMotorLeader.setControl(flywheelControl.withVelocity(flywheelSetpoint));
      hoodMotor.setControl(
          hoodControl.withPosition(
              (hoodParked ? kHoodParkAngle.get() : hoodSetpoint) * kHoodConversionFactor));
    }
  }

  private void reset() {
    hoodMotor.setPosition(0);
  }

  private void setAngle(double valueInDegrees) {
    hoodSetpoint = MathUtil.clamp(valueInDegrees, kHoodPositionMin, KHoodPositionMax);
  }

  private void setVelocity(double valueInRPM) {
    hoodSetpoint = MathUtil.clamp(valueInRPM, 0, 3000);
  }

  /** Determines whether the subsystem is at its commanded setpoint. */
  public boolean isAtSetpoint() {
    return MathUtil.isNear(hoodSetpoint, getAngle(), kHoodPositionTolerance)
        && MathUtil.isNear(flywheelSetpoint, getVelocity(), kFlywheelVelocityTolerance);
  }

  @Logged(name = "Angle", importance = Importance.INFO)
  public double getAngle() {
    return hoodMotor.getPositionRotations() / kHoodConversionFactor;
  }

  @Logged(name = "Velocity", importance = Importance.INFO)
  public double getVelocity() {
    return flywheelMotorLeader.getVelocityRPM();
  }

  /** Initializes the shooter by zeroing the hood. */
  public Command initialize() {
    return new FunctionalCommand(
            () -> {},
            () -> {
              hoodMotor.set(-0.1);
            },
            interrupted -> {
              reset();
              hoodMotor.stopMotor();
              initialized = true;
            },
            () ->
                hoodMotor.getStatorCurrentAmps() > kHoodStatorCurrentLimit * 0.9
                    && Math.abs(hoodMotor.getVelocityRPM()) < 5,
            this)
        .withName("Initialize");
  }

  /** Disables the subsystem. */
  public Command disable() {
    return runOnce(
            () -> {
              disabled = true;
            })
        .withName("ShooterDisable");
  }

  /** Enables the subsystem. */
  public Command enable() {
    return runOnce(
            () -> {
              disabled = false;
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
  public Command setParameters(double angleInDegrees, double speedInRPM) {
    return runOnce(
            () -> {
              setAngle(angleInDegrees);
              setVelocity(speedInRPM);
            })
        .withName("SetParameters");
  }

  // spotless:off
  private static Tunable.DoubleValue kHoodParkAngle = Tunable.value("Hood/ParkAngle", __kHoodParkAngle);
  // spotless:on
}

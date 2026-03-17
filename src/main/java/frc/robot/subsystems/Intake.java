package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.LoggedTalonFX;
import frc.robot.lib.PhoenixUtil;
import frc.robot.lib.Tunable;

public final class Intake extends SubsystemBase {
  // MAKE SURE TO UPDATE THESE VALUES AFTER TUNING
  // DO NOT USE THESE VALUES DIRECTLY IN PERIODIC CODE,
  // USE THEM TO INITIALIZE TUNABLE VALUES AND CONTROLLERS
  private static final double __kRollerS = 2.5;
  private static final double __kRollerV = 0.016;
  private static final double __kRollerP = 5;
  private static final double __kRollerD = 0;

  private static final double __kArmMoveG = 20;
  private static final double __kArmMoveS = 1.5;
  private static final double __kArmMoveP = 310;
  private static final double __kArmMoveD = 35;

  private static final double __kArmHoldG = 0;
  private static final double __kArmHoldS = 5;
  private static final double __kArmHoldP = 1500;
  private static final double __kArmHoldD = 0;

  private static final double __kRollerVelocity = 2700; // RPMs
  private static final double __kHoverPosition = 20; // degrees
  private static final double __kDeployPosition = 0; // degrees

  @Logged(name = "Roller", importance = Importance.CRITICAL)
  private final LoggedTalonFX rollerMotor;

  @Logged(name = "Arm", importance = Importance.CRITICAL)
  private final LoggedTalonFX armMotor;

  @Logged(name = "Setpoint/Arm", importance = Importance.DEBUG)
  private Angle armSetpoint = Degrees.of(0);

  @Logged(name = "Setpoint/Roller", importance = Importance.DEBUG)
  private double rollerSetpoint = 0;

  private final VelocityTorqueCurrentFOC rollerControl;
  private final PositionTorqueCurrentFOC armMoveControl;
  private final PositionTorqueCurrentFOC armHoldControl;

  @Logged(name = "Enabled", importance = Importance.CRITICAL)
  private boolean enabled = false;

  private boolean deployed;
  private boolean stopped;

  private static final double kRollerStatorCurrentLimit = 50.0;
  private static final double kArmStatorCurrentLimit = 50.0;

  private static final double kRollerConversionFactor = 36.0 / 18.0;
  private static final double kArmConversionFactor = 61.1;
  private static final double kRollerTolerance = 50.0; // RPMs
  private static final double kArmTolerance = 2.0; // degrees
  private static final double kArmHoldThreshold = 7.0; // degrees
  private static final Angle kStowedPosition = Degrees.of(90);
  private static final Angle kMaxPosition = Degrees.of(120);
  private static final Angle kMinPosition = Degrees.of(-5);

  public Intake(CANBus canbus) {
    rollerMotor =
        new LoggedTalonFX(
            Constants.CanivoreID.MOTOR_INTAKE_ROLLER,
            canbus,
            (config) -> {
              config.CurrentLimits.StatorCurrentLimit = kRollerStatorCurrentLimit;
              config.CurrentLimits.StatorCurrentLimitEnable = true;
              config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
              config.MotorOutput.ControlTimesyncFreqHz = 100;

              config.Slot0.kS = __kRollerS;
              config.Slot0.kV = __kRollerV;
              config.Slot0.kP = __kRollerP;
              config.Slot0.kD = __kRollerD;
            });

    armMotor =
        new LoggedTalonFX(
            Constants.CanivoreID.MOTOR_INTAKE_ARM,
            canbus,
            (config) -> {
              config.CurrentLimits.StatorCurrentLimit = kArmStatorCurrentLimit;
              config.CurrentLimits.StatorCurrentLimitEnable = true;
              config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
              config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
              config.MotorOutput.ControlTimesyncFreqHz = 250;
              config.Feedback.SensorToMechanismRatio = kArmConversionFactor;
              config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
              config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = kMaxPosition.in(Rotations);
              config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
              config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = kMinPosition.in(Rotations);

              config.Slot0.kS = __kArmMoveS;
              config.Slot0.kG = __kArmMoveG;
              config.Slot0.kP = __kArmMoveP;
              config.Slot0.kD = __kArmMoveD;
              config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
              config.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

              config.Slot1.kS = __kArmHoldS;
              config.Slot1.kG = __kArmHoldG;
              config.Slot1.kP = __kArmHoldP;
              config.Slot1.kD = __kArmHoldD;
              config.Slot1.GravityType = GravityTypeValue.Arm_Cosine;
              config.Slot1.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
            });

    rollerControl =
        new VelocityTorqueCurrentFOC(0).withSlot(0).withUseTimesync(true).withUpdateFreqHz(0);

    armMoveControl =
        new PositionTorqueCurrentFOC(0).withSlot(0).withUseTimesync(true).withUpdateFreqHz(0);

    armHoldControl =
        new PositionTorqueCurrentFOC(0).withSlot(1).withUseTimesync(true).withUpdateFreqHz(0);

    // make configuration tunable (must be done after configuration is bound to the motor)
    Tunable.bind("Roller", rollerMotor);
    Tunable.bind("Arm/Move", armMotor);
    Tunable.bind1("Arm/Hold", armMotor);

    // zero position based on the initial configuration
    PhoenixUtil.run("Intake Arm Zero", armMotor.getDeviceID(), () -> armMotor.setPosition(0));
  }

  @Override
  public void periodic() {
    if (enabled) {
      rollerSetpoint = kRollerVelocity.get();
      rollerMotor.setControl(
          rollerControl.withVelocity(
              RotationsPerSecond.convertFrom(rollerSetpoint, RPM) * kRollerConversionFactor));

      armSetpoint =
          Degrees.of(rollerMotor.isStalled() ? kHoverPosition.get() : kDeployPosition.get());

      // we have two control modes here to consider: when we are at zero, we are resting
      // on the hard stops and we don't need kG/kS to maintain position, however, for
      // the full motion we need to use control mode with kG/kS
      //
      // note that we only switch to hold mode when we are _actually_ close to the zero setpoint
      if (armSetpoint.in(Degrees) <= kArmHoldThreshold
          && getAngle().in(Degrees) <= kArmHoldThreshold)
        armMotor.setControl(armHoldControl.withPosition(armSetpoint.in(Rotations)));
      else armMotor.setControl(armMoveControl.withPosition(armSetpoint.in(Rotations)));

      // if (deployed) {
      //   armSetpoint = Degrees.of(kDeployPosition.get());

      //   if (stopped) {
      //     rollerSetpoint = 0;
      //     rollerMotor.stopMotor();
      //   } else {
      //     rollerSetpoint = kRollerVelocity.get();
      //     rollerMotor.setControl(
      //         rollerControl.withVelocity(
      //             RotationsPerSecond.convertFrom(rollerSetpoint, RPM) *
      // kRollerConversionFactor));
      //   }
      // } else {
      //   armSetpoint = kStowedPosition;
      //   rollerSetpoint = 0;
      //   rollerMotor.stopMotor();
      // }

      // armMotor.setControl(
      //     armControl.withPosition(armSetpoint.in(Rotations) * kArmConversionFactor));
    } else {
      rollerMotor.stopMotor();
      armMotor.stopMotor();
    }
  }

  @Logged(name = "Deployed", importance = Importance.CRITICAL)
  public boolean isDeployed() {
    return deployed;
  }

  /** Determines whether the subsystem is at its nominal state. */
  @Logged(name = "Nominal", importance = Importance.CRITICAL)
  public boolean isNominal() {
    return MathUtil.isNear(armSetpoint.in(Degrees), getAngle().in(Degrees), kArmTolerance)
        && MathUtil.isNear(rollerSetpoint, getVelocity(), kRollerTolerance);
  }

  @Logged(name = "Velocity", importance = Importance.CRITICAL)
  public double getVelocity() {
    return rollerMotor.getMotorVelocity().in(RPM) / kRollerConversionFactor;
  }

  @Logged(name = "Angle", importance = Importance.CRITICAL)
  public Angle getAngle() {
    return armMotor.getMotorPosition();
  }

  /** Disables the subsystem. */
  public Command disable() {
    return runOnce(() -> enabled = false).withName("IntakeDisable");
  }

  /** Enables the subsystem. */
  public Command enable() {
    return runOnce(() -> enabled = true).withName("IntakeEnable");
  }

  /** Deploys the intake. */
  public Command deploy() {
    return runOnce(() -> deployed = true).withName("IntakeDeploy");
  }

  /** Retracts the intake. */
  public Command retract() {
    return runOnce(() -> deployed = false).withName("IntakeRetract");
  }

  /** Stops the intake roller. */
  public Command stop() {
    return runOnce(
            () -> {
              stopped = true;
            })
        .withName("IntakeStop");
  }

  /** Starts the intake roller. */
  public Command start() {
    return runOnce(
            () -> {
              stopped = false;
            })
        .withName("IntakeStart");
  }

  // spotless:off
  private static Tunable.DoubleValue kRollerVelocity = Tunable.value("Roller/Velocity", __kRollerVelocity);
  private static Tunable.DoubleValue kDeployPosition = Tunable.value("Arm/Deploy", __kDeployPosition);
  private static Tunable.DoubleValue kHoverPosition = Tunable.value("Arm/Hover", __kHoverPosition);
  // spotless:on
}

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.EnergyReporter;
import frc.robot.lib.LoggedTalonFX;
import frc.robot.lib.PhoenixUtil;
import frc.robot.lib.Tunable;

public final class Intake extends SubsystemBase implements EnergyReporter {
  // MAKE SURE TO UPDATE THESE VALUES AFTER TUNING
  // DO NOT USE THESE VALUES DIRECTLY IN PERIODIC CODE,
  // USE THEM TO INITIALIZE TUNABLE VALUES AND CONTROLLERS
  private static final double __kRollerS = 2.5;
  private static final double __kRollerV = 0.016;
  private static final double __kRollerP = 5;
  private static final double __kRollerD = 0;

  // roller SPID values for VoltageVelocity control w/o FOC (in case we need it)
  // S = 0.18, V = 0.11, P = 7, D = 0

  private static final double __kDeployHoldS = 2;
  private static final double __kDeployHoldP = 10;
  private static final double __kDeployHoldD = 0;

  private static final double __kDeployExtendS = 0;
  private static final double __kDeployExtendP = 5;
  private static final double __kDeployExtendI = 0.5;
  private static final double __kDeployExtendD = 0.5;

  private static final double __kDeployRetractS = 0;
  private static final double __kDeployRetractP = 5;
  private static final double __kDeployRetractI = 0.5;
  private static final double __kDeployRetractD = 0.5;

  private static final double __kRollerVelocity = 2750; // RPMs
  private static final double __kRetractedPosition = 1; // inches
  private static final double __kExtendedPosition = 11; // inches

  @Logged(name = "Roller", importance = Importance.CRITICAL)
  private final LoggedTalonFX rollerMotor;

  @Logged(name = "Deploy", importance = Importance.CRITICAL)
  private final LoggedTalonFX deployMotor;

  @Logged(name = "Setpoint/Deploy", importance = Importance.DEBUG)
  private double deploySetpoint = 0;

  @Logged(name = "Setpoint/Roller", importance = Importance.DEBUG)
  private double rollerSetpoint = 0;

  private final VelocityTorqueCurrentFOC rollerControl;
  private final PositionTorqueCurrentFOC deployHoldControl;
  private final PositionTorqueCurrentFOC deployExtendControl;
  private final PositionTorqueCurrentFOC deployRetractControl;

  @Logged(name = "Enabled", importance = Importance.CRITICAL)
  private boolean enabled = false;

  private static enum State {
    kHolding,
    kExtending,
    kRetracting,
  };

  private State state = State.kHolding;

  private boolean deployed;

  private static final double kRollerStatorCurrentLimit = 50.0;
  private static final double kDeployStatorCurrentLimit = 40.0;

  private static final double kRollerConversionFactor = 36.0 / 18.0;
  private static final double kDeployConversionFactor = 10.0 / (1.6 * Math.PI);
  private static final double kRollerTolerance = 50.0; // RPMs
  private static final double kDeployTolerance = 0.5; // inches
  private static final double kMaxPosition = 11.5;
  private static final double kMinPosition = 0;

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

    deployMotor =
        new LoggedTalonFX(
            Constants.CanivoreID.MOTOR_INTAKE_DEPLOY,
            canbus,
            (config) -> {
              config.CurrentLimits.StatorCurrentLimit = kDeployStatorCurrentLimit;
              config.CurrentLimits.StatorCurrentLimitEnable = true;
              config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
              config.MotorOutput.ControlTimesyncFreqHz = 250;
              config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
              config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
                  kMaxPosition * kDeployConversionFactor;
              config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
              config.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
                  kMinPosition * kDeployConversionFactor;

              config.Slot0.kS = __kDeployHoldS;
              config.Slot0.kP = __kDeployHoldP;
              config.Slot0.kD = __kDeployHoldD;
              config.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

              config.Slot1.kS = __kDeployExtendS;
              config.Slot1.kP = __kDeployExtendP;
              config.Slot1.kI = __kDeployExtendI;
              config.Slot1.kD = __kDeployExtendD;
              config.Slot1.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

              config.Slot2.kS = __kDeployRetractS;
              config.Slot2.kP = __kDeployRetractP;
              config.Slot2.kI = __kDeployRetractI;
              config.Slot2.kD = __kDeployRetractD;
              config.Slot2.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
            });

    rollerControl =
        new VelocityTorqueCurrentFOC(0).withSlot(0).withUseTimesync(true).withUpdateFreqHz(0);

    deployHoldControl =
        new PositionTorqueCurrentFOC(0).withSlot(0).withUseTimesync(true).withUpdateFreqHz(0);

    deployExtendControl =
        new PositionTorqueCurrentFOC(0).withSlot(1).withUseTimesync(true).withUpdateFreqHz(0);

    deployRetractControl =
        new PositionTorqueCurrentFOC(0).withSlot(2).withUseTimesync(true).withUpdateFreqHz(0);

    // make configuration tunable (must be done after configuration is bound to the motor)
    Tunable.bind("Roller", rollerMotor);
    Tunable.bind("Deploy/Hold", deployMotor);
    Tunable.bind1("Deploy/Extend", deployMotor);
    Tunable.bind2("Deploy/Retract", deployMotor);

    // zero position based on the initial configuration
    PhoenixUtil.run(
        "Intake Deploy Zero", deployMotor.getDeviceID(), () -> deployMotor.setPosition(0));
  }

  @Override
  public void periodic() {
    ControlRequest controlRequest;
    switch (state) {
      case kExtending:
        controlRequest = deployExtendControl.withPosition(deploySetpoint * kDeployConversionFactor);
        break;
      case kRetracting:
        controlRequest =
            deployRetractControl.withPosition(deploySetpoint * kDeployConversionFactor);
        break;
      default:
        controlRequest = deployHoldControl.withPosition(deploySetpoint * kDeployConversionFactor);
        break;
    }

    // switch to holding pattern once the position is reached
    if (state != State.kHolding
        && MathUtil.isNear(deploySetpoint, getPosition(), kDeployTolerance)) {
      deployed = state == State.kExtending;
      state = State.kHolding;
    }

    deployMotor.setControl(controlRequest);

    if (enabled) {
      rollerSetpoint = kRollerVelocity.get();
      rollerMotor.setControl(
          rollerControl.withVelocity(
              RotationsPerSecond.convertFrom(rollerSetpoint, RPM) * kRollerConversionFactor));
    } else {
      rollerMotor.stopMotor();
    }
  }

  @Logged(name = "Deployed", importance = Importance.CRITICAL)
  public boolean isDeployed() {
    return deployed;
  }

  /** Determines whether the subsystem is at its nominal state. */
  @Logged(name = "Nominal", importance = Importance.CRITICAL)
  public boolean isNominal() {
    return MathUtil.isNear(deploySetpoint, getPosition(), kDeployTolerance)
        && MathUtil.isNear(rollerSetpoint, getVelocity(), kRollerTolerance);
  }

  @Logged(name = "Velocity", importance = Importance.CRITICAL)
  public double getVelocity() {
    return rollerMotor.getMotorVelocity().in(RPM) / kRollerConversionFactor;
  }

  @Logged(name = "Position", importance = Importance.CRITICAL)
  public double getPosition() {
    return deployMotor.getMotorPosition().in(Rotations) / kDeployConversionFactor;
  }

  /** Disables the subsystem. */
  public Command disable() {
    return runOnce(() -> enabled = false).withName("IntakeDisable");
  }

  /** Enables the subsystem. */
  public Command enable() {
    return runOnce(() -> enabled = true).withName("IntakeEnable");
  }

  /** Extends the intake. */
  public Command extend() {
    return runOnce(
            () -> {
              state = State.kExtending;
              deploySetpoint = kExtendedPosition.get();
            })
        .withName("IntakeExtend");
  }

  /** Retracts the intake. */
  public Command retract() {
    return runOnce(
            () -> {
              state = State.kRetracting;
              deploySetpoint = kRetractedPosition.get();
            })
        .withName("IntakeRetract");
  }

  @Override
  public double getTotalSupplyCurrent() {
    return Math.abs(deployMotor.getSupplyCurrentAmps())
        + Math.abs(rollerMotor.getSupplyCurrentAmps());
  }

  /** Runs self-check. */
  public Command selfCheck() {
    return Commands.sequence(
        Commands.print("Intake self-check started"), Commands.print("Intake self-check completed"));
  }

  // spotless:off
  private static Tunable.DoubleValue kRollerVelocity = Tunable.value("Roller/Velocity", __kRollerVelocity);
  private static Tunable.DoubleValue kExtendedPosition = Tunable.value("Deploy/Extended", __kExtendedPosition);
  private static Tunable.DoubleValue kRetractedPosition = Tunable.value("Deploy/Retracted", __kRetractedPosition);
  // spotless:on
}

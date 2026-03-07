package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.signals.NeutralModeValue;
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
  private static final double __kRollerS = 0;
  private static final double __kRollerV = 0;
  private static final double __kRollerP = 5;
  private static final double __kRollerD = 0;

  private static final double __kDeployS = 0;
  private static final double __kDeployV = 0;
  private static final double __kDeployP = 2;
  private static final double __kDeployD = 0;

  private static final double __kRollerVelocity = 2500; // RPMs
  private static final double __kDeployPosition =
      0; // degrees (change to +90 appox. once properly stowed at the beginning)

  @Logged(name = "Roller", importance = Importance.INFO)
  private final LoggedTalonFX rollerMotor;

  @Logged(name = "Deploy", importance = Importance.INFO)
  private final LoggedTalonFX deployMotor;

  @Logged(name = "Setpoint/Deploy", importance = Importance.INFO)
  private Angle deploySetpoint;

  @Logged(name = "Setpoint/Roller", importance = Importance.INFO)
  private double rollerSetpoint;

  private final VelocityTorqueCurrentFOC rollerControl;
  private final PositionTorqueCurrentFOC deployControl;

  private boolean disabled;
  private boolean deployed;
  private boolean stopped;

  private static final double kRollerStatorCurrentLimit = 30.0;
  private static final double kDeployStatorCurrentLimit = 30.0;

  private static final double kRollerConversionFactor = 36.0 / 18.0;
  private static final double kDeployConversionFactor = 61.1;
  private static final double kRollerTolerance = 5.0; // RPMs
  private static final double kDeployTolerance = 2.0; // degrees
  private static final double kStowedPosition = 0.0; // degrees

  public Intake(CANBus canbus) {
    rollerMotor =
        new LoggedTalonFX(
            Constants.CanivoreID.MOTOR_INTAKE_ROLLER,
            canbus,
            (config) -> {
              config.CurrentLimits.StatorCurrentLimit = kRollerStatorCurrentLimit;
              config.CurrentLimits.StatorCurrentLimitEnable = true;
              config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

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

              config.Slot0.kS = __kDeployS;
              config.Slot0.kV = __kDeployV;
              config.Slot0.kP = __kDeployP;
              config.Slot0.kD = __kDeployD;
            });

    rollerControl = new VelocityTorqueCurrentFOC(0).withSlot(0);
    deployControl = new PositionTorqueCurrentFOC(0).withSlot(0);

    // make configuration tunable (must be done after configuration is bound to the motor)
    Tunable.bind("Roller", rollerMotor);
    Tunable.bind("Deploy", deployMotor);

    // zero position based on the initial configuration
    PhoenixUtil.run(
        "Intake Deploy Zero", deployMotor.getDeviceID(), () -> deployMotor.setPosition(0));

    disabled = true;
    deploySetpoint = Degrees.of(kStowedPosition);
    rollerSetpoint = 0;
  }

  @Override
  public void periodic() {
    if (disabled) {
      rollerMotor.stopMotor();
      deployMotor.stopMotor();
    } else {
      rollerSetpoint = kRollerVelocity.get();
      rollerMotor.setControl(
          rollerControl.withVelocity(
              RotationsPerSecond.convertFrom(rollerSetpoint, RPM) * kRollerConversionFactor));

      deploySetpoint = Degrees.of(kDeployPosition.get());
      deployMotor.setControl(
          deployControl.withPosition(deploySetpoint.in(Rotations) * kDeployConversionFactor));

      // if (deployed) {
      //   deploySetpoint = Degrees.of(kDeployPosition.get());

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
      //   deploySetpoint = Degrees.of(kStowedPosition);
      //   rollerSetpoint = 0;
      //   rollerMotor.stopMotor();
      // }

      // deployMotor.setControl(
      //     deployControl.withPosition(deploySetpoint.in(Rotations) * kDeployConversionFactor));
    }
  }

  @Logged(name = "Deployed", importance = Importance.INFO)
  public boolean isDeployed() {
    return deployed;
  }

  /** Determines whether the subsystem is at its nominal state. */
  @Logged(name = "Nominal", importance = Importance.INFO)
  public boolean isNominal() {
    return MathUtil.isNear(deploySetpoint.in(Degrees), getAngle().in(Degrees), kDeployTolerance)
        && MathUtil.isNear(rollerSetpoint, getVelocity(), kRollerTolerance);
  }

  @Logged(name = "Velocity", importance = Importance.INFO)
  public double getVelocity() {
    return rollerMotor.getVelocityRPM() / kRollerConversionFactor;
  }

  @Logged(name = "Angle", importance = Importance.INFO)
  public Angle getAngle() {
    return deployMotor.getPositionMeasured().div(kDeployConversionFactor);
  }

  /** Disables the subsystem. */
  public Command disable() {
    return runOnce(
            () -> {
              disabled = true;
            })
        .withName("IntakeDisable");
  }

  /** Enables the subsystem. */
  public Command enable() {
    return runOnce(
            () -> {
              disabled = false;
            })
        .withName("IntakeEnable");
  }

  /** Deploys the intake. */
  public Command deploy() {
    return runOnce(
            () -> {
              deployed = true;
            })
        .withName("IntakeDeploy");
  }

  /** Retracts the intake. */
  public Command retract() {
    return runOnce(
            () -> {
              deployed = false;
            })
        .withName("IntakeRetract");
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
  private static Tunable.DoubleValue kDeployPosition = Tunable.value("Deploy/Position", __kDeployPosition);
  // spotless:on
}

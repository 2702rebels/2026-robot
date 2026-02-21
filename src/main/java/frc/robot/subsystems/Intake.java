package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.LoggedTalonFX;
import frc.robot.lib.Tunable;

public final class Intake extends SubsystemBase {
  // MAKE SURE TO UPDATE THESE VALUES AFTER TUNING
  // DO NOT USE THESE VALUES DIRECTLY IN PERIODIC CODE,
  // USE THEM TO INITIALIZE TUNABLE VALUES AND CONTROLLERS
  private static final double __kRollerS = 0;
  private static final double __kRollerV = 0;
  private static final double __kRollerP = 0;
  private static final double __kRollerD = 0;

  private static final double __kDeployS = 0;
  private static final double __kDeployV = 0;
  private static final double __kDeployP = 0;
  private static final double __kDeployD = 0;

  private static final double __kRollerVelocity = 100; // RPMs
  private static final double __kDeployPosition = 80;

  @Logged(name = "Roller", importance = Importance.INFO)
  private final LoggedTalonFX rollerMotor;

  @Logged(name = "Deploy", importance = Importance.INFO)
  private final LoggedTalonFX deployMotor;

  private final VelocityTorqueCurrentFOC rollerControl;
  private final PositionTorqueCurrentFOC deployControl;

  private boolean disabled;
  private boolean deployed;
  private boolean stopped;

  private static final double kRollerStatorCurrentLimit = 30;
  private static final double kDeployStatorCurrentLimit = 30;

  private static final double kRollerConversionFactor = 29 / 18;
  private static final double kDeployConversionFactor = (44 / 12) * 360; // converted to degrees
  private static final double kRollerTolerance = 5; // RPMs
  private static final double kDeployTolerance = 1; // degrees

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
    deployMotor.setPosition(0);
  }

  @Override
  public void periodic() {
    if (disabled) {
      rollerMotor.stopMotor();
      deployMotor.stopMotor();
    } else {
      if (deployed) {
        if (stopped) {
          rollerMotor.stopMotor();
        } else {
          rollerMotor.setControl(
              rollerControl.withVelocity(kRollerVelocity.get() * kRollerConversionFactor));
        }

        deployMotor.setControl(
            deployControl.withPosition(kDeployPosition.get() * kDeployConversionFactor));
      } else {
        rollerMotor.stopMotor();
        deployMotor.setControl(deployControl.withPosition(0));
      }
    }
  }

  @Logged(name = "Deployed", importance = Importance.INFO)
  public boolean isDeployed() {
    return deployed;
  }

  /** Determines whether the subsystem is at its nominal state. */
  @Logged(name = "Nominal", importance = Importance.INFO)
  public boolean isNominal() {
    return deployed
        ? MathUtil.isNear(kDeployPosition.get(), getAngle(), kDeployTolerance)
            && MathUtil.isNear(kRollerVelocity.get(), getVelocity(), kRollerTolerance)
        : MathUtil.isNear(0, getAngle(), kDeployTolerance)
            && MathUtil.isNear(0, getVelocity(), kRollerTolerance);
  }

  @Logged(name = "Velocity", importance = Importance.INFO)
  public double getVelocity() {
    return rollerMotor.getVelocityRPM() / kRollerConversionFactor;
  }

  @Logged(name = "Angle", importance = Importance.INFO)
  public double getAngle() {
    return deployMotor.getPositionRotations() / kDeployConversionFactor;
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

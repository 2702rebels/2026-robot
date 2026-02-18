package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.LoggedTalonFXS;
import frc.robot.lib.Tunable;

public final class Climb extends SubsystemBase {
  // MAKE SURE TO UPDATE THESE VALUES AFTER TUNING
  // DO NOT USE THESE VALUES DIRECTLY IN PERIODIC CODE,
  // USE THEM TO INITIALIZE TUNABLE VALUES AND CONTROLLERS
  private static final double __kS = 0;
  private static final double __kP = 0;
  private static final double __kD = 0;

  @Logged(name = "Motor", importance = Importance.INFO)
  private final LoggedTalonFXS motor;

  @Logged(name = "Setpoint", importance = Importance.INFO)
  private double setpoint;

  private final PositionTorqueCurrentFOC control;

  private boolean disabled;

  private static final double kStatorCurrentLimit = 60;
  private static final double kConversionFactor = 1;
  private static final double kTolerance = 1;

  public Climb(CANBus canbus) {
    motor =
        new LoggedTalonFXS(
            Constants.CanivoreID.MOTOR_CLIMB,
            canbus,
            (config) -> {
              config.CurrentLimits.StatorCurrentLimit = kStatorCurrentLimit;
              config.CurrentLimits.StatorCurrentLimitEnable = true;
              config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

              config.Slot0.kS = __kS;
              config.Slot0.kP = __kP;
              config.Slot0.kD = __kD;
            });

    control = new PositionTorqueCurrentFOC(0).withSlot(0);

    // make configuration tunable (must be done after configuration is bound to the motor)
    Tunable.bind("Motor", motor);

    // TODO: support for ToF sensor
  }

  @Override
  public void periodic() {
    if (disabled) {
      motor.stopMotor();
    } else {
      motor.setControl(control.withPosition(setpoint * kConversionFactor));
    }
  }

  /** Determines whether the subsystem is at its commanded setpoint. */
  public boolean isAtSetpoint() {
    return MathUtil.isNear(setpoint, getPosition(), kTolerance);
  }

  @Logged(name = "Position", importance = Importance.INFO)
  public double getPosition() {
    return motor.getPositionRotations() / kConversionFactor;
  }

  /** Disables the subsystem. */
  public Command disable() {
    return runOnce(
            () -> {
              disabled = true;
            })
        .withName("TurretDisable");
  }

  /** Enables the subsystem. */
  public Command enable() {
    return runOnce(
            () -> {
              disabled = false;
            })
        .withName("TurretEnable");
  }
}

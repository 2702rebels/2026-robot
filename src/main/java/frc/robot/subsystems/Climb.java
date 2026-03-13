package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.signals.AdvancedHallSupportValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.CANduit;
import frc.robot.lib.LoggedTalonFXS;
import frc.robot.lib.Tunable;

public final class Climb extends SubsystemBase {
  // MAKE SURE TO UPDATE THESE VALUES AFTER TUNING
  // DO NOT USE THESE VALUES DIRECTLY IN PERIODIC CODE,
  // USE THEM TO INITIALIZE TUNABLE VALUES AND CONTROLLERS
  private static final double __kP = 5;
  private static final double __kD = 0;

  private static final double __kPd = 3;
  private static final double __kDd = 0;

  private static final double __kPositionExtended = 8.0; // inches
  private static final double __kPositionEngaged = 2.0;
  private static final double __kPositionRetracted = 0;
  private static final double __kPositionManual = 0;

  // 2.73 is effective spool diameter in inches
  private static final double kConversionFactor = 36.0 / (Math.PI * 2.73);

  // motion profile for lowering the robot
  private static final double __kMaxVelocity = kConversionFactor * 6.0; // rot/sec (6 in/sec)
  private static final double __kMaxAcceleration =
      __kMaxVelocity / 0.2; // rot/sec^2 (200ms to max velocity)

  @Logged(name = "Motor", importance = Importance.INFO)
  private final LoggedTalonFXS motor;

  @Logged(name = "Setpoint", importance = Importance.INFO)
  private Distance setpoint = Inches.zero();

  private final PositionVoltage control;
  private final MotionMagicVoltage controlLower;

  private final CANduit.PWMInput sensor;

  @Logged(name = "Enabled", importance = Importance.INFO)
  private boolean enabled = true;

  // indicates that the subsystem has been initialized by zeroing the position of the climb
  private boolean initialized = false;

  // indicates that the robot has attempted to climb/climbed and should be lowered in a controlled
  // manner
  private boolean engaged = false;

  private static final double kStatorCurrentLimit = 100.0;
  private static final double kTolerance = 0.5; // inches

  // manual mode used for testing
  private boolean manual;

  public Climb(CANBus canbus, CANduit canduit) {
    motor =
        new LoggedTalonFXS(
            Constants.CanivoreID.MOTOR_CLIMB,
            canbus,
            (config) -> {
              config.Commutation.AdvancedHallSupport = AdvancedHallSupportValue.Enabled;
              config.Commutation.MotorArrangement = MotorArrangementValue.NEO_JST;
              config.CurrentLimits.StatorCurrentLimit = kStatorCurrentLimit;
              config.CurrentLimits.StatorCurrentLimitEnable = true;
              config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
              config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
              config.MotorOutput.ControlTimesyncFreqHz = 100;

              config.Slot0.kP = __kP;
              config.Slot0.kD = __kD;

              config.Slot1.kP = __kPd;
              config.Slot1.kD = __kDd;

              config.MotionMagic.MotionMagicCruiseVelocity = __kMaxVelocity;
              config.MotionMagic.MotionMagicAcceleration = __kMaxAcceleration;
            });

    control =
        new PositionVoltage(0)
            .withSlot(0)
            .withUseTimesync(true)
            .withUpdateFreqHz(0)
            .withEnableFOC(true);

    controlLower =
        new MotionMagicVoltage(0)
            .withSlot(1)
            .withUseTimesync(true)
            .withUpdateFreqHz(0)
            .withEnableFOC(true);

    // make configuration tunable (must be done after configuration is bound to the motor)
    Tunable.bind("Climb", motor);
    Tunable.bind1("Climb/Lower", motor);
    Tunable.bindMotionProfile("Climb/Lower", motor);

    sensor = canduit.createPWMInput(Constants.CANduitID.CLIMB_SENSOR);

    // manual = true; // FOR TESTING ONLY
  }

  /**
   * Converts raw PWM input into distance in meters for ToF sensor.
   * https://www.pololu.com/product/4064
   */
  private static Distance getSensorDistance(CANduit.PWMInput encoder) {
    // the sensor was found to be consistently off by 10mm
    final double fudgeFactor = -0.1;
    final double v = encoder.getPulseWidth();

    return Meters.of((v - 1000000.0) * 0.03 / 4000.0 + fudgeFactor);
  }

  @Override
  public void periodic() {
    // run initialization sequence regardless of subsystem disabled state
    if (!initialized) {
      motor.set(-0.05);

      // climb is zeroed against a hard stop
      //
      // note that since we typically require very high stator current for actual climb,
      // we use much lower threhsold here to detect stall during initialization sequence
      if (motor.isStalled(10)) {
        motor.stopMotor();
        motor.resetPosition(0);
        initialized = true;
      }
    } else {
      if (enabled) {
        if (manual) {
          setpoint = Inches.of(kPositionManual.get());
        }

        // use profiled motion if the climb is engaged to lower the robot
        // otherwise, use simple PID to jump to the setpoint
        final var position = setpoint.in(Inches) * kConversionFactor;
        motor.setControl(
            engaged ? controlLower.withPosition(position) : control.withPosition(position));
      } else {
        motor.stopMotor();
      }
    }

    // reset "engaged" flag once the hook is fully extended
    if (engaged && getPosition().in(Inches) >= kPositionExtended.get() - kTolerance) {
      engaged = false;
    }
  }

  private static final Distance kDistanceNotAvailable = Meters.of(-1);

  @Logged(name = "SensorDistance", importance = Importance.INFO)
  public Distance getSensorDistance() {
    return sensor.hasValue() ? getSensorDistance(sensor) : kDistanceNotAvailable;
  }

  /** Determines whether the subsystem is at its commanded setpoint. */
  @Logged(name = "AtSetpoint", importance = Importance.INFO)
  public boolean isAtSetpoint() {
    return MathUtil.isNear(setpoint.in(Inches), getPosition().in(Inches), kTolerance);
  }

  @Logged(name = "Position", importance = Importance.INFO)
  public Distance getPosition() {
    return Inches.of(motor.getMotorPosition().in(Rotations) / kConversionFactor);
  }

  @Logged(name = "Initialized", importance = Importance.INFO)
  public boolean isInitialized() {
    return initialized;
  }

  @Logged(name = "Ready", importance = Importance.INFO)
  public boolean isReady() {
    return MathUtil.isNear(kPositionExtended.get(), getPosition().in(Inches), kTolerance);
  }

  @Logged(name = "Engaged", importance = Importance.INFO)
  public boolean isEngaged() {
    return engaged;
  }

  /** Disables the subsystem. */
  public Command disable() {
    return runOnce(
            () -> {
              enabled = false;
            })
        .withName("ClimbDisable");
  }

  /** Enables the subsystem. */
  public Command enable() {
    return runOnce(
            () -> {
              enabled = true;
            })
        .withName("ClimbEnable");
  }

  /** Extends the hook. */
  public Command extend() {
    return runOnce(
            () -> {
              this.setpoint = Inches.of(kPositionExtended.get());
            })
        .withName("ClimbExtend");
  }

  /** Retracts the hook. */
  public Command retract() {
    return runOnce(
            () -> {
              this.setpoint = Inches.of(kPositionRetracted.get());
            })
        .withName("ClimbRetract");
  }

  /** Engages the climb by rapdidly retracting the hook to a pre-defined position. */
  public Command engage() {
    return runOnce(
            () -> {
              this.engaged = true;
              this.setpoint = Inches.of(kPositionEngaged.get());
            })
        .withName("ClimbEngage");
  }

  /** Disengages the climb by lowering the robot to the ground in a controllable manner. */
  public Command disengage() {
    return runOnce(
            () -> {
              this.setpoint = Inches.of(kPositionExtended.get());
            })
        .withName("ClimbDisengage")
        // a "hack" that resets the "engaged" flag after a timeout in case we never fully
        // reached the setpoint somehow, note that we do not require this subsystem in
        // the command being scheduled
        //
        // this way we can quickly extend/retract the hook manually, however it opens up
        // a possibility of a race, if the climb is re-engaged in the meantime,
        // yet this specific scenario is extremely rare in practice
        .andThen(
            new ScheduleCommand(
                Commands.sequence(
                    Commands.waitSeconds(2),
                    Commands.runOnce(
                        () -> {
                          this.engaged = false;
                        }))));
  }

  // spotless:off
  private static Tunable.DoubleValue kPositionExtended = Tunable.value("Climb/Extended", __kPositionExtended);
  private static Tunable.DoubleValue kPositionRetracted = Tunable.value("Climb/Retracted", __kPositionRetracted);
  private static Tunable.DoubleValue kPositionEngaged = Tunable.value("Climb/Engaged", __kPositionEngaged);
  private static Tunable.DoubleValue kPositionManual = Tunable.value("Climb/Manual", __kPositionManual);
  // spotless:on
}

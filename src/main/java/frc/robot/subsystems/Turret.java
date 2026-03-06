package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.util.CircularBuffer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.CANduit;
import frc.robot.lib.EasyCRT;
import frc.robot.lib.EasyCRT.CRTStatus;
import frc.robot.lib.EasyCRTConfig;
import frc.robot.lib.LoggedTalonFX;
import frc.robot.lib.PhoenixUtil;
import frc.robot.lib.Tunable;

public final class Turret extends SubsystemBase {
  // MAKE SURE TO UPDATE THESE VALUES AFTER TUNING
  // DO NOT USE THESE VALUES DIRECTLY IN PERIODIC CODE,
  // USE THEM TO INITIALIZE TUNABLE VALUES AND CONTROLLERS
  private static final double __kS = 0;
  private static final double __kP = 0;
  private static final double __kD = 0;
  private static final double __kMaxVelocity = 0; // rps
  private static final double __kMaxAcceleration = 0; // rps/s
  private static final double __kJerk = 0;

  // Expo kV and kA characterize the profile not gains
  // https://v6.docs.ctr-electronics.com/en/latest/docs/api-reference/device-specific/talonfx/motion-magic.html#motion-magic-expo
  //
  // max velocity (rps) = min(supply voltage / kV, cruise velocity)
  // max acceleration (rps/s) = supply voltage / kA
  //
  // For example,
  // kV = 0.03 at 12V -> max velocity = 400 rps
  // kA = 0.04 at 12V -> max acceleration = 300 rps/s
  //
  // SMALLER VALUES REPRESENT HIGHER MAX VELOCITY / ACCELERATION
  private static final double __kVExpo = 0.3; // 40 rps at 12V, 4 rps at 10:1 ratio
  private static final double __kAExpo =
      0.06; // 200 rps at 12V, 20 rps at 10:1 ratio, 200ms to max velocity

  private static final double __kSetpointManual = 0;

  @Logged(name = "Motor", importance = Importance.INFO)
  private final LoggedTalonFX motor;

  @Logged(name = "Setpoint", importance = Importance.INFO)
  private Angle setpoint = Degrees.zero();

  @Logged(name = "CrtAngle", importance = Importance.DEBUG)
  private Angle crtAngle = Degrees.zero();

  @Logged(name = "CrtStatus", importance = Importance.DEBUG)
  private CRTStatus crtStatus = CRTStatus.NOT_ATTEMPTED;

  @Logged(name = "Offset/Mean", importance = Importance.DEBUG)
  private Angle offsetMean = Degrees.zero();

  @Logged(name = "Offset/Stdev", importance = Importance.DEBUG)
  private Angle offsetStdev = Degrees.zero();

  private static final int kOffsetEstimatorSize = 10;
  private final CircularBuffer<Double> offsetEstimator = new CircularBuffer<>(kOffsetEstimatorSize);

  private final LinearFilter encoder1Filter = LinearFilter.movingAverage(5);
  private final LinearFilter encoder2Filter = LinearFilter.movingAverage(5);

  private final MotionMagicExpoTorqueCurrentFOC control;

  private final CANduit.PWMInput encoder1;
  private final CANduit.PWMInput encoder2;
  private final EasyCRT crt;

  private boolean disabled;

  // indicates that the subsystem has been initialized by determining the position of the turret
  private boolean initialized;

  // manual mode used for testing
  private boolean manual;

  private static final double kTurretTeeth = 218.0;
  private static final double kEncoder1Ratio = kTurretTeeth / 17.0;
  private static final double kEncoder2Ratio = kTurretTeeth / 19.0;

  private static final Angle kEncoder1Offset = Degrees.of(-325.29);
  private static final Angle kEncoder2Offset = Degrees.of(-201.14);
  private static final Angle kMinRange = Degrees.of(-260.0);
  private static final Angle kMaxRange = Degrees.of(260.0);

  private static final double kStatorCurrentLimit = 40.0;
  private static final double kConversionFactor = kTurretTeeth / 22.0;
  private static final double kMinPosition = -150.0; // degrees
  private static final double kMaxPosition = 150.0; // degrees
  private static final double kTolerance = 1.0; // degrees

  public Turret(CANBus canbus, CANduit canduit) {
    motor =
        new LoggedTalonFX(
            Constants.CanivoreID.MOTOR_TURRET,
            canbus,
            (config) -> {
              config.CurrentLimits.StatorCurrentLimit = kStatorCurrentLimit;
              config.CurrentLimits.StatorCurrentLimitEnable = true;
              config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
              config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

              config.Slot0.kS = __kS;
              config.Slot0.kP = __kP;
              config.Slot0.kD = __kD;
              config.MotionMagic.MotionMagicCruiseVelocity = __kMaxVelocity;
              config.MotionMagic.MotionMagicAcceleration = __kMaxAcceleration;
              config.MotionMagic.MotionMagicJerk = __kJerk;
              config.MotionMagic.MotionMagicExpo_kV = __kVExpo;
              config.MotionMagic.MotionMagicExpo_kA = __kAExpo;
            });

    control = new MotionMagicExpoTorqueCurrentFOC(0).withSlot(0);

    // make configuration tunable (must be done after configuration is bound to the motor)
    Tunable.bind("Turret", motor);
    Tunable.bindMotionProfile("Turret/Motion", motor);

    encoder1 = canduit.createPWMInput(Constants.CANduitID.TURRET_ENCODER1);
    encoder2 = canduit.createPWMInput(Constants.CANduitID.TURRET_ENCODER2);
    crt =
        new EasyCRT(
            new EasyCRTConfig(this::getEncoder1Angle, this::getEncoder2Angle)
                .withEncoderRatios(kEncoder1Ratio, kEncoder2Ratio)
                .withAbsoluteEncoderOffsets(kEncoder1Offset, kEncoder2Offset)
                .withMechanismRange(kMinRange, kMaxRange)
                .withMatchTolerance(Degrees.of(kTolerance)));

    initialized = false;
    disabled = true;

    manual = true; // FOR TESTING ONLY
  }

  /**
   * Converts raw PWM input into encoder position in rotations for REV through-borer encoder V1.
   * https://www.revrobotics.com/rev-11-1271/
   */
  private static double getEncoderRotations(CANduit.PWMInput encoder) {
    // 0°: 1μs, 360°: 1024μs
    //
    // REV encoders were found to be not agreeing to the specification,
    // of those tested reported periods were 1010μs and 1034μs;
    // we have also observed that pulse width does not go above 1025μs as per spec,
    // when the reported period is higher than 1025μs;
    //
    // below formula seems to handle both situations reasonably well

    final double v = encoder.getPulseWidth();
    final double p = Math.min(encoder.getPeriod(), 1025000); // nanoseconds
    return MathUtil.inputModulus(v / p, 0, 1);
  }

  /** Logic to estimate turret angle using a history of CRT readings. */
  private void estimateTurretAngle(Angle estimatedAngle) {
    // record CRT angle for telemetry/debugging
    crtAngle = estimatedAngle;

    final var position = getPosition();
    final var offsetInDegrees =
        MathUtil.inputModulus(crtAngle.in(Degrees) - position.in(Degrees), -180, 180);

    offsetEstimator.addLast(offsetInDegrees);

    if (offsetEstimator.size() >= kOffsetEstimatorSize) {
      double mean = 0;
      for (int i = 0; i < kOffsetEstimatorSize; ++i) {
        final var v = offsetEstimator.get(i);
        mean += v;
      }
      mean /= kOffsetEstimatorSize;

      double stdev = 0;
      for (int i = 0; i < kOffsetEstimatorSize; ++i) {
        final var v = offsetEstimator.get(i);
        stdev += Math.pow(v - mean, 2);
      }
      stdev = Math.sqrt(stdev / kOffsetEstimatorSize);

      // record offset values for telemetry/debugging
      offsetMean = Degrees.of(mean);
      offsetStdev = Degrees.of(stdev);

      // reset turret motor position to the derived angle when the offset stabilizes
      if (stdev < 2 * kTolerance && Math.abs(mean) > kTolerance) {
        PhoenixUtil.run(
            "Turret Position Init",
            motor.getDeviceID(),
            () -> motor.setPosition(crtAngle.in(Rotations) * kConversionFactor));

        initialized = true;
      }
    }
  }

  @Logged(name = "Encoder1", importance = Importance.INFO)
  public Angle getEncoder1Angle() {
    return Rotations.of(encoder1Filter.calculate(getEncoderRotations(encoder1)));
  }

  @Logged(name = "Encoder2", importance = Importance.INFO)
  public Angle getEncoder2Angle() {
    return Rotations.of(encoder2Filter.calculate(getEncoderRotations(encoder2)));
  }

  @Override
  public void periodic() {
    if (!initialized) {
      // ensure both encoders are reporting valid readings
      if (encoder1.hasValue() && encoder2.hasValue()) {
        crtStatus = crt.getLastStatus();
        crt.getAngleOptional().ifPresent(this::estimateTurretAngle);
      }
    }

    if (disabled) {
      motor.stopMotor();
    } else if (initialized) {
      if (manual) {
        setpoint = Degrees.of(kSetpointManual.get());
      }

      motor.setControl(control.withPosition(setpoint.in(Rotations) * kConversionFactor));
    }
  }

  @Logged(name = "Initialized", importance = Importance.INFO)
  public boolean isInitialized() {
    return initialized;
  }

  /** Determines whether the subsystem is at its commanded setpoint. */
  @Logged(name = "AtSetpoint", importance = Importance.INFO)
  public boolean isAtSetpoint() {
    return MathUtil.isNear(setpoint.in(Degrees), getPosition().in(Degrees), kTolerance);
  }

  @Logged(name = "Position", importance = Importance.INFO)
  public Angle getPosition() {
    return motor.getPositionMeasured().div(kConversionFactor);
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

  /** Sets turret position. */
  public Command setPosition(double position) {
    return runOnce(
            () -> {
              setpoint = Degrees.of(MathUtil.clamp(position, kMinPosition, kMaxPosition));
            })
        .withName("SetPosition");
  }

  // spotless:off
  private static Tunable.DoubleValue kSetpointManual = Tunable.value("Turret/Manual", __kSetpointManual);
  // spotless:on
}

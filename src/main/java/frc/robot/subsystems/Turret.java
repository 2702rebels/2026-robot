package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.CANduit;
import frc.robot.lib.EasyCRT;
import frc.robot.lib.EasyCRT.CRTStatus;
import frc.robot.lib.EasyCRTConfig;
import frc.robot.lib.LoggedTalonFX;
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
  private static final double __kVExpo = 10;
  private static final double __kAExpo = 10;

  @Logged(name = "Motor", importance = Importance.INFO)
  private final LoggedTalonFX motor;

  @Logged(name = "Setpoint", importance = Importance.INFO)
  private Angle setpoint;

  @Logged(name = "CrtAngle", importance = Importance.INFO)
  private Angle crtAngle;

  private Angle turretAngle;

  @Logged(name = "CrtStatus", importance = Importance.INFO)
  private CRTStatus crtStatus = CRTStatus.NOT_ATTEMPTED;

  private final MotionMagicExpoTorqueCurrentFOC control;

  private final CANduit.PWMInput encoder1;
  private final CANduit.PWMInput encoder2;
  private final EasyCRT crt;

  private boolean disabled;

  // indicates that the subsystem has been initialized by determining the position of the turret
  private boolean initialized;

  private static final double kTurretTeeth = 218.0;
  private static final double kEncoder1Ratio = kTurretTeeth / 17.0;
  private static final double kEncoder2Ratio = kTurretTeeth / 19.0;
  private static final Angle kEncoder1Offset = Degrees.of(291.39666601294374);
  private static final Angle kEncoder2Offset = Degrees.of(186.03676383105295);

  private static final double kStatorCurrentLimit = 30.0;
  private static final double kConversionFactor = kTurretTeeth / 22.0;
  private static final double kMinPosition = -200.0; // degrees
  private static final double kMaxPosition = 200.0; // degrees
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
                .withMechanismRange(Degrees.of(kMinPosition), Degrees.of(kMaxPosition))
                .withMatchTolerance(Degrees.of(kTolerance)));

    initialized = false;
    disabled = true;
    setpoint = Degrees.zero();
    crtAngle = Degrees.zero();
  }

  /**
   * Converts raw PWM input into encoder position in degrees for REV through-borer encoder V1.
   * https://www.revrobotics.com/rev-11-1271/
   */
  private static Angle getEncoderAngle(CANduit.PWMInput encoder) {
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
    SmartDashboard.putNumber(encoder.toString() + " Period", p);
    return Radians.of(MathUtil.inputModulus(2 * Math.PI * v / p, 0, 2 * Math.PI));
  }

  private static Angle mod(Angle x) {
    return Rotations.of(MathUtil.inputModulus(x.in(Rotations), 0, 1));
  }

  private static Angle getTurretRotation(Angle encoder1, Angle encoder2) {
    encoder1 = mod(encoder1);
    encoder2 = mod(encoder2);
    // When x is the turret rotation, encoder1 is 218x/17 and encoder2 is 218x/19
    // Difference between encoders = 218x/17 - 218x/19 = 436x/323
    // Therefore, turret rotation can be derived from encoder difference as x = 323 * (encoder1 -
    // encoder2) / 436
    SmartDashboard.putNumber("Encoder Difference", mod(encoder1.minus(encoder2)).in(Degrees));
    Angle baseRotation = mod(encoder1.minus(encoder2)).times(323.0 / 436.0);
    // Predict encoder1 reading based off of derived turret rotation
    Angle encoder1Prediction = mod(baseRotation.times(218.0 / 17.0));
    // Encoder difference wraps every 323/436 rotations of the turret
    // Prediction will match if encoder difference has not yet wrapped;
    // If prediction is close enough to actual encoder1 reading, we are likely in the first wrap;
    // otherwise, we are likely in the second wrap and need to add one wrap to the base rotation
    if (Math.abs(encoder1Prediction.minus(encoder1).abs(Rotations) - 0.5) < 0.15) {
      return baseRotation.plus(Rotations.of(323.0 / 436.0));
    } else if (encoder1Prediction.minus(encoder1).abs(Rotations) < 0.15) {
      return baseRotation;
    } else {
      return Rotations.zero();
    }
  }

  @Logged(name = "Encoder1", importance = Importance.INFO)
  public Angle getEncoder1Angle() {
    return getEncoderAngle(encoder1);
  }

  @Logged(name = "Encoder2", importance = Importance.INFO)
  public Angle getEncoder2Angle() {
    return Rotation.one().minus(getEncoderAngle(encoder2));
  }

  @Override
  public void periodic() {
    if (!initialized) {
      // ensure both encoders are reporting valid readings
      if (encoder1.hasValue() && encoder2.hasValue()) {
        // final var angle = crt.getAngleOptional();
        // crtStatus = crt.getLastStatus();
        // if (angle.isPresent()) {
        //   crtAngle = angle.get();

        //   // reset turret motor position to the derived angle
        //   // PhoenixUtil.run(
        //   //     "Turret Position Init",
        //   //     motor.getDeviceID(),
        //   //     () -> motor.setPosition(crtAngle.in(Rotations) * kConversionFactor));
        // }

        turretAngle =
            getTurretRotation(
                getEncoder1Angle().minus(kEncoder1Offset).plus(Rotations.of(0.5)),
                getEncoder2Angle().minus(kEncoder2Offset).plus(Rotations.of(0.5)));

        SmartDashboard.putNumber("Encoder1 Raw Angle", getEncoder1Angle().in(Degrees));
        SmartDashboard.putNumber("Encoder2 Raw Angle", getEncoder2Angle().in(Degrees));
        SmartDashboard.putNumber(
            "Encoder1 Angle",
            getEncoder1Angle().minus(kEncoder1Offset).plus(Rotations.of(0.5)).in(Degrees));
        SmartDashboard.putNumber(
            "Encoder2 Angle",
            getEncoder2Angle().minus(kEncoder2Offset).plus(Rotations.of(0.5)).in(Degrees));
        SmartDashboard.putNumber("Turret Angle", turretAngle.in(Degrees));
      }
    }

    if (disabled) {
      motor.stopMotor();
    } else if (initialized) {
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
}

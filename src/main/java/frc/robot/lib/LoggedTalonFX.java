package frc.robot.lib;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import java.util.function.Consumer;

public class LoggedTalonFX extends TalonFX {
  private final StatusSignal<Temperature> temperatureSignal;
  private final StatusSignal<Voltage> motorVoltageSignal;
  private final StatusSignal<Current> supplyCurrentSignal;
  private final StatusSignal<Current> statorCurrentSignal;
  private final StatusSignal<Angle> positionSignal;
  private final StatusSignal<AngularVelocity> velocitySignal;
  private final StatusSignal<Double> closedLoopReferenceSignal;
  private final StatusSignal<Double> closedLoopReferenceSlopeSignal;

  public LoggedTalonFX(int deviceId, CANBus canbus) {
    this(deviceId, canbus, null);
  }

  public LoggedTalonFX(int deviceId, CANBus canbus, Consumer<TalonFXConfiguration> configurator) {
    super(deviceId, canbus);

    final var config = new TalonFXConfiguration();
    configure(config);

    // apply external configuration if specified
    if (configurator != null) {
      configurator.accept(config);
    }

    PhoenixUtil.config(this, config);

    temperatureSignal = getDeviceTemp();
    motorVoltageSignal = getMotorVoltage();
    supplyCurrentSignal = getSupplyCurrent();
    statorCurrentSignal = getStatorCurrent();
    positionSignal = getPosition();
    velocitySignal = getVelocity();
    closedLoopReferenceSignal = getClosedLoopReference();
    closedLoopReferenceSlopeSignal = getClosedLoopReferenceSlope();

    PhoenixUtil.registerSignals(
        this,
        temperatureSignal,
        motorVoltageSignal,
        supplyCurrentSignal,
        statorCurrentSignal,
        positionSignal,
        velocitySignal,
        closedLoopReferenceSignal,
        closedLoopReferenceSlopeSignal);
  }

  protected void configure(TalonFXConfiguration config) {}

  @Logged(name = "Temperature", importance = Importance.INFO)
  public double getMotorTemperature() {
    return temperatureSignal.getValue().in(Celsius);
  }

  @Logged(name = "AppliedVolts", importance = Importance.DEBUG)
  public double getAppliedVolts() {
    return motorVoltageSignal.getValue().in(Volts);
  }

  @Logged(name = "StatorCurrentAmps", importance = Importance.INFO)
  public double getStatorCurrentAmps() {
    return statorCurrentSignal.getValue().in(Amps);
  }

  @Logged(name = "SupplyCurrentAmps", importance = Importance.INFO)
  public double getSupplyCurrentAmps() {
    return supplyCurrentSignal.getValue().in(Amps);
  }

  @Logged(name = "MotorPosition", importance = Importance.DEBUG)
  public Angle getMotorPosition() {
    return positionSignal.getValue();
  }

  @Logged(name = "MotorVelocity", importance = Importance.DEBUG)
  public AngularVelocity getMotorVelocity() {
    return velocitySignal.getValue();
  }

  @Logged(name = "MotorClosedLoopReference", importance = Importance.DEBUG)
  public double getMotorClosedLoopReference() {
    return closedLoopReferenceSignal.getValue();
  }

  @Logged(name = "MotorClosedLoopReferenceSlope", importance = Importance.DEBUG)
  public double getMotorClosedLoopReferenceSlope() {
    return closedLoopReferenceSlopeSignal.getValue();
  }
}

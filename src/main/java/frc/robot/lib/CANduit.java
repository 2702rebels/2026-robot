package frc.robot.lib;

import edu.wpi.first.hal.CANData;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.HashMap;
import java.util.Map;

/**
 * The main class for the CANduit library. This class manages the CAN communication and GPIO pin
 * allocation for the CANduit device.
 */
public class CANduit {
  private Map<Integer, Channel> channels = new HashMap<>();
  private final CAN can;
  private final boolean connected;

  private static final int CLASS_SET_MODE = 1;
  private static final int CLASS_DIO = 2;
  private static final int CLASS_BCAST_STATUS = 20;
  private static final int CLASS_BCAST_PWM_VALUE = 21;

  private static final int MODE_NONE = 0;
  private static final int MODE_DIO_IN = 1;
  private static final int MODE_DIO_OUT = 2;
  private static final int MODE_PWM_IN = 3;

  private abstract class Channel implements AutoCloseable {
    protected final CANduit canduit;
    protected final int gpio;
    protected long timestamp;
    protected final CANData data = new CANData();

    protected Channel(CANduit canduit, int gpio, int mode) {
      this.canduit = canduit;
      this.gpio = gpio;
      canduit.acquireGPIO(gpio, this);
      canduit.configureGPIO(gpio, mode);
    }

    /** Updates the input. Must be invoked in the robot periodic. */
    protected abstract void update();

    /** Returns timestamp of the most recent value. */
    public long getTimestamp() {
      return timestamp;
    }

    /** Releases the channel. */
    public void close() {
      canduit.releaseGPIO(gpio);
    }

    public String toString() {
      return "GPIO " + gpio;
    }
  }

  private abstract class DigitalIO extends Channel {
    protected static final byte[] k0 = new byte[] {0b00000000};
    protected static final byte[] k1 = new byte[] {0b00000001};

    protected boolean value;

    private DigitalIO(CANduit canduit, int gpio, int mode) {
      super(canduit, gpio, mode);
    }

    @Override
    protected void update() {
      // [!] CLASS_BCAST_STATUS returns the status for ALL pins and always uses API index 0
      if (canduit.readDataLatest(0, CLASS_BCAST_STATUS, data) && data.length > 0) {
        // byte 0: GPIO state - bits 0 through 7 correspond to GPIO 0 through 7
        value = (data.data[0] & (0b00000001 << gpio)) != 0;
        timestamp = data.timestamp;
      }

      SmartDashboard.putNumber("PhilWasHere", 555);
      // Hack to test the reset reason counts
      if (canduit.readDataLatest(1, CLASS_BCAST_STATUS, data) && data.length == 8) {
        SmartDashboard.putNumber("PowerOn Count", data.data[0]);
        SmartDashboard.putNumber("Glitch Count", data.data[1]);
        SmartDashboard.putNumber("Other Count", data.data[2]);
        SmartDashboard.putNumber("Disable Count", data.data[3]);
      }

      // Hack to test the gpio mode broadcast
      if (canduit.readDataLatest(2, CLASS_BCAST_STATUS, data) && data.length == 8) {
        SmartDashboard.putNumber("GPIO 0 mode", data.data[0]);
        SmartDashboard.putNumber("GPIO 1 mode", data.data[1]);
        SmartDashboard.putNumber("GPIO 2 mode", data.data[2]);
        SmartDashboard.putNumber("GPIO 3 mode", data.data[3]);
      }
    }

    /** Returns most recent value received. */
    public boolean get() {
      return value;
    }
  }

  public class DigitalInput extends DigitalIO {
    private DigitalInput(CANduit canduit, int gpio) {
      super(canduit, gpio, MODE_DIO_IN);
    }
  }

  public class DigitalOutput extends DigitalIO {
    private DigitalOutput(CANduit canduit, int gpio) {
      super(canduit, gpio, MODE_DIO_OUT);
    }

    /** Sets the output. */
    public void set(boolean value) {
      canduit.writeData(gpio, CLASS_DIO, value ? k1 : k0);
    }
  }

  public class PWMInput extends Channel {
    protected int period = -1;
    protected int width = -1;
    protected int gen_count = -1;

    private PWMInput(CANduit canduit, int gpio) {
      super(canduit, gpio, MODE_PWM_IN);
    }

    @Override
    protected void update() {
      if (canduit.readDataLatest(gpio, CLASS_BCAST_PWM_VALUE, data) && data.length == 8) {
        final var bb = ByteBuffer.wrap(data.data).order(ByteOrder.LITTLE_ENDIAN);
        width = bb.getInt();
        period = bb.getInt();
        // In new fw, width & period are 28bit values
        int lower = (width & 0xF0000000) >> 28;
        int upper = (period & 0xF0000000) >> 28;
        gen_count = lower | (upper << 4);
        width &= 0xFFFFFFF;
        period &= 0xFFFFFFF;
        timestamp = data.timestamp;
      }

      SmartDashboard.putNumber("PhilWasHere", 1234);
      // Hack to test the reset reason counts
      if (canduit.readDataLatest(1, CLASS_BCAST_STATUS, data) && data.length == 8) {
        SmartDashboard.putNumber("PowerOn Count", data.data[0]);
        SmartDashboard.putNumber("Glitch Count", data.data[1]);
        SmartDashboard.putNumber("Other Count", data.data[2]);
        SmartDashboard.putNumber("Disable Count", data.data[3]);
      }

      // Hack to test the gpio mode broadcast
      if (canduit.readDataLatest(2, CLASS_BCAST_STATUS, data) && data.length == 8) {
        SmartDashboard.putNumber("GPIO 0 mode", data.data[0]);
        SmartDashboard.putNumber("GPIO 1 mode", data.data[1]);
        SmartDashboard.putNumber("GPIO 2 mode", data.data[2]);
        SmartDashboard.putNumber("GPIO 3 mode", data.data[3]);
      }
    }

    /**
     * Returns most recent period received (in nanoseconds).
     *
     * <p>Returns -1 if no valid value is present.
     */
    public int getPeriod() {
      return period;
    }

    /**
     * Returns most recent pulse width received (in nanoseconds). Returns -1 if no valid value is
     * present.
     */
    public int getPulseWidth() {
      return width;
    }

    /** Returns most recent generation count */
    public int getGenCount() {
      return gen_count;
    }

    /** Determines whether valid reading is present. */
    public boolean hasValue() {
      return period > 0 && width >= 0;
    }
  }

  /**
   * Initializes a CANduit device.
   *
   * @param canID The CAN ID of the CANduit device. Negative identifier can be used to initialized
   *     the device in disconnected mode.
   */
  public CANduit(int canID) {
    if (canID >= 0) {
      this.can = new CAN(canID);
      this.connected = true;
    } else {
      this.can = null;
      this.connected = false;
    }
  }

  /** Acquires the specified GPIO and exposes it as a digital input. */
  public DigitalInput createDigitalInput(int gpio) {
    return new DigitalInput(this, gpio);
  }

  /** Acquires the specified GPIO and exposes it as a digital output. */
  public DigitalOutput createDigitalOutput(int gpio) {
    return new DigitalOutput(this, gpio);
  }

  /** Acquires the specified GPIO and exposes it as a generic PWM input. */
  public PWMInput createPWMInput(int gpio) {
    return new PWMInput(this, gpio);
  }

  /** Updates all registered input channels. Must be invoked in the robot periodic. */
  public void update() {
    for (var channel : channels.values()) {
      channel.update();
    }
  }

  /** Acquires GPIO. */
  private void acquireGPIO(int gpio, Channel channel) {
    if (gpio < 0 || gpio > 7) {
      throw new IllegalArgumentException("GPIO pin must be between 0 and 7.");
    }

    if (channels.containsKey(gpio)) {
      throw new IllegalArgumentException("GPIO pin " + gpio + " in use.");
    }

    channels.put(gpio, channel);
  }

  /** Releases GPIO. */
  private void releaseGPIO(int gpio) {
    channels.remove(gpio);
    configureGPIO(gpio, MODE_NONE);
  }

  /** Configures GPIO mode. */
  private void configureGPIO(int gpio, int mode) {
    if (!connected) return;
    int apiIndex = gpio;
    int apiId = CLASS_SET_MODE << 4 | apiIndex;
    byte[] data = {(byte) mode};
    can.writePacket(data, apiId);
  }

  /** Reads low-level data from the CAN. Returns latest packet. */
  // THis assumes gpio as the API index which isn't always the case.  Maybe rename gpio to index?
  private boolean readDataLatest(int gpio, int apiClass, CANData data) {
    if (!connected) return false;
    return can.readPacketLatest(apiClass << 4 | gpio, data);
  }

  /** Writes low-level data to the CAN. */
  private void writeData(int gpio, int apiClass, byte[] data) {
    if (!connected) return;
    can.writePacket(data, apiClass << 4 | gpio);
  }
}

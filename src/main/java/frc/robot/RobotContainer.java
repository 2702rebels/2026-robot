// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.RobotState.Intent;
import frc.robot.generated.TunerConstants;
import frc.robot.lib.CANduit;
import frc.robot.lib.Tunable;
import frc.robot.subsystems.AprilTagDetector;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LedFeedback;
import frc.robot.subsystems.LedResource;
import frc.robot.subsystems.LedVision;

public class RobotContainer {
  private final RobotState state = RobotState.getInstance();
  private final CommandXboxController controllerDriver = new CommandXboxController(0);
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  private final LedResource led = new LedResource(Constants.PwmID.LED);
  private final LedFeedback ledFeedback = new LedFeedback(led, this::getDefaultLedFeedback);
  private final LedVision ledVision = new LedVision(led, this::getDefaultVisionFeedback);

  @Logged(name = "AprilTagDetector", importance = Importance.INFO)
  private final AprilTagDetector aprilTagDetector = new AprilTagDetector();

  private static final Trigger disabled = new Trigger(() -> false);
  private static final Trigger isAutonomousMode = new Trigger(DriverStation::isAutonomousEnabled);

  // slows down max rotation speed for more precise movements
  private final Trigger preciseRotationActive = controllerDriver.leftStick();

  // button mappings
  private final Trigger resetOrientationButton = controllerDriver.start();
  private final Trigger brakeButton = controllerDriver.b();
  private final Trigger resetIntentButton = controllerDriver.x();

  public RobotContainer(CANduit canduit) {
    state.bind(drivetrain);
    state.setVisionStateSupplier(aprilTagDetector::getState);

    // [!] order matters here: we want to poll driver input first, since in most cases
    // we set intent via instant command that updates the robot state in place;
    // thus allowing triggers bound to the intent-based conditions to fire in the same
    // update cycle (this only works if the logic runs within command init method)
    configureDriverBindings();
    configureTriggers();
  }

  /** Smooth out control input by squaring. */
  private double smoothInput(double value) {
    final var v = MathUtil.clamp(value, -1, 1);
    return Math.signum(value) * v * v;
  }

  /** Use this method to configure driver controller bindings. */
  private void configureDriverBindings() {
    if (!Constants.Settings.drivingDisabled) {
      // note that X is defined as forward according to WPILib convention,
      // and Y is defined as to the left according to WPILib convention.
      drivetrain.setDefaultCommand(
          // drivetrain will execute this command periodically
          drivetrain.driveTeleop(
              // drive forward with negative Y (forward)
              () -> -smoothInput(controllerDriver.getLeftY()),
              // drive left with negative X (left)
              () -> -smoothInput(controllerDriver.getLeftX()),
              // drive counterclockwise with negative X (left)
              () -> -smoothInput(controllerDriver.getRightX()) * getOmegaModifier()));
    }

    resetOrientationButton.onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
    brakeButton.whileTrue(drivetrain.brake());

    // run SysId routines when holding back/start and X/Y
    // note that each routine should be run exactly once in a single log
    if (Constants.Settings.sysIdBindings) {
      controllerDriver.povRight().whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
      controllerDriver.povLeft().whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
      controllerDriver.povUp().whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
      controllerDriver.povDown().whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
      // controllerDriver.povLeft().whileTrue(new WheelDiameterCharacterization(drivetrain, 1));
      // controllerDriver.povRight().whileTrue(new WheelDiameterCharacterization(drivetrain, -1));
    }

    drivetrain.registerTelemetry(state::updateSwerveTelemetry);

    resetIntentButton.onTrue(state.setIntent(Intent.kNone));
  }

  /** Use this method to configure trigger bindings. */
  private void configureTriggers() {
    // endgame alerts
    new Trigger(
            () ->
                DriverStation.isTeleopEnabled()
                    && DriverStation.getMatchTime() > 0
                    && DriverStation.getMatchTime() <= Math.round(kEndgameAlert1.get()))
        .onTrue(rumble(Milliseconds.of(500)));

    new Trigger(
            () ->
                DriverStation.isTeleopEnabled()
                    && DriverStation.getMatchTime() > 0
                    && DriverStation.getMatchTime() <= Math.round(kEndgameAlert2.get()))
        .onTrue(
            Commands.sequence(
                rumble(Milliseconds.of(200)),
                Commands.waitTime(Milliseconds.of(100)),
                rumble(Milliseconds.of(200)),
                Commands.waitTime(Milliseconds.of(100)),
                rumble(Milliseconds.of(200))));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  /** Returns feedback for LED subsystem */
  private LedFeedback.Type getDefaultLedFeedback() {
    switch (state.intent) {
      default:
        return LedFeedback.Type.kNone;
    }
  }

  private LedVision.Type getDefaultVisionFeedback() {
    return state.isPoseTrusted.getAsBoolean() ? LedVision.Type.kTrusted : LedVision.Type.kUnknown;
  }

  private Command clearOrientationLock() {
    return Commands.runOnce(drivetrain::clearOrientationLock);
  }

  /** Runs validation checklist at the startup. Must be invoked from the robotInit(). */
  public void initValidation() {}

  /** Creates a command to rumble the controllers. */
  private Command rumble(Time time) {
    return Commands.sequence(
        Commands.runOnce(() -> controllerDriver.getHID().setRumble(RumbleType.kBothRumble, 1.0)),
        Commands.waitTime(time),
        Commands.runOnce(() -> controllerDriver.getHID().setRumble(RumbleType.kBothRumble, 0.0)));
  }

  /** Gets modifier of the rotational velocity when precise rotation mode is active. */
  private double getOmegaModifier() {
    return preciseRotationActive.getAsBoolean() ? kPreciseRotation.get() : 1;
  }

  // spotless:off
  private static final Tunable.DoubleValue kEndgameAlert1 = Tunable.value("Endgame/Alert1", 30.0);
  private static final Tunable.DoubleValue kEndgameAlert2 = Tunable.value("Endgame/Alert2", 10.0);
  private static final Tunable.DoubleValue kPreciseRotation = Tunable.value("Control/PreciseRotation", 0.2);
  // spotless:on
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.lib.CANduit;
import frc.robot.lib.GamePhaseTracker;
import frc.robot.lib.Tunable;
import frc.robot.subsystems.AprilTagDetector;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LedFeedback;
import frc.robot.subsystems.LedResource;
import frc.robot.subsystems.LedVision;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class RobotContainer {
  private final RobotState state = RobotState.getInstance();
  private final CommandXboxController controllerDriver = new CommandXboxController(0);
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  private final GamePhaseTracker gamePhaseTracker = new GamePhaseTracker();

  private final LedResource led = new LedResource(Constants.PwmID.LED);
  private final LedFeedback ledFeedback = new LedFeedback(led, this::getDefaultLedFeedback);
  private final LedVision ledVision = new LedVision(led, this::getDefaultVisionFeedback);

  @Logged(name = "CANduit", importance = Importance.CRITICAL)
  private final CANduit canduit;

  @Logged(name = "AprilTagDetector", importance = Importance.CRITICAL)
  private final AprilTagDetector aprilTagDetector = new AprilTagDetector();

  @Logged(name = "Intake", importance = Importance.CRITICAL)
  private final Intake intake = new Intake(TunerConstants.kCANBus);

  @Logged(name = "Hopper", importance = Importance.CRITICAL)
  private final Hopper hopper = new Hopper(TunerConstants.kCANBus);

  @Logged(name = "Turret", importance = Importance.CRITICAL)
  private final Turret turret;

  @Logged(name = "Shooter", importance = Importance.CRITICAL)
  private final Shooter shooter = new Shooter(TunerConstants.kCANBus);

  @Logged(name = "Climb", importance = Importance.CRITICAL)
  private final Climb climb;

  private static final Trigger robotEnabled = new Trigger(DriverStation::isEnabled);

  // slows down max rotation speed for more precise movements
  private final Trigger preciseRotationActive = controllerDriver.leftStick();

  // button mappings
  private final Trigger resetOrientationButton = controllerDriver.start();
  private final Trigger brakeButton = controllerDriver.b();
  private final Trigger resetButton = controllerDriver.x();
  private final Trigger enableIntakeButton = controllerDriver.a();
  private final Trigger shootButton = controllerDriver.rightTrigger();
  private final Trigger parkHoodButton = controllerDriver.rightBumper();
  private final Trigger climbButton = controllerDriver.leftBumper();
  private final Trigger climbUpButton = controllerDriver.povUp();
  private final Trigger climbDownButton = controllerDriver.povDown();

  // alerts
  private static final Time kMissingAutoWinAllianceAlertThreshold = Seconds.of(1);
  private final Alert alertMissingAutoWinAlliance =
      new Alert("AUTO WINNER NOT SET!", AlertType.kError);

  private final Alert alertLowBattery =
      new Alert(
          "Battery voltage is low, turn off the robot and replace the battery!",
          AlertType.kWarning);

  private final Alert alertControllerDriverDisconnected =
      new Alert("Driver controller disconnected (port 0).", AlertType.kWarning);

  public RobotContainer(CANduit canduit) {
    this.canduit = canduit;
    turret = new Turret(TunerConstants.kCANBus, canduit);
    climb = new Climb(TunerConstants.kCANBus, canduit);

    state.bind(drivetrain);
    state.setVisionStateSupplier(aprilTagDetector::getState);
    state.setGamePhaseSupplier(gamePhaseTracker::getPhase);

    turret.setTrackingSupplier(state::getTurretAngleSetpoint);
    shooter.setTrackingSupplier(state::getShooterParameters);

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

    shootButton
        .and(turret::isAtSetpoint)
        .debounce(250, DebounceType.kFalling)
        .whileTrue(hopper.enable());

    shootButton.whileTrue(shooter.enable());

    enableIntakeButton.onTrue(intake.enable());
    resetButton.onTrue(intake.disable());

    climbUpButton.and(() -> !climb.isEngaged()).onTrue(climb.extend());
    climbDownButton.and(() -> !climb.isEngaged()).onTrue(climb.retract());

    parkHoodButton.or(state::isShooterWithinTrench).whileTrue(shooter.parkHood());

    climbButton.and(climb::isReady).onTrue(climb.engage());
    resetButton.and(climb::isEngaged).onTrue(climb.disengage());

    // controllerDriver.povLeft().onTrue(drivetrain.align(() -> new
    // APTarget(state.getShootPose())));
    // controllerDriver
    //     .povLeft()
    //     .whileTrue(
    //         new DriveToPose(
    //             drivetrain, () -> Optional.of(state.getShootPose()), state::getRobotPose));

    // controllerDriver.y().onTrue();
    // controllerDriver.b().onTrue(intake.enable().andThen(intake.deploy()));
  }

  /** Use this method to configure trigger bindings. */
  private void configureTriggers() {
    // auto-initialize turret on enable
    robotEnabled.and(turret::shouldInitialize).onTrue(turret.initialize());

    RobotModeTriggers.autonomous().onTrue(Commands.runOnce(gamePhaseTracker::startPeriod));
    RobotModeTriggers.teleop().onTrue(Commands.runOnce(gamePhaseTracker::startPeriod));
    RobotModeTriggers.disabled()
        .onTrue(Commands.runOnce(gamePhaseTracker::startPeriod).ignoringDisable(true));

    // alert for missing auto win alliance
    RobotModeTriggers.teleop()
        .and(gamePhaseTracker.missingAutoWinAlliance)
        .and(
            () ->
                gamePhaseTracker.getPeriodElapsedTime().gte(kMissingAutoWinAllianceAlertThreshold))
        .whileTrue(
            Commands.startEnd(
                () -> {
                  state.missingAutoWinAlliance = true;
                  alertMissingAutoWinAlliance.set(true);
                },
                () -> {
                  state.missingAutoWinAlliance = false;
                  alertMissingAutoWinAlliance.set(false);
                }));

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

  public void periodic() {
    // alert for low battery in disabled state
    alertLowBattery.set(DriverStation.isDisabled() && state.isLowBattery());

    // alert for disconnected controllers
    alertControllerDriverDisconnected.set(
        !DriverStation.isJoystickConnected(controllerDriver.getHID().getPort()));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  /** Returns feedback for LED subsystem */
  private LedFeedback.Type getDefaultLedFeedback() {
    return LedFeedback.Type.kNone;
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

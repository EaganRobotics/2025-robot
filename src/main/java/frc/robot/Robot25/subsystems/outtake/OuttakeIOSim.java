package frc.robot.Robot25.subsystems.outtake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Robot25.subsystems.outtake.OuttakeConstants.CURRENT_LIMIT;
import static frc.robot.Robot25.subsystems.outtake.OuttakeConstants.GEARING;

import java.util.Optional;
import java.util.concurrent.CompletableFuture;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.tunables.LoggedTunableBoolean;
import frc.robot.SimConstants;
import frc.robot.Robot25.RobotContainer;
import frc.robot.Robot25.subsystems.elevator.Elevator;
import frc.robot.Robot25.subsystems.elevator.Elevator.Level;
import frc.robot.Robot25.subsystems.outtake.OuttakeConstants.Sim;
import static frc.robot.Robot25.util.Pose2dNearLine.isNearSegment;

import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.IntakeSimulation.IntakeSide;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.ironmaple.simulation.motorsims.MapleMotorSim;
import org.ironmaple.simulation.motorsims.SimMotorConfigs;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;
import org.littletonrobotics.junction.Logger;

public class OuttakeIOSim implements OuttakeIO {

  private final AbstractDriveTrainSimulation driveSim = RobotContainer.DRIVE_SIMULATION;

  private final IntakeSimulation intakeSim = IntakeSimulation.InTheFrameIntake("Coral", driveSim, Inches.of(16.743),
      IntakeSide.BACK, 1);

  private static final Transform3d CORAL_MIDDLE_POSE = new Transform3d(Inches.of(-1.356 + 9), Inches.of(-0.012),
      Inches.of(20.196 - 0.75),
      new Rotation3d(Degrees.zero(), Degrees.of(34.411), Degrees.zero()))
      .plus(new Transform3d(Inches.of(-2), Inches.zero(), Inches.zero(), Rotation3d.kZero));
  private static final Transform3d CORAL_LOADING_POSE = CORAL_MIDDLE_POSE
      .plus(new Transform3d(Inches.of(-10), Inches.zero(), Inches.zero(), Rotation3d.kZero));
  private static final Transform3d CORAL_LOADED_POSE = CORAL_MIDDLE_POSE
      .plus(new Transform3d(Inches.of(4), Inches.zero(), Inches.zero(), Rotation3d.kZero));
  private static final double CORAL_LOAD_TIME_SECONDS = 0.1;

  private double loadTimeSeconds = -1;
  private Optional<Pose3d> coralPose = Optional.empty();

  private static final DCMotor outtakeGearbox = DCMotor.getKrakenX60(1);
  private final SimulatedMotorController.GenericMotorController outtakeMotorController;
  private final MapleMotorSim outtakeMotor;
  private boolean isClosedLoop = false;
  private Voltage outtakeAppliedVoltage = Volts.of(0);
  private final SimpleMotorFeedforward feedForwardController = new SimpleMotorFeedforward(Sim.kS, Sim.kV, Sim.kA);

  private final FlywheelSim outtakeSim = new FlywheelSim(
      LinearSystemId.createFlywheelSystem(outtakeGearbox, 0.1, GEARING), outtakeGearbox, 0.000015);

  LoggedTunableBoolean LoadSideSensor = new LoggedTunableBoolean("Tuning/Outtake/LoadSideSensor", false);
  LoggedTunableBoolean ScoreSideSensor = new LoggedTunableBoolean("Tuning/Outtake/ScoreSideSensor", false);

  private Trigger hasGamePiece = new Trigger(LoadSideSensor::get);
  private Trigger canGetGamePiece = hasGamePiece.negate().debounce(SimConstants.LOAD_CORAL_DELAY.in(Seconds));

  public OuttakeIOSim() {
    outtakeMotor = new MapleMotorSim(
        new SimMotorConfigs(outtakeGearbox, GEARING, Sim.MOTOR_LOAD_MOI, Sim.FRICTION_VOLTAGE));
    outtakeMotorController = outtakeMotor.useSimpleDCMotorController().withCurrentLimit(CURRENT_LIMIT);
  }

  public void simStageCoral() {
    intakeSim.addGamePieceToIntake();
  }

  @Override
  public void setOpenLoop(Voltage output) {
    outtakeAppliedVoltage = output;
    isClosedLoop = false;
    if (output.in(Volts) > 0 && coralPose.isPresent()) {
      var coralTranslation = CORAL_LOADED_POSE.getTranslation().toTranslation2d();
      var height = Elevator.getInstance().getCurrentHeight();
      var angle = Degrees.of(-34.411);
      if (Elevator.getInstance().isAtHeight(Level.L4).getAsBoolean()) {
        coralTranslation = coralTranslation.plus(new Translation2d(Inches.of(4), Inches.zero()));
        height = height.plus(Inches.of(4));
        angle = Degrees.of(-70);
      }
      var projectile = new ReefscapeCoralOnFly(driveSim.getSimulatedDriveTrainPose().getTranslation(),
          coralTranslation, driveSim.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
          driveSim.getSimulatedDriveTrainPose().getRotation(), height,
          MetersPerSecond.of(output.in(Volts) / 3), angle);
      coralPose = Optional.empty();
      CompletableFuture.runAsync(() -> {
        System.out.println("Disabling sensors");
        try {
          Thread.sleep(50);
          LoadSideSensor.set(false);
          Thread.sleep(100);
          ScoreSideSensor.set(false);
        } catch (InterruptedException e) {
          e.printStackTrace();
          LoadSideSensor.set(false);
          ScoreSideSensor.set(false);
          Thread.currentThread().interrupt();
        }
      });
      SimulatedArena.getInstance().addGamePieceProjectile(projectile);
    }
  }

  @Override
  public void setRollerOpenLoop(Voltage output) {
    // TODO simulate another motor?
    outtakeAppliedVoltage = output;
    isClosedLoop = false;
  }

  @Override
  public void updateInputs(OuttakeIOInputs inputs) {
    var angularVelocity = outtakeSim.getAngularVelocityRadPerSec();
    if (isClosedLoop) {
      var feedForwardVolts = feedForwardController.calculate(angularVelocity);
      outtakeAppliedVoltage = Volts.of(feedForwardVolts);
    }

    outtakeMotorController.requestVoltage(outtakeAppliedVoltage);
    outtakeSim.setInputVoltage(outtakeMotor.getAppliedVoltage().in(Volts));
    outtakeMotor.update(Seconds.of(TimedRobot.kDefaultPeriod));
    outtakeSim.update(TimedRobot.kDefaultPeriod);

    var nearLoadingStation = isNearSegment(driveSim.getSimulatedDriveTrainPose(), SimConstants.BR_LOADING_STATION,
        SimConstants.FR_LOADING_STATION, SimConstants.LOADING_STATION_TOLERANCE)
        || isNearSegment(driveSim.getSimulatedDriveTrainPose(), SimConstants.BL_LOADING_STATION,
            SimConstants.FL_LOADING_STATION, SimConstants.LOADING_STATION_TOLERANCE);
    Logger.recordOutput("Outtake/NearLoadingStation", nearLoadingStation);
    var joystickName = DriverStation.getJoystickName(2);
    var humanPlayerControllerConnected = joystickName != null && !joystickName.isBlank();
    Logger.recordOutput("HumanPlayerConnected", humanPlayerControllerConnected);
    Logger.recordOutput("HasGamePiece", hasGamePiece);
    Logger.recordOutput("CanGetGamePiece", canGetGamePiece);
    if (humanPlayerControllerConnected && outtakeAppliedVoltage.in(Volts) > 0 && coralPose.isEmpty()
        && nearLoadingStation) {
      intakeSim.startIntake();
    } else if (!humanPlayerControllerConnected && nearLoadingStation
        && canGetGamePiece.getAsBoolean()) {
      CompletableFuture.runAsync(() -> {
        try {
          Thread.sleep(100);
        } catch (InterruptedException e) {
          e.printStackTrace();
          Thread.currentThread().interrupt();
        } finally {
          intakeSim.addGamePieceToIntake();
        }
      });
    } else {
      intakeSim.stopIntake();
    }

    var robotPose = new Pose3d(driveSim.getSimulatedDriveTrainPose())
        .plus(new Transform3d(Inches.of(0), Inches.of(0),
            Elevator.getInstance().getCurrentHeight().minus(Elevator.Level.Intake.getHeight()),
            Rotation3d.kZero));
    if (intakeSim.obtainGamePieceFromIntake()) {
      LoadSideSensor.set(true);
      coralPose = Optional.of(robotPose.plus(CORAL_LOADING_POSE));
      loadTimeSeconds = Timer.getFPGATimestamp();
    } else if (coralPose.isPresent()) {
      var t = (Timer.getFPGATimestamp() - loadTimeSeconds) / CORAL_LOAD_TIME_SECONDS;
      if (t >= 1) {
        ScoreSideSensor.set(true);
        coralPose = Optional.of(robotPose.plus(CORAL_LOADED_POSE));
      } else if (t > 0 && t < 1) {
        coralPose = Optional.of(
            robotPose.plus(CORAL_LOADING_POSE).interpolate(robotPose.plus(CORAL_LOADED_POSE), t));
      }
    }

    // Record coral pose simulation output
    RobotContainer.simCoralPoses[0] = coralPose.orElse(SimConstants.QUEENED_GAMEPIECE_POSE);

    // Update motor inputs
    inputs.outtakeConnected = true;
    inputs.outtakeAppliedVolts = outtakeAppliedVoltage;
    inputs.outtakeCurrent = Amps.of(outtakeSim.getCurrentDrawAmps());
    inputs.outtakeVelocity = AngularVelocity.ofBaseUnits(angularVelocity, RadiansPerSecond);
    inputs.seesCoralAtInput = LoadSideSensor.get();
    inputs.seesCoralAtOutput = ScoreSideSensor.get();
  }
}

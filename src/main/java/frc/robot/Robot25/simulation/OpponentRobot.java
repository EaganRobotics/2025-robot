package frc.robot.Robot25.simulation;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.wpilibj.DriverStation;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.PIDConstants;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SelfControlledSwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.ironmaple.utils.FieldMirroringUtils;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class OpponentRobot extends SubsystemBase {
  /*
   * If an opponent robot is not on the field, it is placed in a queening position for performance.
   */
  private static final Pose2d[] ROBOT_QUEENING_POSITIONS =
      new Pose2d[] {new Pose2d(-6, 0, new Rotation2d()), new Pose2d(-5, 0, new Rotation2d()),
          new Pose2d(-4, 0, new Rotation2d()), new Pose2d(-3, 0, new Rotation2d()),
          new Pose2d(-2, 0, new Rotation2d())};

  /* The robots will be teleported to these positions when teleop begins. */
  private static final Pose2d[] ROBOTS_STARTING_POSITIONS =
      new Pose2d[] {new Pose2d(15, 6, Rotation2d.fromDegrees(180)),
          new Pose2d(15, 4, Rotation2d.fromDegrees(180)),
          new Pose2d(15, 2, Rotation2d.fromDegrees(180)), new Pose2d(1.6, 6, new Rotation2d()),
          new Pose2d(1.6, 4, new Rotation2d())};

  private static final double ROBOT_MASS_KG = 62;
  private static final double ROBOT_MOI = 8;
  private static final double DRIVE_REDUCTION = 8.14;
  private static final double WHEEL_RADIUS_IN = 2;

  private static final List<OpponentRobot> opponentRobots = new ArrayList<>(5);

  /*
   * The drivetrain configuration for the opponent robots in the maple-sim simulation.
   */
  private static final DriveTrainSimulationConfig DRIVETRAIN_CONFIG =
      DriveTrainSimulationConfig.Default().withRobotMass(Kilograms.of(ROBOT_MASS_KG))
          .withSwerveModule(new SwerveModuleSimulationConfig(DCMotor.getKrakenX60(1),
              DCMotor.getKrakenX60(1), DRIVE_REDUCTION, 150.0 / 7.0, Volts.of(0), Volts.of(0), Inches.of(WHEEL_RADIUS_IN),
              KilogramSquareMeters.of(0.04), 1.6));

  // PathPlanner configuration
  private static final RobotConfig PP_CONFIG = new RobotConfig(
          ROBOT_MASS_KG, // Robot mass in kg
          ROBOT_MOI,  // Robot MOI
          new ModuleConfig(
                  Inches.of(WHEEL_RADIUS_IN).in(Meters), 3.5, 1.2, DCMotor.getKrakenX60(1).withReduction(DRIVE_REDUCTION), 60, 1), // Swerve module config
          new Translation2d(0.69, 0.69) // Track length and width
  );

  // PathPlanner PID settings
  private final PPHolonomicDriveController driveController =
          new PPHolonomicDriveController(new PIDConstants(5.0, 0.02), new PIDConstants(7.0, 0.05));

  private final SelfControlledSwerveDriveSimulation driveSimulation;
  private final Pose2d queeningPose;
  private final int id;
  private String behavior = "Initializing";

  public OpponentRobot(int id) {
    this.id = id;
    this.queeningPose = ROBOT_QUEENING_POSITIONS[id];
    this.driveSimulation = new SelfControlledSwerveDriveSimulation(
        new SwerveDriveSimulation(DRIVETRAIN_CONFIG, queeningPose));

    SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation.getDriveTrainSimulation());
    opponentRobots.add(this);
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Opponent/Robot" + id + "/Pose", driveSimulation.getDriveTrainSimulation().getSimulatedDriveTrainPose());
    Logger.recordOutput("Opponent/Robot" + id + "/Behavior", behavior);
  }

  /**
   * Joystick drive command for controlling the opponent robots. This command allows the robot to be
   * driven using an Xbox controller.
   */
  public Command joystickDrive(XboxController joystick) {
    // Obtain chassis speeds from the joystick inputs
    final Supplier<ChassisSpeeds> joystickSpeeds = () -> new ChassisSpeeds(
        -joystick.getLeftY() * driveSimulation.maxLinearVelocity().in(MetersPerSecond), // Forward/Backward
        -joystick.getLeftX() * driveSimulation.maxLinearVelocity().in(MetersPerSecond), // Left/Right
        -joystick.getRightX() * driveSimulation.maxAngularVelocity().in(RadiansPerSecond) // Rotation
    );

    return this.run(() -> {
      // Calculate field-centric speed from the driver station-centric speed
      final ChassisSpeeds fieldCentricSpeeds =
          ChassisSpeeds.fromRobotRelativeSpeeds(joystickSpeeds.get(), driveSimulation
              .getDriveTrainSimulation().getSimulatedDriveTrainPose().getRotation().unaryMinus());
      // Run the field-centric speed to control the robot's movement
      driveSimulation.runChassisSpeeds(fieldCentricSpeeds, new Translation2d(), true, true);
    })
        // Before the command starts, reset the robot to its starting position on the
        // field
        .beforeStarting(() -> {
          driveSimulation.setSimulationWorldPose(
            FieldMirroringUtils.toCurrentAlliancePose(ROBOTS_STARTING_POSITIONS[id - 1]));
        behavior = "JoystickDrive";
        });
  }

  public Command friendlyCycleCoral() {
    // TODO
    return Commands.none();
  }

  public Command friendlyCycleAlgae() {
    // TODO
    return Commands.none();
  }

  public Command friendlyBumpInto() {
    // TODO
    return Commands.none();
  }

  public Command enemyDefense() {
    // TODO
    return Commands.none();
  }

  /** Follow path command for opponent robots */
  private Command followPath(PathPlannerPath path) {
      return new FollowPathCommand(
              path,
              driveSimulation::getActualPoseInSimulationWorld,
              driveSimulation::getActualSpeedsRobotRelative,
              (speeds, feedforwards) ->
                  driveSimulation.runChassisSpeeds(speeds, new Translation2d(), false, false),
              driveController,
              PP_CONFIG,
              () -> DriverStation.getAlliance()
                  .orElse(DriverStation.Alliance.Blue)
                  .equals(DriverStation.Alliance.Red),
              this
      );
  }

  private List<OpponentRobot> otherRobots() {
    // return opponentRobots.where(robot -> robot != this);
    return new ArrayList<>();
  }
}

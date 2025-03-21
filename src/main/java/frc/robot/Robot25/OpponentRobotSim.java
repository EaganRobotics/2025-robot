package frc.robot.Robot25;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.function.Supplier;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SelfControlledSwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.utils.FieldMirroringUtils;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class OpponentRobotSim extends SubsystemBase {
  /*
   * If an opponent robot is not on the field, it is placed in a queening position
   * for performance.
   */
  public static final Pose2d[] ROBOT_QUEENING_POSITIONS = new Pose2d[] {
      new Pose2d(-6, 0, new Rotation2d()),
      new Pose2d(-5, 0, new Rotation2d()),
      new Pose2d(-4, 0, new Rotation2d()),
      new Pose2d(-3, 0, new Rotation2d()),
      new Pose2d(-2, 0, new Rotation2d())
  };

  /* The robots will be teleported to these positions when teleop begins. */
  public static final Pose2d[] ROBOTS_STARTING_POSITIONS = new Pose2d[] {
      new Pose2d(15, 6, Rotation2d.fromDegrees(180)),
      new Pose2d(15, 4, Rotation2d.fromDegrees(180)),
      new Pose2d(15, 2, Rotation2d.fromDegrees(180)),
      new Pose2d(1.6, 6, new Rotation2d()),
      new Pose2d(1.6, 4, new Rotation2d())
  };

  /*
   * The drivetrain configuration for the opponent robots in the maple-sim
   * simulation.
   */
  private static final DriveTrainSimulationConfig DRIVETRAIN_CONFIG = DriveTrainSimulationConfig.Default()
      .withRobotMass(Pounds.of(140));

  private final SelfControlledSwerveDriveSimulation driveSimulation;
  private final Pose2d queeningPose;
  private final int id;

  public OpponentRobotSim(int id) {
    this.id = id;
    this.queeningPose = ROBOT_QUEENING_POSITIONS[id];
    this.driveSimulation = new SelfControlledSwerveDriveSimulation(new SwerveDriveSimulation(
        DRIVETRAIN_CONFIG,
        queeningPose));

    SimulatedArena.getInstance().addDriveTrainSimulation(
        driveSimulation.getDriveTrainSimulation());
  }

  public Pose2d opponentRobotPose() {
    return driveSimulation.getDriveTrainSimulation().getSimulatedDriveTrainPose();
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Opponent Robot Pose", opponentRobotPose());
  }

  /**
   * Joystick drive command for controlling the opponent robots. This command
   * allows the robot to be driven using an
   * Xbox controller.
   */
  public Command joystickDrive(XboxController joystick) {
    // Obtain chassis speeds from the joystick inputs
    final Supplier<ChassisSpeeds> joystickSpeeds = () -> new ChassisSpeeds(
        -joystick.getLeftY() * driveSimulation.maxLinearVelocity().in(MetersPerSecond), // Forward/Backward
        -joystick.getLeftX() * driveSimulation.maxLinearVelocity().in(MetersPerSecond), // Left/Right
        -joystick.getRightX() * driveSimulation.maxAngularVelocity().in(RadiansPerSecond) // Rotation
    );

    // Obtain the driver station facing for the opponent alliance
    // Used in defense practice, where two tabs of AScope show the driver stations
    // of both alliances
    final Supplier<Rotation2d> opponentDriverStationFacing = () -> FieldMirroringUtils
        .getCurrentAllianceDriverStationFacing().plus(Rotation2d.fromDegrees(180));

    return Commands.run(
        () -> {
          // Calculate field-centric speed from the driver station-centric speed
          final ChassisSpeeds fieldCentricSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
              joystickSpeeds.get(),
              FieldMirroringUtils.getCurrentAllianceDriverStationFacing()
                  .plus(Rotation2d.fromDegrees(180)));
          // Run the field-centric speed to control the robot's movement
          driveSimulation.runChassisSpeeds(fieldCentricSpeeds, new Translation2d(), true, true);
        },
        this)
        // Before the command starts, reset the robot to its starting position on the
        // field
        .beforeStarting(() -> driveSimulation.setSimulationWorldPose(
            FieldMirroringUtils.toCurrentAlliancePose(ROBOTS_STARTING_POSITIONS[id - 1])));
  }
}

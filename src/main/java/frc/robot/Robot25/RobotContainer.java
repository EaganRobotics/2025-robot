// Copyright Copyright Copyright Copyright

package frc.robot.Robot25;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.devices.DigitalInputWrapper;
import frc.lib.devices.TalonFXWrapper;
import frc.lib.devices.TalonFXWrapper.FollowerConfig;
import frc.robot.Robot;
import frc.robot.Robot25.commands.DriveCommands;
import frc.robot.Robot25.subsystems.drive.Drive;
import frc.robot.Robot25.subsystems.drive.DriveConstants;
import frc.robot.Robot25.subsystems.drive.ModuleIO;
import frc.robot.Robot25.subsystems.drive.ModuleIOSim;
import frc.robot.Robot25.subsystems.drive.ModuleIOTalonFX;
import frc.robot.Robot25.subsystems.elevator.Elevator;
import frc.robot.Robot25.subsystems.elevator.ElevatorIO;
import frc.robot.Robot25.subsystems.elevator.ElevatorIOSim;
import frc.robot.Robot25.subsystems.elevator.ElevatorIOTalonFX;
import frc.robot.Robot25.subsystems.elevator.ElevatorIOTalonFXNew;
import frc.robot.Robot25.subsystems.gyro.GyroIO;
import frc.robot.Robot25.subsystems.gyro.GyroIOPigeon2;
import frc.robot.Robot25.subsystems.gyro.GyroIOSim;
import frc.robot.Robot25.subsystems.outtake.Outtake;
import frc.robot.Robot25.subsystems.outtake.OuttakeIO;
import frc.robot.Robot25.subsystems.outtake.OuttakeIOSim;
import frc.robot.Robot25.subsystems.outtake.OuttakeIOTalonFX;
import frc.robot.Robot25.subsystems.vision.Vision;
import frc.robot.Robot25.subsystems.vision.VisionIO;
import frc.robot.Robot25.subsystems.vision.VisionIOLimelight;
import frc.robot.Robot25.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.SimConstants;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer extends frc.lib.RobotContainer {

  // Subsystems
  private final Drive drive;
  private final Elevator elevator;
  private final Outtake outtake;
  private final Vision vision;

  // Drive simulation
  private static final SwerveDriveSimulation driveSimulation =
      new SwerveDriveSimulation(Drive.MAPLE_SIM_CONFIG, SimConstants.SIM_INITIAL_FIELD_POSE);

  // Controller
  private final CommandXboxController DriverController = new CommandXboxController(0);
  private final CommandXboxController OperatorController = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  // coast buttion
  private static DigitalInputWrapper coastButton = new DigitalInputWrapper(4, "coastButton", false);

  @AutoLogOutput
  public final Pose3d[] mechanismPoses = new Pose3d[] {Pose3d.kZero, Pose3d.kZero, Pose3d.kZero,};

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    super(driveSimulation);
    // Check for valid swerve config
    var modules = new SwerveModuleConstants[] {DriveConstants.FrontLeft, DriveConstants.FrontRight,
        DriveConstants.BackLeft, DriveConstants.BackRight,};
    for (var constants : modules) {
      if (constants.DriveMotorType != DriveMotorArrangement.TalonFX_Integrated
          || constants.SteerMotorType != SteerMotorArrangement.TalonFX_Integrated) {
        throw new RuntimeException(
            "You are using an unsupported swerve configuration, which this template does not support without manual customization. The 2025 release of Phoenix supports some swerve configurations which were not available during 2025 beta testing, preventing any development and support from the AdvantageKit developers.");
      }
    }

    switch (SimConstants.CURRENT_MODE) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive = new Drive(new GyroIOPigeon2(), new ModuleIOTalonFX(DriveConstants.FrontLeft),
            new ModuleIOTalonFX(DriveConstants.FrontRight),
            new ModuleIOTalonFX(DriveConstants.BackLeft),
            new ModuleIOTalonFX(DriveConstants.BackRight));

        elevator = new Elevator(new ElevatorIOTalonFXNew());
        outtake = new Outtake(new OuttakeIOTalonFX());
        vision = new Vision(drive, new VisionIO() {});

        // vision =
        // new Vision(drive, new VisionIOLimelight("bob", () -> drive.getPose().getRotation()));
        break;
      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive = new Drive(new GyroIOSim(driveSimulation.getGyroSimulation()),
            new ModuleIOSim(driveSimulation.getModules()[0]),
            new ModuleIOSim(driveSimulation.getModules()[1]),
            new ModuleIOSim(driveSimulation.getModules()[2]),
            new ModuleIOSim(driveSimulation.getModules()[3]));

        elevator = new Elevator(new ElevatorIOSim());
        outtake = new Outtake(new OuttakeIOSim());
        vision = new Vision(drive,
            new VisionIOPhotonVisionSim("Northstar 0",
                new Transform3d(0.225425, 0.2667, 0.20955,
                    new Rotation3d(Degrees.zero(), Degrees.of(-28.125), Degrees.of(30.0))),
                () -> drive.getPose()));
        break;
      default:
        // Replayed robot, disable IO implementations
        drive = new Drive(new GyroIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {},
            new ModuleIO() {});

        elevator = new Elevator(new ElevatorIO() {});

        outtake = new Outtake(new OuttakeIO() {});

        vision = new Vision(drive, new VisionIO() {});
        break;
    }

    NamedCommands.registerCommand("MinHeight", elevator.minHeight());

    NamedCommands.registerCommand("L1", elevator.L1());
    NamedCommands.registerCommand("L2", elevator.L2());
    NamedCommands.registerCommand("L3", elevator.L3());
    NamedCommands.registerCommand("L4", elevator.L4());
    NamedCommands.registerCommand("Exhaust", outtake.depositCoral());

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    autoChooser.addOption("Static Drive Voltage", Commands.run(() -> drive.driveOpenLoop(10)));
    autoChooser.addOption("Static Turn Voltage", Commands.run(() -> drive.TurnOpenLoop(10)));

    // Set up SysId routines
    autoChooser.addOption("Drive Wheel Radius Characterization",
        DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption("Drive Simple FF Characterization",
        DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption("Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption("Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption("Drive SysId (Dynamic Forward)",
        drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption("Drive SysId (Dynamic Reverse)",
        drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // toggle coast on true
    coastButton.asTrigger().onChange(Commands.runOnce(() -> {
      // TODO Add coast for more subsystems once we have them
      drive.toggleCoast();
      System.out.println("COAST TOGGLED");
    }));

    // Xbox controller is mapped incorrectly on Mac OS
    DoubleSupplier xSupplier = () -> DriverController.getLeftX();
    DoubleSupplier ySupplier = () -> DriverController.getLeftY();
    DoubleSupplier omegaSupplier = () -> -DriverController.getRightX();
    BooleanSupplier slowModeSupplier =
        () -> !SimConstants.IS_MAC ? DriverController.getRightTriggerAxis() > 0.5
            : DriverController.getRightX() > 0.0;

    // Default command, normal field-relative drive
    drive
        .setDefaultCommand(DriveCommands.joystickDrive(drive, ySupplier, xSupplier, omegaSupplier));
    outtake.setDefaultCommand(outtake.autoQueueCoral(OperatorController.leftBumper())
        .onlyWhile(elevator.elevatorAtMinHeight()));

    // POV snap to angles
    // DriverController.povUp().onTrue(DriveCommands.snapToRotation(drive,
    // Rotation2d.kZero));
    DriverController.povUpRight()
        .onTrue(DriveCommands.snapToRotation(drive, Rotation2d.fromDegrees(-45)));
    DriverController.povRight()
        .onTrue(DriveCommands.snapToRotation(drive, Rotation2d.fromDegrees(-90)));
    DriverController.povDownRight()
        .onTrue(DriveCommands.snapToRotation(drive, Rotation2d.fromDegrees(-135)));
    // DriverController.povDown()
    // .onTrue(DriveCommands.snapToRotation(drive, Rotation2d.fromDegrees(-180)));
    DriverController.povDownLeft()
        .onTrue(DriveCommands.snapToRotation(drive, Rotation2d.fromDegrees(135)));
    DriverController.povLeft()
        .onTrue(DriveCommands.snapToRotation(drive, Rotation2d.fromDegrees(90)));
    DriverController.povUpLeft()
        .onTrue(DriveCommands.snapToRotation(drive, Rotation2d.fromDegrees(45)));

    // DriverController.povDown().onTrue(elevator.downLevel());
    // DriverController.povUp().onTrue(elevator.upLevel());
    // DriverController.rightTrigger().onTrue(outtake.depositCoral());
    // DriverController.leftTrigger().onTrue(outtake.reverseCoral());
    // DriverController.a().onTrue(elevator.minHeight());
    // DriverController.x().onTrue(elevator.L2());
    // DriverController.b().onTrue(elevator.L3());
    // DriverController.y().onTrue(elevator.L4());

    // Reset gyro to 0° when START button is pressed
    DriverController.start()
        .onTrue(Commands.runOnce(
            () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
            drive).ignoringDisable(true));

    // OperatorController.povDown().onTrue(elevator.downLevel());
    // OperatorController.povUp().onTrue(elevator.upLevel());
    // DriverController.rightTrigger().onTrue(outtake.depositCoral());
    // DriverController.leftTrigger().onTrue(outtake.reverseCoral());
    // OperatorController.rightBumper().onTrue(elevator.L1());
    // DriverController.a().onTrue(elevator.minHeight());
    // DriverController.x().onTrue(elevator.L2());
    // DriverController.b().onTrue(elevator.L3());
    // DriverController.y().onTrue(elevator.L4());

    OperatorController.povDown().onTrue(elevator.downLevel());
    OperatorController.povUp().onTrue(elevator.upLevel());
    OperatorController.rightTrigger().onTrue(outtake.depositCoral());
    OperatorController.leftTrigger().onTrue(outtake.reverseCoral());
    OperatorController.rightBumper().onTrue(elevator.L1());
    OperatorController.a().onTrue(elevator.minHeight());
    OperatorController.x().onTrue(elevator.L2());
    OperatorController.b().onTrue(elevator.L3());
    OperatorController.y().onTrue(elevator.L4());

    OperatorController.axisMagnitudeGreaterThan(1, 0.1)
        .whileTrue(elevator.openLoop(OperatorController::getLeftY));

    // elevator.setDefaultCommand(elevator.openLoop(OperatorController::getLeftY));

    // DriverController.y();
    // .onTrue(DriveCommands.snapToPosition(drive, new Pose2d(5, 5,
    // Rotation2d.fromDegrees(90))));
    // elevator.setDefaultCommand(elevator.openLoop(OperatorController::getLeftY));

  }

  @Override
  public Command getAutonomousCommand() {
    return autoChooser.get();

  }

  public Command getTestCommand() {
    var elevatorTalonFX = new TalonFXWrapper(20, "Elevator", false, NeutralModeValue.Brake, 5, 0, 0,
        0, RotationsPerSecondPerSecond.of(0), RotationsPerSecond.of(0), false, false,
        Rotations.of(120.0 / 360.0), Rotations.of(0), new FollowerConfig(21, true),
        Units.Seconds.of(3), Units.Amps.of(75), Units.RotationsPerSecond.of(1));

    return Commands.runEnd(() -> {
      elevatorTalonFX.set(.1);
    }, () -> {
      elevatorTalonFX.set(0);
    }).withTimeout(10);
  }

  @Override
  public void disabledInit() {
    // drive.stopWithX();
  }

  @Override
  public void robotPeriodic() {
    var elevatorPoses = elevator.getElevatorPoses();
    mechanismPoses[0] = elevatorPoses[0];
    mechanismPoses[1] = elevatorPoses[1];
    mechanismPoses[2] = elevatorPoses[2];
  }

  @Override
  public void simulationPeriodic() {
    // var elevatorPoses = elevator.getElevatorPoses();
    // mechanismPoses[0] = elevatorPoses[0];
    // mechanismPoses[1] = elevatorPoses[1];
    // mechanismPoses[2] = elevatorPoses[2];
  }
}

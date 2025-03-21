// Copyright Copyright Copyright Copyright

package frc.robot.Robot25;

import static edu.wpi.first.units.Units.*;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot25.commands.DriveCharacterization;
import frc.robot.Robot25.commands.DriveCommands;
import frc.robot.Robot25.commands.SimDriverPractice;
import frc.robot.Robot25.subsystems.AlgaeEater.Algae;
import frc.robot.Robot25.subsystems.AlgaeEater.AlgaeIO;
import frc.robot.Robot25.subsystems.AlgaeEater.AlgaeIOSim;
import frc.robot.Robot25.subsystems.AlgaeEater.AlgaeIOTalonFX;
import frc.robot.Robot25.subsystems.drive.Drive;
import frc.robot.Robot25.subsystems.drive.DriveConstants;
import frc.robot.Robot25.subsystems.drive.ModuleIO;
import frc.robot.Robot25.subsystems.drive.ModuleIOSim;
import frc.robot.Robot25.subsystems.drive.ModuleIOTalonFX;
import frc.robot.Robot25.subsystems.elevator.Elevator;
import frc.robot.Robot25.subsystems.elevator.Elevator.Level;
import frc.robot.Robot25.subsystems.elevator.ElevatorIO;
import frc.robot.Robot25.subsystems.elevator.ElevatorIOSim;
import frc.robot.Robot25.subsystems.elevator.ElevatorIOTalonFXNew;
import frc.robot.Robot25.subsystems.gyro.GyroIO;
import frc.robot.Robot25.subsystems.gyro.GyroIOPigeon2;
import frc.robot.Robot25.subsystems.gyro.GyroIOSim;
import frc.robot.Robot25.subsystems.outtake.Outtake;
import frc.robot.Robot25.subsystems.outtake.OuttakeIO;
import frc.robot.Robot25.subsystems.outtake.OuttakeIOSim;
import frc.robot.Robot25.subsystems.outtake.OuttakeIOTalonFX;
import frc.robot.Robot25.subsystems.vision.Vision;
import frc.robot.Robot25.subsystems.vision.VisionConstants;
import frc.robot.Robot25.subsystems.vision.VisionIO;
import frc.robot.Robot25.subsystems.vision.VisionIOLimelight;
import frc.robot.Robot25.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.SimConstants;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer extends frc.lib.RobotContainer {

  // Subsystems
  private final Drive drive;
  private final Elevator elevator;
  private final Outtake outtake;
  private final Vision vision;
  private final Algae algae;

  // Drive simulation
  public static final SwerveDriveSimulation DRIVE_SIMULATION = new SwerveDriveSimulation(Drive.MAPLE_SIM_CONFIG,
      SimConstants.SIM_INITIAL_FIELD_POSE);

  // Opponent Robot Simulation
  OpponentRobotSim opponentRobotSim = new OpponentRobotSim(1);

  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);
  private final CommandXboxController humanPlayerController = new CommandXboxController(2);
  private final XboxController opponentController = new XboxController(5);

  private final LoggedDashboardChooser<Command> autoChooser;
  private final LoggedDashboardChooser<Command> testChooser;

  private final Pose3d[] mechanismPoses = new Pose3d[] { Pose3d.kZero, Pose3d.kZero, Pose3d.kZero, };

  public static final Pose3d[] simCoralPoses = new Pose3d[] { Pose3d.kZero, Pose3d.kZero, Pose3d.kZero, };

  public RobotContainer() {
    super(DRIVE_SIMULATION);
    // Check for valid swerve config
    var modules = new SwerveModuleConstants[] { DriveConstants.FrontLeft, DriveConstants.FrontRight,
        DriveConstants.BackLeft, DriveConstants.BackRight, };
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
            new ModuleIOTalonFX(DriveConstants.BackRight),
            DRIVE_SIMULATION::setSimulationWorldPose);

        elevator = new Elevator(new ElevatorIOTalonFXNew());
        outtake = new Outtake(new OuttakeIOTalonFX());
        vision = new Vision(drive,
            new VisionIOLimelight("limelight-front", () -> drive.getPose().getRotation()),
            new VisionIOLimelight("limelight-back", () -> drive.getPose().getRotation()));
        algae = new Algae(new AlgaeIOTalonFX());

        break;
      case SIM:
        drive = new Drive(new GyroIOSim(DRIVE_SIMULATION.getGyroSimulation()),
            new ModuleIOSim(DRIVE_SIMULATION.getModules()[0]),
            new ModuleIOSim(DRIVE_SIMULATION.getModules()[1]),
            new ModuleIOSim(DRIVE_SIMULATION.getModules()[2]),
            new ModuleIOSim(DRIVE_SIMULATION.getModules()[3]),
            DRIVE_SIMULATION::setSimulationWorldPose);
        drive.setPose(SimConstants.SIM_INITIAL_FIELD_POSE);

        elevator = new Elevator(new ElevatorIOSim());
        outtake = new Outtake(new OuttakeIOSim());
        vision = new Vision(drive, new VisionIOPhotonVisionSim(VisionConstants.limelightBackName,
            VisionConstants.limelightBackTransform, DRIVE_SIMULATION::getSimulatedDriveTrainPose),
            new VisionIOPhotonVisionSim(VisionConstants.limelightFrontName,
                VisionConstants.limelightFrontTransform,
                DRIVE_SIMULATION::getSimulatedDriveTrainPose));
        algae = new Algae(new AlgaeIOSim());

        break;
      default:
        // Replayed robot, disable IO implementations
        drive = new Drive(new GyroIO() {
        }, new ModuleIO() {
        }, new ModuleIO() {
        }, new ModuleIO() {
        },
            new ModuleIO() {
            }, DRIVE_SIMULATION::setSimulationWorldPose);

        elevator = new Elevator(new ElevatorIO() {
        });

        outtake = new Outtake(new OuttakeIO() {
        });

        vision = new Vision(drive, new VisionIO() {
        });
        algae = new Algae(new AlgaeIO() {
        });

        break;
    }

    // Values are tuned to speed but may be changed
    NamedCommands.registerCommand("Auto1", DriveCommands.FirstSnapper(drive).withTimeout(2));
    NamedCommands.registerCommand("Align1", DriveCommands.AutoSnapper(drive).withTimeout(2));
    NamedCommands.registerCommand("Align2", DriveCommands.AutoSnapper(drive).withTimeout(2.5));
    NamedCommands.registerCommand("RightSource", DriveCommands.SourceSnapper(drive).withTimeout(2));
    NamedCommands.registerCommand("LeftSource", DriveCommands.SourceSnapper(drive).withTimeout(2));
    // Probably dont change or use
    NamedCommands.registerCommand("Align3", DriveCommands.FirstSnapper(drive).withTimeout(1));
    NamedCommands.registerCommand("Align4", DriveCommands.AutoSnapper(drive).withTimeout(1.5));
    NamedCommands.registerCommand("RightSource2",
        DriveCommands.AutoSourceRight(drive).withTimeout(1));
    NamedCommands.registerCommand("LeftSource2",
        DriveCommands.AutoSourceLeft(drive).withTimeout(1));

    NamedCommands.registerCommand("L0", elevator.L0().andThen(outtake.autoQueueCoral2()));
    NamedCommands.registerCommand("L1", elevator.L1());
    NamedCommands.registerCommand("L2", elevator.L2());
    NamedCommands.registerCommand("L3", elevator.L3());
    NamedCommands.registerCommand("L4", elevator.L4());

    NamedCommands.registerCommand("Maybe1",
        DriveCommands.FullSnapperOuter(drive)
            .andThen(DriveCommands.FullSnapperInner(drive).alongWith(elevator.L4()))
            .andThen(outtake.depositCoral())
            .andThen(DriveCommands.FullSnapperInner(drive).alongWith(elevator.L0())));
    NamedCommands.registerCommand("Maybe2", DriveCommands.SourceSnapper(drive).withTimeout(3));
    NamedCommands.registerCommand("Maybe3",
        DriveCommands.FullSnapperOuterAuto(drive)
            .andThen(DriveCommands.FullSnapperInner(drive).alongWith(elevator.L4()))
            .andThen(outtake.depositCoral()));
    NamedCommands.registerCommand("Maybe4",
        elevator.L0().alongWith(DriveCommands.SourceSnapper(drive).withTimeout(3)));
    NamedCommands.registerCommand("Maybe5",
        DriveCommands.FullSnapperOuter(drive)
            .andThen(DriveCommands.FullSnapperInner(drive).alongWith(elevator.L4()))
            .andThen(outtake.depositCoral()).andThen(elevator.L0()));
    NamedCommands.registerCommand("Exhaust",
        new WaitUntilCommand(elevator.isAtGoal()).andThen(outtake.depositCoral()));

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser("AL.0C.1M"));

    // Set up test routines
    testChooser = new LoggedDashboardChooser<>("Test Choices");

    // Add sim driver practice
    testChooser.addDefaultOption("Sim Driver Practice",
        SimDriverPractice.simDriverPractice(this, this::simResetRobot));

    // Set up SysId routines
    testChooser.addOption("Drive Wheel Radius Characterization",
        DriveCharacterization.wheelRadiusCharacterization(drive));
    testChooser.addOption("Drive Simple FF Characterization",
        DriveCharacterization.feedforwardCharacterization(drive));
    testChooser.addOption("Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    testChooser.addOption("Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    testChooser.addOption("Drive SysId (Dynamic Forward)",
        drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    testChooser.addOption("Drive SysId (Dynamic Reverse)",
        drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    configureButtonBindings();
  }

  private void configureButtonBindings() {

    // Due to field orientation, joystick Y (forward) controls X direction and vice
    // versa
    drive.setDefaultCommand(
        DriveCommands.joystickDriveAssist(drive, () -> driverController.getLeftY(),
            () -> driverController.getLeftX(), () -> -driverController.getRightX() * .85,
            driverController.leftTrigger(), driverController.rightTrigger()));
    outtake.setDefaultCommand(outtake.autoQueueCoral().onlyWhile(elevator.isAtHeight(Level.Intake))
        .withName("RobotContainer.outtakeDefaultCommand"));
    driverController.povUpRight()
        .onTrue(DriveCommands.snapToRotation(drive, Rotation2d.fromDegrees(-45)));
    driverController.povRight()
        .onTrue(DriveCommands.snapToRotation(drive, Rotation2d.fromDegrees(-90)));
    driverController.povDownRight()
        .onTrue(DriveCommands.snapToRotation(drive, Rotation2d.fromDegrees(-135)));
    driverController.povDownLeft()
        .onTrue(DriveCommands.snapToRotation(drive, Rotation2d.fromDegrees(135)));
    driverController.povLeft()
        .onTrue(DriveCommands.snapToRotation(drive, Rotation2d.fromDegrees(90)));
    driverController.povUpLeft()
        .onTrue(DriveCommands.snapToRotation(drive, Rotation2d.fromDegrees(45)));
    driverController.start().onTrue(Commands.runOnce(() -> {
      drive.setPose(new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero));
      CommandScheduler.getInstance().cancelAll(); // clear out any commands that might be stuck
      if (DriverStation.isEnabled()) {
        drive.startIgnoringVision();
      }
    }, drive).ignoringDisable(true).withName("RobotContainer.driverZeroCommand"));
    driverController.rightBumper().whileTrue(DriveCommands.Snapper(drive));
    driverController.leftBumper().whileTrue(DriveCommands.SourceSnapper(drive));

    driverController.x().whileTrue(DriveCommands.AlgaeSnapper(drive));
    driverController.y()
        .whileTrue(DriveCommands.FullSnapperOuter(drive)
            .andThen(DriveCommands.FullSnapperInner(drive).alongWith(elevator.L4()))
            .andThen(outtake.depositCoral()).andThen(elevator.L0()));
    driverController.b()
        .whileTrue(DriveCommands.FullSnapperOuter(drive)
            .andThen(DriveCommands.FullSnapperInner(drive).alongWith(elevator.L3()))
            .andThen(outtake.depositCoral()).andThen(elevator.L0()));
    driverController.a()
        .whileTrue(DriveCommands.FullSnapperOuter(drive)
            .andThen(DriveCommands.FullSnapperInner(drive).alongWith(elevator.L2()))
            .andThen(outtake.depositCoral()).andThen(elevator.L0()));

    driverController.povUp().whileTrue(DriveCommands.BargeSnapper(drive));

    operatorController.leftTrigger().whileTrue(outtake.autoQueueCoralOveride());
    operatorController.rightTrigger().whileTrue(outtake.reverseCoral());
    operatorController.povDown().onTrue(elevator.downLevel());
    operatorController.povUp().onTrue(elevator.upLevel());
    operatorController.start().onTrue(elevator.zeroElevator());
    operatorController.back().onTrue(elevator.zeroElevator());
    operatorController.leftBumper().onTrue(outtake.depositCoral().andThen(elevator.intakeHeight())
        .withName("RobotContainer.intakeHeight"));
    operatorController.rightBumper().onTrue(elevator.intakeHeight());
    operatorController.a().onTrue(elevator.L2());
    operatorController.x().onTrue(elevator.L1());
    operatorController.b().onTrue(elevator.L3());
    operatorController.y().onTrue(elevator.L4());
    operatorController.povRight().whileTrue(algae.setOpenLoop(Volts.of(10)));
    operatorController.povLeft().whileTrue(algae.setOpenLoop(Volts.of(-6)));

    operatorController.axisMagnitudeGreaterThan(1, 0.1)
        .whileTrue(outtake.openLoop(operatorController::getLeftY));

    operatorController.axisMagnitudeGreaterThan(5, 0.1)
        .whileTrue(elevator.openLoop(operatorController::getRightY));

    opponentRobotSim.setDefaultCommand(opponentRobotSim.joystickDrive(opponentController));
    // ##########################################################################################################
  }

  private Command simResetRobot() {
    return Commands.none();
  }

  @Override
  public Command getAutonomousCommand() {
    return autoChooser.get();

  }

  @Override
  public Command getTestCommand() {
    return testChooser.get();
  }

  @Override
  public void disabledInit() {
    drive.stop();
  }

  @Override
  public void teleopInit() {
    drive.stop();
  }

  @Override
  public void disabledPeriodic() {
    var autoCommand = autoChooser.get();

    if (autoCommand instanceof PathPlannerAuto) {
      var auto = (PathPlannerAuto) autoCommand;
      Pose2d startPose = auto.getStartingPose();
      Logger.recordOutput("AutoStartPose", startPose);
    }
  }

  @Override
  public void robotPeriodic() {
    var elevatorPoses = elevator.getElevatorPoses();
    mechanismPoses[0] = elevatorPoses[0];
    mechanismPoses[1] = elevatorPoses[1];
    mechanismPoses[2] = elevatorPoses[2];
  }

  private Transform3d leftCoralTransform, rightCoralTransform;
  private double lastDropTimeSec = 0;
  private Trigger canDropCoral = new Trigger(
      () -> Timer.getFPGATimestamp() - lastDropTimeSec > SimConstants.LOAD_CORAL_DELAY.in(Seconds));

  private Command dropCoral(boolean leftCoral) {
    return Commands.runOnce(() -> {
      lastDropTimeSec = Timer.getFPGATimestamp();
      System.out.println("Dropping " + (leftCoral ? "left" : "right") + " coral");
      Pose3d coralPose;
      if (leftCoral) {
        coralPose = simCoralPoses[1];
        simCoralPoses[1] = SimConstants.HIDDEN_CORAL_POSE;
      } else {
        coralPose = simCoralPoses[2];
        simCoralPoses[2] = SimConstants.HIDDEN_CORAL_POSE;
      }
      var droppedCoral = new ReefscapeCoralOnFly(coralPose.getTranslation().toTranslation2d(),
          Translation2d.kZero, new ChassisSpeeds(), coralPose.getRotation().toRotation2d(),
          coralPose.getMeasureZ().plus(Meters.of(0.07)), MetersPerSecond.of(1.5),
          coralPose.getRotation().getMeasureY().unaryMinus());
      SimulatedArena.getInstance().addGamePieceProjectile(droppedCoral);
    });
  }

  @Override
  public void simulationInit() {
    if (SimConstants.CURRENT_MODE == SimConstants.Mode.SIM) {

      var lb = humanPlayerController.leftBumper();
      var rb = humanPlayerController.rightBumper();

      canDropCoral.whileTrue(Commands.run(() -> {
        leftCoralTransform = new Transform3d(Inches.of(10),
            SimConstants.LOADING_STATION_WIDTH.times(-humanPlayerController.getLeftX() / 2),
            Inches.zero(), Rotation3d.kZero);
        rightCoralTransform = new Transform3d(Inches.of(10),
            SimConstants.LOADING_STATION_WIDTH.times(-humanPlayerController.getRightX() / 2),
            Inches.zero(), Rotation3d.kZero);
        simCoralPoses[1] = SimConstants.LEFT_STATION_CORAL_POSE.plus(leftCoralTransform);
        simCoralPoses[2] = SimConstants.RIGHT_STATION_CORAL_POSE.plus(rightCoralTransform);
      }).ignoringDisable(true));

      lb.and(canDropCoral).onTrue(dropCoral(true));
      rb.and(canDropCoral).onTrue(dropCoral(false));
    }
  }

  @Override
  public void simulationPeriodic() {

    Logger.recordOutput("CanDropCoral", canDropCoral);

    Logger.recordOutput("MechanismPoses", mechanismPoses);

    if (SimConstants.CURRENT_MODE == SimConstants.Mode.SIM) {

      Logger.recordOutput("SimCoralPoses", simCoralPoses);
    }
  }

  @Override
  public void resetSimulation() {
    super.resetSimulation();
    outtake.simStageCoral();
  }
}

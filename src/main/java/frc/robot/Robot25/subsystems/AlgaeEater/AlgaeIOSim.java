package frc.robot.Robot25.subsystems.AlgaeEater;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.Optional;
import java.util.concurrent.CompletableFuture;

import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeAlgaeOnFly;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot25.RobotContainer;
import frc.robot.Robot25.simulation.Gamepieces;
import frc.robot.Robot25.subsystems.elevator.Elevator;
import frc.robot.Robot25.subsystems.elevator.ElevatorConstants;

public class AlgaeIOSim implements AlgaeIO {

  private final AbstractDriveTrainSimulation driveSim = RobotContainer.DRIVE_SIMULATION;

  private static final Transform3d ALGAE_LOADED_POSE =
      new Transform3d(Inches.of(12.49), Inches.zero(), Inches.of(28.278), Rotation3d.kZero);
  private static final Transform3d MANIPULATOR_POSE =
      new Transform3d(Inches.of(15.042), Inches.zero(), Inches.of(22.987), Rotation3d.kZero);
  private static final Distance MAX_ALGAE_DISTANCE = Inches.of(9.3);
  private static final double ALGAE_LOAD_TIME = 0.2;

  private Voltage algaeIntakeAppliedVolts = Volts.of(0.0);

  private boolean hasAlgae = false;
  private String algaeKey = null;
  private double algaeLoadTime = 0;
  private Pose3d grabbedAlgaePose = null;

  /** Updates the set of loggable inputs. */
  @Override
  public void updateInputs(AlgaeIOInputs inputs) {
    var t = (Timer.getFPGATimestamp() - algaeLoadTime) / ALGAE_LOAD_TIME;
    var height =
        Elevator.getInstance().getCurrentHeight().minus(ElevatorConstants.MIN_HEIGHT).in(Meters);
    var algaeLoadedPose = new Pose3d(driveSim.getSimulatedDriveTrainPose()).plus(ALGAE_LOADED_POSE)
        .plus(new Transform3d(0, 0, height, Rotation3d.kZero));
    var manipulatorPose = new Pose3d(driveSim.getSimulatedDriveTrainPose()).plus(MANIPULATOR_POSE)
        .plus(new Transform3d(0, 0, height, Rotation3d.kZero));

    // Logger.recordOutput("Algae/LoadedPose", algaeLoadedPose);
    // Logger.recordOutput("Algae/ManipulatorPose", manipulatorPose);
    if (!hasAlgae) {
      if (algaeIntakeAppliedVolts.lt(Volts.of(-2.0))) {
        grabbableAlgae(manipulatorPose).ifPresent(key -> {
          algaeKey = key;
          hasAlgae = true;
          algaeLoadTime = Timer.getFPGATimestamp();
          grabbedAlgaePose = Gamepieces.removeAlgae(key).get();
        });
      }
    } else if (t < 1) {
      if (grabbedAlgaePose.minus(manipulatorPose).getTranslation().getNorm() > MAX_ALGAE_DISTANCE
          .in(Meters) || algaeIntakeAppliedVolts.gt(Volts.of(-2.0))) {
        hasAlgae = false;
        Gamepieces.removeAlgae("AlgaeEater");
        Gamepieces.setAlgae(algaeKey, grabbedAlgaePose);
      } else {
        var algaePose = grabbedAlgaePose.interpolate(algaeLoadedPose, t);
        Gamepieces.setAlgae("AlgaeEater", algaePose);
      }
    } else {
      Gamepieces.setAlgae("AlgaeEater", algaeLoadedPose);
      if (algaeIntakeAppliedVolts.gt(Volts.of(2.0))) {
        hasAlgae = false;
        var algaeTranslation = ALGAE_LOADED_POSE.getTranslation().toTranslation2d();
        var shootHeight = Elevator.getInstance().getCurrentHeight()
            .plus(MANIPULATOR_POSE.getMeasureZ()).minus(ElevatorConstants.MIN_HEIGHT);
        var projectile =
            new ReefscapeAlgaeOnFly(driveSim.getSimulatedDriveTrainPose().getTranslation(),
                algaeTranslation, driveSim.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                driveSim.getSimulatedDriveTrainPose().getRotation(), shootHeight,
                MetersPerSecond.of(algaeIntakeAppliedVolts.in(Volts) / 3), Degrees.of(5));
        CompletableFuture.runAsync(() -> {
          try {
            Thread.sleep(50);
          } catch (InterruptedException e) {
            e.printStackTrace();
            Thread.currentThread().interrupt();
          } finally {
            Gamepieces.shootAlgae("AlgaeEater", (pose) -> projectile);
          }
        });
      }
    }

    inputs.algaeConnected = true;
    inputs.algaeAppliedVolts = algaeIntakeAppliedVolts;
  }

  /** Run the drive side at the specified open loop value. */
  @Override
  public void setOpenLoop(Voltage output) {
    algaeIntakeAppliedVolts = output;
  }

  private Optional<String> grabbableAlgae(Pose3d manipulatorCenter) {
    double minDistance = MAX_ALGAE_DISTANCE.in(Meters);
    Optional<String> nearestAlgaeKey = Optional.empty();

    for (var entry : Gamepieces.ALGAE_POSES.entrySet()) {
      double distance =
          manipulatorCenter.getTranslation().getDistance(entry.getValue().getTranslation());
      if (distance < minDistance) {
        minDistance = distance;
        nearestAlgaeKey = Optional.of(entry.getKey());
      }
    }

    return nearestAlgaeKey;
  }
}

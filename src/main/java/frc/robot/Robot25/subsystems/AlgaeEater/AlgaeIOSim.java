package frc.robot.Robot25.subsystems.AlgaeEater;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;

import java.util.List;
import java.util.Optional;

import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.SimConstants;
import frc.robot.Robot25.RobotContainer;
import frc.robot.Robot25.subsystems.elevator.Elevator;
import frc.robot.Robot25.subsystems.elevator.ElevatorConstants;

public class AlgaeIOSim implements AlgaeIO {

  private final AbstractDriveTrainSimulation driveSim = RobotContainer.DRIVE_SIMULATION;

  private static final Transform3d ALGAE_LOADED_POSE = new Transform3d(Inches.of(12.49), Inches.of(1.25),
      Inches.of(28.278), Rotation3d.kZero);
  private static final Distance MAX_ALGAE_DISTANCE = Inches.of(12);
  private static final double ALGAE_LOAD_TIME = 0.3;

  private Voltage algaeIntakeAppliedVolts = Volts.of(0.0);

  private boolean hasAlgae = false;
  private double algaeLoadTime = 0;
  private Pose3d grabbedAlgaePose = null;

  /** Updates the set of loggable inputs. */
  @Override
  public void updateInputs(AlgaeIOInputs inputs) {
    var t = (Timer.getFPGATimestamp() - algaeLoadTime) / ALGAE_LOAD_TIME;
    var height = Elevator.getInstance().getCurrentHeight().minus(ElevatorConstants.MIN_HEIGHT).in(Meters);
    var algaeLoadedPose = new Pose3d(driveSim.getSimulatedDriveTrainPose()).plus(ALGAE_LOADED_POSE)
        .plus(new Transform3d(0, 0, height, Rotation3d.kZero));
    var manipulatorCenter = new Pose3d(driveSim.getSimulatedDriveTrainPose()).plus(ALGAE_LOADED_POSE)
        .plus(new Transform3d(0, 0, height, Rotation3d.kZero));
    if (!hasAlgae) {
      if (algaeIntakeAppliedVolts.lt(Volts.of(-2.0))) {
        var algaePoseIndex = nearestAlgae(algaeLoadedPose);
        if (algaePoseIndex.isPresent()) {
          var index = algaePoseIndex.get();
          hasAlgae = true;
          algaeLoadTime = Timer.getFPGATimestamp();
          grabbedAlgaePose = RobotContainer.simAlgaePoses[index];
          RobotContainer.simAlgaePoses[index] = SimConstants.QUEENED_GAMEPIECE_POSE;
        }
      }
    } else if (t < 1) {
      if (grabbedAlgaePose.minus(algaeLoadedPose).getTranslation().getNorm() > MAX_ALGAE_DISTANCE.in(Meters)) {
        hasAlgae = false;
        RobotContainer.simAlgaePoses[0] = SimConstants.QUEENED_GAMEPIECE_POSE;
      } else {
        var algaePose = grabbedAlgaePose.interpolate(algaeLoadedPose, t);
        RobotContainer.simAlgaePoses[0] = algaePose;
      }
    } else {
      RobotContainer.simAlgaePoses[0] = algaeLoadedPose;
    }

    inputs.algaeConnected = true;
    inputs.algaeAppliedVolts = algaeIntakeAppliedVolts;
  }

  /** Run the drive side at the specified open loop value. */
  @Override
  public void setOpenLoop(Voltage output) {
    algaeIntakeAppliedVolts = output;
  }

  private Optional<Integer> grabbableAlgaeIndex(Pose3d manipulatorCenter) {
    double minDistance = MAX_ALGAE_DISTANCE.in(Meters);
    Optional<Integer> nearestPoseIndex = Optional.empty();

    for (int i = 1; i < 7; i++) {
      var p = RobotContainer.simAlgaePoses[i];
      double distance = manipulatorCenter.getTranslation().getDistance(p.getTranslation());
      if (distance < minDistance) {
        minDistance = distance;
        nearestPoseIndex = Optional.of(i);
      }
    }

    return nearestPoseIndex;
  }
}

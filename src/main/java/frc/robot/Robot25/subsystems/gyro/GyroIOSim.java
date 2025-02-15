package frc.robot.Robot25.subsystems.gyro;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.stream.Stream;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.Robot25.util.PhoenixUtil;
import org.ironmaple.simulation.drivesims.GyroSimulation;

public class GyroIOSim implements GyroIO {
  private final GyroSimulation gyroSimulation;

  public GyroIOSim(GyroSimulation gyroSimulation) {
    this.gyroSimulation = gyroSimulation;
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = true;
    inputs.yawPosition = gyroSimulation.getGyroReading();
    inputs.gyroRotation = new Rotation3d(inputs.yawPosition);
    inputs.yawVelocityRadPerSec = Units
        .degreesToRadians(gyroSimulation.getMeasuredAngularVelocity().in(RadiansPerSecond));

    inputs.odometryYawTimestamps = PhoenixUtil.getSimulationOdometryTimeStamps();
    inputs.odometryYawPositions = gyroSimulation.getCachedGyroReadings();
    inputs.odometryRotation3d = Stream.of(inputs.odometryYawPositions).map((r) -> new Rotation3d(r))
        .toArray(Rotation3d[]::new);
  }
}

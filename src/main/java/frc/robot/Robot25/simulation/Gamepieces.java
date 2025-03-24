package frc.robot.Robot25.simulation;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;
import java.util.function.Function;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeAlgaeOnFly;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.SimConstants;
import frc.robot.Robot25.util.Pose2dNearSegment;

public final class Gamepieces {

  public static final Pose3d QUEENED_GAMEPIECE_POSE =
      new Pose3d(-100, -100, -100, new Rotation3d(0, 0, 0));
  public static final Pose3d BLUE_LEFT_STATION_CORAL_POSE = new Pose3d(0.636, 7.693, 1.265,
      new Rotation3d(Degrees.zero(), Degrees.of(35), Degrees.of(-54)));
  public static final Pose3d BLUE_RIGHT_STATION_CORAL_POSE = new Pose3d(0.641, 0.359, 1.280,
      new Rotation3d(Degrees.zero(), Degrees.of(35), Degrees.of(54)));
  public static final Pose3d RED_LEFT_STATION_CORAL_POSE = flipPose3d(BLUE_LEFT_STATION_CORAL_POSE);
  public static final Pose3d RED_RIGHT_STATION_CORAL_POSE =
      flipPose3d(BLUE_RIGHT_STATION_CORAL_POSE);
  public static final Distance LOADING_STATION_WIDTH = Meters.of(1.626);

  public static final Translation2d FR_LOADING_STATION = new Translation2d(1.66, 0.52);
  public static final Translation2d BR_LOADING_STATION = new Translation2d(0.53, 1.38);
  public static final Translation2d FL_LOADING_STATION = new Translation2d(1.66, 7.53);
  public static final Translation2d BL_LOADING_STATION = new Translation2d(0.53, 6.71);
  public static final Distance LOADING_STATION_TOLERANCE = Inches.of(2);
  public static final Time LOAD_CORAL_DELAY = Seconds.of(1);
  public static final Transform3d DEFAULT_LOADING_STATION_CORAL_TRANSFORM =
      new Transform3d(Inches.of(10), Inches.zero(), Inches.zero(), Rotation3d.kZero);

  public static final Pose3d ALGAE_AB_POSE = new Pose3d(3.811, 4.025, 1.313, Rotation3d.kZero);
  public static final Pose3d ALGAE_CD_POSE = new Pose3d(4.151, 3.437, 0.909, Rotation3d.kZero);
  public static final Pose3d ALGAE_EF_POSE = new Pose3d(4.830, 3.437, 1.313, Rotation3d.kZero);
  public static final Pose3d ALGAE_GH_POSE = new Pose3d(5.170, 4.025, 0.909, Rotation3d.kZero);
  public static final Pose3d ALGAE_IJ_POSE = new Pose3d(4.830, 4.613, 1.313, Rotation3d.kZero);
  public static final Pose3d ALGAE_KL_POSE = new Pose3d(4.151, 4.613, 0.909, Rotation3d.kZero);

  public static final Pose3d RED_NET_CENTER = new Pose3d(8.785, 1.906, 2.1, Rotation3d.kZero);
  public static final Pose3d BLUE_NET_CENTER = flipPose3d(RED_NET_CENTER);

  public static final Map<String, Pose3d> CORAL_POSES = new HashMap<>();
  public static final Map<String, Pose3d> ALGAE_POSES = new HashMap<>();

  private static final int HUMAN_PLAYER_PORT = 5;
  private static final CommandXboxController humanPlayerController =
      new CommandXboxController(HUMAN_PLAYER_PORT);

  private static double lastCoralDropTimeSec = 0;

  @AutoLogOutput
  private static Trigger canDropCoral = new Trigger(
      () -> Timer.getFPGATimestamp() - lastCoralDropTimeSec > LOAD_CORAL_DELAY.in(Seconds));

  static {
    if (SimConstants.CURRENT_MODE != SimConstants.Mode.SIM) {
      throw new IllegalStateException("Simulation mode is not enabled");
    }

    var lb = humanPlayerController.leftBumper();
    var rb = humanPlayerController.rightBumper();

    lb.and(canDropCoral).onTrue(dropCoral(true));
    rb.and(canDropCoral).onTrue(dropCoral(false));
  }

  public static void addCoral(String key, Pose3d pose) {
    if (CORAL_POSES.containsKey(key)) {
      throw new IllegalArgumentException("Coral already exists");
    }

    CORAL_POSES.put(key, pose);
  }

  public static void setCoral(String key, Pose3d pose) {
    CORAL_POSES.put(key, pose);
  }

  public static Optional<Pose3d> removeCoral(String key) {
    var pose = CORAL_POSES.remove(key);
    return Optional.ofNullable(pose);
  }

  public static void shootCoral(String key,
      Function<Pose3d, ReefscapeCoralOnFly> projectileSupplier) {
    if (!CORAL_POSES.containsKey(key)) {
      throw new IllegalArgumentException("Coral does not exist");
    }

    var pose = CORAL_POSES.remove(key);
    SimulatedArena.getInstance().addGamePieceProjectile(projectileSupplier.apply(pose));
  }

  public static void addAlgae(String key, Pose3d pose) {
    if (ALGAE_POSES.containsKey(key)) {
      throw new IllegalArgumentException("Algae already exists");
    }

    ALGAE_POSES.put(key, pose);
  }

  public static void setAlgae(String key, Pose3d pose) {
    ALGAE_POSES.put(key, pose);
  }

  public static Optional<Pose3d> removeAlgae(String key) {
    var pose = ALGAE_POSES.remove(key);
    return Optional.ofNullable(pose);
  }

  public static void shootAlgae(String key,
      Function<Pose3d, ReefscapeAlgaeOnFly> projectileSupplier) {
    if (!ALGAE_POSES.containsKey(key)) {
      throw new IllegalArgumentException("Algae does not exist");
    }

    class PoseContainer {
      Pose3d pose;
    }

    var scoredKey = String.format("Scored@%.1f", Timer.getFPGATimestamp());
    var pose = ALGAE_POSES.remove(key);
    PoseContainer scoredPose = new PoseContainer();
    SimulatedArena.getInstance().addGamePieceProjectile(
        projectileSupplier.apply(pose).withProjectileTrajectoryDisplayCallBack((poses) -> {
          if (poses.size() > 0) {
            scoredPose.pose = poses.getLast();
          }
        }, (poses) -> {
        }).withHitTargetCallBack(() -> Gamepieces.setAlgae(scoredKey, scoredPose.pose))
            .withTargetPosition(
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                    ? BLUE_NET_CENTER.getTranslation()
                    : RED_NET_CENTER.getTranslation()));
  }

  public static void resetField() {
    CORAL_POSES.clear();
    ALGAE_POSES.clear();
    SimulatedArena.getInstance().resetFieldForAuto();
    stageAlgaeOnReef();
  }

  public static void stageAlgaeOnReef() {
    setAlgae("Blue_ReefAB", ALGAE_AB_POSE);
    setAlgae("Red_ReefAB", flipPose3d(ALGAE_AB_POSE));
    setAlgae("Blue_ReefCD", ALGAE_CD_POSE);
    setAlgae("Red_ReefCD", flipPose3d(ALGAE_CD_POSE));
    setAlgae("Blue_ReefEF", ALGAE_EF_POSE);
    setAlgae("Red_ReefEF", flipPose3d(ALGAE_EF_POSE));
    setAlgae("Blue_ReefGH", ALGAE_GH_POSE);
    setAlgae("Red_ReefGH", flipPose3d(ALGAE_GH_POSE));
    setAlgae("Blue_ReefIJ", ALGAE_IJ_POSE);
    setAlgae("Red_ReefIJ", flipPose3d(ALGAE_IJ_POSE));
    setAlgae("Blue_ReefKL", ALGAE_KL_POSE);
    setAlgae("Red_ReefKL", flipPose3d(ALGAE_KL_POSE));
  }

  public static boolean isNearLoadingStation(Pose2d drivePose) {
    var backRightLoadingStation =
        DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? BR_LOADING_STATION
            : FlippingUtil.flipFieldPosition(BR_LOADING_STATION);
    var frontRightLoadingStation =
        DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? FR_LOADING_STATION
            : FlippingUtil.flipFieldPosition(FR_LOADING_STATION);
    var backLeftLoadingStation =
        DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? BL_LOADING_STATION
            : FlippingUtil.flipFieldPosition(BL_LOADING_STATION);
    var frontLeftLoadingStation =
        DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? FL_LOADING_STATION
            : FlippingUtil.flipFieldPosition(FL_LOADING_STATION);
    return Pose2dNearSegment.isNearSegment(drivePose, backRightLoadingStation,
        frontRightLoadingStation, LOADING_STATION_TOLERANCE)
        || Pose2dNearSegment.isNearSegment(drivePose, backLeftLoadingStation,
            frontLeftLoadingStation, LOADING_STATION_TOLERANCE);
  }

  private static Transform3d leftCoralTransform, rightCoralTransform;

  public static void periodic() {
    if (hasHumanPlayer()) {
      leftCoralTransform = new Transform3d(Inches.of(10),
          LOADING_STATION_WIDTH.times(-humanPlayerController.getLeftX() / 2), Inches.zero(),
          Rotation3d.kZero);
      rightCoralTransform = new Transform3d(Inches.of(10),
          LOADING_STATION_WIDTH.times(-humanPlayerController.getRightX() / 2), Inches.zero(),
          Rotation3d.kZero);
    } else {
      leftCoralTransform = DEFAULT_LOADING_STATION_CORAL_TRANSFORM;
      rightCoralTransform = DEFAULT_LOADING_STATION_CORAL_TRANSFORM;
    }

    var leftStationCoralPose = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
        ? BLUE_LEFT_STATION_CORAL_POSE
        : RED_LEFT_STATION_CORAL_POSE;
    var rightStationCoralPose = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
        ? BLUE_RIGHT_STATION_CORAL_POSE
        : RED_RIGHT_STATION_CORAL_POSE;

    setCoral("Left_LoadingStation", leftStationCoralPose.plus(leftCoralTransform));
    setCoral("Right_LoadingStation", rightStationCoralPose.plus(rightCoralTransform));

    Logger.recordOutput("FieldSimulation/Coral", getCoralPoses());
    Logger.recordOutput("FieldSimulation/Algae", getAlgaePoses());
  }

  private static Command dropCoral(boolean leftCoral) {
    return Commands.runOnce(() -> {
      lastCoralDropTimeSec = Timer.getFPGATimestamp();
      System.out.println("Dropping " + (leftCoral ? "left" : "right") + " coral");
      var key = leftCoral ? "Left_LoadingStation" : "Right_LoadingStation";
      shootCoral(key,
          (coralPose) -> new ReefscapeCoralOnFly(coralPose.getTranslation().toTranslation2d(),
              Translation2d.kZero, new ChassisSpeeds(), coralPose.getRotation().toRotation2d(),
              coralPose.getMeasureZ().plus(Meters.of(0.07)), MetersPerSecond.of(1),
              coralPose.getRotation().getMeasureY().unaryMinus()));
    });
  }

  private static Pose3d[] getCoralPoses() {
    var simGamePieces = new ArrayList<Pose3d>();
    simGamePieces
        .addAll(Arrays.asList(SimulatedArena.getInstance().getGamePiecesArrayByType("Coral")));
    simGamePieces.addAll(CORAL_POSES.values());
    return simGamePieces.toArray(new Pose3d[simGamePieces.size()]);
  }

  public static Pose3d[] getAlgaePoses() {
    var simGamePieces = new ArrayList<Pose3d>();
    simGamePieces
        .addAll(Arrays.asList(SimulatedArena.getInstance().getGamePiecesArrayByType("Algae")));
    simGamePieces.addAll(ALGAE_POSES.values());
    return simGamePieces.toArray(new Pose3d[simGamePieces.size()]);
  }

  private static Pose3d flipPose3d(Pose3d pose) {
    var t2d = FlippingUtil.flipFieldPosition(pose.getTranslation().toTranslation2d());
    var r3d = pose.getRotation();
    var r2d = FlippingUtil.flipFieldRotation(pose.getRotation().toRotation2d());
    return new Pose3d(t2d.getX(), t2d.getY(), pose.getZ(),
        new Rotation3d(r3d.getX(), r3d.getY(), r2d.getRadians()));
  }

  @AutoLogOutput
  public static boolean hasHumanPlayer() {
    var joystickName = DriverStation.getJoystickName(HUMAN_PLAYER_PORT);
    return joystickName != null && !joystickName.isBlank();
  }
}

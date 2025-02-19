package org.texastorque.subsystems;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.texastorque.AlignPose2d;
import org.texastorque.AlignPose2d.Relation;
import org.texastorque.AprilTagList;
import org.texastorque.LimelightHelpers;
import org.texastorque.LimelightHelpers.PoseEstimate;
import org.texastorque.LimelightHelpers.RawFiducial;
import org.texastorque.Subsystems;
import org.texastorque.torquelib.Debug;
import org.texastorque.torquelib.base.TorqueMode;
import org.texastorque.torquelib.base.TorqueStatelessSubsystem;
import org.texastorque.torquelib.control.TorqueFieldZone;
import org.texastorque.torquelib.control.TorqueRollingMedian;
import org.texastorque.torquelib.sensors.TorqueNavXGyro;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public class Perception extends TorqueStatelessSubsystem implements Subsystems {
	private static volatile Perception instance;

	private final SwerveDrivePoseEstimator poseEstimator;

	/**
	 * Standard deviations of model states. Increase these numbers to trust your
	 * model's state estimates less. This matrix is in the form [x, y, theta]ᵀ,
	 * with units in meters and radians, then meters.
	 */
	private static final Vector<N3> ODOMETRY_STDS = VecBuilder.fill(.05, .05, Units.degreesToRadians(2));

	/**
	 * Standard deviations of the vision measurements. Increase these numbers to
	 * trust global measurements from vision less. This matrix is in the form
	 * [x, y, theta]ᵀ, with units in meters and radians.
	 */
	private static final Vector<N3> VISION_STDS = VecBuilder.fill(.1, .1, Units.degreesToRadians(1));

	private final TorqueNavXGyro gyro = TorqueNavXGyro.getInstance();
	private double gyro_simulated = 0;

	// Used to filter some noise directly out of the pose measurements.
	private final TorqueRollingMedian filteredX, filteredY;
	private Pose2d finalPose = new Pose2d();

	private final Field2d field = new Field2d();
	private ArrayList<TorqueFieldZone> zones;
	private RawFiducial lastDetection;
	
	public Perception() {
		LimelightHelpers.setCameraPose_RobotSpace(LIMELIGHT_HIGH, -0.150752, -0.105425, 0.77653, -90, 45, 180);
		LimelightHelpers.setCameraPose_RobotSpace(LIMELIGHT_LOW, 0.0916686, 0.127, 0.164267, 0, 25, 0);

		poseEstimator = new SwerveDrivePoseEstimator(drivebase.kinematics, getHeading(), drivebase.getModulePositions(), new Pose2d(), ODOMETRY_STDS, VISION_STDS);

		filteredX = new TorqueRollingMedian(15);
		filteredY = new TorqueRollingMedian(15);

		Debug.field("Field", field);

		createZones();
	}

	final String LIMELIGHT_HIGH = "limelight-high";
	final String LIMELIGHT_LOW = "limelight-low";

	@Override
	public void initialize(TorqueMode mode) {}

	@Override
	public void update(TorqueMode mode) {
		gyro_simulated += drivebase.inputSpeeds.omegaRadiansPerSecond / 180 * Math.PI;
		gyro_simulated %= 2 * Math.PI;

		LimelightHelpers.SetRobotOrientation(LIMELIGHT_HIGH, getHeading().getDegrees(), 0, 0, 0, 0, 0);
		LimelightHelpers.SetRobotOrientation(LIMELIGHT_LOW, getHeading().getDegrees(), 0, 0, 0, 0, 0);
		LimelightHelpers.PoseEstimate visionEstimateHigh = getVisionEstimate(LIMELIGHT_HIGH);
		LimelightHelpers.PoseEstimate visionEstimateLow = getVisionEstimate(LIMELIGHT_LOW);
		
		final Pose2d odometryPose = poseEstimator.update(getHeading(), drivebase.getModulePositions());

		if (seesTag() && visionEstimateHigh != null && visionEstimateLow != null) {
			if (visionEstimateHigh.tagCount > 0) {
				if ((drivebase.getState() == Drivebase.State.ALIGN && containsAnyID(visionEstimateHigh.rawFiducials, 16, 13, 12, 3, 2, 1))
					|| (drivebase.getState() == Drivebase.State.PATHING && visionEstimateHigh.avgTagDist > 1)
					|| visionEstimateLow.avgTagDist > 1.5) {
						// disregard
				} else {
					poseEstimator.addVisionMeasurement(visionEstimateHigh.pose, visionEstimateHigh.timestampSeconds);
				}
			}
			if (visionEstimateLow.tagCount > 0) {
				if ((drivebase.getState() == Drivebase.State.ALIGN && containsAnyID(visionEstimateLow.rawFiducials, 16, 13, 12, 3, 2, 1))
					|| (drivebase.getState() == Drivebase.State.PATHING && visionEstimateLow.avgTagDist > 1)
					|| visionEstimateLow.avgTagDist > 1.5) {
						// disregard
				} else {
					poseEstimator.addVisionMeasurement(visionEstimateLow.pose, visionEstimateLow.timestampSeconds);
				}
			}

			finalPose = new Pose2d(
				filteredX.calculate(poseEstimator.getEstimatedPosition().getX()),
				filteredY.calculate(poseEstimator.getEstimatedPosition().getY()),
				getHeading()
			);
		} else {
			finalPose = odometryPose;
		}

		field.setRobotPose(finalPose);

		Debug.log("Current Pose", getPose().toString());
		Debug.log("Sees Tag", seesTag());

		// Simulation poses
		Logger.recordOutput("Robot Pose", getPose());
		Logger.recordOutput("Animated Component Poses", new Pose3d[] {
			new Pose3d(0, 0, Math.sin(Timer.getTimestamp() % Math.PI) / 4, new Rotation3d()),
			new Pose3d(0, 0, Math.sin(Timer.getTimestamp() % Math.PI) / 3, new Rotation3d()),
			new Pose3d(0, 0, Math.sin(Timer.getTimestamp() % Math.PI) / 2, new Rotation3d()),
			new Pose3d(.109, 0, .578 + Math.sin(Timer.getTimestamp() % Math.PI) / 2, new Rotation3d(0, Math.sin(Timer.getTimestamp()) - 1, 0))
		});
		Logger.recordOutput("Zeroed Component Poses", new Pose3d[] {
			new Pose3d(),
			new Pose3d(),
			new Pose3d(),
			new Pose3d()
		});
		Logger.recordOutput("Real Component Poses", getRealComponentPoses());

		for (TorqueFieldZone zone : zones) {
			Logger.recordOutput("Zone ID " + zone.getID(), zone.getPolygon());
		}
	}

	@Override
	public void clean(TorqueMode mode) {}

	public Pose3d[] getRealComponentPoses() {
		final double elevatorPos = elevator.getElevatorPosition();
		final double shoulderAngle = claw.getShoulderAngle();
		final double elevatorMultiplier = elevatorPos / 12.5;
		
		return new Pose3d[] {
			new Pose3d(0, 0, .6 * elevatorMultiplier, new Rotation3d()),
			new Pose3d(0, 0, 1.25 * elevatorMultiplier, new Rotation3d()),
			new Pose3d(0, 0, 1.74 * elevatorMultiplier, new Rotation3d()),
			new Pose3d(.109, .02, .278 + (1.74 * elevatorMultiplier), new Rotation3d(0, Math.toRadians((shoulderAngle + 180 + 360) % 360), 0))
		};
	}

	public boolean containsID(final RawFiducial[] rawFiducials, final int id) {
		for (RawFiducial fiducial : rawFiducials) {
			if (fiducial.id == id) {
				return true;
			}
		}
		return false;
	}

	public boolean containsAnyID(final RawFiducial[] rawFiducials, final int ...ids) {
		for (int id : ids) {
			if (containsID(rawFiducials, id)) {
				return true;
			}
		}
		return false;
	}

	public RawFiducial getAlignDetection() {
		// Get best apriltag detection (closest to center of frame)
		final PoseEstimate estimate = getVisionEstimate(LIMELIGHT_HIGH);
		if (estimate == null) return lastDetection;
		if (estimate.rawFiducials.length == 0) return lastDetection;
		
		final List<RawFiducial> detections = Arrays.asList(estimate.rawFiducials);
		RawFiducial bestDetection = detections.get(0);

		for (RawFiducial raw : detections) {
			if (Math.abs(raw.txnc) < Math.abs(bestDetection.txnc)) {
				bestDetection = raw;
			}
		}
		lastDetection = bestDetection;
		return bestDetection;
	}

	public Rotation2d getHeading() {
		if (RobotBase.isSimulation()) {
			return Rotation2d.fromRadians(gyro_simulated);
		}
		if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
			return gyro.getHeadingCCW();
		}
		return gyro.getHeadingCCW();
	}

	public Optional<Pose2d> getAlignPose(final Pose2d currentPose, final Relation relation) {
		if (relation == null) return Optional.empty();
		TorqueFieldZone currentZone = null;
		for (TorqueFieldZone zone : zones) {
			if (zone.contains(currentPose)) {
				currentZone = zone;
			}
		}

		if (currentZone == null) return Optional.empty();
		final int id = currentZone.getID();

		final AlignPose2d[] alignPoses = AprilTagList.values()[id - 1].alignPoses;
		for (AlignPose2d alignPose : alignPoses) {
			if (alignPose.getRelation() == relation) {
				return Optional.of(alignPose.getPose());
			}
		}
		return Optional.empty();
	}

	public void resetHeading() {
		gyro_simulated = 0;

		final boolean isRedAlliance = DriverStation.getAlliance().isPresent()
                    ? DriverStation.getAlliance().get() == Alliance.Red
                    : false;
		gyro.setOffsetCW(Rotation2d.fromDegrees(isRedAlliance ? 180 : 0));
		setPose(new Pose2d(0, 0, getHeading()));
	}

	public void setCurrentTrajectory(final Trajectory trajectory) {
		field.getObject("trajectory").setTrajectory(trajectory);
		Logger.recordOutput("Auto Trajectory", trajectory);
	}

	public boolean seesTag() {
		return LimelightHelpers.getTargetCount(LIMELIGHT_HIGH) > 0 || LimelightHelpers.getTargetCount(LIMELIGHT_LOW) > 0;
	}

	public void setPose(final Pose2d pose) {
		poseEstimator.resetPosition(getHeading(), drivebase.getModulePositions(), pose);
	}

	private PoseEstimate getVisionEstimate(final String limelightName) {
		return LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
	}

	public Pose2d getPose() {
		return finalPose;
	}

	public void createZones() {
		final double allianceOffset = 8.56957565;
		final Translation2d centerBlue = new Translation2d(4.5, 4.0259);
		final Translation2d rightBlue = new Translation2d(4.5, 4.0259 - 3);
		final Translation2d farRightBlue = new Translation2d(4.5 + 2.5980644, 4.0259 - 1.5);
		final Translation2d farLeftBlue = new Translation2d(4.5 + 2.5980644, 4.0259 + 1.5);
		final Translation2d leftBlue = new Translation2d(4.5, 4.0259 + 3);
		final Translation2d closeRightBlue = new Translation2d(4.5 - 2.5980644, 4.0259 - 1.5);
		final Translation2d closeLeftBlue = new Translation2d(4.5 - 2.5980644, 4.0259 + 1.5);

		final Translation2d centerRed = new Translation2d(4.5 + allianceOffset, 4.0259);
		final Translation2d leftRed = new Translation2d(4.5 + allianceOffset, 4.0259 - 3);
		final Translation2d closeLeftRed = new Translation2d(4.5 + allianceOffset + 2.5980644, 4.0259 - 1.5);
		final Translation2d closeRightRed = new Translation2d(4.5 + 2.5980644 + allianceOffset, 4.0259 + 1.5);
		final Translation2d rightRed = new Translation2d(4.5 + allianceOffset, 4.0259 + 3);
		final Translation2d farLeftRed = new Translation2d(4.5 - 2.5980644 + allianceOffset, 4.0259 - 1.5);
		final Translation2d farRightRed = new Translation2d(4.5 - 2.5980644 + allianceOffset, 4.0259 + 1.5);

		final Translation2d csLeftBackLeftBlue = new Translation2d(0, 1.194053);
		final Translation2d csLeftBackRightBlue = new Translation2d(1.644160, 0);
		final Translation2d csLeftFrontLeftBlue = new Translation2d(0, 3.429942);
		final Translation2d csLeftFrontRightBlue = new Translation2d(4.345927, 0);

		final Translation2d csRightBackLeftBlue = new Translation2d(0, 6.857747);
		final Translation2d csRightBackRightBlue = new Translation2d(1.644160, 8.0518);
		final Translation2d csRightFrontLeftBlue = new Translation2d(0, 4.621858);
		final Translation2d csRightFrontRightBlue = new Translation2d(4.345927, 8.0518);

		final Translation2d csLeftBackLeftRed = new Translation2d(17.548249, 1.194053);
		final Translation2d csLeftBackRightRed = new Translation2d(17.548249 - 1.644160, 0);
		final Translation2d csLeftFrontLeftRed = new Translation2d(17.548249, 3.429942);
		final Translation2d csLeftFrontRightRed = new Translation2d(17.548249 - 4.345927, 0);

		final Translation2d csRightBackLeftRed = new Translation2d(17.548249, 6.857747);
		final Translation2d csRightBackRightRed = new Translation2d(17.548249 - 1.644160, 8.0518);
		final Translation2d csRightFrontLeftRed = new Translation2d(17.548249, 4.621858);
		final Translation2d csRightFrontRightRed = new Translation2d(17.548249 - 4.345927, 8.0518);

		zones = new ArrayList<>();

        zones.add(new TorqueFieldZone(18, centerBlue, closeLeftBlue, closeRightBlue, centerBlue));
        zones.add(new TorqueFieldZone(17, centerBlue, closeRightBlue, rightBlue, centerBlue));
        zones.add(new TorqueFieldZone(22, centerBlue, rightBlue, farRightBlue, centerBlue));
        zones.add(new TorqueFieldZone(21, centerBlue, farRightBlue, farLeftBlue, centerBlue));
        zones.add(new TorqueFieldZone(20, centerBlue, farLeftBlue, leftBlue, centerBlue));
        zones.add(new TorqueFieldZone(19, centerBlue, leftBlue, closeLeftBlue, centerBlue));

        zones.add(new TorqueFieldZone(7, centerRed, closeLeftRed, closeRightRed, centerRed));
        zones.add(new TorqueFieldZone(6, centerRed, closeLeftRed, leftRed, centerRed));
        zones.add(new TorqueFieldZone(11, centerRed, leftRed, farLeftRed, centerRed));
        zones.add(new TorqueFieldZone(10, centerRed, farLeftRed, farRightRed, centerRed));
        zones.add(new TorqueFieldZone(9, centerRed, farRightRed, rightRed, centerRed));
        zones.add(new TorqueFieldZone(8, centerRed, rightRed, closeRightRed, centerRed));

		zones.add(new TorqueFieldZone(12, csLeftBackLeftBlue, csLeftBackRightBlue, csLeftFrontRightBlue, csLeftFrontLeftBlue, csLeftBackLeftBlue));
		zones.add(new TorqueFieldZone(13, csRightBackLeftBlue, csRightBackRightBlue, csRightFrontRightBlue, csRightFrontLeftBlue, csRightBackLeftBlue));

		zones.add(new TorqueFieldZone(1, csLeftBackLeftRed, csLeftBackRightRed, csLeftFrontRightRed, csLeftFrontLeftRed, csLeftBackLeftRed));
		zones.add(new TorqueFieldZone(2, csRightBackLeftRed, csRightBackRightRed, csRightFrontRightRed, csRightFrontLeftRed, csRightBackLeftRed));
	}

	public static Perception getInstance() {
		return instance == null ? instance = new Perception() : instance;
	}
}

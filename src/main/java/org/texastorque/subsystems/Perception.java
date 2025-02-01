package org.texastorque.subsystems;

import java.util.Arrays;
import java.util.List;
import java.util.Optional;

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
import org.texastorque.torquelib.control.TorqueRollingMedian;
import org.texastorque.torquelib.sensors.TorqueNavXGyro;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
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
	private Pose2d finalPose;

	private final Field2d field = new Field2d();

	public Perception() {
		LimelightHelpers.setCameraPose_RobotSpace(LIMELIGHT_TOP, -0.150752, -0.105425, 0.77653, 90, 45, 180);
		LimelightHelpers.setCameraPose_RobotSpace(LIMELIGHT_BOTTOM, 0.298645, 0.127, 0.164267, 0, 25, 0);

		poseEstimator = new SwerveDrivePoseEstimator(drivebase.kinematics, getHeading(), drivebase.getModulePositions(), new Pose2d(), ODOMETRY_STDS, VISION_STDS);

		filteredX = new TorqueRollingMedian(5);
		filteredY = new TorqueRollingMedian(5);

		Debug.field("Field", field);
	}

	final String LIMELIGHT_TOP = "limelight-top";
	final String LIMELIGHT_BOTTOM = "limelight-bottom";

	@Override
	public void initialize(TorqueMode mode) {}

	@Override
	public void update(TorqueMode mode) {
		gyro_simulated += drivebase.inputSpeeds.omegaRadiansPerSecond / 180 * Math.PI;
		gyro_simulated %= 2 * Math.PI;

		LimelightHelpers.SetRobotOrientation(LIMELIGHT_TOP, getHeading().getDegrees(), 0, 0, 0, 0, 0);
		LimelightHelpers.SetRobotOrientation(LIMELIGHT_BOTTOM, getHeading().getDegrees(), 0, 0, 0, 0, 0);
		LimelightHelpers.PoseEstimate visionEstimateTop = getVisionEstimate(LIMELIGHT_TOP);
		LimelightHelpers.PoseEstimate visionEstimateBottom = getVisionEstimate(LIMELIGHT_BOTTOM);
		
		final Pose2d odometryPose = poseEstimator.update(getHeading(), drivebase.getModulePositions());
		final Pose2d visionPose = getFusedVisionPose(visionEstimateTop, visionEstimateBottom);

		if (visionPose != null) {
			if (seesTag()) {
				poseEstimator.addVisionMeasurement(visionPose, visionEstimateTop.timestampSeconds);
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

		Debug.log("Current Pose", finalPose.toString());
		Debug.log("Sees Tag", seesTag());
	}

	@Override
	public void clean(TorqueMode mode) {}

	public void resetHeading() {
		gyro_simulated = 0;
        gyro.setOffsetCW(Rotation2d.fromRadians(0));
        setPose(new Pose2d(0, 0, getHeading()));
    }

	public void setPose(Pose2d pose) {
        poseEstimator.resetPosition(getHeading(), drivebase.getModulePositions(), pose);
    }

	private PoseEstimate getVisionEstimate(final String limelightName) {
		return LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
	}
	
	private Pose2d getFusedVisionPose(final PoseEstimate top, final PoseEstimate bottom) {
		final Pose2d pastPose = getPose();

		// Pose filtering
		if (top == null && bottom != null) return bottom.pose;
		if (bottom == null && top != null) return top.pose;
		if (bottom == null && top == null) return null;

		// If the the two poses are more than a half-meter away from each other, disregard both
		if (top.pose.getTranslation().getDistance(bottom.pose.getTranslation()) > .5) {
			return pastPose;
		}

		// If not, return average pose
		final Pose2d fusedPose = new Pose2d(
			(top.pose.getX() + bottom.pose.getX()) / 2,
			(top.pose.getY() + bottom.pose.getY()) / 2,
			getHeading()
		);
		return fusedPose;
	}

	private boolean seesTag() {
		return LimelightHelpers.getTargetCount(LIMELIGHT_TOP) > 0 || LimelightHelpers.getTargetCount(LIMELIGHT_BOTTOM) > 0;
	}

	public Rotation2d getHeading() {
		if (RobotBase.isSimulation()) {
			return Rotation2d.fromRadians(gyro_simulated);
		}
		return gyro.getHeadingCCW();
	}

	public Pose2d getPose() {
		return finalPose;
	}

	public RawFiducial getAlignDetection() {
		// Get best apriltag detection (closest to center of frame)
		final PoseEstimate estimate = getVisionEstimate(LIMELIGHT_TOP); // Whichever limelight is facing towards where we score
		if (estimate.rawFiducials.length == 0) return null;
		
		final List<RawFiducial> detections = Arrays.asList(estimate.rawFiducials);
		RawFiducial bestDetection = detections.get(0);

		for (RawFiducial raw : detections) {
			if (Math.abs(raw.txnc) < Math.abs(bestDetection.txnc)) {
				bestDetection = raw;
			}
		}
		return bestDetection;
	}

	public Optional<Pose2d> getAlignPose(final Relation relation) {
		final RawFiducial bestDetection = getAlignDetection();
		if (bestDetection == null) return Optional.empty();
		
		final AlignPose2d[] alignPoses = AprilTagList.values()[bestDetection.id - 1].alignPoses;
		for (AlignPose2d alignPose : alignPoses) {
			if (alignPose.getRelation() == relation) {
				return Optional.of(alignPose.getPose());
			}
		}
		return Optional.empty();
	}

	public static Perception getInstance() {
		return instance == null ? instance = new Perception() : instance;
	}
}

package org.texastorque.subsystems;

import java.util.Arrays;
import java.util.List;
import java.util.Optional;

import org.texastorque.AprilTagList.AlignPose2d;
import org.texastorque.AprilTagList.AlignPose2d.Relation;
import org.texastorque.AprilTagList;
import org.texastorque.LimeLightHelpers;
import org.texastorque.LimeLightHelpers.PoseEstimate;
import org.texastorque.LimeLightHelpers.RawFiducial;
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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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

	private final TorqueFieldZone zone = new TorqueFieldZone(new Translation2d(), new Translation2d(1, 1));

	public Perception() {
		poseEstimator = new SwerveDrivePoseEstimator(drivebase.kinematics, getHeading(), drivebase.getModulePositions(), new Pose2d(), ODOMETRY_STDS, VISION_STDS);

		filteredX = new TorqueRollingMedian(5);
		filteredY = new TorqueRollingMedian(5);

		Debug.field("Field", field);
	}

	final String LIMELIGHT = "limelight"; // 'limelight' - The name for the LL3G that is used for pose estimation
	final String LIMELIGHT_AI = "limelight-ai"; // 'limelight-ai' - The name for the LL3 that is used for note detection

	@Override
	public void initialize(TorqueMode mode) {}

	@Override
	public void update(TorqueMode mode) {
		gyro_simulated += drivebase.inputSpeeds.omegaRadiansPerSecond / 180 * Math.PI;
		gyro_simulated %= 2 * Math.PI;

		LimeLightHelpers.SetRobotOrientation(LIMELIGHT, getHeading().getDegrees(), 0, 0, 0, 0, 0);
		LimeLightHelpers.PoseEstimate visionEstimate = getVisionEstimate();
		
		final Pose2d estimatedPose = poseEstimator.update(getHeading(), drivebase.getModulePositions());
		finalPose = estimatedPose;

		if (visionEstimate != null) {
			if (visionEstimate.tagCount > 0) {
				poseEstimator.addVisionMeasurement(visionEstimate.pose, visionEstimate.timestampSeconds);
			}

			finalPose = new Pose2d(
					filteredX.calculate(estimatedPose.getX()),
					filteredY.calculate(estimatedPose.getY()),
					getHeading()
			);
			
			SmartDashboard.putBoolean("Sees Tag", LimeLightHelpers.getTargetCount(LIMELIGHT) > 0);
		}
		field.setRobotPose(finalPose);

		// final int notesDetected = LimeLightHelpers.getTargetCount(LIMELIGHT_AI);

		SmartDashboard.putBoolean("In Zone", zone.contains(finalPose));
		SmartDashboard.putString("Current Pose", finalPose.toString());
		// SmartDashboard.putNumber("Notes Detected", notesDetected);
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

	private PoseEstimate getVisionEstimate() {
		return LimeLightHelpers.getBotPoseEstimate_wpiBlue("limelight");
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

	public Optional<Pose2d> getAlignPose(final Relation relation) {
		// Get best apriltag detection (closest to center of frame)
		final PoseEstimate estimate = getVisionEstimate();
		if (estimate.rawFiducials.length == 0) return Optional.empty();
		
		final List<RawFiducial> detections = Arrays.asList(estimate.rawFiducials);
		RawFiducial bestDetection = detections.get(0);

		for (RawFiducial raw : detections) {
			if (Math.abs(raw.txnc) < Math.abs(bestDetection.txnc)) {
				bestDetection = raw;
			}
		}
		
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

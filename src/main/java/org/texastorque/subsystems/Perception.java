package org.texastorque.subsystems;

import java.util.ArrayList;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.texastorque.AlignPose2d;
import org.texastorque.AlignPose2d.Relation;
import org.texastorque.AprilTagList;
import org.texastorque.LimelightHelpers;
import org.texastorque.LimelightHelpers.PoseEstimate;
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
	private Pose2d finalPose;

	private final Field2d field = new Field2d();
	private final ArrayList<TorqueFieldZone> zones;

	public Perception() {
		LimelightHelpers.setCameraPose_RobotSpace(LIMELIGHT_HIGH, -0.150752, -0.105425, 0.77653, 90, 45, 180);
		LimelightHelpers.setCameraPose_RobotSpace(LIMELIGHT_LOW, 0.298645, 0.127, 0.164267, 0, 25, 0);

		poseEstimator = new SwerveDrivePoseEstimator(drivebase.kinematics, getHeading(), drivebase.getModulePositions(), new Pose2d(), ODOMETRY_STDS, VISION_STDS);

		filteredX = new TorqueRollingMedian(5);
		filteredY = new TorqueRollingMedian(5);

		Debug.field("Field", field);

		final boolean isRedAlliance = (DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() == Alliance.Red : false); // ?? This doesn't work...
		final double allianceOffset = isRedAlliance ? 8.56957565 : 0;
		final Translation2d center = new Translation2d(4.5 + allianceOffset, 4.0259);
        final Translation2d right = new Translation2d(4.5 + allianceOffset, 4.0259 - 3);
        final Translation2d farRight = new Translation2d(4.5 + allianceOffset + 2.5980644, 4.0259 - 1.5);
        final Translation2d farLeft = new Translation2d(4.5 + 2.5980644 + allianceOffset, 4.0259 + 1.5);
        final Translation2d left = new Translation2d(4.5 + allianceOffset, 4.0259 + 3);
        final Translation2d closeRight = new Translation2d(4.5 - 2.5980644 + allianceOffset, 4.0259 - 1.5);
        final Translation2d closeLeft = new Translation2d(4.5 - 2.5980644 + allianceOffset, 4.0259 + 1.5);

        zones = new ArrayList<>();

        zones.add(new TorqueFieldZone(18, center, closeLeft, closeRight));
        zones.add(new TorqueFieldZone(17, center, closeRight, right));
        zones.add(new TorqueFieldZone(22, center, right, farRight));
        zones.add(new TorqueFieldZone(21, center, farRight, farLeft));
        zones.add(new TorqueFieldZone(20, center, farLeft, left));
        zones.add(new TorqueFieldZone(19, center, left, closeLeft));
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
		final Pose2d visionPose = getFusedVisionPose(visionEstimateHigh, visionEstimateLow);

		if (visionPose != null) {
			if (seesTag()) {
				poseEstimator.addVisionMeasurement(visionPose, visionEstimateHigh.timestampSeconds);
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

		// Simulation poses
        Logger.recordOutput("Robot Pose", perception.getPose());
        Logger.recordOutput("Animated Component Poses", new Pose3d[] {
            new Pose3d(0, 0, Math.sin(Timer.getTimestamp() % Math.PI) / 4, new Rotation3d()),
            new Pose3d(0, 0, Math.sin(Timer.getTimestamp() % Math.PI) / 3, new Rotation3d()),
            new Pose3d(0, 0, Math.sin(Timer.getTimestamp() % Math.PI) / 2, new Rotation3d()),
            new Pose3d(.109, 0, .578 + Math.sin(Timer.getTimestamp() % Math.PI) / 2, new Rotation3d(0, Math.sin(Timer.getTimestamp()) - 1, 0))
        });
        Logger.recordOutput("Real Component Poses", getRealComponentPoses());

        for (TorqueFieldZone zone : zones) {
            Logger.recordOutput("Zone ID " + zone.getID() , zone.getPolygon());
        }
	}

	@Override
	public void clean(TorqueMode mode) {}

	public Pose3d[] getRealComponentPoses() {
		final double shoulderPos = elevator.getElevatorPosition();
		final double clawPos = claw.getClawAngle();
		final double shoulderMultiplier = shoulderPos / Elevator.State.NET.position;
		
		return new Pose3d[] {
            new Pose3d(0, 0, .6 * shoulderMultiplier, new Rotation3d()),
            new Pose3d(0, 0, 1.25 * shoulderMultiplier, new Rotation3d()),
            new Pose3d(0, 0, 1.43 * shoulderMultiplier, new Rotation3d()),
            new Pose3d(.109, .02, .278 + (1.43 * shoulderMultiplier), new Rotation3d(0, Math.toRadians((clawPos + 220 + 360) % 360), 0))
        };
	}

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
	
	private Pose2d getFusedVisionPose(final PoseEstimate high, final PoseEstimate low) {
		final Pose2d pastPose = getPose();

		// Pose filtering
		if (high == null && low != null) return low.pose;
		if (low == null && high != null) return high.pose;
		if (low == null && high == null) return null;

		// If the the two poses are more than a half-meter away from each other, disregard both
		if (high.pose.getTranslation().getDistance(low.pose.getTranslation()) > .5) {
			return pastPose;
		}

		// If not, return average pose
		final Pose2d fusedPose = new Pose2d(
			(high.pose.getX() + low.pose.getX()) / 2,
			(high.pose.getY() + low.pose.getY()) / 2,
			getHeading()
		);
		return fusedPose;
	}

	private boolean seesTag() {
		return LimelightHelpers.getTargetCount(LIMELIGHT_HIGH) > 0 || LimelightHelpers.getTargetCount(LIMELIGHT_LOW) > 0;
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

	public Optional<Pose2d> getAlignPose(final Pose2d currentPose, final Relation relation) {
		TorqueFieldZone currentZone = null;
		for (TorqueFieldZone zone : zones) {
			if (zone.contains(currentPose)) {
				currentZone = zone;
			}
		}
		if (currentZone == null) return Optional.empty();

		final int id = currentZone.getID();
		final AlignPose2d[] alignPoses = AprilTagList.values()[id + 1].alignPoses;

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

/**
 * Copyright 2025 Texas Torque.
 *
 * This file is part of Bravo/Charlie/Crashout-2025, which is not licensed for distribution.
 * For more details, see ./license.txt or write <davey.adams.three@gmail.com>.
 */
package org.texastorque.subsystems;

import java.util.ArrayList;

import org.littletonrobotics.junction.Logger;
import org.texastorque.Field.AlignPosition.Placement;
import org.texastorque.Field.AlignPosition.Relation;
import org.texastorque.Field.AlignPosition.AlignableTarget;
import org.texastorque.Field.AprilTagList;
import org.texastorque.Field;
import org.texastorque.LimelightHelpers;
import org.texastorque.LimelightHelpers.PoseEstimate;
import org.texastorque.LimelightHelpers.RawFiducial;
import org.texastorque.Ports;
import org.texastorque.Subsystems;
import org.texastorque.torquelib.Debug;
import org.texastorque.torquelib.base.TorqueMode;
import org.texastorque.torquelib.base.TorqueStatelessSubsystem;
import org.texastorque.torquelib.control.TorqueFieldZone;
import org.texastorque.torquelib.control.TorqueRollingMedian;

import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.Pair;
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
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public class Perception extends TorqueStatelessSubsystem implements Subsystems {
	private static volatile Perception instance;

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

	private final TorqueRollingMedian filteredX, filteredY, filteredHPDistance;
	private final Pigeon2 gyro = new Pigeon2(Ports.GYRO);
	private final CANrange canRange;
	private final Field2d field = new Field2d();
	private final SwerveDrivePoseEstimator poseEstimator;
	private AlignableTarget desiredAlignTarget = AlignableTarget.NONE;
	private Relation relation = Relation.NONE;
	private ArrayList<TorqueFieldZone> zones;
	private Pose2d filteredPose = new Pose2d();
	private boolean isHighInvalid, isLowInvalid, shouldNotUseVision;
	private double gyro_simulated = 0;
	public boolean useDistance;
	public Pose2d currentTagPose;
	
	public Perception() {
		LimelightHelpers.setCameraPose_RobotSpace(LIMELIGHT_HIGH, -0.152, -0.135, 0.747 + .046, 0, 45, 180);
		LimelightHelpers.setCameraPose_RobotSpace(LIMELIGHT_LOW, 0.0916686, 0.127, 0.164267, 0, 25, 0);

		poseEstimator = new SwerveDrivePoseEstimator(drivebase.kinematics, getHeading(), drivebase.getModulePositions(), new Pose2d(), ODOMETRY_STDS, VISION_STDS);

		canRange = new CANrange(Ports.CAN_RANGE);

		filteredX = new TorqueRollingMedian(15);
		filteredY = new TorqueRollingMedian(15);
		filteredHPDistance = new TorqueRollingMedian(15);

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
		
		updateOdometryLocalization();
		if (RobotBase.isReal()) updateVisionLocalization();

		field.setRobotPose(getPose());

		filteredPose = new Pose2d(
			filteredX.calculate(getPose().getX()),
			filteredY.calculate(getPose().getY()),
			getHeading()
		);

		useDistance = getCurrentZone() != null && (getCurrentZone().getID() == 1 || getCurrentZone().getID() == 2 || getCurrentZone().getID() == 12 || getCurrentZone().getID() == 13);

		if (getCurrentZone() != null) {
			currentTagPose = AprilTagList.values()[getCurrentZone().getID()-1].pose;
		}

		updateVisualization();

		Logger.recordOutput("Filtered Pose", getFilteredPose());
		Debug.log("Gyro Angle", getHeading().getDegrees());
		Debug.log("Current Pose", getPose().toString());
		Debug.log("Sees Tag", seesTag());
		Debug.log("CANrange Distance", getHPDistance());
		Debug.log("Gyro Angle", getHeading().getDegrees());
		Debug.log("Relation", relation.toString());
		Debug.log("Align Target", desiredAlignTarget.toString());
		Debug.log("Current Tag ID", getCurrentZone() != null ? currentTagPose.getRotation().toString() : "none");
	}

	@Override
	public void clean(TorqueMode mode) {}

	public void updateVisionLocalization() {
		LimelightHelpers.SetRobotOrientation(LIMELIGHT_HIGH, getHeading().getDegrees(), 0, 0, 0, 0, 0);
		LimelightHelpers.SetRobotOrientation(LIMELIGHT_LOW, getHeading().getDegrees(), 0, 0, 0, 0, 0);
		LimelightHelpers.PoseEstimate visionEstimateHigh = getVisionEstimate(LIMELIGHT_HIGH);
		LimelightHelpers.PoseEstimate visionEstimateLow = getVisionEstimate(LIMELIGHT_LOW);

		shouldNotUseVision = drivebase.getState() == Drivebase.State.PATHING
				|| gyro.getAngularVelocityYDevice().getValueAsDouble() > Math.PI;

		if (shouldNotUseVision) return;

		isHighInvalid = visionEstimateHigh == null
				|| visionEstimateHigh.avgTagDist > 6
				|| visionEstimateHigh.tagCount == 0
				|| (drivebase.getState() == Drivebase.State.ALIGN && visionEstimateHigh.rawFiducials.length > 2)
				|| (getCurrentZone() != null && AprilTagList.values()[getCurrentZone().getID() - 1].placement == Placement.REEF);
		
		isLowInvalid = visionEstimateLow == null
				|| visionEstimateLow.avgTagDist > 6
				|| visionEstimateLow.tagCount == 0
				|| (drivebase.getState() == Drivebase.State.ALIGN && visionEstimateLow.rawFiducials.length > 2)
				|| (getCurrentZone() != null && AprilTagList.values()[getCurrentZone().getID() - 1].placement == Placement.CORAL_STATION);
		
		Debug.log("High Invalid", isHighInvalid);
		Debug.log("Low Invalid", isLowInvalid);

		if (!isHighInvalid) {
			poseEstimator.addVisionMeasurement(visionEstimateHigh.pose, visionEstimateHigh.timestampSeconds);
			Logger.recordOutput("High Limelight Pose", visionEstimateHigh.pose);
		}
		if (!isLowInvalid) {
			poseEstimator.addVisionMeasurement(visionEstimateLow.pose, visionEstimateLow.timestampSeconds);
			Logger.recordOutput("Low Limelight Pose", visionEstimateLow.pose);
		}
	}

	public void updateOdometryLocalization() {
		poseEstimator.update(getHeading(), drivebase.getModulePositions());
	}

	public Pair<Double, Double> getOffsets(final Pose2d targetPose, final Pose2d currentPose) {
		final double dx = targetPose.getX() - currentPose.getX();
		final double dy = targetPose.getY() - currentPose.getY();

		final double forward = dx * Math.cos(targetPose.getRotation().getRadians()) + dy * Math.sin(targetPose.getRotation().getRadians());
		final double right = -dx * Math.sin(targetPose.getRotation().getRadians()) + dy * Math.cos(targetPose.getRotation().getRadians());

		return new Pair<Double, Double>(forward, right);
	}

	public void updateVisualization() {
		Logger.recordOutput("Robot Pose", getPose());
		Logger.recordOutput("Animated Component Poses", new Pose3d[] {
			new Pose3d(0, 0, 0, new Rotation3d()),
			new Pose3d(0, 0, 0, new Rotation3d()),
			new Pose3d(0.11, 0, 0.275, new Rotation3d()),
			new Pose3d(-0.31, 0, 0.24, new Rotation3d())
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

	public Pose3d[] getRealComponentPoses() {
		final double elevatorPos = elevator.getElevatorPosition();
		final double shoulderAngle = claw.getShoulderAngle();
		final double elevatorMultiplier = elevatorPos / elevator.MAX_HEIGHT;
		
		return new Pose3d[] {
			new Pose3d(0, 0, .5 * elevatorMultiplier, new Rotation3d()),
			new Pose3d(0, 0, 1.05 * elevatorMultiplier, new Rotation3d()),
			new Pose3d(.109, 0, .8 + (1.05 * elevatorMultiplier), new Rotation3d(0, Math.toRadians((shoulderAngle + 360) % 360), 0)),
			new Pose3d(-0.31, 0, 0.24, new Rotation3d(0, Math.toRadians((-(climb.getClimbPosition() / 2.75) + 360) % 360), 0))
		};
	}

	public Pose3d[] getDesiredComponentPoses() {
		final double elevatorPos = elevator.getState().position;
		final double shoulderAngle = claw.getState().getAngle();
		final double elevatorMultiplier = elevatorPos / elevator.MAX_HEIGHT;

		return new Pose3d[] {
			new Pose3d(0, 0, .5 * elevatorMultiplier, new Rotation3d()),
			new Pose3d(0, 0, 1.05 * elevatorMultiplier, new Rotation3d()),
			new Pose3d(.109, 0, .8 + (1.05 * elevatorMultiplier), new Rotation3d(0, Math.toRadians((shoulderAngle + 360) % 360), 0)),
			new Pose3d(-0.31, 0, 0.24, new Rotation3d(0, Math.toRadians((-(climb.getClimbPosition() / 2.75) + 360) % 360), 0))
		};
	}

	public boolean inCoralStationZone() {
		if (getCurrentZone() == null) return false;
		return AprilTagList.values()[getCurrentZone().getID() - 1].placement == Placement.CORAL_STATION;
	}

	public boolean inReefZone() {
		if (getCurrentZone() == null) return false;
		return AprilTagList.values()[getCurrentZone().getID() - 1].placement == Placement.REEF;
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

	public TorqueFieldZone getCurrentZone() {
		for (TorqueFieldZone zone : zones) {
			if (zone.contains(getFilteredPose())) {
				return zone;
			}
		}
		return null;
	}

	public Rotation2d getHeading() {
		if (RobotBase.isSimulation()) {
			return Rotation2d.fromRadians(gyro_simulated);
		}
		return Rotation2d.fromDegrees((gyro.getYaw().getValueAsDouble() + (100 * 360)) % 360);
	}

	public Pose2d getAlignPose() {
		if (getCurrentZone() != null && AprilTagList.values()[getCurrentZone().getID() - 1].placement == Placement.REEF && !DriverStation.isAutonomous() && drivebase.getAlignOverride() == null) {
			return Field.getInstance().getAlignPose(filteredPose, AlignableTarget.of(elevator.getSelectedState()), relation);
		}
		return Field.getInstance().getAlignPose(filteredPose, desiredAlignTarget, relation);
	}

	public AlignableTarget getDesiredAlignTarget() {
		return desiredAlignTarget;
	}

	public void resetHeading(final double offset) {
		gyro_simulated = 0;
		gyro.setYaw(offset);
		setPose(new Pose2d(0, 0, getHeading()));
	}
	
	public void resetHeading() {
		final boolean isRedAlliance = DriverStation.getAlliance().isPresent()
                    ? DriverStation.getAlliance().get() == Alliance.Red
                    : false;

		drivebase.lastRotationTarget = isRedAlliance ? 180 : 0;
		
		resetHeading(isRedAlliance ? 180 : 0);
	}

	public void setCurrentTrajectory(final Trajectory trajectory) {
		field.getObject("trajectory").setTrajectory(trajectory);
		Logger.recordOutput("Auto Trajectory", trajectory);
	}

	public boolean seesTag() {
		return !shouldNotUseVision && (!isHighInvalid || !isLowInvalid);
	}

	public void setDesiredAlignTarget(final AlignableTarget alignableTarget) {
		this.desiredAlignTarget = alignableTarget;
	}

	public void setRelation(Relation relation) {
	  	this.relation = relation;
	}

	public void setPose(final Pose2d pose) {
		poseEstimator.resetPosition(getHeading(), drivebase.getModulePositions(), pose);
	}

	private PoseEstimate getVisionEstimate(final String limelightName) {
		return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
	}

	public double getHPDistance() {
		if (RobotBase.isSimulation() && currentTagPose != null) {
			Rotation2d rotation = currentTagPose.getRotation().rotateBy(Rotation2d.fromDegrees(90));
			Pose2d currentPose = getPose();

			return Math.abs(Math.cos(rotation.getRadians()) * (currentTagPose.getY() - currentPose.getY()) - Math.sin(rotation.getRadians()) * (currentTagPose.getX() - currentPose.getX()));
		}
		return filteredHPDistance.calculate(canRange.getDistance().getValueAsDouble());
	}

	public Pose2d getFilteredPose() {
		return filteredPose;
	}

	public Pose2d getPose() {
		return new Pose2d(poseEstimator.getEstimatedPosition().getX(), poseEstimator.getEstimatedPosition().getY(), getHeading());
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

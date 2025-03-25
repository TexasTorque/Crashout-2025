package org.texastorque;

import java.util.ArrayList;
import org.texastorque.Field.AlignPosition.Placement;
import org.texastorque.Field.AlignPosition.Relation;
import org.texastorque.subsystems.Elevator;
import org.texastorque.Field.AlignPosition.AlignableTarget;
import org.texastorque.torquelib.control.TorqueFieldZone;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class Field implements Subsystems {
	private static volatile Field instance;

	public enum AprilTagList {
		ID_1(1, new Pose2d(16.700198, 0.65532, Rotation2d.fromDegrees(126)), Placement.CORAL_STATION),
		ID_2(2, new Pose2d(16.700198, 7.39448, Rotation2d.fromDegrees(234)), Placement.CORAL_STATION),
		ID_3(3, new Pose2d(11.55981, 8.05541, Rotation2d.fromDegrees(270)), Placement.PROCESSOR),
		ID_4(4, new Pose2d(9.27308, 6.137656, Rotation2d.fromDegrees(0)), Placement.BARGE),
		ID_5(5, new Pose2d(9.27308, 1.915906, Rotation2d.fromDegrees(0)), Placement.BARGE),
		ID_6(6, new Pose2d(13.474446, 3.306318, Rotation2d.fromDegrees(300)), Placement.REEF),
		ID_7(7, new Pose2d(13.890798, 4.02659, Rotation2d.fromDegrees(0)), Placement.REEF),
		ID_8(8, new Pose2d(13.474446, 4.746862, Rotation2d.fromDegrees(60)), Placement.REEF),
		ID_9(9, new Pose2d(12.644158, 4.746862, Rotation2d.fromDegrees(120)), Placement.REEF),
		ID_10(10, new Pose2d(12.228806, 4.02659, Rotation2d.fromDegrees(180)), Placement.REEF),
		ID_11(11, new Pose2d(12.644158, 3.306318, Rotation2d.fromDegrees(240)), Placement.REEF),
		ID_12(12, new Pose2d(0.851154, 0.65532, Rotation2d.fromDegrees(54)), Placement.CORAL_STATION),
		ID_13(13, new Pose2d(0.851154, 7.39448, Rotation2d.fromDegrees(306)), Placement.CORAL_STATION),
		ID_14(14, new Pose2d(8.274272, 6.137656, Rotation2d.fromDegrees(180)), Placement.BARGE),
		ID_15(15, new Pose2d(8.274272, 1.915906, Rotation2d.fromDegrees(180)), Placement.BARGE),
		ID_16(16, new Pose2d(5.988542, -0.00381, Rotation2d.fromDegrees(90)), Placement.PROCESSOR),
		ID_17(17, new Pose2d(4.075906, 3.306318, Rotation2d.fromDegrees(240)), Placement.REEF),
		ID_18(18, new Pose2d(3.6576, 4.02659, Rotation2d.fromDegrees(180)), Placement.REEF),
		ID_19(19, new Pose2d(4.075906, 4.746862, Rotation2d.fromDegrees(120)), Placement.REEF),
		ID_20(20, new Pose2d(4.905194, 4.746862, Rotation2d.fromDegrees(60)), Placement.REEF),
		ID_21(21, new Pose2d(5.321546, 4.02659, Rotation2d.fromDegrees(0)), Placement.REEF),
		ID_22(22, new Pose2d(4.905194, 3.306318, Rotation2d.fromDegrees(300)), Placement.REEF);

		public final int id;
		public final Pose2d pose;
		public final Placement placement;

		AprilTagList(final int id, final Pose2d pose, final Placement placement) {
			this.id = id;
			this.pose = pose;
			this.placement = placement;
		}
	}

	public class AlignPosition {

		public enum Relation {
			FAR_LEFT,
			LEFT,
			CENTER,
			RIGHT,
			FAR_RIGHT,
			NONE;
		}
	
		public enum Placement {
			REEF,
			CORAL_STATION,
			PROCESSOR,
			BARGE;
		}
	
		public enum AlignableTarget {
			L1,
			L2,
			L3,
			L4,
			ALGAE_HIGH,
			ALGAE_LOW,
			CORAL_STATION,
			NET,
			PROCESSOR,
			NONE;

			public static AlignableTarget of(final Elevator.State elevatorSelectedState) {
				if (elevatorSelectedState == Elevator.State.SCORE_L1) return L1;
				if (elevatorSelectedState == Elevator.State.SCORE_L2) return L2;
				if (elevatorSelectedState == Elevator.State.SCORE_L3) return L3;
				if (elevatorSelectedState == Elevator.State.SCORE_L4) return L4;
				if (elevatorSelectedState == Elevator.State.ALGAE_REMOVAL_HIGH) return ALGAE_HIGH;
				if (elevatorSelectedState == Elevator.State.ALGAE_REMOVAL_LOW) return ALGAE_LOW;
				if (elevatorSelectedState == Elevator.State.CORAL_HP) return CORAL_STATION;
				if (elevatorSelectedState == Elevator.State.CORAL_HP_SHIFT) return CORAL_STATION;
				if (elevatorSelectedState == Elevator.State.NET) return NET;
				if (elevatorSelectedState == Elevator.State.PROCESSOR) return PROCESSOR;
				return NONE;
			}
		}
		
		private final Placement placement;
		private final Relation relation;
		private final AlignableTarget alignableTarget;
		private final double forward;
		private final double right;
	
		public AlignPosition(final Placement placement, final Relation relation, final AlignableTarget alignableTarget, final double forward, final double right) {
			this.placement = placement;
			this.relation = relation;
			this.alignableTarget = alignableTarget;
			this.forward = forward;
			this.right = right;
		}
	
		public Placement getPlacement() {
		  return placement;
		}

		public Relation getRelation() {
			return relation;
		}
	
		public AlignableTarget getAlignableTarget() {
			return alignableTarget;
		}
	
		public double getForward() {
			return forward;
		}
	
		public double getRight() {
			return right;
		}
	}

	// bumpers against .4118
	// slit to slit is 0.2032m
	// pole to pole is 0.328619002m

	private ArrayList<AlignPosition> alignPositions = new ArrayList<>();
    private AlignPosition leftL2 = new AlignPosition(Placement.REEF, Relation.LEFT, AlignableTarget.L2, .5304, .0154 - 0.328619002);
    private AlignPosition leftL3 = new AlignPosition(Placement.REEF, Relation.LEFT, AlignableTarget.L3, .4118, .0189 - 0.328619002);
    private AlignPosition leftL4 = new AlignPosition(Placement.REEF, Relation.LEFT, AlignableTarget.L4, .5020, .0189 - 0.308619002);
    private AlignPosition centerHigh = new AlignPosition(Placement.REEF, Relation.CENTER, AlignableTarget.ALGAE_HIGH, .4118, -.1834);
    private AlignPosition centerLow = new AlignPosition(Placement.REEF, Relation.CENTER, AlignableTarget.ALGAE_LOW, .4118, -.1834);
    private AlignPosition backup = new AlignPosition(Placement.REEF, Relation.NONE, AlignableTarget.NONE, .75, -.2);
    private AlignPosition rightL2 = new AlignPosition(Placement.REEF, Relation.RIGHT, AlignableTarget.L2, .5304, .0154);
    private AlignPosition rightL3 = new AlignPosition(Placement.REEF, Relation.RIGHT, AlignableTarget.L3, .4118, .0189);
    private AlignPosition rightL4 = new AlignPosition(Placement.REEF, Relation.RIGHT, AlignableTarget.L4, .5020, .0189);
    private AlignPosition leftCoralStation = new AlignPosition(Placement.CORAL_STATION, Relation.LEFT, AlignableTarget.CORAL_STATION, .4118, .0127 - 0.2032 - 0.2032);
    private AlignPosition centerCoralStation = new AlignPosition(Placement.CORAL_STATION, Relation.CENTER, AlignableTarget.CORAL_STATION, .4118, .0127);
    private AlignPosition rightCoralStation = new AlignPosition(Placement.CORAL_STATION, Relation.RIGHT, AlignableTarget.CORAL_STATION, .4118, .0127 + 0.2032 + 0.2032);

	public Field() {
		alignPositions.add(leftL2);
		alignPositions.add(leftL3);
		alignPositions.add(leftL4);
		alignPositions.add(centerHigh);
		alignPositions.add(centerLow);
		alignPositions.add(backup);
		alignPositions.add(rightL2);
		alignPositions.add(rightL3);
		alignPositions.add(rightL4);
		alignPositions.add(leftCoralStation);
		alignPositions.add(centerCoralStation);
		alignPositions.add(rightCoralStation);
	}

	public Pose2d getAlignPose(final Pose2d currentPose, final AlignableTarget alignableTarget, final Relation relation) {
		TorqueFieldZone currentZone = perception.getCurrentZone();

		if (currentZone == null || currentPose == null) return null;

		AlignPosition alignPosition = null;
		for (AlignPosition pos : alignPositions) {
			if (pos.getAlignableTarget() == alignableTarget && pos.getRelation() == relation && pos.placement == AprilTagList.values()[currentZone.getID() - 1].placement) {
				alignPosition = pos;
			}
		}

		if (alignPosition == null) return null;

		Pose2d tagPose = AprilTagList.values()[currentZone.getID() - 1].pose;

		Pose2d forwardPosition = new Pose2d(
			tagPose.getX() + alignPosition.getForward() * Math.cos(tagPose.getRotation().getRadians()),
			tagPose.getY() + alignPosition.getForward() * Math.sin(tagPose.getRotation().getRadians()),
			tagPose.getRotation()
		);

		Pose2d rightPosition = new Pose2d(
			forwardPosition.getX() + alignPosition.getRight() * Math.cos(tagPose.getRotation().getRadians() + Math.toRadians(90)),
			forwardPosition.getY() + alignPosition.getRight() * Math.sin(tagPose.getRotation().getRadians() + Math.toRadians(90)),
			forwardPosition.getRotation().rotateBy(alignPosition.getPlacement() == Placement.CORAL_STATION  || alignableTarget == AlignableTarget.L1 ? new Rotation2d() : Rotation2d.fromDegrees(180))
		);
		
		return rightPosition;
	}

	public static synchronized final Field getInstance() {
        return instance == null ? instance = new Field() : instance;
    }
}

package org.texastorque;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;

import org.texastorque.AlignPose2d.Placement;
import org.texastorque.AlignPose2d.Relation;

import edu.wpi.first.math.geometry.Rotation3d;

public enum AprilTagList {
	ID_1(1, new Pose3d(16.700198, 0.65532, 1.4859, new Rotation3d(0, 0, 126)),
			Placement.CORAL_STATION,
			new AlignPose2d(Relation.LEFT, new Pose2d(17.548249 - .575, 1.375, Rotation2d.fromDegrees(126))),
			new AlignPose2d(Relation.CENTER, new Pose2d(16.37, 1.12, Rotation2d.fromDegrees(126))),
			new AlignPose2d(Relation.RIGHT, new Pose2d(17.548249 - 1.6, .6, Rotation2d.fromDegrees(126)))
	),
	ID_2(2, new Pose3d(16.700198, 7.39448, 1.4859, new Rotation3d(0, 0, 234)),
			Placement.CORAL_STATION,
			new AlignPose2d(Relation.LEFT, new Pose2d(17.548249 - 1.6, 8.0518 - .6, Rotation2d.fromDegrees(234))),
			new AlignPose2d(Relation.CENTER, new Pose2d(16.35, 6.95, Rotation2d.fromDegrees(234))),
			new AlignPose2d(Relation.RIGHT, new Pose2d(17.548249 - .575, 8.0518 - 1.375, Rotation2d.fromDegrees(234)))
	),
	ID_3(3, new Pose3d(11.55981, 8.05541, 1.30275, new Rotation3d(0, 0, 270)),
			Placement.PROCESSOR
	),
	ID_4(4, new Pose3d(9.27308, 6.137656, 1.867916, new Rotation3d(0, 30, 0)),
			Placement.BARGE
	),
	ID_5(5, new Pose3d(9.27308, 1.915906, 1.867916, new Rotation3d(0, 30, 0)),
			Placement.BARGE
	),
	ID_6(6, new Pose3d(13.474446, 3.306318, 0.308202, new Rotation3d(0, 0, 300)),
			Placement.REEF,
			new AlignPose2d(Relation.LEFT, new Pose2d(13.39, 2.80, Rotation2d.fromDegrees(120))),
			new AlignPose2d(Relation.CENTER, new Pose2d(13.49, 2.85, Rotation2d.fromDegrees(120))),
			new AlignPose2d(Relation.RIGHT, new Pose2d(13.69, 2.94, Rotation2d.fromDegrees(120)))
	),
	ID_7(7, new Pose3d(13.890798, 4.02659, 0.308202, new Rotation3d(0, 0, 0)),
			Placement.REEF,
			new AlignPose2d(Relation.LEFT, new Pose2d(14.29, 3.75, Rotation2d.fromDegrees(180))),
			new AlignPose2d(Relation.CENTER, new Pose2d(14.29, 3.88, Rotation2d.fromDegrees(180))),
			new AlignPose2d(Relation.RIGHT, new Pose2d(14.29, 4.07, Rotation2d.fromDegrees(180)))
	),
	ID_8(8, new Pose3d(13.474446, 4.746862, 0.308202, new Rotation3d(0, 0, 60)),
			Placement.REEF,
			new AlignPose2d(Relation.LEFT, new Pose2d(13.94, 4.97, Rotation2d.fromDegrees(240))),
			new AlignPose2d(Relation.CENTER, new Pose2d(13.79, 5.025, Rotation2d.fromDegrees(240))),
			new AlignPose2d(Relation.RIGHT, new Pose2d(13.64, 5.11, Rotation2d.fromDegrees(240)))
	),
	ID_9(9, new Pose3d(12.644158, 4.746862, 0.308202, new Rotation3d(0, 0, 120)),
			Placement.REEF,
			new AlignPose2d(Relation.LEFT, new Pose2d(12.69, 5.26, Rotation2d.fromDegrees(300))),
			new AlignPose2d(Relation.CENTER, new Pose2d(12.56, 5.18, Rotation2d.fromDegrees(300))),
			new AlignPose2d(Relation.RIGHT, new Pose2d(12.38, 5.05, Rotation2d.fromDegrees(300)))
	),
	ID_10(10, new Pose3d(12.228806, 4.02659, 0.308202, new Rotation3d(0, 0, 180)),
			Placement.REEF,
			new AlignPose2d(Relation.LEFT, new Pose2d(11.82, 4.31, Rotation2d.fromDegrees(0))),
			new AlignPose2d(Relation.CENTER, new Pose2d(11.82, 4.19, Rotation2d.fromDegrees(0))),
			new AlignPose2d(Relation.RIGHT, new Pose2d(11.82, 3.98, Rotation2d.fromDegrees(0)))
	),
	ID_11(11, new Pose3d(12.644158, 3.306318, 0.308202, new Rotation3d(0, 0, 240)),
			Placement.REEF,
			new AlignPose2d(Relation.LEFT, new Pose2d(12.17, 3.08, Rotation2d.fromDegrees(60))),
			new AlignPose2d(Relation.CENTER, new Pose2d(12.28, 3.05, Rotation2d.fromDegrees(60))),
			new AlignPose2d(Relation.RIGHT, new Pose2d(12.46, 2.94, Rotation2d.fromDegrees(60)))
	),
	ID_12(12, new Pose3d(0.851154, 0.65532, 1.4859, new Rotation3d(0, 0, 54)),
			Placement.CORAL_STATION,
			new AlignPose2d(Relation.LEFT, new Pose2d(1.6, .6, Rotation2d.fromDegrees(54))),
			new AlignPose2d(Relation.CENTER, new Pose2d(1.12, 1.11, Rotation2d.fromDegrees(54))),
			new AlignPose2d(Relation.RIGHT, new Pose2d(.575, 1.375, Rotation2d.fromDegrees(54)))
	),
	ID_13(13, new Pose3d(0.851154, 7.39448, 1.4859, new Rotation3d(0, 0, 306)),
			Placement.CORAL_STATION,
			new AlignPose2d(Relation.LEFT, new Pose2d(.575, 8.0518 - 1.375, Rotation2d.fromDegrees(306))),
			new AlignPose2d(Relation.CENTER, new Pose2d(1.16, 6.91, Rotation2d.fromDegrees(306))),
			new AlignPose2d(Relation.RIGHT, new Pose2d(1.6, 8.0518 - .6, Rotation2d.fromDegrees(306)))
	),
	ID_14(14, new Pose3d(8.274272, 6.137656, 1.867916, new Rotation3d(0, 30, 180)),
			Placement.BARGE
	),
	ID_15(15, new Pose3d(8.274272, 1.915906, 1.867916, new Rotation3d(0, 30, 180)),
			Placement.BARGE
	),
	ID_16(16, new Pose3d(5.988542, -0.00381, 1.30275, new Rotation3d(0, 0, 90)),
			Placement.PROCESSOR
	),
	ID_17(17, new Pose3d(4.075906, 3.306318, 0.308202, new Rotation3d(0, 0, 240)),
			Placement.REEF,
			new AlignPose2d(Relation.LEFT, new Pose2d(3.57, 3.10, Rotation2d.fromDegrees(60))),
			new AlignPose2d(Relation.CENTER, new Pose2d(3.64, 3.03, Rotation2d.fromDegrees(60))),
			new AlignPose2d(Relation.RIGHT, new Pose2d(3.88, 2.93, Rotation2d.fromDegrees(60)))
	),
	ID_18(18, new Pose3d(3.6576, 4.02659, 0.308202, new Rotation3d(0, 0, 180)),
			Placement.REEF,
			new AlignPose2d(Relation.LEFT, new Pose2d(3.25, 4.37, Rotation2d.fromDegrees(0))),
			new AlignPose2d(Relation.CENTER, new Pose2d(3.25, 4.20, Rotation2d.fromDegrees(0))),
			new AlignPose2d(Relation.RIGHT, new Pose2d(3.25, 4.02, Rotation2d.fromDegrees(0)))
	),
	ID_19(19, new Pose3d(4.075906, 4.746862, 0.308202, new Rotation3d(0, 0, 120)),
			Placement.REEF,
			new AlignPose2d(Relation.LEFT, new Pose2d(4.15, 5.25, Rotation2d.fromDegrees(300))),
			new AlignPose2d(Relation.CENTER, new Pose2d(4.06, 5.22, Rotation2d.fromDegrees(300))),
			new AlignPose2d(Relation.RIGHT, new Pose2d(3.87, 5.11, Rotation2d.fromDegrees(300)))
	),
	ID_20(20, new Pose3d(4.905194, 4.746862, 0.308202, new Rotation3d(0, 0, 60)),
			Placement.REEF,
			new AlignPose2d(Relation.RIGHT, new Pose2d(5.15, 5.09, Rotation2d.fromDegrees(240))),
			new AlignPose2d(Relation.CENTER, new Pose2d(5.30, 5.01, Rotation2d.fromDegrees(240))),
			new AlignPose2d(Relation.LEFT, new Pose2d(5.40, 4.95, Rotation2d.fromDegrees(240)))
	),
	ID_21(21, new Pose3d(5.321546, 4.02659, 0.308202, new Rotation3d(0, 0, 0)),
			Placement.REEF,
			new AlignPose2d(Relation.RIGHT, new Pose2d(5.74, 4.03, Rotation2d.fromDegrees(180))),
			new AlignPose2d(Relation.CENTER, new Pose2d(5.77, 3.83, Rotation2d.fromDegrees(180))),
			new AlignPose2d(Relation.LEFT, new Pose2d(5.74, 3.71, Rotation2d.fromDegrees(180)))
	),
	ID_22(22, new Pose3d(4.905194, 3.306318, 0.308202, new Rotation3d(0, 0, 300)),
			Placement.REEF,
			new AlignPose2d(Relation.RIGHT, new Pose2d(5.11, 2.94, Rotation2d.fromDegrees(120))),
			new AlignPose2d(Relation.CENTER, new Pose2d(4.93, 2.82, Rotation2d.fromDegrees(120))),
			new AlignPose2d(Relation.LEFT, new Pose2d(4.82, 2.80, Rotation2d.fromDegrees(120)))
	);

	public final int id;
	public final Pose3d pose;
	public final Placement placement;
	public final AlignPose2d[] alignPoses; // Poses related to the april tag that we want to align to.

	AprilTagList(final int id, final Pose3d pose, final Placement placement, final AlignPose2d ...alignPoses) {
		this.id = id;
		this.pose = pose;
		this.placement = placement;
		this.alignPoses = alignPoses;
	}
}

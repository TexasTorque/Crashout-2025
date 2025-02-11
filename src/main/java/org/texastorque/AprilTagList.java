package org.texastorque;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;

import org.texastorque.AlignPose2d.Placement;
import org.texastorque.AlignPose2d.Relation;

import edu.wpi.first.math.geometry.Rotation3d;

public enum AprilTagList {
	ID_1(1, new Pose3d(16.700198, 0.65532, 1.4859, new Rotation3d(0, 0, 126)),
			Placement.CORAL_STATION
	),
	ID_2(2, new Pose3d(16.700198, 7.39448, 1.4859, new Rotation3d(0, 0, 234)),
			Placement.CORAL_STATION
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
			Placement.REEF
	),
	ID_7(7, new Pose3d(13.890798, 4.02659, 0.308202, new Rotation3d(0, 0, 0)),
			Placement.REEF
	),
	ID_8(8, new Pose3d(13.474446, 4.746862, 0.308202, new Rotation3d(0, 0, 60)),
			Placement.REEF
	),
	ID_9(9, new Pose3d(12.644158, 4.746862, 0.308202, new Rotation3d(0, 0, 120)),
			Placement.REEF
	),
	ID_10(10, new Pose3d(12.228806, 4.02659, 0.308202, new Rotation3d(0, 0, 180)),
			Placement.REEF
	),
	ID_11(11, new Pose3d(12.644158, 3.306318, 0.308202, new Rotation3d(0, 0, 240)),
			Placement.REEF
	),
	ID_12(12, new Pose3d(0.851154, 0.65532, 1.4859, new Rotation3d(0, 0, 54)),
			Placement.CORAL_STATION,
			new AlignPose2d(Relation.CENTER, new Pose2d(1.38, 1, Rotation2d.fromDegrees(55)))
	),
	ID_13(13, new Pose3d(0.851154, 7.39448, 1.4859, new Rotation3d(0, 0, 306)),
			Placement.CORAL_STATION
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
			Placement.REEF
	),
	ID_18(18, new Pose3d(3.6576, 4.02659, 0.308202, new Rotation3d(0, 0, 180)),
			Placement.REEF
	),
	ID_19(19, new Pose3d(4.075906, 4.746862, 0.308202, new Rotation3d(0, 0, 120)),
			Placement.REEF
	),
	ID_20(20, new Pose3d(4.905194, 4.746862, 0.308202, new Rotation3d(0, 0, 60)),
			Placement.REEF
	),
	ID_21(21, new Pose3d(5.321546, 4.02659, 0.308202, new Rotation3d(0, 0, 0)),
			Placement.REEF,
			new AlignPose2d(Relation.CENTER, new Pose2d(5.8, 4, Rotation2d.fromDegrees(180)))
	),
	ID_22(22, new Pose3d(4.905194, 3.306318, 0.308202, new Rotation3d(0, 0, 300)),
			Placement.REEF
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

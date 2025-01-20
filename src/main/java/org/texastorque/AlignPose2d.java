package org.texastorque;

import edu.wpi.first.math.geometry.Pose2d;

public class AlignPose2d {
    public enum Relation {
        FAR_LEFT,
        LEFT,
        CENTER,
        RIGHT,
        FAR_RIGHT;
    }

    public enum Placement {
		REEF,
		CORAL_STATION,
		PROCESSOR,
		BARGE;
	}

    private Relation relation;
    private Pose2d pose;

    public AlignPose2d(final Relation relation, final Pose2d pose) {
        this.relation = relation;
        this.pose = pose;
    }

    public Relation getRelation() {
        return relation;
    }

    public Pose2d getPose() {
        return pose;
    }
}
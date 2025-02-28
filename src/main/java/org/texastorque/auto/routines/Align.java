package org.texastorque.auto.routines;

import java.util.function.Supplier;

import org.texastorque.AlignPose2d.Relation;
import org.texastorque.Subsystems;
import org.texastorque.subsystems.Drivebase;
import org.texastorque.torquelib.auto.TorqueSequence;
import org.texastorque.torquelib.auto.commands.TorqueRun;
import org.texastorque.torquelib.auto.commands.TorqueWaitTimeUntil;
import org.texastorque.torquelib.swerve.TorqueSwerveSpeeds;

import edu.wpi.first.math.geometry.Pose2d;

public class Align extends TorqueSequence implements Subsystems {

	public Align(final Relation relation, final double timeToAlign) {
        addBlock(new TorqueRun(() -> {
            drivebase.setRelation(relation);
            drivebase.setState(Drivebase.State.ALIGN);
        }));
        addBlock(new TorqueWaitTimeUntil(timeToAlign, () -> drivebase.isNearAligned()));
        addBlock(new TorqueRun(() -> {
            drivebase.setInputSpeeds(new TorqueSwerveSpeeds());
            drivebase.setState(Drivebase.State.ROBOT_RELATIVE);
        }));
	}

	public Align(final Supplier<Pose2d> pose, final double timeToAlign) {
		addBlock(new TorqueRun(() -> {
            drivebase.setAlignPoseOverride(pose.get());
            drivebase.setState(Drivebase.State.ALIGN);
        }));
        addBlock(new TorqueWaitTimeUntil(timeToAlign, () -> drivebase.isNearAligned()));
        addBlock(new TorqueRun(() -> {
            drivebase.setInputSpeeds(new TorqueSwerveSpeeds());
            drivebase.setState(Drivebase.State.ROBOT_RELATIVE);
            drivebase.setAlignPoseOverride(null);
        }));
	}
}

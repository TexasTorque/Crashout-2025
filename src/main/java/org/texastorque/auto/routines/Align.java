package org.texastorque.auto.routines;

import org.texastorque.AlignPose2d.Relation;
import org.texastorque.Subsystems;
import org.texastorque.subsystems.Drivebase;
import org.texastorque.torquelib.auto.TorqueSequence;
import org.texastorque.torquelib.auto.commands.TorqueRun;
import org.texastorque.torquelib.auto.commands.TorqueWaitTimeUntil;
import org.texastorque.torquelib.swerve.TorqueSwerveSpeeds;

import edu.wpi.first.math.geometry.Pose2d;

public class Align extends TorqueSequence implements Subsystems {

	public Align(final Relation relation) {
        addBlock(new TorqueRun(() -> {
            drivebase.setRelation(relation);
            drivebase.setState(Drivebase.State.ALIGN);
        }));
        addBlock(new TorqueWaitTimeUntil(5, () -> drivebase.isAligned()));
        addBlock(new TorqueRun(() -> drivebase.setState(Drivebase.State.ROBOT_RELATIVE)));
	}

	public Align(final Pose2d pose) {
		addBlock(new TorqueRun(() -> {
            drivebase.setAlignPoseOverride(pose);
            drivebase.setState(Drivebase.State.ALIGN);
        }));
        addBlock(new TorqueWaitTimeUntil(5, () -> drivebase.isAligned()));
        addBlock(new TorqueRun(() -> {
            drivebase.setInputSpeeds(new TorqueSwerveSpeeds());
            drivebase.setState(Drivebase.State.ROBOT_RELATIVE);
            drivebase.setAlignPoseOverride(null);
        }));
	}
}

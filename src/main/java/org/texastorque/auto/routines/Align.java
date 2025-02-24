package org.texastorque.auto.routines;

import org.texastorque.AlignPose2d.Relation;
import org.texastorque.Subsystems;
import org.texastorque.subsystems.Drivebase;
import org.texastorque.torquelib.auto.TorqueSequence;
import org.texastorque.torquelib.auto.commands.TorqueRun;
import org.texastorque.torquelib.auto.commands.TorqueWaitUntil;
import org.texastorque.torquelib.base.TorqueMode;
import org.texastorque.torquelib.swerve.TorqueSwerveSpeeds;

import edu.wpi.first.math.geometry.Pose2d;

public class Align extends TorqueSequence implements Subsystems {

	public Align(final Relation relation) {
        addBlock(new TorqueRun(() -> {
            drivebase.setRelation(relation);
            drivebase.setState(Drivebase.State.ALIGN);
        }));
        addBlock(new TorqueWaitUntil(() -> drivebase.isAligned(TorqueMode.AUTO)));
        addBlock(new TorqueRun(() -> drivebase.setState(Drivebase.State.ROBOT_RELATIVE)));
	}

	public Align(final Pose2d pose) {
		addBlock(new TorqueRun(() -> {
            drivebase.setAlignPoseOverride(pose);
            drivebase.setState(Drivebase.State.ALIGN);
        }));
        addBlock(new TorqueWaitUntil(() -> drivebase.isAligned(TorqueMode.AUTO)));
        addBlock(new TorqueRun(() -> {
            drivebase.setInputSpeeds(new TorqueSwerveSpeeds());
            drivebase.setState(Drivebase.State.ROBOT_RELATIVE);
            drivebase.setAlignPoseOverride(null);
        }));
	}
}

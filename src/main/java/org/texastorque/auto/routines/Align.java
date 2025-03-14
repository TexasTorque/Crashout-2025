package org.texastorque.auto.routines;

import java.util.function.Supplier;

import org.texastorque.Field.AlignPosition.AlignableTarget;
import org.texastorque.Field.AlignPosition.Relation;
import org.texastorque.Subsystems;
import org.texastorque.subsystems.Drivebase;
import org.texastorque.torquelib.auto.TorqueSequence;
import org.texastorque.torquelib.auto.commands.TorqueRun;
import org.texastorque.torquelib.auto.commands.TorqueWaitTimeUntil;
import org.texastorque.torquelib.swerve.TorqueSwerveSpeeds;
import edu.wpi.first.math.geometry.Pose2d;

public class Align extends TorqueSequence implements Subsystems {

	public Align(final Relation relation, final AlignableTarget alignableTarget, final double timeToAlign) {
        addBlock(new TorqueRun(() -> {
            perception.setRelation(relation);
            perception.setDesiredAlignTarget(alignableTarget);
            drivebase.setState(Drivebase.State.ALIGN);
            drivebase.resetAlign();
        }));
        addBlock(new TorqueWaitTimeUntil(timeToAlign, () -> drivebase.isAligned()));
        addBlock(new TorqueRun(() -> {
            drivebase.setInputSpeeds(new TorqueSwerveSpeeds());
            drivebase.setState(Drivebase.State.ROBOT_RELATIVE);
        }));
	}

	public Align(final Supplier<Pose2d> pose, final double timeToAlign) {
		addBlock(new TorqueRun(() -> {
            drivebase.setAlignPoseOverride(pose.get());
            drivebase.setState(Drivebase.State.ALIGN);
            drivebase.resetAlign();
        }));
        addBlock(new TorqueWaitTimeUntil(timeToAlign, () -> drivebase.isAligned()));
        addBlock(new TorqueRun(() -> {
            drivebase.setInputSpeeds(new TorqueSwerveSpeeds());
            drivebase.setState(Drivebase.State.ROBOT_RELATIVE);
            drivebase.setAlignPoseOverride(null);
        }));
	}
}

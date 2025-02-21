package org.texastorque.auto.routines;

import org.texastorque.AlignPose2d.Relation;
import org.texastorque.Subsystems;
import org.texastorque.subsystems.Claw;
import org.texastorque.subsystems.Drivebase;
import org.texastorque.subsystems.Elevator;
import org.texastorque.torquelib.auto.TorqueSequence;
import org.texastorque.torquelib.auto.commands.TorqueRun;
import org.texastorque.torquelib.auto.commands.TorqueWaitTime;
import org.texastorque.torquelib.auto.commands.TorqueWaitUntil;
import org.texastorque.torquelib.base.TorqueMode;
import org.texastorque.torquelib.swerve.TorqueSwerveSpeeds;

import edu.wpi.first.math.geometry.Pose2d;

public class QuickSwap extends TorqueSequence implements Subsystems {

	public QuickSwap(final Pose2d backupPose) {
		// Alignment
        addBlock(new Align(Relation.CENTER).command());

        // Algae extraction
        addBlock(new TorqueRun(() -> claw.setAlgaeState(Claw.AlgaeState.INTAKE)));
        addBlock(new TorqueWaitTime(.5)); // Wait until we intake algae
        addBlock(new TorqueRun(() -> claw.setAlgaeState(Claw.AlgaeState.OFF)));

        // Move back to avoid claw hitting reef poles
        addBlock(new TorqueRun(() -> {
            drivebase.setAlignPoseOverride(backupPose);
            drivebase.setState(Drivebase.State.ALIGN);
        }));
        addBlock(new TorqueWaitUntil(() -> drivebase.isAligned(TorqueMode.AUTO)));
        addBlock(new TorqueRun(() -> {
            drivebase.setInputSpeeds(new TorqueSwerveSpeeds());
            drivebase.setState(Drivebase.State.ROBOT_RELATIVE);
            drivebase.setAlignPoseOverride(null);
        }));

        // Start moving subsystems while aligning
        addBlock(new TorqueRun(() -> {
            elevator.setState(Elevator.State.SCORE_L3);
            claw.setState(Claw.State.MID_SCORE);
        }));

        addBlock(new TorqueWaitUntil(() -> elevator.isNearState() && claw.isNearState()));

        // Alignment
        addBlock(new Align(Relation.LEFT).command());

        // Coral placement
        addBlock(new TorqueRun(() -> claw.setCoralState(Claw.CoralState.SHOOT)));
        addBlock(new TorqueWaitTime(.5)); // Wait until we shoot coral
        addBlock(new TorqueRun(() -> claw.setCoralState(Claw.CoralState.OFF)));
	}
	
}

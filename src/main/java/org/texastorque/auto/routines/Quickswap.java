package org.texastorque.auto.routines;

import org.texastorque.Field.AlignPosition.AlignableTarget;
import org.texastorque.Field.AlignPosition.Relation;
import org.texastorque.Subsystems;
import org.texastorque.subsystems.Claw;
import org.texastorque.subsystems.Elevator;
import org.texastorque.torquelib.auto.TorqueSequence;
import org.texastorque.torquelib.auto.commands.TorqueRun;
import org.texastorque.torquelib.auto.commands.TorqueWaitTime;
import org.texastorque.torquelib.auto.commands.TorqueWaitUntil;

public class Quickswap extends TorqueSequence implements Subsystems {

    public Quickswap() {
        this(false);
    }

	public Quickswap(final boolean isL4) {
        // Algae extraction
        addBlock(new TorqueRun(() -> claw.setAlgaeState(Claw.AlgaeState.INTAKE)));

		// Alignment
        addBlock(new Align(Relation.CENTER, AlignableTarget.ALGAE_HIGH, .75).command());

        addBlock(new TorqueRun(() -> claw.setAlgaeState(Claw.AlgaeState.OFF)));

        // Start moving subsystems while aligning
        addBlock(new TorqueRun(() -> {
            elevator.setState(isL4 ? Elevator.State.SCORE_L4 : Elevator.State.SCORE_L3);
            claw.setState(isL4 ? Claw.State.SCORE_L4 : Claw.State.SCORE_L3);
        }));

        // Move back to avoid claw hitting reef poles
        addBlock(new TorqueRun(() -> {
            perception.setRelation(Relation.NONE);
            perception.setDesiredAlignTarget(AlignableTarget.NONE);
        }));
        addBlock(new Align(() -> perception.getAlignPose(), 1).command());

        addBlock(new TorqueWaitUntil(() -> elevator.isNearState() && claw.isNearState()));

        // Alignment
        addBlock(new Align(Relation.LEFT, isL4 ? AlignableTarget.L4 : AlignableTarget.L3, 1.2).command());

        // Coral placement & Algae Shot
        addBlock(new TorqueRun(() -> claw.setCoralState(Claw.CoralState.SHOOT)));
        addBlock(new TorqueRun(() -> claw.setAlgaeState(Claw.AlgaeState.INTAKE)));
        addBlock(new TorqueWaitTime(.5)); // Wait until we shoot coral
        addBlock(new TorqueRun(() -> claw.setCoralState(Claw.CoralState.OFF)));
        addBlock(new TorqueRun(() -> claw.setAlgaeState(Claw.AlgaeState.OFF)));
	}
	
}

/**
 * Copyright 2025 Texas Torque.
 *
 * This file is part of Bravo/Charlie/Crashout-2025, which is not licensed for distribution.
 * For more details, see ./license.txt or write <davey.adams.three@gmail.com>.
 */
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
import org.texastorque.torquelib.auto.marker.Marker;

public class Quickswap extends TorqueSequence implements Subsystems {

    public Quickswap() {
        this(false, Relation.LEFT);
    }

	public Quickswap(final boolean isL4, final Relation relation) {
        // Algae extraction
        addBlock(new TorqueRun(() -> claw.setAlgaeState(Claw.AlgaeState.INTAKE)));

		// Alignment
        addBlock(new Align(Relation.CENTER, AlignableTarget.ALGAE_HIGH, .7).command());

        // Move back to avoid claw hitting reef poles
        addBlock(new TorqueRun(() -> {
            perception.setRelation(Relation.NONE);
            perception.setDesiredAlignTarget(AlignableTarget.NONE);
        }));
        addBlock(new Align(() -> perception.getAlignPose(), .7, 
            new Marker(() -> {
                elevator.setState(isL4 ? Elevator.State.SCORE_L4 : Elevator.State.SCORE_L3);
                claw.setState(isL4 ? Claw.State.SCORE_L4 : Claw.State.SCORE_L3);
            }, .3)
        ).command());

        addBlock(new TorqueWaitUntil(() -> elevator.isNearState() && claw.isNearState()));

        addBlock(new TorqueRun(() -> {
            claw.setAlgaeState(Claw.AlgaeState.OFF);
        }));

        // Alignment
        addBlock(new Align(relation, isL4 ? AlignableTarget.L4 : AlignableTarget.L3, 1.2).command());

        // Coral placement & Algae Shot
        addBlock(new TorqueRun(() -> claw.setCoralState(Claw.CoralState.SHOOT)));
        addBlock(new TorqueWaitTime(.5)); // Wait until we shoot coral
        addBlock(new TorqueRun(() -> claw.setCoralState(Claw.CoralState.OFF)));
	}
}

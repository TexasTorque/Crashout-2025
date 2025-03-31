/**
 * Copyright 2025 Texas Torque.
 *
 * This file is part of Bravo/Charlie/Crashout-2025, which is not licensed for distribution.
 * For more details, see ./license.txt or write <davey.adams.three@gmail.com>.
 */
package org.texastorque.auto.sequences;

import org.texastorque.Field.AlignPosition.AlignableTarget;
import org.texastorque.Field.AlignPosition.Relation;
import org.texastorque.auto.routines.Align;
import org.texastorque.auto.routines.Quickswap;
import org.texastorque.Subsystems;
import org.texastorque.subsystems.Claw;
import org.texastorque.subsystems.Elevator;
import org.texastorque.torquelib.auto.TorqueSequence;
import org.texastorque.torquelib.auto.commands.TorqueFollowPath;
import org.texastorque.torquelib.auto.commands.TorqueRun;
import org.texastorque.torquelib.auto.commands.TorqueWaitUntil;
import org.texastorque.torquelib.auto.marker.Marker;

public class CenterNet extends TorqueSequence implements Subsystems {

	public CenterNet() {
		addBlock(new TorqueRun(() -> {
			elevator.setState(Elevator.State.ALGAE_REMOVAL_LOW);
			claw.setState(Claw.State.ALGAE_EXTRACTION);
		}));

        // Drive center to far
		addBlock(new TorqueFollowPath("CNET_1", drivebase));

		addBlock(new TorqueWaitUntil(() -> elevator.isNearState() && claw.isNearState()));

		addBlock(new Quickswap(true, Relation.RIGHT).command());

		// Shoot algae
		addBlock(new TorqueFollowPath("CNET_2", drivebase).withMarkers(
			new Marker(() -> {
				elevator.setState(Elevator.State.NET);
				claw.setState(Claw.State.NET);
			}, .05),
			new Marker(() -> {
				claw.setAlgaeState(Claw.AlgaeState.SHOOT_FAST);
			}, .25),
			new Marker(() -> {
				elevator.setState(Elevator.State.ALGAE_REMOVAL_HIGH);
				claw.setState(Claw.State.ALGAE_EXTRACTION);
				claw.setAlgaeState(Claw.AlgaeState.INTAKE);
			}, .5)
		));

		// Pick up high algae
		addBlock(new Align(Relation.CENTER, AlignableTarget.ALGAE_HIGH, 1).command());

		// Drive to center net and shoot
		addBlock(new TorqueFollowPath("CNET_3", drivebase).withMarkers(
			new Marker(() -> {
				elevator.setState(Elevator.State.NET);
				claw.setState(Claw.State.NET);
				claw.setAlgaeState(Claw.AlgaeState.OFF);
			}, .1),
			new Marker(() -> {
				claw.setAlgaeState(Claw.AlgaeState.SHOOT_FAST);
			}, .3),
			new Marker(() -> {
				elevator.setState(Elevator.State.ALGAE_REMOVAL_HIGH);
				claw.setState(Claw.State.ALGAE_EXTRACTION);
				claw.setAlgaeState(Claw.AlgaeState.INTAKE);
			}, .6)
		));

		// Alignment to pickup high algae
		addBlock(new Align(Relation.CENTER, AlignableTarget.ALGAE_HIGH, 0.5).command());
		
		// Drive to center and shoot
		addBlock(new TorqueFollowPath("CNET_4", drivebase).withMarkers(
			new Marker(() -> {
				elevator.setState(Elevator.State.NET);
				claw.setState(Claw.State.NET);
				claw.setAlgaeState(Claw.AlgaeState.OFF);
			}, .2),
			new Marker(() -> {
				claw.setAlgaeState(Claw.AlgaeState.SHOOT_FAST);
			}, .7)
		));

		// Go to stow at end
		addBlock(new TorqueRun(() -> {
			claw.setState(Claw.State.STOW);
			elevator.setState(Elevator.State.STOW);
			claw.setAlgaeState(Claw.AlgaeState.OFF);
		}));
	}
}

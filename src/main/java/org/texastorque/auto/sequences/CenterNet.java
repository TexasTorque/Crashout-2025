package org.texastorque.auto.sequences;

import org.texastorque.Field.AlignPosition.AlignableTarget;
import org.texastorque.Field.AlignPosition.Relation;
import org.texastorque.auto.routines.Align;
import org.texastorque.auto.routines.Quickswap;
import org.texastorque.Subsystems;
import org.texastorque.subsystems.Claw;
import org.texastorque.subsystems.Elevator;
import org.texastorque.subsystems.Claw.AlgaeState;
import org.texastorque.torquelib.auto.TorqueSequence;
import org.texastorque.torquelib.auto.commands.TorqueFollowPath;
import org.texastorque.torquelib.auto.commands.TorqueRun;
import org.texastorque.torquelib.auto.commands.TorqueWaitTime;
import org.texastorque.torquelib.auto.commands.TorqueWaitUntil;
import org.texastorque.torquelib.auto.marker.Marker;

public class CenterNet extends TorqueSequence implements Subsystems {

	public CenterNet() {
		// addBlock(new Push().command());

        // Drive center to far right
		addBlock(new TorqueFollowPath("CNET_1", drivebase).withMarkers(
			new Marker(() -> {
				elevator.setState(Elevator.State.SCORE_L4);
				claw.setState(Claw.State.SCORE_L4);
			}, .2)
		));

		// Alignment and scoreee
		addBlock(new Align(Relation.RIGHT, AlignableTarget.L4, 0.5).command()); // Alignt time is low as path drives close to align target
		addBlock(new TorqueRun(() -> claw.setCoralState(Claw.CoralState.SHOOT)));
		addBlock(new TorqueRun(() -> claw.setAlgaeState(Claw.AlgaeState.INTAKE)));
		addBlock(new TorqueWaitTime(0.5));

		// Pick up low algae
		addBlock(new Align(Relation.CENTER, AlignableTarget.ALGAE_LOW, 1.5).command());
		addBlock(new TorqueRun(() -> {
			elevator.setState(Elevator.State.ALGAE_REMOVAL_LOW);
			claw.setState(Claw.State.ALGAE_EXTRACTION);
		}));
		addBlock(new TorqueWaitTime(0.5));


		// Shoot algae
		addBlock(new TorqueFollowPath("CNET_2", drivebase).withMarkers(
			new Marker(() -> {
				claw.setAlgaeState(Claw.AlgaeState.OFF);
				elevator.setState(Elevator.State.NET);
				claw.setState(Claw.State.NET);
			}, .2),
			new Marker(() -> {
				claw.setAlgaeState(Claw.AlgaeState.SHOOT_FAST);
			}, .8)
		));

		// Reset to pickup
		addBlock(new TorqueRun(() -> {
			elevator.setState(Elevator.State.ALGAE_REMOVAL_HIGH);
			claw.setState(Claw.State.ALGAE_EXTRACTION);
			claw.setAlgaeState(AlgaeState.INTAKE);
		}));


		// Drive to far left
		addBlock(new TorqueFollowPath("CNET_3", drivebase));

		// Pick up high algae
		addBlock(new Align(Relation.CENTER, AlignableTarget.ALGAE_HIGH, 0.8).command());

		// Drive to center net and shoot
		addBlock(new TorqueFollowPath("CNET_4", drivebase).withMarkers(
			new Marker(() -> {
				elevator.setState(Elevator.State.NET);
				claw.setState(Claw.State.NET);
				claw.setAlgaeState(Claw.AlgaeState.OFF);
			}, .2),
			new Marker(() -> {
				claw.setAlgaeState(Claw.AlgaeState.SHOOT_FAST);
			}, .8)
		));

		// Reset to pickup 
		addBlock(new TorqueRun(() -> {
			elevator.setState(Elevator.State.ALGAE_REMOVAL_HIGH);
			claw.setState(Claw.State.ALGAE_EXTRACTION);
			claw.setAlgaeState(AlgaeState.INTAKE);
		}));

		// Drive far right side
		addBlock(new TorqueFollowPath("CNET_5", drivebase));

		// Alignment to pickup high algae
		addBlock(new Align(Relation.CENTER, AlignableTarget.ALGAE_HIGH, 0.5).command());
		
		// Drive to center and shoot
		addBlock(new TorqueFollowPath("CNET_6", drivebase).withMarkers(
			new Marker(() -> {
				elevator.setState(Elevator.State.NET);
				claw.setState(Claw.State.NET);
			}, .2),
			new Marker(() -> {
				claw.setAlgaeState(Claw.AlgaeState.SHOOT_FAST);
			}, .8)
		));

		// Go to stow at end
		addBlock(new TorqueRun(() -> {
			claw.setState(Claw.State.STOW);
			elevator.setState(Elevator.State.STOW);
			claw.setAlgaeState(Claw.AlgaeState.OFF);
			claw.setCoralState(Claw.CoralState.OFF);
		}));
		
	}
}

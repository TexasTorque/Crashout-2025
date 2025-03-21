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
		addBlock(new Align(Relation.RIGHT, AlignableTarget.L4, 1.7).command());
		addBlock(new TorqueRun(() -> claw.setCoralState(Claw.CoralState.SHOOT)));

		// Pick up low algae
		addBlock(new Align(Relation.CENTER, AlignableTarget.ALGAE_LOW, 1.7).command());
		addBlock(new TorqueRun(() -> {
			elevator.setState(Elevator.State.ALGAE_REMOVAL_LOW);
			claw.setAlgaeState(AlgaeState.INTAKE);
			claw.setState(Claw.State.ALGAE_EXTRACTION);
		}));


		// Drive far right to center net
		addBlock(new TorqueFollowPath("CNET_2", drivebase).withMarkers(
			new Marker(() -> {
				elevator.setState(Elevator.State.NET);
				claw.setState(Claw.State.NET);
			}, .2),
			new Marker(() -> {
				claw.setAlgaeState(Claw.AlgaeState.SHOOT_FAST);
			}, .8)
		));

		addBlock(new TorqueRun(() -> claw.setAlgaeState(Claw.AlgaeState.OFF)));

		addBlock(new TorqueRun(() -> {
			elevator.setState(Elevator.State.ALGAE_REMOVAL_HIGH);
			claw.setState(Claw.State.ALGAE_EXTRACTION);
		}));

		addBlock(new TorqueRun(() -> claw.setAlgaeState(Claw.AlgaeState.INTAKE)));

		// Drive to far left
		addBlock(new TorqueFollowPath("CNET_3", drivebase));

		// Alignment
		addBlock(new Align(Relation.CENTER, AlignableTarget.ALGAE_HIGH, 1.7).command());

		// Pick up high algae
		addBlock(new Align(Relation.CENTER, AlignableTarget.ALGAE_HIGH, 1.7).command());
		addBlock(new TorqueRun(() -> {
			elevator.setState(Elevator.State.ALGAE_REMOVAL_HIGH);
			claw.setAlgaeState(AlgaeState.INTAKE);
			claw.setState(Claw.State.ALGAE_EXTRACTION);
		}));

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

		// Drive right side
		addBlock(new TorqueFollowPath("CNET_5", drivebase));

		// Alignment
		addBlock(new Align(Relation.CENTER, AlignableTarget.ALGAE_HIGH, 1.7).command());

		// Pick up high algae
		addBlock(new Align(Relation.CENTER, AlignableTarget.ALGAE_HIGH, 1.7).command());
		addBlock(new TorqueRun(() -> {
			elevator.setState(Elevator.State.ALGAE_REMOVAL_HIGH);
			claw.setAlgaeState(AlgaeState.INTAKE);
			claw.setState(Claw.State.ALGAE_EXTRACTION);
		}));

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


		
	}
}

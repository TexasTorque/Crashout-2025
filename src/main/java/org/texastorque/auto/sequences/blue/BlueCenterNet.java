package org.texastorque.auto.sequences.blue;

import org.texastorque.Field.AlignPosition.Relation;
import org.texastorque.auto.routines.Align;
import org.texastorque.auto.routines.Quickswap;
import org.texastorque.Subsystems;
import org.texastorque.subsystems.Claw;
import org.texastorque.subsystems.Elevator;
import org.texastorque.torquelib.auto.TorqueSequence;
import org.texastorque.torquelib.auto.commands.TorqueFollowPath;
import org.texastorque.torquelib.auto.commands.TorqueRun;
import org.texastorque.torquelib.auto.commands.TorqueWaitTime;
import org.texastorque.torquelib.auto.commands.TorqueWaitUntil;
import org.texastorque.torquelib.auto.marker.Marker;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class BlueCenterNet extends TorqueSequence implements Subsystems {

	public BlueCenterNet() {
		// Elevator & claw setpoints
        addBlock(new TorqueRun(() -> {
            elevator.setState(Elevator.State.ALGAE_REMOVAL_HIGH);
            claw.setState(Claw.State.ALGAE_EXTRACTION);
        }));

        // Drive center to far left
		addBlock(new TorqueFollowPath("BLUE_CTR_FL", drivebase));

		// Quickswap
		addBlock(new Quickswap(new Pose2d(5.365, 5.325, Rotation2d.fromDegrees(-120))).command());

		// Drive far left to net
		addBlock(new TorqueFollowPath("BLUE_FL_NET", drivebase).withMarkers(
			new Marker(() -> {
				elevator.setState(Elevator.State.NET);
				claw.setState(Claw.State.NET);
			}, .9)
		));

		addBlock(new TorqueWaitUntil(() -> elevator.isAtState() && claw.isAtState()));

		addBlock(new TorqueRun(() -> claw.setAlgaeState(Claw.AlgaeState.SHOOT)));
		addBlock(new TorqueWaitTime(.5));
		addBlock(new TorqueRun(() -> claw.setAlgaeState(Claw.AlgaeState.OFF)));

		addBlock(new TorqueRun(() -> {
			elevator.setState(Elevator.State.ALGAE_REMOVAL_LOW);
			claw.setState(Claw.State.ALGAE_EXTRACTION);
		}));

		addBlock(new TorqueFollowPath("BLUE_NET_FF", drivebase));

		addBlock(new TorqueWaitUntil(() -> elevator.isNearState() && claw.isNearState()));

		addBlock(new TorqueRun(() -> claw.setAlgaeState(Claw.AlgaeState.INTAKE)));

		// Alignment
        addBlock(new Align(Relation.CENTER, 1.2).command());

		addBlock(new TorqueRun(() -> claw.setAlgaeState(Claw.AlgaeState.OFF)));

		addBlock(new TorqueFollowPath("BLUE_FF_NET", drivebase).withMarkers(
			new Marker(() -> {
				elevator.setState(Elevator.State.NET);
				claw.setState(Claw.State.NET);
			}, .9)
		));

		addBlock(new TorqueWaitUntil(() -> elevator.isAtState() && claw.isAtState()));

		addBlock(new TorqueRun(() -> claw.setAlgaeState(Claw.AlgaeState.SHOOT)));
		addBlock(new TorqueWaitTime(.5));
		addBlock(new TorqueRun(() -> claw.setAlgaeState(Claw.AlgaeState.OFF)));

		addBlock(new TorqueRun(() -> {
			elevator.setState(Elevator.State.STOW);
			claw.setState(Claw.State.STOW);
		}));
	}
}

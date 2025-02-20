package org.texastorque.auto.sequences;

import org.texastorque.AlignPose2d.Relation;
import org.texastorque.Subsystems;
import org.texastorque.subsystems.Claw;
import org.texastorque.subsystems.Drivebase;
import org.texastorque.subsystems.Elevator;
import org.texastorque.torquelib.auto.TorqueSequence;
import org.texastorque.torquelib.auto.commands.TorqueFollowPath;
import org.texastorque.torquelib.auto.commands.TorqueRun;
import org.texastorque.torquelib.auto.commands.TorqueWaitTime;
import org.texastorque.torquelib.auto.commands.TorqueWaitUntil;
import org.texastorque.torquelib.auto.marker.Marker;
import org.texastorque.torquelib.base.TorqueMode;

public class CenterNet extends TorqueSequence implements Subsystems {

	// E Algae Extraction -> Net -> D Algae Extraction -> Net -> C Algae Extraction
	public CenterNet() {
		// Elevator & claw setpoints
        addBlock(new TorqueRun(() -> {
            elevator.setState(Elevator.State.ALGAE_REMOVAL_HIGH);
            claw.setState(Claw.State.ALGAE_EXTRACTION);
        }));

        // Drive center to far left
		addBlock(new TorqueFollowPath("CTR_FL", drivebase));

		// Alignment
        addBlock(new TorqueRun(() -> {
            drivebase.setRelation(Relation.CENTER);
            drivebase.setState(Drivebase.State.ALIGN);
        }));
        addBlock(new TorqueWaitUntil(() -> drivebase.isAligned(TorqueMode.AUTO)));
        addBlock(new TorqueRun(() -> drivebase.setState(Drivebase.State.ROBOT_RELATIVE)));

        // Algae extraction
        addBlock(new TorqueRun(() -> claw.setAlgaeState(Claw.AlgaeState.INTAKE)));
        addBlock(new TorqueWaitTime(.5)); // Wait until we intake algae
        addBlock(new TorqueRun(() -> claw.setAlgaeState(Claw.AlgaeState.OFF)));

		// Drive far left to net
		addBlock(new TorqueFollowPath("FL_NET", drivebase).withMarkers(
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

		addBlock(new TorqueWaitTime(1));

		addBlock(new TorqueFollowPath("NET_FF", drivebase));

		// Elevator & claw setpoints
        addBlock(new TorqueRun(() -> {
            elevator.setState(Elevator.State.ALGAE_REMOVAL_LOW);
            claw.setState(Claw.State.ALGAE_EXTRACTION);
        }));

		addBlock(new TorqueWaitUntil(() -> elevator.isNearState() && claw.isNearState()));

		// Alignment
        addBlock(new TorqueRun(() -> {
            drivebase.setRelation(Relation.CENTER);
            drivebase.setState(Drivebase.State.ALIGN);
        }));
        addBlock(new TorqueWaitUntil(() -> drivebase.isAligned(TorqueMode.AUTO)));
        addBlock(new TorqueRun(() -> drivebase.setState(Drivebase.State.ROBOT_RELATIVE)));

		addBlock(new TorqueFollowPath("FF_NET", drivebase).withMarkers(
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

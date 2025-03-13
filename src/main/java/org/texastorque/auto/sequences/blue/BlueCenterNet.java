package org.texastorque.auto.sequences.blue;

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
import org.texastorque.torquelib.auto.commands.TorqueWaitTime;
import org.texastorque.torquelib.auto.commands.TorqueWaitUntil;
import org.texastorque.torquelib.auto.marker.Marker;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class BlueCenterNet extends TorqueSequence implements Subsystems {

	public BlueCenterNet() {
		// addBlock(new TorqueRun(() -> drivebase.setPose(new Pose2d(7.318, 3.972, Rotation2d.fromDegrees(180)))));

		// addBlock(new Push().command());

		// Elevator & claw setpoints
        addBlock(new TorqueRun(() -> {
            elevator.setState(Elevator.State.ALGAE_REMOVAL_HIGH);
            claw.setState(Claw.State.ALGAE_EXTRACTION);
        }));

        // Drive center to far right
		addBlock(new TorqueFollowPath("BCNET_1", drivebase));

		// Quickswap
		addBlock(new Quickswap(true).command());

		// Drive shot setpoint to FF
		addBlock(new TorqueFollowPath("BCNET_2", drivebase).withMarkers(
			new Marker(() -> {
				elevator.setState(Elevator.State.NET);
				claw.setState(Claw.State.NET);
			}, .2)
		));

		addBlock(new TorqueWaitUntil(() -> elevator.isAtState() && claw.isAtState()));

		// Shoot algae into net
		addBlock(new TorqueRun(() -> claw.setAlgaeState(Claw.AlgaeState.SHOOT)));
		addBlock(new TorqueWaitTime(.5));
		addBlock(new TorqueRun(() -> claw.setAlgaeState(Claw.AlgaeState.OFF)));

		addBlock(new TorqueRun(() -> {
			elevator.setState(Elevator.State.ALGAE_REMOVAL_LOW);
			claw.setState(Claw.State.ALGAE_EXTRACTION);
		}));

		addBlock(new TorqueWaitUntil(() -> elevator.isNearState() && claw.isNearState()));

		addBlock(new TorqueRun(() -> claw.setAlgaeState(Claw.AlgaeState.INTAKE)));

		// Alignment
		addBlock(new Align(Relation.CENTER, AlignableTarget.ALGAE_LOW, 1.2).command());

		addBlock(new TorqueRun(() -> claw.setAlgaeState(Claw.AlgaeState.OFF)));

		addBlock(new TorqueRun(() -> {
			elevator.setState(Elevator.State.NET);
			claw.setState(Claw.State.NET);
		}));

		// Alignment
        addBlock(new Align(() -> new Pose2d(6.085, 4.033, Rotation2d.fromDegrees(216.123)), 1).command());

		addBlock(new TorqueWaitUntil(() -> elevator.isAtState() && claw.isAtState()));

		addBlock(new TorqueRun(() -> claw.setAlgaeState(Claw.AlgaeState.SHOOT)));
		addBlock(new TorqueWaitTime(.5));
		addBlock(new TorqueRun(() -> claw.setAlgaeState(Claw.AlgaeState.OFF)));

		addBlock(new TorqueRun(() -> {
			elevator.setState(Elevator.State.ALGAE_REMOVAL_HIGH);
			claw.setState(Claw.State.ALGAE_EXTRACTION);
		}));

		addBlock(new TorqueFollowPath("BCNET_3", drivebase));

		addBlock(new TorqueWaitUntil(() -> elevator.isAtState() && claw.isAtState()));

		addBlock(new TorqueRun(() -> claw.setAlgaeState(Claw.AlgaeState.INTAKE)));

		// Alignment
		addBlock(new Align(Relation.CENTER, AlignableTarget.ALGAE_HIGH, 1.2).command());

		addBlock(new TorqueRun(() -> claw.setAlgaeState(Claw.AlgaeState.OFF)));

		addBlock(new TorqueFollowPath("BCNET_4", drivebase).withMarkers(
			new Marker(() -> {
				elevator.setState(Elevator.State.NET);
			claw.setState(Claw.State.NET);
			}, .2)
		));

		addBlock(new TorqueWaitUntil(() -> elevator.isAtState() && claw.isAtState()));

		addBlock(new TorqueRun(() -> claw.setAlgaeState(Claw.AlgaeState.SHOOT)));
		addBlock(new TorqueWaitTime(.5));
		addBlock(new TorqueRun(() -> {
			claw.setAlgaeState(Claw.AlgaeState.OFF);

			elevator.setState(Elevator.State.CORAL_HP);
			claw.setState(Claw.State.CORAL_HP);
		}));
	}
}

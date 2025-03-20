package org.texastorque.auto.sequences;

import org.texastorque.Field.AlignPosition.AlignableTarget;
import org.texastorque.Field.AlignPosition.Relation;
import org.texastorque.Subsystems;
import org.texastorque.auto.routines.Align;
import org.texastorque.subsystems.Claw;
import org.texastorque.subsystems.Elevator;
import org.texastorque.subsystems.Elevator.State;
import org.texastorque.torquelib.auto.TorqueSequence;
import org.texastorque.torquelib.auto.commands.TorqueFollowPath;
import org.texastorque.torquelib.auto.commands.TorqueRun;
import org.texastorque.torquelib.auto.commands.TorqueWaitTime;
import org.texastorque.torquelib.auto.commands.TorqueWaitUntil;
import org.texastorque.torquelib.auto.marker.Marker;

public class RightL4Auto extends TorqueSequence implements Subsystems {
    
    public RightL4Auto() {
        addBlock(new TorqueRun(() -> {
            elevator.setState(Elevator.State.STOW);
            claw.setState(Claw.State.STOW);
        }));

        // Drive right to close right
        addBlock(new TorqueFollowPath("BRL4_1", drivebase).withMarkers(
            new Marker(() -> {
                elevator.setState(Elevator.State.SCORE_L4);
                claw.setState(Claw.State.SCORE_L4);
            }, .5)
        ));

        addBlock(new TorqueWaitUntil(() -> elevator.isNearState() && claw.isNearState()));

        // Alignment
        addBlock(new Align(Relation.LEFT, AlignableTarget.L4, 2).command());

        // Coral placement
        addBlock(new TorqueRun(() -> claw.setCoralState(Claw.CoralState.SHOOT)));
        addBlock(new TorqueWaitTime(.5)); // Wait until we shoot coral
        addBlock(new TorqueRun(() -> claw.setCoralState(Claw.CoralState.OFF)));

        // Drive close right to coral station right
        addBlock(new TorqueFollowPath("BRL4_2", drivebase).withMarkers(
            new Marker(() -> {
                elevator.setState(State.CORAL_HP);
                claw.setState(Claw.State.CORAL_HP);
            }, .2)
        ));

        addBlock(new TorqueRun(() -> claw.setCoralState(Claw.CoralState.INTAKE)));

        // Alignment
        addBlock(new Align(Relation.CENTER, AlignableTarget.CORAL_STATION, 1.2).command());

        // Pickup coral from coral station
        addBlock(new TorqueWaitTime(.5));
        addBlock(new TorqueRun(() -> claw.setCoralState(Claw.CoralState.OFF)));

        // Drive coral station right to close right
        addBlock(new TorqueFollowPath("BRL4_3", drivebase).withMarkers(
            new Marker(() -> {
                elevator.setState(Elevator.State.SCORE_L4);
                claw.setState(Claw.State.SCORE_L4);
            }, .2)
        ));

        // Alignment
        addBlock(new Align(Relation.RIGHT, AlignableTarget.L4, 2).command());

        // Coral placement
        addBlock(new TorqueRun(() -> claw.setCoralState(Claw.CoralState.SHOOT)));
        addBlock(new TorqueWaitTime(.5)); // Wait until we shoot coral
        addBlock(new TorqueRun(() -> claw.setCoralState(Claw.CoralState.OFF)));

        // Move back to avoid claw hitting reef poles
        addBlock(new TorqueRun(() -> {
            perception.setRelation(Relation.NONE);
            perception.setDesiredAlignTarget(AlignableTarget.NONE);
        }));
        addBlock(new Align(() -> perception.getAlignPose(), 1).command());

        // Move to algae setpoints
        addBlock(new TorqueRun(() -> {
            elevator.setState(Elevator.State.ALGAE_REMOVAL_LOW);
            claw.setState(Claw.State.ALGAE_EXTRACTION);
            claw.setAlgaeState(Claw.AlgaeState.INTAKE);
        }));

        addBlock(new TorqueWaitUntil(() -> elevator.isNearState() && claw.isNearState()));

        // Alignment
        addBlock(new Align(Relation.CENTER, AlignableTarget.ALGAE_LOW, .75).command());

        addBlock(new TorqueRun(() -> claw.setAlgaeState(Claw.AlgaeState.OFF)));

        // Move back to avoid claw hitting reef poles
        addBlock(new TorqueRun(() -> {
            perception.setRelation(Relation.NONE);
            perception.setDesiredAlignTarget(AlignableTarget.NONE);
        }));
        addBlock(new Align(() -> perception.getAlignPose(), 1).command());
    }
}

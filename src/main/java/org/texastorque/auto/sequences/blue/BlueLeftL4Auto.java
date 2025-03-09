package org.texastorque.auto.sequences.blue;

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

public class BlueLeftL4Auto extends TorqueSequence implements Subsystems {
    
    public BlueLeftL4Auto() {
        // Drive left to close left
        addBlock(new TorqueFollowPath("BLUE_LFT_CL", drivebase).withMarkers(
            new Marker(() -> {
                elevator.setState(Elevator.State.SCORE_L4);
                claw.setState(Claw.State.SCORE_L4);
            }, .3)
        ));

        addBlock(new TorqueWaitUntil(() -> elevator.isNearState() && claw.isNearState()));

        // Alignment
        addBlock(new Align(Relation.LEFT, AlignableTarget.L4, 2).command());

        // Coral placement
        addBlock(new TorqueRun(() -> claw.setCoralState(Claw.CoralState.SHOOT)));
        addBlock(new TorqueWaitTime(.5)); // Wait until we shoot coral
        addBlock(new TorqueRun(() -> claw.setCoralState(Claw.CoralState.OFF)));

        // Drive close left to coral station left
        addBlock(new TorqueFollowPath("BLUE_CL_CSL", drivebase).withMarkers(
            new Marker(() -> {
                elevator.setState(State.CORAL_HP);
                claw.setState(Claw.State.CORAL_HP);
            }, .2)
        ));

        addBlock(new TorqueRun(() -> claw.setCoralState(Claw.CoralState.INTAKE)));

        // Alignment
        addBlock(new Align(Relation.CENTER, AlignableTarget.CORAL_STATION, 1.2).command());

        // Pickup coral from coral station
        addBlock(new TorqueWaitTime(1.5));
        addBlock(new TorqueRun(() -> claw.setCoralState(Claw.CoralState.OFF)));

        // Drive coral station left to close left
        addBlock(new TorqueFollowPath("BLUE_CSL_CL", drivebase).withMarkers(
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
    }
}

package org.texastorque.auto.sequences.red;

import org.texastorque.Field.AlignPosition.AlignableTarget;
import org.texastorque.Field.AlignPosition.Relation;
import org.texastorque.Subsystems;
import org.texastorque.auto.routines.Align;
import org.texastorque.auto.routines.Quickswap;
import org.texastorque.subsystems.Claw;
import org.texastorque.subsystems.Elevator;
import org.texastorque.subsystems.Elevator.State;
import org.texastorque.torquelib.auto.TorqueSequence;
import org.texastorque.torquelib.auto.commands.TorqueFollowPath;
import org.texastorque.torquelib.auto.commands.TorqueRun;
import org.texastorque.torquelib.auto.commands.TorqueWaitTime;
import org.texastorque.torquelib.auto.commands.TorqueWaitUntil;
import org.texastorque.torquelib.auto.marker.Marker;

public class RedRightQuickswapAuto extends TorqueSequence implements Subsystems {
    
    public RedRightQuickswapAuto() {
        // Drive right to close right
        addBlock(new TorqueFollowPath("RED_RGT_CR", drivebase).withMarkers(
            new Marker(() -> {
                elevator.setState(State.ALGAE_REMOVAL_LOW);
                claw.setState(Claw.State.ALGAE_EXTRACTION);
            }, .3)
        ));

        addBlock(new TorqueWaitUntil(() -> elevator.isNearState() && claw.isNearState()));

        // Quickswap
        addBlock(new Quickswap().command());

        // Drive close right to coral station right
        addBlock(new TorqueFollowPath("RED_CR_CSR", drivebase).withMarkers(
            new Marker(() -> {
                claw.setAlgaeState(Claw.AlgaeState.SHOOT_SLOW);
            }, .5),
            new Marker(() -> {
                claw.setAlgaeState(Claw.AlgaeState.OFF);
                elevator.setState(State.CORAL_HP);
                claw.setState(Claw.State.CORAL_HP);
            }, .7)
        ));

        addBlock(new TorqueRun(() -> claw.setCoralState(Claw.CoralState.INTAKE)));

        // Alignment
        addBlock(new Align(Relation.CENTER, AlignableTarget.CORAL_STATION, 1.2).command());

        // Pickup coral from coral station
        addBlock(new TorqueWaitTime(1.5));
        addBlock(new TorqueRun(() -> claw.setCoralState(Claw.CoralState.OFF)));

        // Drive coral station right to close right
        addBlock(new TorqueFollowPath("RED_CSR_CR", drivebase).withMarkers(
            new Marker(() -> {
                elevator.setState(Elevator.State.SCORE_L3);
                claw.setState(Claw.State.SCORE_L3);
            }, .2)
        ));

        // Alignment
        addBlock(new Align(Relation.RIGHT, AlignableTarget.L3, 2).command());

        // Coral placement
        addBlock(new TorqueRun(() -> claw.setCoralState(Claw.CoralState.SHOOT)));
        addBlock(new TorqueWaitTime(.5)); // Wait until we shoot coral
        addBlock(new TorqueRun(() -> claw.setCoralState(Claw.CoralState.OFF)));
    }
}

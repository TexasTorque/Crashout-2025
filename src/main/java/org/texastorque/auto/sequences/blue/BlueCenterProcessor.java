package org.texastorque.auto.sequences.blue;

import org.texastorque.Subsystems;
import org.texastorque.Field.AlignPosition.AlignableTarget;
import org.texastorque.Field.AlignPosition.Relation;
import org.texastorque.auto.routines.Align;
import org.texastorque.auto.routines.Quickswap;
import org.texastorque.subsystems.Claw;
import org.texastorque.subsystems.Elevator;
import org.texastorque.torquelib.auto.TorqueSequence;
import org.texastorque.torquelib.auto.commands.TorqueFollowPath;
import org.texastorque.torquelib.auto.commands.TorqueRun;
import org.texastorque.torquelib.auto.commands.TorqueWaitTime;
import org.texastorque.torquelib.auto.commands.TorqueWaitUntil;
import org.texastorque.torquelib.auto.marker.Marker;

public class BlueCenterProcessor extends TorqueSequence implements Subsystems {
    
    public BlueCenterProcessor() {
        // Elevator & claw setpoints
        addBlock(new TorqueRun(() -> {
            elevator.setState(Elevator.State.ALGAE_REMOVAL_LOW);
            claw.setState(Claw.State.ALGAE_EXTRACTION);
        }));

        // Drive center to far
        addBlock(new TorqueFollowPath("BCPSR_1", drivebase));

        addBlock(new TorqueWaitUntil(() -> elevator.isNearState() && claw.isNearState()));

        // Quickswap
        addBlock(new Quickswap(true).command());

        // Drive far to processor
        addBlock(new TorqueFollowPath("BCPSR_2", drivebase).withMarkers(
            new Marker(() -> {
                elevator.setState(Elevator.State.PROCESSOR);
                claw.setState(Claw.State.PROCESSOR);
            }, .2)
        ));

        addBlock(new TorqueWaitUntil(() -> elevator.isNearState() && claw.isNearState()));

        // Score algae in processor
        addBlock(new TorqueRun(() -> claw.setAlgaeState(Claw.AlgaeState.SHOOT)));
        addBlock(new TorqueWaitTime(.5)); // Wait until we shoot algae
        addBlock(new TorqueRun(() -> claw.setAlgaeState(Claw.AlgaeState.OFF)));
        
        // Drive processor to far right
        addBlock(new TorqueFollowPath("BCPSR_3", drivebase).withMarkers(
            new Marker(() -> {
                elevator.setState(Elevator.State.ALGAE_REMOVAL_HIGH);
                claw.setState(Claw.State.ALGAE_EXTRACTION);
                claw.setAlgaeState(Claw.AlgaeState.INTAKE);
            }, .3)
        ));

        // Align
        addBlock(new Align(Relation.CENTER, AlignableTarget.ALGAE_HIGH, 1).command());

        // Move back to avoid claw hitting reef poles
        addBlock(new TorqueRun(() -> {
            perception.setRelation(Relation.NONE);
            perception.setDesiredAlignTarget(AlignableTarget.NONE);
        }));
        addBlock(new Align(() -> perception.getAlignPose(), 1).command());
    }
}

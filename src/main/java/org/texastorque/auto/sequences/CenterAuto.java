package org.texastorque.auto.sequences;

import org.texastorque.Subsystems;
import org.texastorque.subsystems.Claw;
import org.texastorque.subsystems.Elevator;
import org.texastorque.torquelib.auto.TorqueSequence;
import org.texastorque.torquelib.auto.commands.TorqueFollowPath;
import org.texastorque.torquelib.auto.commands.TorqueRun;
import org.texastorque.torquelib.auto.commands.TorqueWaitTime;
import org.texastorque.torquelib.auto.marker.Marker;

public class CenterAuto extends TorqueSequence implements Subsystems {
    public CenterAuto() {
        // Drive center to far
        addBlock(new TorqueFollowPath("CTR_FF", drivebase).withMarkers(
            new Marker(() -> {
                elevator.setState(Elevator.State.ALGAE_REMOVAL_LOW);
                claw.setState(Claw.State.ALGAE_EXTRACTION);
                claw.setAlgaeState(Claw.AlgaeState.INTAKE);
            }, .2)
        ));

        // Algae extraction & coral placement

        // Drive far to processor
        addBlock(new TorqueFollowPath("FF_PSR", drivebase).withMarkers(

        ));

        // Score algae in processor
        addBlock(new TorqueRun(() -> claw.setAlgaeState(Claw.AlgaeState.INTAKE)));
        addBlock(new TorqueWaitTime(1)); // Wait until we don't have algae
        addBlock(new TorqueRun(() -> claw.setAlgaeState(Claw.AlgaeState.OFF)));
        addBlock(new TorqueRun(() -> elevator.setState(Elevator.State.ALGAE_REMOVAL_HIGH)));

        // Drive processor to far right
        addBlock(new TorqueFollowPath("PSR_FR", drivebase).withMarkers(

        ));

        // Algae extraction

        // Drive far right to processor
        addBlock(new TorqueFollowPath("FR_PSR", drivebase).withMarkers(
            
        ));

        // Score algae in processor
        addBlock(new TorqueRun(() -> claw.setAlgaeState(Claw.AlgaeState.INTAKE)));
        addBlock(new TorqueWaitTime(1)); // Wait until we don't have algae
        addBlock(new TorqueRun(() -> claw.setAlgaeState(Claw.AlgaeState.OFF)));
        addBlock(new TorqueRun(() -> elevator.setState(Elevator.State.ALGAE_REMOVAL_HIGH)));
    }
}

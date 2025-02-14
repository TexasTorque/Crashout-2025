package org.texastorque.auto.sequences;

import org.texastorque.Subsystems;
import org.texastorque.torquelib.auto.TorqueSequence;
import org.texastorque.torquelib.auto.commands.TorqueFollowPath;
import org.texastorque.torquelib.auto.commands.TorqueRun;
import org.texastorque.torquelib.auto.commands.TorqueWaitUntil;
import org.texastorque.torquelib.auto.marker.Marker;
import org.texastorque.torquelib.base.TorqueMode;
import org.texastorque.torquelib.auto.commands.TorqueWaitTime;
import org.texastorque.AlignPose2d.Relation;
import org.texastorque.subsystems.Drivebase;
import org.texastorque.subsystems.Claw;
import org.texastorque.subsystems.Elevator;

public class LeftAuto extends TorqueSequence implements Subsystems {
    
    public LeftAuto() {
        // Drive left to close left
        addBlock(new TorqueFollowPath("LFT_CL", drivebase).withMarkers(
            new Marker(() -> {
                elevator.setState(Elevator.State.ALGAE_REMOVAL_LOW);
                claw.setState(Claw.State.ALGAE_EXTRACTION);
            }, .1)
        ));

        // Alignment
        addBlock(new TorqueRun(() -> drivebase.setRelation(Relation.CENTER)));
        addBlock(new TorqueRun(() -> drivebase.setState(Drivebase.State.ALIGN)));
        addBlock(new TorqueWaitUntil(() -> drivebase.isAligned(TorqueMode.AUTO)));
        addBlock(new TorqueRun(() -> drivebase.setState(Drivebase.State.ROBOT_RELATIVE)));

        // Algae extraction
        addBlock(new TorqueWaitUntil(() -> elevator.isAtState()));
        addBlock(new TorqueRun(() -> claw.setAlgaeState(Claw.AlgaeState.INTAKE)));
        addBlock(new TorqueWaitTime(.5)); // Wait until we intake algae
        addBlock(new TorqueRun(() -> claw.setAlgaeState(Claw.AlgaeState.OFF)));
        
        // Start moving subsystems while aligning
        addBlock(new TorqueRun(() -> elevator.setState(Elevator.State.SCORE_L3)));
        addBlock(new TorqueRun(() -> claw.setState(Claw.State.MID_SCORE)));

        // Alignment
        addBlock(new TorqueRun(() -> drivebase.setRelation(Relation.LEFT)));
        addBlock(new TorqueRun(() -> drivebase.setState(Drivebase.State.ALIGN)));
        addBlock(new TorqueWaitUntil(() -> drivebase.isAligned(TorqueMode.AUTO)));
        addBlock(new TorqueRun(() -> drivebase.setState(Drivebase.State.ROBOT_RELATIVE)));
        
        // Coral placement
        addBlock(new TorqueWaitUntil(() -> elevator.isAtState() && claw.isAtState()));
        addBlock(new TorqueRun(() -> claw.setCoralState(Claw.CoralState.SHOOT)));
        addBlock(new TorqueWaitTime(.5)); // Wait until we shoot coral
        addBlock(new TorqueRun(() -> claw.setCoralState(Claw.CoralState.OFF)));

        // Drive close left to coral station left
        addBlock(new TorqueFollowPath("CL_CSL", drivebase).withMarkers(
            new Marker(() -> {
                elevator.setState(Elevator.State.CORAL_HP);
                claw.setState(Claw.State.CORAL_HP);
                claw.setCoralState(Claw.CoralState.INTAKE);
                claw.coralSpike.reset();
            }, .1)
        ));

        // Pickup coral from coral station
        addBlock(new TorqueWaitTime(.5)); // Wait until we intake coral

        // Drive coral station left to close left
        addBlock(new TorqueFollowPath("CSL_CL", drivebase));

        // Coral placement

        // Drive close left to coral station left
        addBlock(new TorqueFollowPath("CL_CSL", drivebase));

        // Pickup coral from coral station

        // Drive coral station left to close left
        addBlock(new TorqueFollowPath("CSL_CL", drivebase));

        // Coral placement
    }
}

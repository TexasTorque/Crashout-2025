package org.texastorque.auto.sequences;

import org.texastorque.Subsystems;
import org.texastorque.AlignPose2d.Relation;
import org.texastorque.auto.routines.Align;
import org.texastorque.auto.routines.QuickSwap;
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

public class CenterProcessor extends TorqueSequence implements Subsystems {
    
    public CenterProcessor() {
        // Elevator & claw setpoints
        addBlock(new TorqueRun(() -> {
            elevator.setState(Elevator.State.ALGAE_REMOVAL_LOW);
            claw.setState(Claw.State.ALGAE_EXTRACTION);
        }));

        // Drive center to far
        addBlock(new TorqueFollowPath("CTR_FF", drivebase));

        addBlock(new TorqueWaitUntil(() -> elevator.isNearState() && claw.isNearState()));

        // Quickswap
        addBlock(new QuickSwap(new Pose2d(6.15, 4, Rotation2d.fromDegrees(180))).command());

        // Drive far to processor
        addBlock(new TorqueFollowPath("FF_PSR", drivebase).withMarkers(
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
        addBlock(new TorqueFollowPath("PSR_FR", drivebase).withMarkers(
            new Marker(() -> {
                elevator.setState(Elevator.State.ALGAE_REMOVAL_HIGH);
                claw.setState(Claw.State.ALGAE_EXTRACTION);
            }, .2)
        ));

        addBlock(new TorqueWaitUntil(() -> elevator.isNearState() && claw.isNearState()));

        // Alignment
        addBlock(new Align(Relation.CENTER).command());

        // Algae extraction
        addBlock(new TorqueRun(() -> claw.setAlgaeState(Claw.AlgaeState.INTAKE)));
        addBlock(new TorqueWaitTime(.5)); // Wait until we intake algae
        addBlock(new TorqueRun(() -> claw.setAlgaeState(Claw.AlgaeState.OFF)));
    }
}

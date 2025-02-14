package org.texastorque.auto.sequences;

import org.texastorque.Subsystems;
import org.texastorque.AlignPose2d.Relation;
import org.texastorque.subsystems.Claw;
import org.texastorque.subsystems.Elevator;
import org.texastorque.torquelib.auto.TorqueSequence;
import org.texastorque.torquelib.auto.commands.TorqueFollowPath;
import org.texastorque.torquelib.auto.commands.TorqueRun;
import org.texastorque.torquelib.auto.commands.TorqueWaitTime;
import org.texastorque.torquelib.auto.commands.TorqueWaitUntil;
import org.texastorque.torquelib.auto.marker.Marker;
import org.texastorque.torquelib.base.TorqueMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import org.texastorque.subsystems.Drivebase;

public class CenterAuto extends TorqueSequence implements Subsystems {
    
    public CenterAuto() {
        // Drive center to far
        addBlock(new TorqueFollowPath("CTR_FF", drivebase).withMarkers(
            // new Marker(() -> {
            //     elevator.setState(Elevator.State.ALGAE_REMOVAL_LOW);
            //     claw.setState(Claw.State.ALGAE_EXTRACTION);
            // }, 0)
        ));

        // addBlock(new TorqueWaitUntil(() -> elevator.isAtState()));
        // addBlock(new TorqueWaitUntil(() -> claw.isAtState()));

        // Alignment
        // addBlock(new TorqueRun(() -> drivebase.setRelation(Relation.CENTER)));
        // addBlock(new TorqueRun(() -> drivebase.setState(Drivebase.State.ALIGN)));
        // addBlock(new TorqueWaitUntil(() -> drivebase.isAligned(TorqueMode.AUTO)));
        // addBlock(new TorqueRun(() -> drivebase.setState(Drivebase.State.ROBOT_RELATIVE)));

        // Algae extraction
        // addBlock(new TorqueWaitUntil(() -> elevator.isAtState()));
        // addBlock(new TorqueRun(() -> claw.setAlgaeState(Claw.AlgaeState.INTAKE)));
        // addBlock(new TorqueWaitTime(.5)); // Wait until we intake algae
        // addBlock(new TorqueRun(() -> claw.setAlgaeState(Claw.AlgaeState.OFF)));

        // Start moving subsystems while aligning
        // addBlock(new TorqueRun(() -> elevator.setState(Elevator.State.SCORE_L3)));
        // addBlock(new TorqueRun(() -> claw.setState(Claw.State.MID_SCORE)));

        // Move back to avoid claw hitting reef poles
        // addBlock(new TorqueRun(() -> drivebase.setAlignPoseOverride(new Pose2d(6.1, 3.77, Rotation2d.fromDegrees(180)))));
        // addBlock(new TorqueWaitUntil(() -> drivebase.isAligned(TorqueMode.AUTO)));
        // addBlock(new TorqueRun(() -> drivebase.setAlignPoseOverride(null)));

        // addBlock(new TorqueWaitUntil(() -> elevator.isAtState()));
        // addBlock(new TorqueWaitUntil(() -> claw.isAtState()));

        // Alignment
        // addBlock(new TorqueRun(() -> drivebase.setRelation(Relation.LEFT)));
        // addBlock(new TorqueRun(() -> drivebase.setState(Drivebase.State.ALIGN)));
        // addBlock(new TorqueWaitUntil(() -> drivebase.isAligned(TorqueMode.AUTO)));
        // addBlock(new TorqueRun(() -> drivebase.setState(Drivebase.State.ROBOT_RELATIVE)));

        // Coral placement
        // addBlock(new TorqueWaitUntil(() -> elevator.isAtState() && claw.isAtState()));
        // addBlock(new TorqueRun(() -> claw.setCoralState(Claw.CoralState.SHOOT)));
        // addBlock(new TorqueWaitTime(.5)); // Wait until we shoot coral
        // addBlock(new TorqueRun(() -> claw.setCoralState(Claw.CoralState.OFF)));

        // Drive far to processor
        addBlock(new TorqueFollowPath("FF_PSR", drivebase).withMarkers(
            // new Marker(() -> {
            //     elevator.setState(Elevator.State.PROCESSOR);
            //     claw.setState(Claw.State.PROCESSOR);
            // }, .1)
        ));

        // Score algae in processor
        // addBlock(new TorqueWaitUntil(() -> elevator.isAtState()));
        // addBlock(new TorqueRun(() -> claw.setAlgaeState(Claw.AlgaeState.SHOOT)));
        // addBlock(new TorqueWaitTime(.5)); // Wait until we shoot algae
        // addBlock(new TorqueRun(() -> claw.setAlgaeState(Claw.AlgaeState.OFF)));

        // Drive processor to far right
        addBlock(new TorqueFollowPath("PSR_FR", drivebase).withMarkers(
            // new Marker(() -> {
            //     elevator.setState(Elevator.State.ALGAE_REMOVAL_HIGH);
            //     claw.setState(Claw.State.ALGAE_EXTRACTION);
            // }, .1)
        ));

        // addBlock(new TorqueWaitUntil(() -> elevator.isAtState()));
        // addBlock(new TorqueWaitUntil(() -> claw.isAtState()));

        // Alignment
        // addBlock(new TorqueRun(() -> drivebase.setRelation(Relation.CENTER)));
        // addBlock(new TorqueRun(() -> drivebase.setState(Drivebase.State.ALIGN)));
        // addBlock(new TorqueWaitUntil(() -> drivebase.isAligned(TorqueMode.AUTO)));
        // addBlock(new TorqueRun(() -> drivebase.setState(Drivebase.State.ROBOT_RELATIVE)));

        // Algae extraction
        // addBlock(new TorqueWaitUntil(() -> elevator.isAtState()));
        // addBlock(new TorqueRun(() -> claw.setAlgaeState(Claw.AlgaeState.INTAKE)));
        // addBlock(new TorqueWaitTime(.5)); // Wait until we intake algae
        // addBlock(new TorqueRun(() -> claw.setAlgaeState(Claw.AlgaeState.OFF)));

        // Drive far right to processor
        addBlock(new TorqueFollowPath("FR_PSR", drivebase).withMarkers(
            // new Marker(() -> {
            //     elevator.setState(Elevator.State.PROCESSOR);
            //     claw.setState(Claw.State.PROCESSOR);
            // }, .1)
        ));

        // Score algae in processor
        // addBlock(new TorqueWaitUntil(() -> elevator.isAtState()));
        // addBlock(new TorqueRun(() -> claw.setAlgaeState(Claw.AlgaeState.SHOOT)));
        // addBlock(new TorqueWaitTime(.5)); // Wait until we shoot algae
        // addBlock(new TorqueRun(() -> claw.setAlgaeState(Claw.AlgaeState.OFF)));
    }
}

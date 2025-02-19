package org.texastorque.auto.sequences;

import org.texastorque.Subsystems;
import org.texastorque.torquelib.auto.TorqueSequence;
import org.texastorque.torquelib.auto.commands.TorqueFollowPath;
import org.texastorque.torquelib.auto.commands.TorqueRun;
import org.texastorque.torquelib.auto.commands.TorqueWaitUntil;
import org.texastorque.torquelib.auto.marker.Marker;
import org.texastorque.torquelib.base.TorqueMode;
import org.texastorque.torquelib.swerve.TorqueSwerveSpeeds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import org.texastorque.torquelib.auto.commands.TorqueWaitTime;
import org.texastorque.AlignPose2d.Relation;
import org.texastorque.subsystems.Drivebase;
import org.texastorque.subsystems.Claw;
import org.texastorque.subsystems.Elevator;
import org.texastorque.subsystems.Elevator.State;

public class LeftAuto extends TorqueSequence implements Subsystems {
    
    public LeftAuto() {
        // Drive left to close left
        addBlock(new TorqueFollowPath("LFT_CL", drivebase).withMarkers(
            new Marker(() -> {
                elevator.setState(State.ALGAE_REMOVAL_LOW);
                claw.setState(Claw.State.ALGAE_EXTRACTION);
            }, .3)
        ));

        addBlock(new TorqueWaitUntil(() -> elevator.isNearState() && claw.isNearState()));

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

        // Move back to avoid claw hitting reef poles
        addBlock(new TorqueRun(() -> {
            drivebase.setAlignPoseOverride(new Pose2d(3.94, 5.43, Rotation2d.fromDegrees(300)));
            drivebase.setState(Drivebase.State.ALIGN);
        }));
        addBlock(new TorqueWaitUntil(() -> drivebase.isAligned(TorqueMode.AUTO)));
        addBlock(new TorqueRun(() -> {
            drivebase.setInputSpeeds(new TorqueSwerveSpeeds());
            drivebase.setState(Drivebase.State.ROBOT_RELATIVE);
            drivebase.setAlignPoseOverride(null);
        }));

        // Start moving subsystems while aligning
        addBlock(new TorqueRun(() -> {
            elevator.setState(Elevator.State.SCORE_L3);
            claw.setState(Claw.State.MID_SCORE);
        }));

        addBlock(new TorqueWaitUntil(() -> elevator.isNearState() && claw.isNearState()));

        // Alignment
        addBlock(new TorqueRun(() -> drivebase.setRelation(Relation.LEFT)));
        addBlock(new TorqueRun(() -> drivebase.setState(Drivebase.State.ALIGN)));
        addBlock(new TorqueWaitUntil(() -> drivebase.isAligned(TorqueMode.AUTO)));
        addBlock(new TorqueRun(() -> drivebase.setState(Drivebase.State.ROBOT_RELATIVE)));

        // Coral placement
        addBlock(new TorqueRun(() -> claw.setCoralState(Claw.CoralState.SHOOT)));
        addBlock(new TorqueWaitTime(.5)); // Wait until we shoot coral
        addBlock(new TorqueRun(() -> claw.setCoralState(Claw.CoralState.OFF)));

        // Drive close left to coral station left
        addBlock(new TorqueFollowPath("CL_CSL", drivebase).withMarkers(
            new Marker(() -> {
                drivebase.setRelation(Relation.CENTER);
                elevator.setState(State.CORAL_HP);
                claw.setState(Claw.State.CORAL_HP);
            }, .2),
            new Marker(() -> {
                claw.setCoralState(Claw.CoralState.INTAKE);
            }, .75)
        ));
        
        addBlock(new TorqueRun(() -> drivebase.setState(Drivebase.State.ALIGN)));
        addBlock(new TorqueWaitUntil(() -> drivebase.isAligned(TorqueMode.AUTO)));

        // Pickup coral from coral station
        addBlock(new TorqueWaitTime(.5));

        // Drive coral station left to close left
        addBlock(new TorqueFollowPath("CSL_CL", drivebase).withMarkers(
            new Marker(() -> {
                elevator.setState(Elevator.State.SCORE_L3);
                claw.setState(Claw.State.MID_SCORE);
            }, .2)
        ));

        // Alignment
        addBlock(new TorqueRun(() -> drivebase.setRelation(Relation.RIGHT)));
        addBlock(new TorqueRun(() -> drivebase.setState(Drivebase.State.ALIGN)));
        addBlock(new TorqueWaitUntil(() -> drivebase.isAligned(TorqueMode.AUTO)));
        addBlock(new TorqueRun(() -> drivebase.setState(Drivebase.State.ROBOT_RELATIVE)));

        // Coral placement
        addBlock(new TorqueRun(() -> claw.setCoralState(Claw.CoralState.SHOOT)));
        addBlock(new TorqueWaitTime(.5)); // Wait until we shoot coral
        addBlock(new TorqueRun(() -> claw.setCoralState(Claw.CoralState.OFF)));

        // Drive close left to coral station left
        addBlock(new TorqueFollowPath("CL_CSL", drivebase).withMarkers(
            new Marker(() -> {
                drivebase.setRelation(Relation.CENTER);
                elevator.setState(State.CORAL_HP);
                claw.setState(Claw.State.CORAL_HP);
            }, .1),
            new Marker(() -> {
                claw.setCoralState(Claw.CoralState.INTAKE);
            }, .75)
        ));
        
        addBlock(new TorqueRun(() -> drivebase.setState(Drivebase.State.ALIGN)));
        addBlock(new TorqueWaitUntil(() -> drivebase.isAligned(TorqueMode.AUTO)));

        // Pickup coral from coral station
        addBlock(new TorqueWaitTime(.5));

        // Drive coral station left to close left
        addBlock(new TorqueFollowPath("CSL_CL", drivebase).withMarkers(
            new Marker(() -> {
                elevator.setState(Elevator.State.SCORE_L2);
                claw.setState(Claw.State.MID_SCORE);
            }, .2)
        ));

        // Alignment
        addBlock(new TorqueRun(() -> drivebase.setRelation(Relation.RIGHT)));
        addBlock(new TorqueRun(() -> drivebase.setState(Drivebase.State.ALIGN)));
        addBlock(new TorqueWaitUntil(() -> drivebase.isAligned(TorqueMode.AUTO)));
        addBlock(new TorqueRun(() -> drivebase.setState(Drivebase.State.ROBOT_RELATIVE)));

        // Coral placement
        addBlock(new TorqueRun(() -> claw.setCoralState(Claw.CoralState.SHOOT)));
        addBlock(new TorqueWaitTime(.5)); // Wait until we shoot coral
        addBlock(new TorqueRun(() -> claw.setCoralState(Claw.CoralState.OFF)));
    }
}

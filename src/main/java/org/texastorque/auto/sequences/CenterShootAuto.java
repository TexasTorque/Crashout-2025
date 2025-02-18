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
import org.texastorque.torquelib.base.TorqueMode;
import org.texastorque.torquelib.swerve.TorqueSwerveSpeeds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import org.texastorque.subsystems.Drivebase;

public class CenterShootAuto extends TorqueSequence implements Subsystems {
    
    public CenterShootAuto() {
        // Elevator & claw setpoints
        addBlock(new TorqueRun(() -> {
            elevator.setState(Elevator.State.ALGAE_REMOVAL_LOW);
            claw.setState(Claw.State.ALGAE_EXTRACTION);
        }));

        // Drive center to far
        addBlock(new TorqueFollowPath("CTR_FF", drivebase));

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

        // // Move back to avoid claw hitting reef poles
        addBlock(new TorqueRun(() -> {
            drivebase.setAlignPoseOverride(new Pose2d(6.15, 4, Rotation2d.fromDegrees(180)));
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

        // Move back to avoid claw hitting reef poles & align to processor
        addBlock(new TorqueRun(() -> {
            drivebase.setAlignPoseOverride(new Pose2d(6.15, 3.0, Rotation2d.fromDegrees(-85)));
            drivebase.setState(Drivebase.State.ALIGN);
        }));
        addBlock(new TorqueWaitUntil(() -> drivebase.isAligned(TorqueMode.AUTO)));
        addBlock(new TorqueRun(() -> {
            drivebase.setInputSpeeds(new TorqueSwerveSpeeds());
            drivebase.setState(Drivebase.State.ROBOT_RELATIVE);
            drivebase.setAlignPoseOverride(null);
        }));

        // Move to processor state
        addBlock(new TorqueRun(() -> {
            elevator.setState(Elevator.State.PROCESSOR);
            claw.setState(Claw.State.PROCESSOR);
        }));

        addBlock(new TorqueWaitUntil(() -> elevator.isAtState() && claw.isAtState()));

        // SHOOT ALGAE!
        addBlock(new TorqueRun(() -> claw.setAlgaeState(Claw.AlgaeState.SHOOT)));
        addBlock(new TorqueWaitTime(.5)); // Wait until we shoot algae
        addBlock(new TorqueRun(() -> claw.setAlgaeState(Claw.AlgaeState.OFF)));
    }
}

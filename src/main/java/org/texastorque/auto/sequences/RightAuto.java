package org.texastorque.auto.sequences;

import org.texastorque.AlignPose2d.Relation;
import org.texastorque.Subsystems;
import org.texastorque.auto.routines.Align;
import org.texastorque.auto.routines.QuickSwap;
import org.texastorque.subsystems.Claw;
import org.texastorque.subsystems.Elevator;
import org.texastorque.subsystems.Elevator.State;
import org.texastorque.torquelib.auto.TorqueSequence;
import org.texastorque.torquelib.auto.commands.TorqueFollowPath;
import org.texastorque.torquelib.auto.commands.TorqueRun;
import org.texastorque.torquelib.auto.commands.TorqueWaitTime;
import org.texastorque.torquelib.auto.commands.TorqueWaitUntil;
import org.texastorque.torquelib.auto.marker.Marker;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class RightAuto extends TorqueSequence implements Subsystems {
    
    public RightAuto() {
        // Drive right to close right
        addBlock(new TorqueFollowPath("RGT_CR", drivebase).withMarkers(
            new Marker(() -> {
                elevator.setState(State.ALGAE_REMOVAL_LOW);
                claw.setState(Claw.State.ALGAE_EXTRACTION);
            }, .3)
        ));

        addBlock(new TorqueWaitUntil(() -> elevator.isNearState() && claw.isNearState()));

        // Quickswap
        addBlock(new QuickSwap(new Pose2d(3.6, 2.6, Rotation2d.fromDegrees(60))).command());

        // Drive close right to coral station right
        addBlock(new TorqueFollowPath("CR_CSR_TOSS", drivebase).withMarkers(
            new Marker(() -> {
                claw.setAlgaeState(Claw.AlgaeState.SHOOT);
            }, .5),
            new Marker(() -> {
                claw.setAlgaeState(Claw.AlgaeState.OFF);
                elevator.setState(State.CORAL_HP);
                claw.setState(Claw.State.CORAL_HP);
            }, .7)
        ));

        // Alignment
        addBlock(new Align(Relation.CENTER).command());

        // Pickup coral from coral station
        addBlock(new TorqueRun(() -> claw.setCoralState(Claw.CoralState.INTAKE)));
        addBlock(new TorqueWaitTime(.5));
        addBlock(new TorqueRun(() -> claw.setCoralState(Claw.CoralState.OFF)));

        // Drive coral station right to close right
        addBlock(new TorqueFollowPath("CSR_CR", drivebase).withMarkers(
            new Marker(() -> {
                elevator.setState(Elevator.State.SCORE_L3);
                claw.setState(Claw.State.MID_SCORE);
            }, .2)
        ));

        // Alignment
        addBlock(new Align(Relation.RIGHT).command());

        // Coral placement
        addBlock(new TorqueRun(() -> claw.setCoralState(Claw.CoralState.SHOOT)));
        addBlock(new TorqueWaitTime(.5)); // Wait until we shoot coral
        addBlock(new TorqueRun(() -> claw.setCoralState(Claw.CoralState.OFF)));

        // Drive close right to coral station right
        addBlock(new TorqueFollowPath("CR_CSR", drivebase).withMarkers(
            new Marker(() -> {
                elevator.setState(State.CORAL_HP);
                claw.setState(Claw.State.CORAL_HP);
            }, .1)
        ));

        // Alignment
        addBlock(new Align(Relation.CENTER).command());

        // Pickup coral from coral station
        addBlock(new TorqueRun(() -> claw.setCoralState(Claw.CoralState.INTAKE)));
        addBlock(new TorqueWaitTime(.5));
        addBlock(new TorqueRun(() -> claw.setCoralState(Claw.CoralState.OFF)));

        // Drive coral station right to close right
        addBlock(new TorqueFollowPath("CSR_CR", drivebase).withMarkers(
            new Marker(() -> {
                elevator.setState(Elevator.State.SCORE_L2);
                claw.setState(Claw.State.MID_SCORE);
            }, .2)
        ));

        // Alignment
        addBlock(new Align(Relation.RIGHT).command());

        // Coral placement
        addBlock(new TorqueRun(() -> claw.setCoralState(Claw.CoralState.SHOOT)));
        addBlock(new TorqueWaitTime(.5)); // Wait until we shoot coral
        addBlock(new TorqueRun(() -> claw.setCoralState(Claw.CoralState.OFF)));
    }
}

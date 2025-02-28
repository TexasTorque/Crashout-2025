package org.texastorque.auto.sequences.wip;

import org.texastorque.Subsystems;
import org.texastorque.auto.routines.QuickSwap;
import org.texastorque.subsystems.Claw;
import org.texastorque.subsystems.Elevator;
import org.texastorque.torquelib.auto.TorqueSequence;
import org.texastorque.torquelib.auto.commands.TorqueFollowPath;
import org.texastorque.torquelib.auto.commands.TorqueRun;
import org.texastorque.torquelib.auto.commands.TorqueWaitTime;
import org.texastorque.torquelib.auto.commands.TorqueWaitUntil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

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

        // Quickswap
        addBlock(new QuickSwap(() -> new Pose2d(6.15, 4, Rotation2d.fromDegrees(180))).command());

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

package org.texastorque.auto.sequences;

import org.texastorque.Field.AlignPosition.AlignableTarget;
import org.texastorque.Field.AlignPosition.Relation;
import org.texastorque.Subsystems;
import org.texastorque.auto.routines.Align;
import org.texastorque.auto.routines.DrivePickupCoral;
import org.texastorque.auto.routines.DriveScoreCoral;
import org.texastorque.subsystems.Claw;
import org.texastorque.subsystems.Elevator;
import org.texastorque.subsystems.Elevator.State;
import org.texastorque.torquelib.auto.TorqueSequence;
import org.texastorque.torquelib.auto.commands.TorqueFollowPath;
import org.texastorque.torquelib.auto.commands.TorqueRun;
import org.texastorque.torquelib.auto.commands.TorqueWaitTime;
import org.texastorque.torquelib.auto.commands.TorqueWaitUntil;
import org.texastorque.torquelib.auto.marker.Marker;

import edu.wpi.first.wpilibj.RobotBase;

public class LeftL4Auto extends TorqueSequence implements Subsystems {
    
    public LeftL4Auto() {
        addBlock(new TorqueRun(() -> {
            elevator.setState(Elevator.State.STOW);
            claw.setState(Claw.State.STOW);
        }));

        // // Drive left to close left
        // addBlock(new TorqueFollowPath("LL4_1", drivebase).withMarkers(
        //     new Marker(() -> {
        //         elevator.setState(State.SCORE_L4); 
        //         claw.setState(Claw.State.SCORE_L4);
        //     }, 0.9)
        // ));

        // // Alignment
        // addBlock(new Align(Relation.RGHT, AlignableTarget.L4, 1).command());

        // // Wait
        // addBlock(new TorqueWaitUntil(() -> elevator.isNearState() && claw.isNearState()));

        // // Coral placement
        // addBlock(new TorqueRun(() -> claw.setCoralState(Claw.CoralState.SHOOT)));
        // addBlock(new TorqueWaitTime(.5)); // Wait until we shoot coral
        // addBlock(new TorqueRun(() -> claw.setCoralState(Claw.CoralState.OFF)));

        addBlock(new DriveScoreCoral("LL4_1", Relation.RIGHT, AlignableTarget.L4, 1, 
            new Marker(() -> {
                elevator.setState(State.SCORE_L4); 
                claw.setState(Claw.State.SCORE_L4);
            }, 0.9)
        ).command());
        
        addBlock(new DrivePickupCoral("LL4_2", 1, 
            new Marker(() -> {
                elevator.setState(State.CORAL_HP);
                claw.setState(Claw.State.CORAL_HP);
            }, 0.2)
        ).command());

        // // Drive close right to coral station right
        // addBlock(new TorqueFollowPath("RL4_2", drivebase).withMarkers(
        //     new Marker(() -> {
        //         elevator.setState(State.CORAL_HP);
        //         claw.setState(Claw.State.CORAL_HP);
        //     }, .2)
        // ));

        // // Intake mf
        // addBlock(new TorqueRun(() -> claw.setCoralState(Claw.CoralState.INTAKE)));

        // // Alignment
        // addBlock(new Align(Relation.CENTER, AlignableTarget.CORAL_STATION, 1.2).command());

        // // Pickup coral from coral station
        // if (RobotBase.isReal()) {
        //     addBlock(new TorqueWaitUntil(() -> claw.hasCoral()));
        // } else {
        //     addBlock(new TorqueWaitTime(.5));
        // }
        // addBlock(new TorqueRun(() -> claw.setCoralState(Claw.CoralState.OFF)));

        // Drive coral station right to close right
        // addBlock(new TorqueFollowPath("RL4_3", drivebase).withMarkers(
        //     new Marker(() -> {
        //         elevator.setState(State.SCORE_L4); 
        //         claw.setState(Claw.State.SCORE_L4);
        //     }, 0.9)
        // ));

        addBlock(new DriveScoreCoral("LL4_3", Relation.LEFT, AlignableTarget.L4, 1, 
            new Marker(() -> {
                elevator.setState(State.SCORE_L4); 
                claw.setState(Claw.State.SCORE_L4);
            }, 0.9)
        ).command());


        // // Drive close right to coral station right
        // addBlock(new TorqueFollowPath("RL4_4", drivebase).withMarkers(
        //     new Marker(() -> {
        //         elevator.setState(State.CORAL_HP);
        //         claw.setState(Claw.State.CORAL_HP);
        //     }, .2)
        // ));

        // addBlock(new TorqueRun(() -> claw.setCoralState(Claw.CoralState.INTAKE)));

        // // Alignment
        // addBlock(new Align(Relation.CENTER, AlignableTarget.CORAL_STATION, 1.2).command());

        // // Pickup coral from coral station
        // if (RobotBase.isReal()) {
        //     addBlock(new TorqueWaitUntil(() -> claw.hasCoral()));
        // } else {
        //     addBlock(new TorqueWaitTime(.5));
        // }
        // addBlock(new TorqueRun(() -> claw.setCoralState(Claw.CoralState.OFF)));

        addBlock(new DrivePickupCoral("LL4_4", 1.2, 
            new Marker(() -> {
                elevator.setState(State.CORAL_HP);
                claw.setState(Claw.State.CORAL_HP);
            }, 0.2)
        ).command());

        // // Drive coral station right to close right
        // addBlock(new TorqueFollowPath("RL4_5", drivebase).withMarkers(
        //     new Marker(() -> {
        //         elevator.setState(State.SCORE_L4); 
        //         claw.setState(Claw.State.SCORE_L4);
        //     }, 0.9)
        // ));

        // // Alignment
        // addBlock(new Align(Relation.LEFT, AlignableTarget.L4, 2).command());

        // // Wait
        // addBlock(new TorqueWaitUntil(() -> elevator.isNearState() && claw.isNearState()));

        // // Coral placement
        // addBlock(new TorqueRun(() -> claw.setCoralState(Claw.CoralState.SHOOT)));
        // addBlock(new TorqueWaitTime(.5)); // Wait until we shoot coral
        // addBlock(new TorqueRun(() -> claw.setCoralState(Claw.CoralState.OFF)));

        addBlock(new DriveScoreCoral("LL4_5", Relation.RIGHT, AlignableTarget.L4, 1, 
            new Marker(() -> {
                elevator.setState(State.SCORE_L4); 
                claw.setState(Claw.State.SCORE_L4);
            }, 0.9)
        ).command());

        // Move back to avoid claw hitting reef poles
        addBlock(new TorqueRun(() -> {
            perception.setRelation(Relation.NONE);
            perception.setDesiredAlignTarget(AlignableTarget.NONE);
        }));
        addBlock(new Align(() -> perception.getAlignPose(), 1).command());
        
		// Go to stow at end
		addBlock(new TorqueRun(() -> {
			claw.setState(Claw.State.STOW);
			elevator.setState(Elevator.State.STOW);
			claw.setAlgaeState(Claw.AlgaeState.OFF);
		}));
    }
}

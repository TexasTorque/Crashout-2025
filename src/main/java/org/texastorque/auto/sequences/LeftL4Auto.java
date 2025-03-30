/**
 * Copyright 2025 Texas Torque.
 *
 * This file is part of Bravo/Charlie/Crashout-2025, which is not licensed for distribution.
 * For more details, see ./license.txt or write <davey.adams.three@gmail.com>.
 */
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
import org.texastorque.torquelib.auto.commands.TorqueRun;
import org.texastorque.torquelib.auto.marker.Marker;
public class LeftL4Auto extends TorqueSequence implements Subsystems {
    
    public LeftL4Auto() {
        addBlock(new TorqueRun(() -> {
            elevator.setState(Elevator.State.STOW);
            claw.setState(Claw.State.STOW);
        }));

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

        addBlock(new DriveScoreCoral("LL4_3", Relation.LEFT, AlignableTarget.L4, 1, 
            new Marker(() -> {
                elevator.setState(State.SCORE_L4); 
                claw.setState(Claw.State.SCORE_L4);
            }, 0.9)
        ).command());

        addBlock(new DrivePickupCoral("LL4_4", 1.2, 
            new Marker(() -> {
                elevator.setState(State.CORAL_HP);
                claw.setState(Claw.State.CORAL_HP);
            }, 0.2)
        ).command());

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

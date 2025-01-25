package org.texastorque.auto.sequences;

import org.texastorque.Subsystems;
import org.texastorque.auto.ReefSequence;
import org.texastorque.auto.ReefSequence.EndAction;
import org.texastorque.subsystems.Claw;
import org.texastorque.subsystems.Elevator;
import org.texastorque.torquelib.auto.TorqueSequence;
import org.texastorque.torquelib.auto.commands.TorqueFollowPath;
import org.texastorque.torquelib.auto.commands.TorqueRun;
import org.texastorque.torquelib.auto.commands.TorqueWaitTime;
import org.texastorque.torquelib.auto.commands.TorqueWaitUntil;

public class BaseAuto extends TorqueSequence implements Subsystems {
    
    public BaseAuto(final ReefSequence ...sequences) {

        for (ReefSequence sequence : sequences) {
            addBlock(new TorqueFollowPath(sequence.getPathName(), drivebase));

            for (EndAction endAction : sequence.getActions()) {
                addBlock(new TorqueRun(() -> elevator.setState(endAction.elevatorState)));
                addBlock(new TorqueWaitUntil(() -> elevator.isAtState()));

                addBlock(new TorqueRun(() -> claw.setState(endAction.clawState)));
                addBlock(new TorqueWaitUntil(() -> claw.isAtState()));

                addBlock(new TorqueRun(() -> claw.setAlgaeState(endAction.algaeState)));
                addBlock(new TorqueRun(() -> claw.setCoralState(endAction.coralState)));

                addBlock(new TorqueWaitTime(2)); // Probably way less
                
                addBlock(new TorqueRun(() -> claw.setAlgaeState(Claw.AlgaeState.OFF)));
                addBlock(new TorqueRun(() -> claw.setCoralState(Claw.CoralState.OFF)));

                addBlock(new TorqueRun(() -> claw.setState(Claw.State.STOW)));

                addBlock(new TorqueWaitUntil(() -> claw.isAtState()));

                addBlock(new TorqueRun(() -> elevator.setState(Elevator.State.STOW)));
            }
        }
    }
}

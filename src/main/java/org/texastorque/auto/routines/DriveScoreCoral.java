package org.texastorque.auto.routines;

import org.texastorque.Field.AlignPosition.AlignableTarget;
import org.texastorque.Field.AlignPosition.Relation;
import org.texastorque.Subsystems;
import org.texastorque.subsystems.Claw;
import org.texastorque.torquelib.auto.TorqueSequence;
import org.texastorque.torquelib.auto.commands.TorqueFollowPath;
import org.texastorque.torquelib.auto.commands.TorqueRun;
import org.texastorque.torquelib.auto.commands.TorqueWaitTime;
import org.texastorque.torquelib.auto.commands.TorqueWaitUntil;
import org.texastorque.torquelib.auto.marker.Marker;

public class DriveScoreCoral extends TorqueSequence implements Subsystems {

	public DriveScoreCoral(String pathName, Relation relation, AlignableTarget target, double timeToAlign, Marker... markers) {
		// Drive path
        addBlock(new TorqueFollowPath(pathName, drivebase).withMarkers(markers));

        // Alignment
        addBlock(new Align(relation, target, timeToAlign).command());

        // Wait
        addBlock(new TorqueWaitUntil(() -> elevator.isNearState() && claw.isNearState()));

        // Coral placement
        addBlock(new TorqueRun(() -> claw.setCoralState(Claw.CoralState.SHOOT)));
        addBlock(new TorqueWaitTime(.5)); // Wait until we shoot coral
        addBlock(new TorqueRun(() -> claw.setCoralState(Claw.CoralState.OFF)));
	}
}

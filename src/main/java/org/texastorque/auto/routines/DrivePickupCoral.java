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

import edu.wpi.first.wpilibj.RobotBase;

public class DrivePickupCoral extends TorqueSequence implements Subsystems {

	public DrivePickupCoral(String pathName, double alignTime, Marker... markers) {
		// Drive path
        addBlock(new TorqueFollowPath(pathName, drivebase).withMarkers(markers));

        // Intake coral
        addBlock(new TorqueRun(() -> claw.setCoralState(Claw.CoralState.INTAKE)));

        // Alignment
        addBlock(new Align(Relation.CENTER, AlignableTarget.CORAL_STATION, alignTime).command());

        // Pickup coral from coral station
        if (RobotBase.isReal()) {
            addBlock(new TorqueWaitUntil(() -> claw.hasCoral()));
        } else {
            addBlock(new TorqueWaitTime(.5));
        }
        addBlock(new TorqueRun(() -> claw.setCoralState(Claw.CoralState.OFF)));

	}
}

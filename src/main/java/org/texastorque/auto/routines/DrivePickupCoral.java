/**
 * Copyright 2025 Texas Torque.
 *
 * This file is part of Bravo/Charlie/Crashout-2025, which is not licensed for distribution.
 * For more details, see ./license.txt or write <davey.adams.three@gmail.com>.
 */
package org.texastorque.auto.routines;

import org.texastorque.Field.AlignPosition.AlignableTarget;
import org.texastorque.Field.AlignPosition.Relation;
import org.texastorque.Subsystems;
import org.texastorque.subsystems.Claw;
import org.texastorque.torquelib.auto.TorqueSequence;
import org.texastorque.torquelib.auto.commands.TorqueFollowPath;
import org.texastorque.torquelib.auto.commands.TorqueRun;
import org.texastorque.torquelib.auto.commands.TorqueWaitTimeUntil;
import org.texastorque.torquelib.auto.marker.Marker;

public class DrivePickupCoral extends TorqueSequence implements Subsystems {

	public DrivePickupCoral(String pathName, double alignTime, Marker... markers) {
		// Drive path
        addBlock(new TorqueFollowPath(pathName, drivebase).withMarkers(markers));

        // Intake coral
        addBlock(new TorqueRun(() -> claw.setCoralState(Claw.CoralState.INTAKE)));

        // Alignment
        addBlock(new Align(Relation.CENTER, AlignableTarget.CORAL_STATION, alignTime).command());

        // Pickup coral from coral station
        addBlock(new TorqueWaitTimeUntil(.5, () -> claw.clawHasCoral()));
        addBlock(new TorqueRun(() -> claw.setCoralState(Claw.CoralState.OFF)));
	}
}

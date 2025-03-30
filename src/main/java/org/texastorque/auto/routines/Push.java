/**
 * Copyright 2025 Texas Torque.
 *
 * This file is part of Bravo/Charlie/Crashout-2025, which is not licensed for distribution.
 * For more details, see ./license.txt or write <davey.adams.three@gmail.com>.
 */
package org.texastorque.auto.routines;

import org.texastorque.Subsystems;
import org.texastorque.torquelib.auto.TorqueSequence;
import org.texastorque.torquelib.auto.commands.TorqueFollowPath;

public class Push extends TorqueSequence implements Subsystems {

	public Push() {
		addBlock(new TorqueFollowPath("PUSH", drivebase));
	}
}

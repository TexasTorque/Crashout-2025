package org.texastorque.auto.routines;

import org.texastorque.Subsystems;
import org.texastorque.torquelib.auto.TorqueSequence;
import org.texastorque.torquelib.auto.commands.TorqueFollowPath;

public class Push extends TorqueSequence implements Subsystems {

	public Push() {
		addBlock(new TorqueFollowPath("PUSH", drivebase));
	}
}

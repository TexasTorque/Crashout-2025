package org.texastorque.auto.routines;

import org.texastorque.Subsystems;
import org.texastorque.torquelib.auto.TorqueSequence;
import org.texastorque.torquelib.auto.commands.TorqueRun;
import org.texastorque.torquelib.auto.commands.TorqueWaitTime;
import org.texastorque.torquelib.swerve.TorqueSwerveSpeeds;

public class Push extends TorqueSequence implements Subsystems {

	public Push() {
		addBlock(new TorqueRun(() -> drivebase.setInputSpeeds(new TorqueSwerveSpeeds(.5, 0, 0))));
		addBlock(new TorqueWaitTime(.1));
		addBlock(new TorqueRun(() -> drivebase.setInputSpeeds(new TorqueSwerveSpeeds(-.5, 0, 0))));
		addBlock(new TorqueWaitTime(.1));
		addBlock(new TorqueRun(() -> drivebase.setInputSpeeds(new TorqueSwerveSpeeds(0, 0, 0))));
	}
}

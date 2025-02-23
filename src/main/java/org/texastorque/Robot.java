package org.texastorque;

import org.texastorque.auto.AutoManager;
import org.texastorque.torquelib.base.TorqueRobotBase;

public class Robot extends TorqueRobotBase implements Subsystems {

	public Robot() {
		super(Input.getInstance(), AutoManager.getInstance());

		// addSubsystem(drivebase);
		// addSubsystem(perception);
		// addSubsystem(elevator);
		// addSubsystem(claw);
		addSubsystem(climb);
	}
}
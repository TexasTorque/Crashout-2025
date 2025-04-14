package org.texastorque;

import org.texastorque.auto.AutoManager;
import org.texastorque.subsystems.Lights;
import org.texastorque.torquelib.base.TorqueRobotBase;

public class Robot extends TorqueRobotBase implements Subsystems {

	public Robot() {
		super(Input.getInstance(), Lights.getInstance(), AutoManager.getInstance());

		addSubsystem(drivebase);
		addSubsystem(perception);
		addSubsystem(elevator);
		addSubsystem(pickup);
		addSubsystem(claw);
		addSubsystem(climb);
	}
}
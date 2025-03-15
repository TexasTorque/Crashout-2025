package org.texastorque.auto.routines;

import org.texastorque.Subsystems;
import org.texastorque.torquelib.auto.TorqueSequence;
import org.texastorque.torquelib.auto.commands.TorqueFollowPath;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Push extends TorqueSequence implements Subsystems {

	public Push() {
		addBlock(new TorqueFollowPath(() -> {
			final boolean isRedAlliance = DriverStation.getAlliance().isPresent()
                    ? DriverStation.getAlliance().get() == Alliance.Red
                    : false;
			
			if (isRedAlliance) return PathPlannerPath.fromPathFile("RED_PUSH");
			return PathPlannerPath.fromPathFile("BLUE_PUSH");
		}, drivebase));
	}
}

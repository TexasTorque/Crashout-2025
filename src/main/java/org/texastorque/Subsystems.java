package org.texastorque;

import org.texastorque.subsystems.Claw;
import org.texastorque.subsystems.Drivebase;
import org.texastorque.subsystems.Elevator;
import org.texastorque.subsystems.Perception;

public interface Subsystems {
    // public final SubsystemName subsystemName = SubsystemName.getInstance();
    public final Drivebase drivebase = Drivebase.getInstance();
    public final Perception perception = Perception.getInstance();
    public final Elevator elevator = Elevator.getInstance();
    public final Claw claw = Claw.getInstance();
}

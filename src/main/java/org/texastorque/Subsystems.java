package org.texastorque;

import org.texastorque.subsystems.Drivebase;
import org.texastorque.subsystems.Perception;

public interface Subsystems {
    // public final SubsystemName subsystemName = SubsystemName.getInstance();
    public final Drivebase drivebase = Drivebase.getInstance();
    public final Perception perception = Perception.getInstance();
}

package org.texastorque;

import org.texastorque.subsystems.Claw;
import org.texastorque.subsystems.Climb;
import org.texastorque.subsystems.Drivebase;
import org.texastorque.subsystems.Elevator;
import org.texastorque.subsystems.Lights;
import org.texastorque.subsystems.Perception;
import org.texastorque.subsystems.Pickup;

public interface Subsystems {
    public final Drivebase drivebase = Drivebase.getInstance();
    public final Perception perception = Perception.getInstance();
    public final Elevator elevator = Elevator.getInstance();
    public final Pickup pickup = Pickup.getInstance();
    public final Claw claw = Claw.getInstance();
    public final Climb climb = Climb.getInstance();
    public final Lights lights = Lights.getInstance();
}

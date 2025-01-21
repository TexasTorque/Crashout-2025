package org.texastorque;

import org.texastorque.torquelib.swerve.base.TorqueSwerveModule.SwervePorts;

public final class Ports {
    // Change According to needs
	public static final SwervePorts BL_MOD = new SwervePorts(1, 2, 9);
    public static final SwervePorts FL_MOD = new SwervePorts(3, 4, 10);
    public static final SwervePorts FR_MOD = new SwervePorts(5, 6, 12);
    public static final SwervePorts BR_MOD = new SwervePorts(7, 8, 11);

    public static final int ELEVATOR = 16;
    public static final int ELEVATOR_ENCODER = 17;

    public static final int CLAW = 18;
    public static final int CLAW_ENCODER = 19;

    public static final int ROLLERS_CORAL = 20;
    public static final int ROLLERS_ALGAE = 21;
}

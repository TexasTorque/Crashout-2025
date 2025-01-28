package org.texastorque;

import org.texastorque.torquelib.swerve.base.TorqueSwerveModule.SwervePorts;

public final class Ports {
	public static final SwervePorts BL_MOD = new SwervePorts(1, 8, 9);
    public static final SwervePorts FL_MOD = new SwervePorts(3, 6, 12);
    public static final SwervePorts FR_MOD = new SwervePorts(5, 8, 10);
    public static final SwervePorts BR_MOD = new SwervePorts(7, 2, 11);

    public static final int ELEVATOR_LEFT = 16;
    public static final int ELEVATOR_RIGHT = 17;
    public static final int ELEVATOR_ENCODER = 18;

    public static final int CLAW = 19;
    public static final int CLAW_ENCODER = 20;

    public static final int ROLLERS_CORAL = 21;
    public static final int ROLLERS_ALGAE = 22;
}

/**
 * Copyright 2025 Texas Torque.
 *
 * This file is part of Bravo/Charlie/Crashout-2025, which is not licensed for distribution.
 * For more details, see ./license.txt or write <davey.adams.three@gmail.com>.
 */
package org.texastorque;

import org.texastorque.torquelib.swerve.base.TorqueSwerveModule.SwervePorts;

public final class Ports {
	public static final SwervePorts FR_MOD = new SwervePorts(5, 6, 11);
    public static final SwervePorts BR_MOD = new SwervePorts(7, 8, 12);
    public static final SwervePorts BL_MOD = new SwervePorts(1, 2, 9);
    public static final SwervePorts FL_MOD = new SwervePorts(3, 4, 10);

    public static final int ELEVATOR_LEFT = 13;
    public static final int ELEVATOR_RIGHT = 14;
    public static final int ELEVATOR_ENCODER = 19;

    public static final int SHOULDER = 15;
    public static final int SHOULDER_ENCODER = 18;

    public static final int ROLLERS_ALGAE = 16;
    public static final int ROLLERS_CORAL = 17;

    public static final int CLIMB = 20;

    public static final int GYRO = 21;
    public static final int RIP_RANGE = 0;

    public static final int PICKUP_PIVOT = 26;
    public static final int PICKUP_ROLLERS = 27;
    public static final int PICKUP_ENCODER = 26;

    public static final int LIGHTS = 9;
}

package org.texastorque;

import org.texastorque.torquelib.swerve.base.TorqueSwerveModule.SwervePorts;

public final class Ports {
    // Change According to needs
	public static final SwervePorts BL_MOD = new SwervePorts(1, 2, 9);
    public static final SwervePorts FL_MOD = new SwervePorts(3, 4, 10);
    public static final SwervePorts FR_MOD = new SwervePorts(5, 6, 12);
    public static final SwervePorts BR_MOD = new SwervePorts(7, 8, 11);
}

/**
 * Copyright 2025 Texas Torque.
 *
 * This file is part of Bravo/Charlie/Crashout-2025, which is not licensed for distribution.
 * For more details, see ./license.txt or write <davey.adams.three@gmail.com>.
 */
package org.texastorque.auto;

import org.texastorque.Subsystems;
import org.texastorque.auto.sequences.CenterNet;
import org.texastorque.auto.sequences.LeftL4Auto;
import org.texastorque.auto.sequences.RightL4Auto;
import org.texastorque.torquelib.auto.TorqueAutoManager;

import com.pathplanner.lib.config.RobotConfig;

public final class AutoManager extends TorqueAutoManager implements Subsystems {
    private static volatile AutoManager instance;

    @Override
    public final void loadSequences() {
        addSequence("LEFT L4", new LeftL4Auto());
        addSequence("CENTER NET", new CenterNet());
        addSequence("RIGHT L4", new RightL4Auto());

        // addSequence("TEST", new TestAuto());
    }

    @Override
    public final void loadPaths() {
        pathLoader.preloadPath("CNET_1");
        pathLoader.preloadPath("CNET_2");
        pathLoader.preloadPath("CNET_A");
        pathLoader.preloadPath("CNET_3");
        pathLoader.preloadPath("CNET_4");
        pathLoader.preloadPath("CPSR_1");
        pathLoader.preloadPath("CPSR_2");
        pathLoader.preloadPath("CPSR_3");
        pathLoader.preloadPath("LL3_1");
        pathLoader.preloadPath("LL3_2");
        pathLoader.preloadPath("LL3_3");
        pathLoader.preloadPath("LL4_1");
        pathLoader.preloadPath("LL4_2");
        pathLoader.preloadPath("LL4_3");
        pathLoader.preloadPath("RL3_1");
        pathLoader.preloadPath("RL3_2");
        pathLoader.preloadPath("RL3_3");
        pathLoader.preloadPath("RL4_1");
        pathLoader.preloadPath("RL4_2");
        pathLoader.preloadPath("RL4_3");
        pathLoader.preloadPath("PUSH");
        pathLoader.preloadPath("NA_NA");
        pathLoader.preloadPath("test");
    }

    public static RobotConfig getRobotConfig() {
        try {
            return RobotConfig.fromGUISettings();
        } catch (Exception e) {}
        return null;
    }

    public static final synchronized AutoManager getInstance() {
        return instance == null ? instance = new AutoManager() : instance;
    }
}
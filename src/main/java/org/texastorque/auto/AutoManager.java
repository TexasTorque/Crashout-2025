/**
 * Copyright 2023 Texas Torque.
 *
 * This file is part of Bravo/Charlie/Takeoff-2024, which is not licensed for distribution.
 * For more details, see ./license.txt or write <jus@justusl.com>.
 */
package org.texastorque.auto;

import org.texastorque.Subsystems;
import org.texastorque.auto.sequences.blue.*;
import org.texastorque.auto.sequences.red.*;
import org.texastorque.torquelib.auto.TorqueAutoManager;

public final class AutoManager extends TorqueAutoManager implements Subsystems {
    private static volatile AutoManager instance;

    @Override
    public final void loadSequences() {
        // addSequence("BLUE LEFT L3", new BlueLeftL3Auto());
        addSequence("BLUE LEFT L4", new BlueLeftL4Auto());
        // addSequence("BLUE CENTER PROCESSOR", new BlueCenterProcessor());
        addSequence("BLUE CENTER NET", new BlueCenterNet());
        addSequence("BLUE RIGHT L4", new BlueRightL4Auto());
        // addSequence("BLUE RIGHT L3", new BlueRightL3Auto());

        // addSequence("RED LEFT L3", new RedLeftL3Auto());
        addSequence("RED LEFT L4", new RedLeftL4Auto());
        // addSequence("RED CENTER PROCESSOR", new RedCenterProcessor());
        addSequence("RED CENTER NET", new RedCenterNet());
        addSequence("RED RIGHT L4", new RedRightL4Auto());
        // addSequence("RED RIGHT L3", new RedRightL3Auto());

        // addSequence("PUSH", new Push());

        // addSequence("TEST", new TestAuto());
    }

    @Override
    public final void loadPaths() {
        pathLoader.preloadPath("BCNET_1");
        pathLoader.preloadPath("BCNET_2");
        pathLoader.preloadPath("BCNET_3");
        pathLoader.preloadPath("BCNET_4");
        pathLoader.preloadPath("BCPSR_1");
        pathLoader.preloadPath("BCPSR_2");
        pathLoader.preloadPath("BCPSR_3");
        pathLoader.preloadPath("BLL3_1");
        pathLoader.preloadPath("BLL3_2");
        pathLoader.preloadPath("BLL3_3");
        pathLoader.preloadPath("BLL4_1");
        pathLoader.preloadPath("BLL4_2");
        pathLoader.preloadPath("BLL4_3");
        pathLoader.preloadPath("BRL3_1");
        pathLoader.preloadPath("BRL3_2");
        pathLoader.preloadPath("BRL3_3");
        pathLoader.preloadPath("BRL4_1");
        pathLoader.preloadPath("BRL4_2");
        pathLoader.preloadPath("BRL4_3");
        pathLoader.preloadPath("NA_NA");
        pathLoader.preloadPath("PUSH");
        pathLoader.preloadPath("RCNET_1");
        pathLoader.preloadPath("RCNET_2");
        pathLoader.preloadPath("RCNET_3");
        pathLoader.preloadPath("RCNET_4");
        pathLoader.preloadPath("RCPSR_1");
        pathLoader.preloadPath("RCPSR_2");
        pathLoader.preloadPath("RCPSR_3");
        pathLoader.preloadPath("RLL3_1");
        pathLoader.preloadPath("RLL3_2");
        pathLoader.preloadPath("RLL3_3");
        pathLoader.preloadPath("RLL4_1");
        pathLoader.preloadPath("RLL4_2");
        pathLoader.preloadPath("RLL4_3");
        pathLoader.preloadPath("RRL3_1");
        pathLoader.preloadPath("RRL3_2");
        pathLoader.preloadPath("RRL3_3");
        pathLoader.preloadPath("RRL4_1");
        pathLoader.preloadPath("RRL4_2");
        pathLoader.preloadPath("RRL4_3");
    }

    public static final synchronized AutoManager getInstance() {
        return instance == null ? instance = new AutoManager() : instance;
    }
}
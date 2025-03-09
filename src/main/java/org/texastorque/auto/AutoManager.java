/**
 * Copyright 2023 Texas Torque.
 *
 * This file is part of Bravo/Charlie/Takeoff-2024, which is not licensed for distribution.
 * For more details, see ./license.txt or write <jus@justusl.com>.
 */
package org.texastorque.auto;

import org.texastorque.Subsystems;
import org.texastorque.auto.sequences.blue.BlueCenterNet;
import org.texastorque.auto.sequences.blue.BlueCenterProcessor;
import org.texastorque.auto.sequences.blue.BlueLeftL4Auto;
import org.texastorque.auto.sequences.blue.BlueLeftQuickswapAuto;
import org.texastorque.auto.sequences.blue.BlueRightL4Auto;
import org.texastorque.auto.sequences.blue.BlueRightQuickswapAuto;
import org.texastorque.auto.sequences.red.RedCenterNet;
import org.texastorque.auto.sequences.red.RedCenterProcessor;
import org.texastorque.auto.sequences.red.RedLeftL4Auto;
import org.texastorque.auto.sequences.red.RedLeftQuickswapAuto;
import org.texastorque.auto.sequences.red.RedRightL4Auto;
import org.texastorque.auto.sequences.red.RedRightQuickswapAuto;
import org.texastorque.torquelib.auto.TorqueAutoManager;

public final class AutoManager extends TorqueAutoManager implements Subsystems {
    private static volatile AutoManager instance;

    @Override
    public final void loadSequences() {
        addSequence("BLUE LEFT -> L3 -> 2 CORAL", new BlueLeftQuickswapAuto());
        addSequence("BLUE LEFT -> L4 -> 3 CORAL", new BlueLeftL4Auto());
        addSequence("BLUE CENTER -> 1 CORAL -> 1 ALGAE PROCESSOR", new BlueCenterProcessor());
        addSequence("BLUE CENTER -> 1 CORAL -> 2 ALGAE NET", new BlueCenterNet());
        addSequence("BLUE RIGHT -> L4 -> 3 CORAL", new BlueRightL4Auto());
        addSequence("BLUE RIGHT -> L3 -> 2 CORAL", new BlueRightQuickswapAuto());

        addSequence("RED LEFT -> L3 -> 2 CORAL", new RedLeftQuickswapAuto());
        addSequence("RED LEFT -> L4 -> 3 CORAL", new RedLeftL4Auto());
        addSequence("RED CENTER -> 1 CORAL -> 1 ALGAE PROCESSOR", new RedCenterProcessor());
        addSequence("RED CENTER -> 1 CORAL -> 2 ALGAE NET", new RedCenterNet());
        addSequence("RED RIGHT -> L4 -> 3 CORAL", new RedRightL4Auto());
        addSequence("RED RIGHT -> L3 -> 2 CORAL", new RedRightQuickswapAuto());

        // addSequence("TEST", new TestAuto());
    }

    @Override
    public final void loadPaths() {
        pathLoader.preloadPath("BLUE_CL_CSL");
        pathLoader.preloadPath("BLUE_CR_CSR");
        pathLoader.preloadPath("BLUE_CSL_CL");
        pathLoader.preloadPath("BLUE_CSR_CR");
        pathLoader.preloadPath("BLUE_CTR_FF");
        pathLoader.preloadPath("BLUE_FF_PSR");
        pathLoader.preloadPath("BLUE_LFT_CL");
        pathLoader.preloadPath("BLUE_RGT_CR");
        pathLoader.preloadPath("RED_CL_CSL");
        pathLoader.preloadPath("RED_CR_CSR");
        pathLoader.preloadPath("RED_CSL_CL");
        pathLoader.preloadPath("RED_CSR_CR");
        pathLoader.preloadPath("RED_CTR_FF");
        pathLoader.preloadPath("RED_FF_PSR");
        pathLoader.preloadPath("RED_LFT_CL");
        pathLoader.preloadPath("RED_RGT_CR");
        pathLoader.preloadPath("NA_NA");
        pathLoader.preloadPath("test");
    }

    public static final synchronized AutoManager getInstance() {
        return instance == null ? instance = new AutoManager() : instance;
    }
}
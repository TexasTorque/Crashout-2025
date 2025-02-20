/**
 * Copyright 2023 Texas Torque.
 *
 * This file is part of Bravo/Charlie/Takeoff-2024, which is not licensed for distribution.
 * For more details, see ./license.txt or write <jus@justusl.com>.
 */
package org.texastorque.auto;

import java.util.Optional;

import org.texastorque.Subsystems;
import org.texastorque.auto.sequences.CenterNet;
import org.texastorque.auto.sequences.CenterProcessor;
import org.texastorque.auto.sequences.LeftAuto;
import org.texastorque.auto.sequences.RightAuto;
import org.texastorque.torquelib.auto.TorqueAutoManager;

import com.pathplanner.lib.path.PathPlannerPath;

public final class AutoManager extends TorqueAutoManager implements Subsystems {
    private static volatile AutoManager instance;

    @Override
    public final void loadSequences() {
        addSequence("LEFT -> 3 CORAL", new LeftAuto());
        addSequence("CENTER -> 1 CORAL -> 1 ALGAE", new CenterProcessor());
        addSequence("CENTER -> 2 ALGAE NET", new CenterNet());
        // addSequence("CENTER -> 1 CORAL -> 1 ALGAE SHOT", new CenterShootAuto());
        addSequence("RIGHT -> 3 CORAL", new RightAuto());
        // addSequence("TEST", new TestAuto());
    }

    @Override
    public final void loadPaths() {
        pathLoader.preloadPath("CL_CSL");
        pathLoader.preloadPath("CR_CSR");
        pathLoader.preloadPath("CSL_CL");
        pathLoader.preloadPath("CSR_CR");
        pathLoader.preloadPath("CTR_FF");
        pathLoader.preloadPath("FF_PSR");
        pathLoader.preloadPath("FR_PSR");
        pathLoader.preloadPath("LFT_CL");
        pathLoader.preloadPath("PSR_FR");
        pathLoader.preloadPath("RGT_CR");
        pathLoader.preloadPath("NA_NA");
        pathLoader.preloadPath("test");
    }

    public final PathPlannerPath getPath(final String pathName) {
        final Optional<PathPlannerPath> pathOpt = pathLoader.getPathSafe(pathName);
        if (pathOpt.isPresent()) {
            return pathOpt.get();
        }
        System.out.println("Failed to load path " + pathName);
        return pathLoader.getPathUnsafe("NA_NA");
    }

    public static final synchronized AutoManager getInstance() {
        return instance == null ? instance = new AutoManager() : instance;
    }
}
/**
 * Copyright 2023 Texas Torque.
 *
 * This file is part of Bravo/Charlie/Takeoff-2024, which is not licensed for distribution.
 * For more details, see ./license.txt or write <jus@justusl.com>.
 */
package org.texastorque.auto;

import java.util.Optional;

import org.texastorque.Subsystems;
import org.texastorque.auto.sequences.CenterAuto;
import org.texastorque.auto.sequences.CenterShootAuto;
import org.texastorque.auto.sequences.LeftAuto;
import org.texastorque.auto.sequences.RightAuto;
import org.texastorque.auto.sequences.TestAuto;
import org.texastorque.torquelib.auto.TorqueAutoManager;

import com.pathplanner.lib.path.PathPlannerPath;

/** Manage the auto loader and selections */
public final class AutoManager extends TorqueAutoManager implements Subsystems {
    private static volatile AutoManager instance;

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

    @Override
    public final void loadSequences() {
        addSequence("LFT -> CL EXT -> CL L3 L -> CSL -> CL L3 R -> CSL -> CL L2 L", new LeftAuto());
        addSequence("CTR -> FF -> PSR", new CenterAuto());
        addSequence("RGT -> CR EXT -> CR L3 L -> CSR -> CR L3 R -> CSR -> CR L2 L", new RightAuto());
        addSequence("CTR -> FF -> PSR SHOT", new CenterShootAuto());
        addSequence("Test", new TestAuto());
    }

    public static final synchronized AutoManager getInstance() {
        return instance == null ? instance = new AutoManager() : instance;
    }
}
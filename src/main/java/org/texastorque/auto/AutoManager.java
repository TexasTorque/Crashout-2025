/**
 * Copyright 2023 Texas Torque.
 *
 * This file is part of Bravo/Charlie/Takeoff-2024, which is not licensed for distribution.
 * For more details, see ./license.txt or write <jus@justusl.com>.
 */
package org.texastorque.auto;

import java.util.ArrayList;
import java.util.Optional;

import org.texastorque.Subsystems;
import org.texastorque.auto.sequences.CenterNet;
import org.texastorque.auto.sequences.CenterProcessor;
import org.texastorque.auto.sequences.LeftAuto;
import org.texastorque.auto.sequences.RightAuto;
import org.texastorque.torquelib.auto.TorqueAutoManager;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;

import edu.wpi.first.math.geometry.Translation2d;

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

    public static final PathPlannerPath shift(final PathPlannerPath path) {
        final double FIELD_LENGTH_2024 = 16.54;
        final double FIELD_LENGTH_2025 = 17.548225;
        final double FIELD_WIDTH_2024 = 8.21055;
        final double FIELD_WIDTH_2025 = 8.0518;
        final Translation2d translation = new Translation2d(FIELD_LENGTH_2025 - FIELD_LENGTH_2024, FIELD_WIDTH_2025 - FIELD_WIDTH_2024);
        ArrayList<PathPoint> shiftedPoints = new ArrayList<>();
        for (PathPoint point : path.getAllPathPoints()) {
            shiftedPoints.add(new PathPoint(point.position.plus(translation)));
        }
        
        PathPlannerPath newPath = PathPlannerPath.fromPathPoints(shiftedPoints, path.getGlobalConstraints(), path.getGoalEndState());
        return newPath;
    }

    public static final synchronized AutoManager getInstance() {
        return instance == null ? instance = new AutoManager() : instance;
    }
}
package org.texastorque.auto;

import org.texastorque.auto.ReefSequence.EndAction;
import org.texastorque.auto.ReefSequence.Location;
import org.texastorque.auto.sequences.BaseAuto;
import org.texastorque.subsystems.Drivebase;
import org.texastorque.torquelib.auto.*;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;

public final class AutoManager extends TorqueAutoManager {
    private static volatile AutoManager instance;

    @Override
    protected void loadSequences() {
        addBaseAuto("CTR -> L3 -> PROCESSOR",
                new ReefSequence(Location.CENTER, Location.FAR, EndAction.L3_L, EndAction.ALGAE_EXTRACTION_LOW),
                new ReefSequence(Location.FAR, Location.PROCESSOR, EndAction.PROCESSOR)
        );
        addBaseAuto("LFT -> L3 -> CSL -> L3",
                new ReefSequence(Location.LEFT, Location.CLOSE_LEFT, EndAction.L3_L, EndAction.ALGAE_EXTRACTION_LOW),
                new ReefSequence(Location.CLOSE_LEFT, Location.CORAL_STATION_LEFT, EndAction.CORAL_PICKUP),
                new ReefSequence(Location.CORAL_STATION_LEFT, Location.CLOSE_LEFT, EndAction.ALGAE_EXTRACTION_LOW, EndAction.L3_R)
        );
    }

    @Override
    public void loadPaths() {
        pathLoader.preloadPath("CST_CL");
        pathLoader.preloadPath("CST_FR");
        pathLoader.preloadPath("CTR_FF");
        pathLoader.preloadPath("FL_CST");
        pathLoader.preloadPath("FR_CST");
        pathLoader.preloadPath("LFT_FL");
        pathLoader.preloadPath("RGT_FL");
    }

    private void addBaseAuto(final String name, final ReefSequence ...sequences) {
        addSequence(name, new BaseAuto(sequences));
    }

    public static RobotConfig getRobotConfig() {
        try {
            return RobotConfig.fromGUISettings();
        } catch (Exception e) {
            return new RobotConfig(
                    74,
                    6,
                    new ModuleConfig(
                            0.0508,
                            4.6,
                            1,
                            new DCMotor(
                                    12,
                                    3.75,
                                    150,
                                    1.8,
                                    35663,
                                    1),
                            35,
                            1),
                    new Translation2d[] {
                            Drivebase.LOC_FL, Drivebase.LOC_FR,
                            Drivebase.LOC_BL, Drivebase.LOC_BR
                    }
            );
        }
    }

    /**
     * Get the AutoManager instance
     *
     * @return AutoManager
     */
    public static final synchronized AutoManager getInstance() {
        return instance == null ? instance = new AutoManager() : instance;
    }
}
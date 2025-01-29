package org.texastorque.auto;

import org.texastorque.Subsystems;
import org.texastorque.auto.sequences.CenterAuto;
import org.texastorque.subsystems.Drivebase;
import org.texastorque.torquelib.auto.*;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;

public final class AutoManager extends TorqueAutoManager implements Subsystems {
    private static volatile AutoManager instance;

    @Override
    protected void loadSequences() {
        addSequence("CTR -> FF EXT -> FF L3 L -> PSR -> FR EXT -> PSR", new CenterAuto());
    }

    @Override
    public void loadPaths() {
        pathLoader.preloadPath("CTR_FF");
        pathLoader.preloadPath("FF_PSR");
        pathLoader.preloadPath("PSR_FR");
        pathLoader.preloadPath("FR_PSR");
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
package org.texastorque.auto;

import org.texastorque.Subsystems;
import org.texastorque.auto.sequences.CenterAuto;
import org.texastorque.auto.sequences.LeftAuto;
import org.texastorque.auto.sequences.RightAuto;
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
        addSequence("LFT -> CL EXT -> CL L3 L -> CSL -> CL L3 R -> CSL -> CL L2 L", new LeftAuto());
        addSequence("CTR -> FF EXT -> FF L3 L -> PSR -> FR EXT -> PSR", new CenterAuto());
        addSequence("RGT -> CR EXT -> CR L3 L -> CSR -> CR L3 R -> CSR -> CR L2 L", new RightAuto());
    }

    @Override
    public void loadPaths() {
        pathLoader.preloadPath("CL_CSL");
        pathLoader.preloadPath("CR_CSR");
        pathLoader.preloadPath("CSL_CL");
        pathLoader.preloadPath("CSR_CR");
        pathLoader.preloadPath("CTR_FF");
        pathLoader.preloadPath("FF_PSR");
        pathLoader.preloadPath("FR_PSR");
        pathLoader.preloadPath("PSR_FR");
        pathLoader.preloadPath("LFT_CL");
        pathLoader.preloadPath("RGT_CR");
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
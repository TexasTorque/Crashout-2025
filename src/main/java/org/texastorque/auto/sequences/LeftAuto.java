package org.texastorque.auto.sequences;

import org.texastorque.Subsystems;
import org.texastorque.torquelib.auto.TorqueSequence;
import org.texastorque.torquelib.auto.commands.TorqueFollowPath;

public class LeftAuto extends TorqueSequence implements Subsystems {
    
    public LeftAuto() {
        // Drive left to close left
        addBlock(new TorqueFollowPath("LFT_CL", drivebase).withMarkers(

        ));

        // Algae extraction & coral placement

        // Drive close left to coral station left
        addBlock(new TorqueFollowPath("CL_CSL", drivebase).withMarkers(

        ));

        // Pickup coral from coral station

        // Drive coral station left to close left
        addBlock(new TorqueFollowPath("CSL_CL", drivebase).withMarkers(

        ));

        // Coral placement

        // Drive close left to coral station left
        addBlock(new TorqueFollowPath("CL_CSL", drivebase).withMarkers(

        ));

        // Pickup coral from coral station

        // Drive coral station left to close left
        addBlock(new TorqueFollowPath("CSL_CL", drivebase).withMarkers(

        ));

        // Coral placement
    }
}

package org.texastorque.auto.sequences;

import org.texastorque.Subsystems;
import org.texastorque.torquelib.auto.TorqueSequence;
import org.texastorque.torquelib.auto.commands.TorqueFollowPath;

public class RightAuto extends TorqueSequence implements Subsystems {
    
    public RightAuto() {
        // Drive right to close right
        addBlock(new TorqueFollowPath("RGT_CR", drivebase).withMarkers(
            
        ));

        // Algae extraction & coral placement

        // Drive close right to coral station right
        addBlock(new TorqueFollowPath("CR_CSR", drivebase).withMarkers(

        ));

        // Pickup coral from coral station

        // Drive coral station right to close right
        addBlock(new TorqueFollowPath("CSR_CR", drivebase).withMarkers(

        ));

        // Coral placement

        // Drive close right to coral station right
        addBlock(new TorqueFollowPath("CR_CSR", drivebase).withMarkers(

        ));

        // Pickup coral from coral station

        // Drive coral station right to close right
        addBlock(new TorqueFollowPath("CSR_CR", drivebase).withMarkers(

        ));

        // Coral placement
    }
}

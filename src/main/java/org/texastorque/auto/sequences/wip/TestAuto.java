package org.texastorque.auto.sequences.wip;

import org.texastorque.Subsystems;
import org.texastorque.torquelib.auto.TorqueSequence;
import org.texastorque.torquelib.auto.commands.TorqueFollowPath;

public class TestAuto extends TorqueSequence implements Subsystems {
    
    public TestAuto() {
        addBlock(new TorqueFollowPath("test", drivebase));
    }
}

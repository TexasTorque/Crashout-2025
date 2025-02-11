package org.texastorque.auto.sequences;

import org.texastorque.Subsystems;
import org.texastorque.auto.AutoManager;
import org.texastorque.torquelib.auto.TorqueSequence;
import org.texastorque.torquelib.auto.commands.TorqueRun;
import org.texastorque.torquelib.auto.commands.TorqueExecuteCommand;

import com.pathplanner.lib.auto.AutoBuilder;

public class TestAuto extends TorqueSequence implements Subsystems {
    
    public TestAuto() {
        // addBlock(new TorqueFollowPath("test", drivebase));
        addBlock(new TorqueExecuteCommand(AutoBuilder.followPath(AutoManager.getInstance().getPathLoader().getPathUnsafe("test"))));
        addBlock(new TorqueRun(() -> System.out.println("ended")));
    }
}

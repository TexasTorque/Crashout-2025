package org.texastorque.auto;

import org.texastorque.torquelib.auto.*;

public final class AutoManager extends TorqueAutoManager {
    private static volatile AutoManager instance;

    /**
     * Get the AutoManager instance
     *
     * @return AutoManager
     */
    public static final synchronized AutoManager getInstance() {
        return instance == null ? instance = new AutoManager() : instance;
    }

    @Override
    protected void loadSequences() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'loadSequences'");
    }

    @Override
    public void loadPaths() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'loadPaths'");
    }
}
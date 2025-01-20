package org.texastorque.subsystems;

import org.texastorque.Subsystems;
import org.texastorque.torquelib.base.TorqueMode;
import org.texastorque.torquelib.base.TorqueState;
import org.texastorque.torquelib.base.TorqueStatorSubsystem;

public final class Empty extends TorqueStatorSubsystem<Empty.State> implements Subsystems {

    private static volatile Empty instance;

    public static enum State implements TorqueState {
        STATE_A, STATE_B;
    }

    private Empty() {
        super(State.STATE_A);
    }

    @Override
    public final void initialize(final TorqueMode mode) {}

    @Override
    public final void update(final TorqueMode mode) {}

    @Override
    public final void clean(final TorqueMode mode) {}

    public static final synchronized Empty getInstance() {
        return instance == null ? instance = new Empty() : instance;
    }
}

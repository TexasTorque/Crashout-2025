package org.texastorque.subsystems;

import org.texastorque.Ports;
import org.texastorque.Subsystems;
import org.texastorque.torquelib.Debug;
import org.texastorque.torquelib.base.TorqueMode;
import org.texastorque.torquelib.base.TorqueState;
import org.texastorque.torquelib.base.TorqueStatorSubsystem;
import org.texastorque.torquelib.motors.TorqueNEO;

public final class Climb extends TorqueStatorSubsystem<Climb.State> implements Subsystems {

    private static volatile Climb instance;
    private final TorqueNEO climbLeft, climbRight;

    public static enum State implements TorqueState {
        UP(4),
        OFF(0),
        DOWN(-4);

        private final double volts;

        private State(double volts) {
            this.volts = volts;
        }

        public double getVolts() {
            return volts;
        }
    }

    private Climb() {
        super(State.OFF);

        climbLeft = new TorqueNEO(Ports.CLIMB_LEFT);
        climbRight = new TorqueNEO(Ports.CLIMB_RIGHT);
    }

    @Override
    public final void initialize(final TorqueMode mode) {}

    @Override
    public final void update(final TorqueMode mode) {
        Debug.log("Climb State", desiredState.toString());
        
        climbLeft.setVolts(desiredState.getVolts());
        climbRight.setVolts(-desiredState.getVolts());
    }

    @Override
    public final void clean(final TorqueMode mode) {
        desiredState = State.OFF;
    }

    public static final synchronized Climb getInstance() {
        return instance == null ? instance = new Climb() : instance;
    }
}
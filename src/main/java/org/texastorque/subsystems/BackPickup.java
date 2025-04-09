/**
 * Copyright 2025 Texas Torque.
 *
 * This file is part of Bravo/Charlie/Crashout-2025, which is not licensed for distribution.
 * For more details, see ./license.txt or write <davey.adams.three@gmail.com>.
 */
package org.texastorque.subsystems;

import org.texastorque.Input;
import org.texastorque.Ports;
import org.texastorque.Subsystems;
import org.texastorque.torquelib.Debug;
import org.texastorque.torquelib.base.TorqueMode;
import org.texastorque.torquelib.base.TorqueState;
import org.texastorque.torquelib.base.TorqueStatorSubsystem;
import org.texastorque.torquelib.motors.TorqueNEO;

import edu.wpi.first.math.controller.PIDController;

public final class BackPickup extends TorqueStatorSubsystem<BackPickup.State> implements Subsystems {

    private static volatile BackPickup instance;
    private final TorqueNEO backPickup, backRollers;
    private PIDController backPID;
    private RollerState rollerState = RollerState.OFF;

    public static enum State implements TorqueState {
        UP(0.092),
        SHOOT(1.0952),
        INTAKE(4);

        private double angle;

        private State(double angle) {
            this.angle = angle;
        }

        public double getAngle() {
            return angle;
        }
    }

    public static enum RollerState implements TorqueState {
        INTAKE(-5), OUTTAKE(2), OFF(0);

        private final double volts;

        private RollerState(double volts) {
            this.volts = volts;
        }

        public double getVolts() {
            return volts;
        }
    }

    private BackPickup() {
        super(State.UP);
        backPickup = new TorqueNEO(Ports.BACKPICKUP);
        backRollers = new TorqueNEO(Ports.BACKPICKUP_ROLLERS);
        backPID = new PIDController(1, 0, 0);
    }

    @Override
    public final void initialize(final TorqueMode mode) {}

    @Override
    public final void update(final TorqueMode mode) {
        Debug.log("Back Pickup", getBackPickupPosition());
        
        double volts = backPID.calculate(getBackPickupPosition(), getState().getAngle());
        backPickup.setVolts(volts);
        backRollers.setVolts(rollerState.getVolts());
        
    }

	@Override
    public final void clean(final TorqueMode mode) {
        desiredState = State.UP;
        rollerState = RollerState.OFF;
    }

    @Override
    public void onStateChange(final State lastState) {}

    public void setBackRollerState(final RollerState state) {
        rollerState = state;
    }


    public double getBackPickupPosition() {
        return backPickup.getPosition();
    }

    public static final synchronized BackPickup getInstance() {
        return instance == null ? instance = new BackPickup() : instance;
    }
}

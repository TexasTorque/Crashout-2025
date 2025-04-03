/**
 * Copyright 2025 Texas Torque.
 *
 * This file is part of Bravo/Charlie/Crashout-2025, which is not licensed for distribution.
 * For more details, see ./license.txt or write <davey.adams.three@gmail.com>.
 */
package org.texastorque.subsystems;

import org.texastorque.Ports;
import org.texastorque.Subsystems;
import org.texastorque.torquelib.Debug;
import org.texastorque.torquelib.base.TorqueMode;
import org.texastorque.torquelib.base.TorqueState;
import org.texastorque.torquelib.base.TorqueStatorSubsystem;
import org.texastorque.torquelib.motors.TorqueNEO;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public final class Climb extends TorqueStatorSubsystem<Climb.State> implements Subsystems {

    private static volatile Climb instance;
    private final TorqueNEO climb;
    // private final PIDController climbPID;

    public State pastState;
    // private double pastStateTime;

    public static enum State implements TorqueState {
        OFF(0),
        OUT(9),
        IN(-8);

        public double volts;

        private State(double volts) {
            this.volts = volts;
        }

        public double getVolts() {
            return volts;
        }
    }

    private Climb() {
        super(State.OFF);
        pastState = State.OFF;
        // pastStateTime = Timer.getFPGATimestamp();

        climb = new TorqueNEO(Ports.CLIMB)
                .idleMode(IdleMode.kBrake)
                .apply();
        }

    @Override
    public final void initialize(final TorqueMode mode) {}

    @Override
    public final void update(final TorqueMode mode) {
        Debug.log("Climb State", desiredState.toString());
        Debug.log("Climb Position", getClimbPosition());

        if (claw.getState() != Claw.State.CLIMB && claw.getState() != Claw.State.HALF_CLIMB) {
            climb.setVolts(0);
        } else {
            if (getClimbPosition() > 285.8833 && desiredState == State.OUT) {
                climb.setVolts(0);
            } else {
                climb.setVolts(desiredState.volts);
            }
        }
    }

    @Override
    public final void clean(final TorqueMode mode) {
        desiredState = State.OFF;
    }

    @Override
    public void onStateChange(final State lastState) {
        // pastStateTime = Timer.getFPGATimestamp();
        pastState = lastState;
    }

    public double getClimbPosition() {
        // if (RobotBase.isSimulation()) {
        //     final double timeToAnimate = Math.abs(desiredState.position - pastState.position) / 120;
        //     final double animationMultiplier = (Timer.getFPGATimestamp() - pastStateTime) / timeToAnimate;
        //     final double position = ((desiredState.position - pastState.position) * animationMultiplier) + pastState.position;

        //     if (claw.isNearState() && (claw.getState() == Claw.State.CLIMB || claw.getState() == Claw.State.HALF_CLIMB)) {
        //         if (animationMultiplier > 1) return desiredState.position;
        //         return position;
        //     }

        //     pastStateTime = Timer.getFPGATimestamp();
        //     return pastState.position;
        // }

        return climb.getPosition();
    }

    // public boolean isAtState() {
    //     return TorqueMath.toleranced(getClimbPosition(), desiredState.getPosition(), 10);
    // }

    public static final synchronized Climb getInstance() {
        return instance == null ? instance = new Climb() : instance;
    }
}

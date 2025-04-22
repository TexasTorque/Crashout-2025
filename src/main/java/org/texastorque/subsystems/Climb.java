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

import edu.wpi.first.wpilibj.RobotBase;

public final class Climb extends TorqueStatorSubsystem<Climb.State> implements Subsystems {

    private static volatile Climb instance;
    private final TorqueNEO climb;
    private double simulatedClimbPosition;

    public static enum State implements TorqueState {
        OFF(0),
        OUT(8),
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
            if (getClimbPosition() > 279.0271 && desiredState == State.OUT) {
                climb.setVolts(0);
            } else if (getClimbPosition() < 100 && desiredState == State.IN) {
                climb.setVolts(0);
            } else {
                climb.setVolts(desiredState.volts);
                simulatedClimbPosition += desiredState.volts;
            }
        }
    }

    @Override
    public final void clean(final TorqueMode mode) {
        desiredState = State.OFF;
    }

    public double getClimbPosition() {
        if (RobotBase.isSimulation()) {
            return simulatedClimbPosition;
        }

        return climb.getPosition();
    }

    public static final synchronized Climb getInstance() {
        return instance == null ? instance = new Climb() : instance;
    }
}

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

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;

public final class Climb extends TorqueStatorSubsystem<Climb.State> implements Subsystems {

    private static volatile Climb instance;
    private final TorqueNEO climb;
    private final PIDController climbPID;

    public State pastState;
    private double pastStateTime;

    public static enum State implements TorqueState {
        STOWED(0),
        OUT(285.8833),
        IN(138);

        public double position;

        private State(double position) {
            this.position = position;
        }

        public double getPosition() {
            return position;
        }
    }

    private Climb() {
        super(State.STOWED);
        pastState = State.STOWED;
        pastStateTime = Timer.getFPGATimestamp();

        climb = new TorqueNEO(Ports.CLIMB)
                .idleMode(IdleMode.kBrake)
                .apply();
        
        climbPID = new PIDController(1, 0, 0);
    }

    @Override
    public final void initialize(final TorqueMode mode) {}

    @Override
    public final void update(final TorqueMode mode) {
        Debug.log("Climb State", desiredState.toString());
        Debug.log("Climb Position", getClimbPosition());

        if (claw.getState() != Claw.State.CLIMB) {
            climb.setVolts(0);
        } else {
            if (claw.isNearState() && claw.getState() == Claw.State.CLIMB) {
                final double CLIMB_MAX_VOLTS_OUT = 12;
                final double CLIMB_MAX_VOLTS_IN = 8;
                double volts = climbPID.calculate(getClimbPosition(), desiredState.position);
                if (volts > CLIMB_MAX_VOLTS_OUT) volts = CLIMB_MAX_VOLTS_OUT;
                if (volts < -CLIMB_MAX_VOLTS_IN) volts = -CLIMB_MAX_VOLTS_IN;
                
                climb.setVolts(volts);
            } else {
                climb.setVolts(0);
            }
        }
    }

    @Override
    public final void clean(final TorqueMode mode) {}

    @Override
    public void onStateChange(final State lastState) {
        pastStateTime = Timer.getFPGATimestamp();
        pastState = lastState;
    }

    public double getClimbPosition() {
        if (RobotBase.isSimulation()) {
            final double timeToAnimate = Math.abs(desiredState.position - pastState.position) / 120;
            final double animationMultiplier = (Timer.getFPGATimestamp() - pastStateTime) / timeToAnimate;
            final double position = ((desiredState.position - pastState.position) * animationMultiplier) + pastState.position;

            if (animationMultiplier > 1) return desiredState.position;
            return position;
        }

        return climb.getPosition();
    }

    public boolean isSafe() {
        return getClimbPosition() > 250;
    }

    public static final synchronized Climb getInstance() {
        return instance == null ? instance = new Climb() : instance;
    }
}

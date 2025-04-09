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
import org.texastorque.torquelib.util.TorqueMath;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;

public final class Intake extends TorqueStatorSubsystem<Intake.State> implements Subsystems {

    private static volatile Intake instance;
    private final TorqueNEO pivot, rollers;
    // public final DigitalInput breakBeam;
    private RollerState rollerState = RollerState.OFF;
    private final PIDController intakePID;

    public static enum State implements TorqueState {
        INTAKE(0),
        SCORE_L1(740.7592),
        HANDOFF(1108.3506),
        ZERO(69);

        private double angle;

        private State(double angle) {
            this.angle = angle;
        }

        public double getAngle() {
            return angle;
        }
    }

    public static enum RollerState implements TorqueState {
        INTAKE(-4), OUTTAKE(4), OFF(0);

        private final double volts;

        private RollerState(double volts) {
            this.volts = volts;
        }

        public double getVolts() {
            return volts;
        }
    }

    private Intake() {
        super(State.ZERO);

        pivot = new TorqueNEO(Ports.INTAKE_PIVOT);

        intakePID = new PIDController(.5, 0, 0);

        rollers = new TorqueNEO(Ports.INTAKE_ROLLERS);

        // breakBeam = new DigitalInput(Ports.INTAKE_BREAK_BEAM);
    }

    @Override
    public final void initialize(final TorqueMode mode) {}

    @Override
    public final void update(final TorqueMode mode) {
        Debug.log("Ground Intake Position", getPivotAngle());
        double volts = intakePID.calculate(getPivotAngle(), desiredState.angle); 

        // First elevator moves up
        // Once intake at state, claw handoffs
        if(desiredState == State.HANDOFF && elevator.isAtState() && claw.isAtState()) {
            // brothaaaa
            rollers.setVolts(0);
            pivot.setVolts(volts);  
        } else {
            rollers.setVolts(rollerState.getVolts());
            pivot.setVolts(volts);
        }
    }

	@Override
    public final void clean(final TorqueMode mode) {
        if (mode.isTeleop()) {
            rollerState = rollerState.OFF;
            desiredState = State.INTAKE;
        }
    }

    @Override
    public void onStateChange(final State lastState) {}

    public void setRollerState(RollerState rollerState) {
        this.rollerState = rollerState;
    }

    public double getPivotAngle() {
        return pivot.getPosition();
    }

    public boolean isAtState() {
        return TorqueMath.toleranced(getPivotAngle(), desiredState.angle, 5);
    }

    // public boolean hasCoral() {
    //     return breakBeam.get();
    // }

    public boolean isIntaking() {
        return wantsState(State.INTAKE);
    }

    public static final synchronized Intake getInstance() {
        return instance == null ? instance = new Intake() : instance;
    }
}

/**
 * Copyright 2025 Texas Torque.
 *
 * This file is part of Bravo/Charlie/Crashout-2025, which is not licensed for distribution.
 * For more details, see ./license.txt or write <davey.adams.three@gmail.com>.
 */
package org.texastorque.subsystems;

import org.texastorque.Ports;
import org.texastorque.Subsystems;
import org.texastorque.torquelib.base.TorqueMode;
import org.texastorque.torquelib.base.TorqueState;
import org.texastorque.torquelib.base.TorqueStatorSubsystem;
import org.texastorque.torquelib.motors.TorqueNEO;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;

public final class Intake extends TorqueStatorSubsystem<Intake.State> implements Subsystems {

    private static volatile Intake instance;
    private final TorqueNEO pivot, rollers;
    public final DigitalInput breakBeam;
    private RollerState rollerState = RollerState.OFF;
    private final PIDController intakePID;

    public static enum State implements TorqueState {
        INTAKE(0),
        SCORE_L1(15),
        HANDOFF(90),
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
        INTAKE(-12), OUTTAKE(12), OFF(0);

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

        pivot = new TorqueNEO(Ports.INTAKE_PIVOT)
            .idleMode(IdleMode.kBrake)
            .apply();

        intakePID = new PIDController(.5, 0, 0);

        rollers = new TorqueNEO(Ports.INTAKE_ROLLERS)
            .idleMode(IdleMode.kBrake)
            .apply();

        breakBeam = new DigitalInput(Ports.INTAKE_BREAK_BEAM);
    }

    @Override
    public final void initialize(final TorqueMode mode) {}

    @Override
    public final void update(final TorqueMode mode) {
        double volts = intakePID.calculate(getPivotAngle(), desiredState.angle);

        rollers.setVolts(rollerState.getVolts());
        pivot.setVolts(volts);
    }

	@Override
    public final void clean(final TorqueMode mode) {
        if (mode.isTeleop()) {
            rollerState = rollerState.INTAKE;
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

    public boolean hasCoral() {
        return breakBeam.get();
    }

    public static final synchronized Intake getInstance() {
        return instance == null ? instance = new Intake() : instance;
    }
}

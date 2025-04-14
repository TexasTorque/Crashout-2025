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
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;

public final class Pickup extends TorqueStatorSubsystem<Pickup.State> implements Subsystems {

    private static volatile Pickup instance;
    private final TorqueNEO pivot, rollers;
    private final PIDController pivotPID;
    private final CANcoder pivotEncoder;
    private RollersState rollersState = RollersState.OFF;
    private State pastState;
    private double pastStateTime;

    public static enum State implements TorqueState {
        ZERO(0),
        STOW(3.6035),
        SHOOT(34),
        INTAKE(122);

        private double angle;

        private State(double angle) {
            this.angle = angle;
        }

        public double getAngle() {
            return angle;
        }
    }

    public static enum RollersState implements TorqueState {
        INTAKE(-12),
        SHOOT(4),
        OFF(0);

        private double volts;

        private RollersState(double volts) {
            this.volts = volts;
        }

        public double getVolts() {
            return volts;
        }
    }

    private Pickup() {
        super(State.ZERO);
        pastState = State.ZERO;
        pastStateTime = Timer.getFPGATimestamp();

        pivot = new TorqueNEO(Ports.PICKUP_PIVOT)
            .idleMode(IdleMode.kBrake)
            .inverted(true)
            .currentLimit(30)
            .apply();
        
        rollers = new TorqueNEO(Ports.PICKUP_ROLLERS)
            .currentLimit(20)
            .apply();

        pivotPID = new PIDController(0.05, 0, 0);
        pivotPID.enableContinuousInput(0, 360);
        
        pivotEncoder = new CANcoder(Ports.PICKUP_ENCODER);
    }

    @Override
    public final void initialize(final TorqueMode mode) {
        State.ZERO.angle = getPivotAngle();
        setState(State.ZERO);
    }

    @Override
    public final void update(final TorqueMode mode) {
        // Calculate volts for current setpoint
        final double PIVOT_MAX_VOLTS = 4;
        double volts = pivotPID.calculate(getPivotAngle(), desiredState.getAngle());
        // final double ff = .5 * Math.cos(Math.toRadians(getPivotAngle() - 1.631949));
        if (Math.abs(volts) > PIVOT_MAX_VOLTS) volts = Math.signum(volts) * PIVOT_MAX_VOLTS;

        // if (desiredState == State.ZERO) {
        //     pivot.setVolts(0);
        // } else {
        //     pivot.setVolts(volts + ff); //Delete volts to tune ff
        // }

        pivot.setVolts(-volts);

        if (isAtState()) {
            rollers.setVolts(rollersState.getVolts());
        }

        Debug.log("Pivot Volts", volts);
        // Debug.log("Pivot FF", ff);
        Debug.log("Pivot Current", pivot.getOutputCurrent());
        Debug.log("Pivot Angle", getPivotAngle());
        Debug.log("Pivot At State", isAtState());
        Debug.log("Pivot State", desiredState.toString());
        Debug.log("Rollers State", rollersState.toString());
        Debug.log("Rollers Current", rollers.getOutputCurrent());
    }

	@Override
    public final void clean(final TorqueMode mode) {
        if (mode.isTeleop()) {
            desiredState = State.STOW;
            rollersState = RollersState.OFF;
        }
    }

    @Override
    public void onStateChange(final State lastState) {
        pastStateTime = Timer.getFPGATimestamp();
        pastState = lastState;
    }

    public double getPivotAngle() {
        if (RobotBase.isSimulation()) {
            double desiredAngle = desiredState.angle;
            final double timeToAnimate = Math.abs(desiredAngle - pastState.angle) / 180;
            final double animationMultiplier = (Timer.getFPGATimestamp() - pastStateTime) / timeToAnimate;
            double position = ((desiredAngle - pastState.angle) * animationMultiplier) + pastState.angle;
            if (animationMultiplier > 1) position = desiredAngle;

            if (desiredState != State.ZERO) {
                return position;
            }

            pastStateTime = Timer.getFPGATimestamp();
            return pastState.angle;
        }
        return pivotEncoder.getAbsolutePosition().getValueAsDouble() * 360;
    }

    public void setRollersState(RollersState rollersState) {
        this.rollersState = rollersState;
    }

    public final boolean isAtState() {
        return isAtState(desiredState);
    }

    public final boolean isAtState(final State state) {
        return TorqueMath.toleranced(getPivotAngle(), state.getAngle(), 5);
    }

    public static final synchronized Pickup getInstance() {
        return instance == null ? instance = new Pickup() : instance;
    }
}
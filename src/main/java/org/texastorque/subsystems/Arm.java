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

public final class Arm extends TorqueStatorSubsystem<Arm.State> implements Subsystems {

    private static volatile Arm instance;
    private final TorqueNEO rotary, rollers;
    private final PIDController rotaryPID;
    private final CANcoder rotaryEncoder;
    private RollersState rollersState = RollersState.OFF;
    private State pastState;
    private double pastStateTime;

    public static enum State implements TorqueState {
        ZERO(0),
        STOW(10),
        SAFE(100),
        OUT(140);

        private double angle;

        private State(double angle) {
            this.angle = angle;
        }

        public double getAngle() {
            return angle;
        }
    }

    public static enum RollersState implements TorqueState {
        INTAKE(-8),
        OUTTAKE(4),
        OFF(0);

        private double volts;

        private RollersState(double volts) {
            this.volts = volts;
        }

        public double getVolts() {
            return volts;
        }
    }

    private Arm() {
        super(State.ZERO);
        pastState = State.ZERO;
        pastStateTime = Timer.getFPGATimestamp();

        rotary = new TorqueNEO(Ports.ARM_ROTARY)
            .idleMode(IdleMode.kBrake)
            .currentLimit(20)
            .inverted(true)
            .apply();
        
        rollers = new TorqueNEO(Ports.ARM_ROLLERS)
            .currentLimit(60)
            .apply();

        rotaryPID = new PIDController(.05, 0, 0);
        
        rotaryEncoder = new CANcoder(Ports.ARM_ENCODER);
    }

    @Override
    public final void initialize(final TorqueMode mode) {
        State.ZERO.angle = getRotaryAngle();
        setState(State.ZERO);
    }

    @Override
    public final void update(final TorqueMode mode) {
        // Calculate volts for current setpoint
        final double ROTARY_MAX_VOLTS = 6;
        double volts = rotaryPID.calculate(getRotaryAngle(), desiredState.getAngle());
        if (Math.abs(volts) > ROTARY_MAX_VOLTS) volts = Math.signum(volts) * ROTARY_MAX_VOLTS;

        if (desiredState == State.ZERO) {
            rotary.setVolts(0);
        } else {
            rotary.setVolts(volts);
        }

        rollers.setVolts(rollersState.getVolts());

        Debug.log("Rotary Volts", volts);
        Debug.log("Rotary Current", rotary.getOutputCurrent());
        Debug.log("Rotary Angle", getRotaryAngle());
        Debug.log("Rotary At State", isAtState());
        Debug.log("Rotary State", desiredState.toString());
        Debug.log("Rollers State", rollersState.toString());
    }

	@Override
    public final void clean(final TorqueMode mode) {
        if (mode.isTeleop()) {
            rollersState = RollersState.OFF;
        }
    }

    @Override
    public void onStateChange(final State lastState) {
        pastStateTime = Timer.getFPGATimestamp();
        pastState = lastState;
    }

    public double getRotaryAngle() {
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
        return rotaryEncoder.getAbsolutePosition().getValueAsDouble() * 360;
    }

    public void setRollersState(RollersState rollersState) {
        this.rollersState = rollersState;
    }

    public final boolean isAtState() {
        return TorqueMath.toleranced(getRotaryAngle(), desiredState.getAngle(), 2);
    }

    public static final synchronized Arm getInstance() {
        return instance == null ? instance = new Arm() : instance;
    }
}

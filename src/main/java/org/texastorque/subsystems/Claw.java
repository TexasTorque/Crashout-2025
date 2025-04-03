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
import org.texastorque.torquelib.control.TorqueCurrentSpike;
import org.texastorque.torquelib.motors.TorqueNEO;
import org.texastorque.torquelib.util.TorqueMath;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;

public final class Claw extends TorqueStatorSubsystem<Claw.State> implements Subsystems {

    private static volatile Claw instance;
    private final TorqueNEO shoulder, algaeRollers, coralRollers;
    private final ProfiledPIDController shoulderPID;
    private final CANcoder shoulderEncoder;
    public final TorqueCurrentSpike coralSpike;
    private AlgaeState algaeState = AlgaeState.OFF;
    private CoralState coralState = CoralState.OFF;
    private State selectedState;
    private State pastState;
    private double pastStateTime;

    public static enum State implements TorqueState {
        ZERO(0),
        STOW(51.2793),
        SCORE_L1(25),
        SCORE_L2(196.6699),
        SCORE_L3(175),
        SCORE_L4(215.0977),
        NET(166.6797),
        ALGAE_EXTRACTION(287.2656),
        PROCESSOR(80.2832),
        REGRESSION_CORAL_HP(20), // It's a half state, used when not in the HP zone, but when in the zone it uses regression
        CORAL_HP(30), 
        HALF_CLIMB(196.6699),
        CLIMB(296);

        private double angle;

        private State(double angle) {
            this.angle = angle;
        }

        public double getAngle() {
            return angle;
        }
    }

    public static enum AlgaeState implements TorqueState {
        INTAKE(-10), SHOOT(6), SHOOT_SEMI_FAST(8), SHOOT_FAST(12), OFF(0);

        private final double volts;

        private AlgaeState(double volts) {
            this.volts = volts;
        }

        public double getVolts() {
            return volts;
        }
    }

    public static enum CoralState implements TorqueState {
        INTAKE(-4), SHOOT(12), SHOOT_SLOW(4), OFF(-1);

        private final double volts;

        private CoralState(double volts) {
            this.volts = volts;
        }

        public double getVolts() {
            return volts;
        }
    }

    private Claw() {
        super(State.ZERO);
        pastState = State.ZERO;
        pastStateTime = Timer.getFPGATimestamp();

        selectedState = State.ZERO;

        shoulder = new TorqueNEO(Ports.SHOULDER)
            .idleMode(IdleMode.kBrake)
            .inverted(true)
            .apply();

        shoulderEncoder = new CANcoder(Ports.SHOULDER_ENCODER);
        shoulderPID = new ProfiledPIDController(.2, 0, 0,
                new TrapezoidProfile.Constraints(2500, 900));

        algaeRollers = new TorqueNEO(Ports.ROLLERS_ALGAE)
            .apply();

        coralRollers = new TorqueNEO(Ports.ROLLERS_CORAL)
            .inverted(true)
            .currentLimit(10)
            .idleMode(IdleMode.kBrake)
            .apply();

        coralSpike = new TorqueCurrentSpike(18);

        shoulderPID.reset(getShoulderAngle());
    }

    @Override
    public final void initialize(final TorqueMode mode) {
        State.ZERO.angle = getShoulderAngle();
        setState(State.ZERO);
        shoulderPID.reset(getShoulderAngle());
    }

    @Override
    public final void update(final TorqueMode mode) {
        // Shoulder regression for HP
        double desiredAngle = desiredState.angle;
        if (desiredState == State.REGRESSION_CORAL_HP && perception.inCoralStationZone()) {
            desiredAngle = getCoralStationAngle();
        }

        // Calculate volts for current setpoint
        final double SHOULDER_MAX_VOLTS = 10;
        double volts = shoulderPID.calculate(getShoulderAngle(), desiredAngle);
        final double ff = .35 * Math.sin(Math.toRadians(getShoulderAngle() + 25));
        if (Math.abs(volts) > SHOULDER_MAX_VOLTS) volts = Math.signum(volts) * SHOULDER_MAX_VOLTS;

        if (climb.getState() == Climb.State.OUT && climb.isAtState()) {
            setState(State.CLIMB);
        }

        // Apply volts
        if (desiredState == State.ZERO) {
            shoulder.setVolts(ff);
            shoulderPID.reset(getShoulderAngle());
        } else {
            shoulder.setVolts(volts + ff);
        }

        algaeRollers.setVolts(algaeState.getVolts());

        // Coral rollers logic
        if (coralState == CoralState.SHOOT || coralState == CoralState.SHOOT_SLOW) 
            coralSpike.reset();
        if (coralState == CoralState.INTAKE && desiredState != State.REGRESSION_CORAL_HP && desiredState != State.CORAL_HP)
            coralState = CoralState.OFF;
        
        if (hasCoral() && (coralState != CoralState.SHOOT || coralState != CoralState.SHOOT_SLOW)) {
            coralState = CoralState.OFF;
            coralRollers.setVolts(coralState.getVolts());
        } else {
            coralRollers.setVolts(coralState.getVolts());
        }

        Debug.log("Shoulder Volts", volts + ff);
        Debug.log("Coral Volts", coralState.getVolts());
        Debug.log("Shoulder Position", getShoulderAngle());
        Debug.log("Claw State", desiredState.toString());
        Debug.log("Has Coral", hasCoral());
        Debug.log("Shoulder At State", isAtState());
        Debug.log("Coral Current", coralRollers.getOutputCurrent());
        Debug.log("Coral State", coralState.toString());
        Debug.log("Algae State", algaeState.toString());
    }

	@Override
    public final void clean(final TorqueMode mode) {
        if (mode.isTeleop()) {
            algaeState = AlgaeState.OFF;
        }
    }

    @Override
    public void onStateChange(final State lastState) {
        pastStateTime = Timer.getFPGATimestamp();
        pastState = lastState;
    }

    public double getCoralStationAngle() {
        double angle = -103.09278 * perception.getHPDistance() + 42.98969;
        if (angle > 30) angle = 30;
        if (angle < 20) angle = 20;
        return angle;
    }

    public final double getShoulderAngle() {
        if (RobotBase.isSimulation()) {
            double desiredAngle = desiredState.angle;
            if (desiredState == State.REGRESSION_CORAL_HP && perception.inCoralStationZone()) {
                desiredAngle = getCoralStationAngle();
            }
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
        return shoulderEncoder.getAbsolutePosition().getValueAsDouble() * 360;
    }

    public final boolean isAtState() {
        return TorqueMath.toleranced(getShoulderAngle(), desiredState.getAngle(), 8);
    }

    public final boolean isNearState() {
        return TorqueMath.toleranced(getShoulderAngle(), desiredState.getAngle(), 40);
    }

    public void setSelectedState(State selectedState) {
      	this.selectedState = selectedState;
    }

    public State getSelectedState() {
      	return selectedState;
    }

    public boolean hasCoral() {
        return coralSpike.calculate(coralRollers.getOutputCurrent());
    }

    public void setAlgaeState(AlgaeState algaeState) {
        this.algaeState = algaeState;
    }

    public void setCoralState(CoralState coralState) {
        this.coralState = coralState;
    }

    public static final synchronized Claw getInstance() {
        return instance == null ? instance = new Claw() : instance;
    }
}

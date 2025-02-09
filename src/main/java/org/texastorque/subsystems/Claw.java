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

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;

public final class Claw extends TorqueStatorSubsystem<Claw.State> implements Subsystems {

    private static volatile Claw instance;
    private final TorqueNEO shoulder, algaeRollers, coralRollers;
    private final PIDController shoulderPID;
    private final CANcoder shoulderEncoder;
    private AlgaeState algaeState = AlgaeState.OFF;
    private CoralState coralState = CoralState.OFF;
    public final TorqueCurrentSpike coralSpike;
    private State pastState;
    private double pastStateTime;

    public static enum State implements TorqueState {
        ZERO(0),
        STOW(180),
        L1_SCORE(240),
        MID_SCORE(160),
        L4_SCORE(180),
        NET(140),
        ALGAE_EXTRACTION(290),
        PROCESSOR(300),
        CORAL_HP(38);

        private final double angle;

        private State(double angle) {
            this.angle = angle;
        }

        public double getAngle() {
            return angle;
        }
    }

    public static enum AlgaeState implements TorqueState {
        INTAKE(-12), SHOOT(12), OFF(0);

        private final double volts;

        private AlgaeState(double volts) {
            this.volts = volts;
        }

        public double getVolts() {
            return volts;
        }
    }

    public static enum CoralState implements TorqueState {
        INTAKE(-4), SHOOT(3), OFF(0);

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

        shoulder = new TorqueNEO(Ports.SHOULDER)
            .idleMode(IdleMode.kBrake)
            .apply();

        shoulderEncoder = new CANcoder(Ports.SHOULDER_ENCODER);
        shoulderPID = new PIDController(.25, 0, 0);

        algaeRollers = new TorqueNEO(Ports.ROLLERS_ALGAE)
            .inverted(true)
            .apply();

        coralRollers = new TorqueNEO(Ports.ROLLERS_CORAL)
            .inverted(true)
            .idleMode(IdleMode.kBrake)
            .apply();

        coralSpike = new TorqueCurrentSpike(12);
    }

    @Override
    public final void initialize(final TorqueMode mode) {}

    @Override
    public final void update(final TorqueMode mode) {
        Debug.log("Shoulder Position", getShoulderAngle());
        Debug.log("Claw State", desiredState.toString());
        Debug.log("Has Coral", hasCoral());
        Debug.log("Shoulder At State", isAtState());
        Debug.log("Coral Current", coralRollers.getOutputCurrent());
        Debug.log("Coral State", coralState.toString());
        Debug.log("Algae State", algaeState.toString());

        shoulder.setVolts(shoulderPID.calculate(getShoulderAngle(), desiredState.angle));

        if (desiredState == State.ZERO) {
            shoulder.setVolts(0);
        }

        algaeRollers.setVolts(algaeState.getVolts());
        coralRollers.setVolts(coralState.getVolts());

        // If we have coral, set volts to 0 to prevent stalling the motor
        if (hasCoral() && coralState != CoralState.SHOOT) {
            coralRollers.setVolts(0);
        }
    }

	@Override
    public final void clean(final TorqueMode mode) {
        algaeState = AlgaeState.OFF;
    }

    @Override
    public void onStateChange(final State lastState) {
        pastStateTime = Timer.getFPGATimestamp();
        pastState = lastState;
    }

    public final double getShoulderAngle() {
        if (RobotBase.isSimulation()) {
            final double timeToAnimate = 1;
            final double animationMultiplier = (Timer.getFPGATimestamp() - pastStateTime) / timeToAnimate;

            if (animationMultiplier > 1) return desiredState.angle;
            return ((desiredState.angle - pastState.angle) * animationMultiplier) + pastState.angle;
        }
        return shoulderEncoder.getAbsolutePosition().getValueAsDouble() * 360;
    }

    public final boolean isAtState() {
        return TorqueMath.toleranced(getShoulderAngle(), desiredState.getAngle(), 8);
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
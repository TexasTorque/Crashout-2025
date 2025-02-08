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

public final class Claw extends TorqueStatorSubsystem<Claw.State> implements Subsystems {

    private static volatile Claw instance;
    private final TorqueNEO shoulder, algaeRollers, coralRollers;
    private final PIDController shoulderPID;
    private final CANcoder shoulderEncoder;
    private AlgaeState algaeState = AlgaeState.OFF;
    private CoralState coralState = CoralState.OFF;
    private final TorqueCurrentSpike coralSpike, algaeSpike;

    // Angle that we intake from HP is 320 degrees (0 is 40 degrees past)
    public static enum State implements TorqueState {
        STOW(121.7392),
        L1_SCORE(181.6025),
        MID_SCORE(108.8867),
        L4_SCORE(121.8035),
        NET(98.3398),
        ALGAE_EXTRACTION(256.9304),
        PROCESSOR(242.1289),
        CORAL_HP(352.7051),
        BABYBIRD(308.8942);

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
        INTAKE(-4), SHOOT(4), OFF(0);

        private final double volts;

        private CoralState(double volts) {
            this.volts = volts;
        }

        public double getVolts() {
            return volts;
        }
    }

    private Claw() {
        super(State.STOW);

        shoulder = new TorqueNEO(Ports.SHOULDER)
            .idleMode(IdleMode.kBrake)
            .apply();

        shoulderEncoder = new CANcoder(Ports.SHOULDER_ENCODER);
        shoulderPID = new PIDController(.25, 0, 0);

        algaeRollers = new TorqueNEO(Ports.ROLLERS_ALGAE)
            .inverted(true)
            .currentLimit(14)
            .apply();

        coralRollers = new TorqueNEO(Ports.ROLLERS_CORAL)
            .inverted(true)
            .currentLimit(8)
            .apply();

        coralSpike = new TorqueCurrentSpike(10);
        algaeSpike = new TorqueCurrentSpike(10); // Needs testing
    }

    @Override
    public final void initialize(final TorqueMode mode) {}

    @Override
    public final void update(final TorqueMode mode) {
        Debug.log("Claw Position", getClawAngle());
        Debug.log("Claw State", desiredState.toString());
        Debug.log("Has Coral", hasCoral());
        Debug.log("Has Algae", hasAlgae());
        Debug.log("Elevator At State", isAtState());

        // If we are moving to a high position (>=3.7), Elevator moves first
        if (elevator.getState().position >= 3.7 || elevator.getState() == Elevator.State.STOW) {
            if (elevator.isAtState()) {
                shoulder.setVolts(shoulderPID.calculate(getClawAngle(), desiredState.getAngle()));
            }
        } else {
            shoulder.setVolts(shoulderPID.calculate(getClawAngle(), desiredState.getAngle()));
        }

        algaeRollers.setVolts(algaeState.getVolts());
        coralRollers.setVolts(coralState.getVolts());

        // If we have coral, set volts to 0 to prevent stalling the motor
        // if (hasCoral() && coralState != CoralState.SHOOT) {
        //     coralRollers.setVolts(0);
        // }

        // If we have algae, set volts to 0 to prevent stalling the motor
        // if (hasAlgae() && algaeState != AlgaeState.SHOOT) {
        //     algaeRollers.setVolts(0);
        // }
    }

	@Override
    public final void clean(final TorqueMode mode) {
        if (desiredState == State.CORAL_HP) {
            desiredState = State.STOW;
        }
    }

    public final double getClawAngle() {
        if (RobotBase.isSimulation()) return desiredState.angle;
        
        return shoulderEncoder.getAbsolutePosition().getValueAsDouble() * 360;
    }

    public final boolean isAtState() {
        return TorqueMath.toleranced(getClawAngle(), desiredState.getAngle(), 5);
    }

    public boolean hasCoral() {
        return coralSpike.calculate(coralRollers.getOutputCurrent());
    }

    public boolean hasAlgae() {
        return algaeSpike.calculate(algaeRollers.getOutputCurrent());
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
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public final class Claw extends TorqueStatorSubsystem<Claw.State> implements Subsystems {

    private static volatile Claw instance;
    private final TorqueNEO claw, algaeRollers, coralRollers;
    private final PIDController clawPID;
    private final CANcoder clawEncoder;
    private AlgaeState algaeState = AlgaeState.OFF;
    private CoralState coralState = CoralState.OFF;
    private final TorqueCurrentSpike spike;

    // Angle that we intake from HP is 0 degrees
    public static enum State implements TorqueState {
        STOW(161.7392 - 40), //
        L1_SCORE(221.6025 - 40), //
        MID_SCORE(148.8867 - 40), //
        L4_SCORE(161.8035 - 40), //
        NET(138.3398 - 40), //
        ALGAE_EXTRACTION(296.9304 - 40), //
        PROCESSOR(282.1289 - 40), //
        CORAL_HP(320), //
        BABYBIRD(348.8942 - 40); //

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
        claw = new TorqueNEO(Ports.CLAW)
            .idleMode(IdleMode.kCoast)
            .apply();
        clawEncoder = new CANcoder(Ports.CLAW_ENCODER);
        clawPID = new PIDController(.25, 0, 0);

        algaeRollers = new TorqueNEO(Ports.ROLLERS_ALGAE)
            .inverted(true)
            .currentLimit(14)
            .apply();
        coralRollers = new TorqueNEO(Ports.ROLLERS_CORAL)
            .currentLimit(3)
            .apply();

        spike = new TorqueCurrentSpike(10);
    }

    @Override
    public final void initialize(final TorqueMode mode) {}

    @Override
    public final void update(final TorqueMode mode) {
        Debug.log("Claw Position", getClawAngle());
        Debug.log("Claw State", desiredState.toString());

        if (elevator.getState().position >= 3.5) { // if greater than 3.5
            if (elevator.isAtState()) { // elevator moves first
                claw.setVolts(clawPID.calculate(getClawAngle(), desiredState.getAngle()));
            }
        } else {
            claw.setVolts(clawPID.calculate(getClawAngle(), desiredState.getAngle()));
        }

        algaeRollers.setVolts(algaeState.getVolts());
        coralRollers.setVolts(-coralState.getVolts());

        boolean hasCoral = spike.calculate(coralRollers.getOutputCurrent());
        SmartDashboard.putBoolean("Has Coral", hasCoral);
        SmartDashboard.putNumber("Coral Current", coralRollers.getOutputCurrent());

        if (hasCoral && coralState != CoralState.SHOOT) {
            coralRollers.setVolts(0);
        }
    }

	@Override
    public final void clean(final TorqueMode mode) {}

    public boolean hasCoral() {
        return coralRollers.getOutputCurrent() > 10;
    }

    public final boolean isAtState() {
        SmartDashboard.putBoolean("Claw At State", TorqueMath.toleranced(getClawAngle(), desiredState.getAngle(), 5));
        return TorqueMath.toleranced(getClawAngle(), desiredState.getAngle(), 5);
    }

    public final double getClawAngle() { // maybe get absolute position
        return clawEncoder.getAbsolutePosition().getValueAsDouble() * 360;
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
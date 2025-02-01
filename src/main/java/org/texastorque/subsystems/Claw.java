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

import edu.wpi.first.math.controller.PIDController;

public final class Claw extends TorqueStatorSubsystem<Claw.State> implements Subsystems {

    private static volatile Claw instance;
    private final TorqueNEO claw, algaeRollers, coralRollers;
    private final PIDController clawPID;
    private final CANcoder clawEncoder;
    private AlgaeState algaeState = AlgaeState.OFF;
    private CoralState coralState = CoralState.OFF;

    // Angle that we intake from HP is 0 degrees
    public static enum State implements TorqueState {
        STOW(90), 
        L1_SCORE(120), 
        MID_SCORE(160), 
        L4_SCORE(180),
        NET(180),
        ALGAE_EXTRACTION(120),
        PROCESSOR(170),
        CORAL_HP(120),
        ALGAE_GROUND_INTAKE(100);

        private final double angle;

        private State(double angle) {
            this.angle = angle;
        }

        public double getAngle() {
            return angle;
        }

    }

    public static enum AlgaeState implements TorqueState {
        INTAKE(-4), SHOOT(4), OFF(0);

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
        claw = new TorqueNEO(Ports.CLAW);
        clawEncoder = new CANcoder(Ports.CLAW_ENCODER);
        clawPID = new PIDController(1, 0, 0);

        algaeRollers = new TorqueNEO(Ports.ROLLERS_ALGAE);
        coralRollers = new TorqueNEO(Ports.ROLLERS_CORAL);
    }

    @Override
    public final void initialize(final TorqueMode mode) {}

    @Override
    public final void update(final TorqueMode mode) {
        Debug.log("Claw Position", getClawAngle());

        // claw.setVolts(clawPID.calculate(getClawAngle(), desiredState.getAngle()));
        // algaeRollers.setVolts(algaeState.getVolts());
        // coralRollers.setVolts(coralState.getVolts());
    }

	@Override
    public final void clean(final TorqueMode mode) {
        if (mode.isTeleop()) {
            algaeState = AlgaeState.OFF;
            coralState = CoralState.OFF;
            desiredState = State.STOW;
        }
    }

    public final boolean isAtState() {
        return TorqueMath.toleranced(getClawAngle(), desiredState.getAngle());
    }

    public final double getClawAngle() {
        return clawEncoder.getPosition().getValueAsDouble();
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
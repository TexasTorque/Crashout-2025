package org.texastorque.subsystems;

import org.texastorque.Input;
import org.texastorque.Ports;
import org.texastorque.Subsystems;
import org.texastorque.torquelib.base.TorqueMode;
import org.texastorque.torquelib.base.TorqueState;
import org.texastorque.torquelib.base.TorqueStatorSubsystem;
import org.texastorque.torquelib.motors.TorqueNEO;
import org.texastorque.torquelib.util.TorqueMath;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public final class Claw extends TorqueStatorSubsystem<Claw.State> implements Subsystems {

    private static volatile Claw instance;
    private final TorqueNEO claw;
    private final PIDController clawPID;
    private final CANcoder clawEncoder;
    private final boolean score;

    public static enum State implements TorqueState {
        ZERO(0), STOW(90), SCORE_LOW(120), SCORE_HIGH(160);

        private final double angle;

        private State(double angle) {
            this.angle = angle;
        }

        public double getAngle() {
            return angle;
        }

    }

    private Claw() {
        super(State.ZERO);
        claw = new TorqueNEO(Ports.CLAW);
        clawEncoder = new CANcoder(Ports.CLAW_ENCODER);
        clawPID = new PIDController(1, 0, 0);
        score = false;
    }

    @Override
    public final void initialize(final TorqueMode mode) {}

    @Override
    public final void update(final TorqueMode mode) {
        if(score){
            claw.setVolts(clawPID.calculate(getClawAngle(), desiredState.getAngle()));
        }
    }

    public final boolean isAtState(){
        return TorqueMath.toleranced(getClawAngle(), desiredState.getAngle());
    }

    public final double getClawAngle() {
        return clawEncoder.getPosition().getValueAsDouble();
    }

    @Override
    public final void clean(final TorqueMode mode) {
        desiredState = State.STOW;
    }

    public static final synchronized Claw getInstance() {
        return instance == null ? instance = new Claw() : instance;
    }
}
package org.texastorque.subsystems;

import org.texastorque.Ports;
import org.texastorque.Subsystems;
import org.texastorque.torquelib.Debug;
import org.texastorque.torquelib.base.TorqueMode;
import org.texastorque.torquelib.base.TorqueState;
import org.texastorque.torquelib.base.TorqueStatorSubsystem;
import org.texastorque.torquelib.motors.TorqueNEO;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj.RobotBase;

public final class Climb extends TorqueStatorSubsystem<Climb.State> implements Subsystems {

    private static volatile Climb instance;
    private final TorqueNEO climb;

    public static enum State implements TorqueState {
        OUT(4),
        IN(-4),
        OFF(0);

        private final double volts;

        private State(double volts) {
            this.volts = volts;
        }

        public double getVolts() {
            return volts;
        }
    }

    private Climb() {
        super(State.OFF);

        climb = new TorqueNEO(Ports.CLIMB)
                .idleMode(IdleMode.kBrake)
                .apply();
    }

    @Override
    public final void initialize(final TorqueMode mode) {}

    @Override
    public final void update(final TorqueMode mode) {
        Debug.log("Climb State", desiredState.toString());
        Debug.log("Climb Position", getClimbPosition());

        if (claw.getState() != Claw.State.CLIMB) return;
        
        climb.setVolts(desiredState.volts);
    }

    @Override
    public final void clean(final TorqueMode mode) {
        desiredState = State.OFF;
    }

    public double getClimbPosition() {
        if (RobotBase.isSimulation()) {
            return desiredState.volts;
        }

        return climb.getPosition();
    }

    public static final synchronized Climb getInstance() {
        return instance == null ? instance = new Climb() : instance;
    }
}
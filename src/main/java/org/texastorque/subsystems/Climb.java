package org.texastorque.subsystems;

import org.texastorque.Ports;
import org.texastorque.Subsystems;
import org.texastorque.torquelib.Debug;
import org.texastorque.torquelib.base.TorqueMode;
import org.texastorque.torquelib.base.TorqueState;
import org.texastorque.torquelib.base.TorqueStatorSubsystem;
import org.texastorque.torquelib.motors.TorqueNEO;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;

public final class Climb extends TorqueStatorSubsystem<Climb.State> implements Subsystems {

    private static volatile Climb instance;
    private final TorqueNEO climb;
    private final PIDController climbPID;
    public State pastState;
    private double pastStateTime;

    public static enum State implements TorqueState {
        OUT(278.7414),
        IN(129.3085),
        STOWED(0);

        private final double position;

        private State(double position) {
            this.position = position;
        }

        public double getPosition() {
            return position;
        }
    }

    private Climb() {
        super(State.STOWED);
        pastState = State.STOWED;
        pastStateTime = Timer.getFPGATimestamp();

        climb = new TorqueNEO(Ports.CLIMB)
                .idleMode(IdleMode.kBrake)
                .apply();

        climbPID = new PIDController(1, 0, 0);
    }

    @Override
    public final void initialize(final TorqueMode mode) {}

    @Override
    public final void update(final TorqueMode mode) {
        Debug.log("Climb State", desiredState.toString());
        Debug.log("Climb Position", getClimbPosition());

        if (claw.getState() != Claw.State.CLIMB) return;
        
        final double MAX_VOLTS = 4;
        double volts = climbPID.calculate(getClimbPosition(), desiredState.position);
        if (Math.abs(volts) > MAX_VOLTS) volts = Math.signum(volts) * MAX_VOLTS;
        climb.setVolts(volts);
    }

    @Override
    public final void clean(final TorqueMode mode) {}

    @Override
    public void onStateChange(final State lastState) {
        pastStateTime = Timer.getFPGATimestamp();
        pastState = lastState;
    }

    public double getClimbPosition() {
        if (RobotBase.isSimulation()) {
            return desiredState.position;
        }

        return climb.getPosition();
    }

    public static final synchronized Climb getInstance() {
        return instance == null ? instance = new Climb() : instance;
    }
}
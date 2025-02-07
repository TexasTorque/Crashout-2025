package org.texastorque.subsystems;

import org.texastorque.Input;
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

public final class Elevator extends TorqueStatorSubsystem<Elevator.State> implements Subsystems {

    private static volatile Elevator instance;
    private final TorqueNEO elevatorLeft, elevatorRight;
    public final PIDController elevatorPID;
    public final CANcoder elevatorEncoder;
    private double debugVolts;

    public static enum State implements TorqueState {
        STOW(3.4895), //
        SCORE_L1(4.5374), //
        SCORE_L2(1.4817), //
        SCORE_L3(4.2421), //
        SCORE_L4(10.043), //
        NET(10.7412), //
        ALGAE_REMOVAL_LOW(6.7678), //
        ALGAE_REMOVAL_HIGH(9.5281), //
        PROCESSOR(2.938), //
        CORAL_HP(3.5115), //
        BABYBIRD(3.645), //
        DEBUG(0);

        public final double position;

        private State(double position) {
            this.position = position;
        }
    }

    private Elevator() {
        super(State.STOW);
        elevatorLeft = new TorqueNEO(Ports.ELEVATOR_LEFT)
            .inverted(true)
            .idleMode(IdleMode.kBrake)
            .apply();
        elevatorRight = new TorqueNEO(Ports.ELEVATOR_RIGHT)
            .inverted(true)
            .idleMode(IdleMode.kBrake)
            .apply();
        elevatorPID = new PIDController(25, 0, 0);
        elevatorEncoder = new CANcoder(Ports.ELEVATOR_ENCODER);
        debugVolts = 0;
    }

    @Override
    public final void initialize(final TorqueMode mode) {}

    @Override
    public final void update(final TorqueMode mode) {
        Debug.log("Elevator Position", getElevatorPosition());
        Debug.log("Elevator State", desiredState.toString());

        double volts = -elevatorPID.calculate(getElevatorPosition(), desiredState.position);
        if (Math.abs(volts) > 5) volts = Math.signum(volts) * 5;

        if (desiredState.position < 3.5) { //if less than 3.5
            if (claw.isAtState()) { // claw moves first
                elevatorLeft.setVolts(-volts);
                elevatorRight.setVolts(-volts);
            }
        } else {
            elevatorLeft.setVolts(-volts);
            elevatorRight.setVolts(-volts);
        }

        if (desiredState == State.DEBUG) {
            elevatorLeft.setVolts(debugVolts);
            elevatorRight.setVolts(debugVolts);
        }
    }

    @Override
    public final void clean(final TorqueMode mode) {
        debugVolts = 0;
    }

    public void setDebugVolts(final double volts) {
        this.debugVolts = volts;
    }

    public final boolean isAtState() {
        return TorqueMath.toleranced(getElevatorPosition(), desiredState.position, .25);
    }

    public final double getElevatorPosition() {
        return elevatorEncoder.getPosition().getValueAsDouble();
    }

    public static final synchronized Elevator getInstance() {
        return instance == null ? instance = new Elevator() : instance;
    }
}
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

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public final class Elevator extends TorqueStatorSubsystem<Elevator.State> implements Subsystems {

    private static volatile Elevator instance;
    private final TorqueNEO elevatorLeft, elevatorRight;
    public final PIDController elevatorPID;
    public final CANcoder elevatorEncoder;
    private double debugVolts;

    public static enum State implements TorqueState {
        SCORE_L1(2000),
        STOW(3000),
        SCORE_L2(4000),
        SCORE_L3(5000), 
        SCORE_L4(6000),
        NET(7000),
        ALGAE_REMOVAL_LOW(4000),
        ALGAE_REMOVAL_HIGH(5000),
        PROCESSOR(0),
        ALGAE_GROUND_INTAKE(0),
        CORAL_HP(1000),
        DEBUG(0);

        public final double position;

        private State(double position) {
            this.position = position;
        }
    }

    private Elevator() {
        super(State.STOW);
        elevatorLeft = new TorqueNEO(Ports.ELEVATOR_LEFT);
        elevatorRight = new TorqueNEO(Ports.ELEVATOR_RIGHT);
        elevatorPID = new PIDController(0.005, 0, 0);
        elevatorEncoder = new CANcoder(Ports.ELEVATOR_ENCODER);
        debugVolts = 0;
    }

    @Override
    public final void initialize(final TorqueMode mode) {}

    @Override
    public final void update(final TorqueMode mode) {
        Debug.log("Elevator Position", getElevatorPosition());
        Debug.log("Elevator State", desiredState.toString());

        // elevatorLeft.setVolts(elevatorPID.calculate(getElevatorPosition(), desiredState.position));
        // elevatorRight.setVolts(elevatorPID.calculate(getElevatorPosition(), desiredState.position));

        if (desiredState == State.DEBUG) {
            elevatorLeft.setVolts(debugVolts);
            elevatorRight.setVolts(debugVolts);
        }
    }

    @Override
    public final void clean(final TorqueMode mode) {
        debugVolts = 0;
        if (mode.isTeleop()) {
            desiredState = State.STOW;
        }
    }

    public void setDebugVolts(final double volts) {
        this.debugVolts = volts;
    }

    public final boolean isAtState() {
        return TorqueMath.toleranced(getElevatorPosition(), desiredState.position, 5);
    }

    public final double getElevatorPosition() {
        return elevatorEncoder.getPosition().getValueAsDouble();
    }

    public static final synchronized Elevator getInstance() {
        return instance == null ? instance = new Elevator() : instance;
    }
}
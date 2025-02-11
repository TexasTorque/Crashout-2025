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
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;

public final class Elevator extends TorqueStatorSubsystem<Elevator.State> implements Subsystems {

    private static volatile Elevator instance;
    private final TorqueNEO elevatorLeft, elevatorRight;
    private final PIDController elevatorPID;
    private final CANcoder elevatorEncoder;
    private double debugVolts;
    private State pastState;
    private double pastStateTime;

    public static enum State implements TorqueState {
        ZERO(0),
        STOW(3.4895),
        SCORE_L1(4.5374),
        SCORE_L2(2.0),
        SCORE_L3(5.0),
        SCORE_L4(10.043),
        NET(10.043),
        ALGAE_REMOVAL_LOW(6.7678),
        ALGAE_REMOVAL_HIGH(9.5281),
        PROCESSOR(2.938),
        CORAL_HP(3.5),
        DEBUG(0); // Doesn't use the position

        public final double position;

        private State(double position) {
            this.position = position;
        }
    }

    private Elevator() {
        super(State.ZERO);
        pastState = State.ZERO;
        pastStateTime = Timer.getFPGATimestamp();

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
        Debug.log("Elevator At State", isAtState());

        double volts = elevatorPID.calculate(getElevatorPosition(), desiredState.position);
        if (Math.abs(volts) > 4) volts = Math.signum(volts) * 4;

        elevatorLeft.setVolts(volts);
        elevatorRight.setVolts(volts);

        if (desiredState == State.ZERO) {
            elevatorLeft.setVolts(0);
            elevatorRight.setVolts(0);
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

    @Override
    public void onStateChange(final State lastState) {
        pastStateTime = Timer.getFPGATimestamp();
        pastState = lastState;
    }

    public final double getElevatorPosition() {
        if (RobotBase.isSimulation()) {
            final double timeToAnimate = 2;
            final double animationMultiplier = (Timer.getFPGATimestamp() - pastStateTime) / timeToAnimate;

            if (animationMultiplier >= 1) return desiredState.position;
            return ((desiredState.position - pastState.position) * animationMultiplier) + pastState.position;
        }
        return elevatorEncoder.getPosition().getValueAsDouble();
    }

    public final boolean isAtState() {
        return TorqueMath.toleranced(getElevatorPosition(), desiredState.position, .25);
    }

    public void setDebugVolts(final double volts) {
        this.debugVolts = volts;
    }

    public static final synchronized Elevator getInstance() {
        return instance == null ? instance = new Elevator() : instance;
    }
}
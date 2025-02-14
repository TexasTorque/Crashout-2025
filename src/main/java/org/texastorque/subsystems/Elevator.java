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

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;

public final class Elevator extends TorqueStatorSubsystem<Elevator.State> implements Subsystems {

    private static volatile Elevator instance;
    private final TorqueNEO elevatorLeft, elevatorRight;
    private final ProfiledPIDController elevatorPID;
    private final CANcoder elevatorEncoder;
    private double debugVolts;
    public State pastState;
    private double pastStateTime;

    private TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(10, 10);
    private final double ELEVATOR_FF = .35;

    public static enum State implements TorqueState {
        ZERO(0), // Not actually a setpoint!! Gets set to whatever we
                          // deployed at so the elevator doesn't try to go to the ZERO position @ startup
        LOW_STOW(1),
        STOW(3.4895),
        SCORE_L1(4.5374),
        SCORE_L2(1.2),
        SCORE_L3(4.1),
        SCORE_L4(10.0),
        NET(10.2),
        ALGAE_REMOVAL_LOW(6.7678),
        ALGAE_REMOVAL_HIGH(9.5281),
        PROCESSOR(3.5),
        CORAL_HP(2.7),
        DEBUG(0); // Doesn't use the position

        public double position;

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
        
        elevatorPID = new ProfiledPIDController(25, 0, 0, constraints);
        elevatorEncoder = new CANcoder(Ports.ELEVATOR_ENCODER);
        debugVolts = 0;

        State.ZERO.position = getElevatorPosition();
    }

    @Override
    public final void initialize(final TorqueMode mode) {}

    @Override
    public final void update(final TorqueMode mode) {
        Debug.log("Elevator Position", getElevatorPosition());
        Debug.log("Elevator State", desiredState.toString());
        Debug.log("Elevator At State", isAtState());

        final double ELEVATOR_MAX_VOLTS = 8;
        double volts = elevatorPID.calculate(getElevatorPosition(), desiredState.position);
        if (Math.abs(volts) > ELEVATOR_MAX_VOLTS) volts = Math.signum(volts) * ELEVATOR_MAX_VOLTS;

        // If we are moving down
        if (desiredState.position < pastState.position) {
            // Wait until claw moves first
            if (claw.isAtState()) {
                elevatorLeft.setVolts(volts + ELEVATOR_FF);
                elevatorRight.setVolts(volts + ELEVATOR_FF);
            }
        } else {
            // Otherwise, move elevator first
            elevatorLeft.setVolts(volts + ELEVATOR_FF);
            elevatorRight.setVolts(volts + ELEVATOR_FF);
        }

        if (desiredState == State.ZERO) {
            elevatorLeft.setVolts(ELEVATOR_FF);
            elevatorRight.setVolts(ELEVATOR_FF);
        }

        if (desiredState == State.DEBUG) {
            elevatorLeft.setVolts(debugVolts + ELEVATOR_FF);
            elevatorRight.setVolts(debugVolts + ELEVATOR_FF);
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

            if (desiredState.position < pastState.position) {
                if (claw.isAtState()) {
                    if (animationMultiplier >= 1) return desiredState.position;
                    return ((desiredState.position - pastState.position) * animationMultiplier) + pastState.position;
                }
            } else {
                if (animationMultiplier >= 1) return desiredState.position;
                return ((desiredState.position - pastState.position) * animationMultiplier) + pastState.position;
            }

            pastStateTime = Timer.getFPGATimestamp();
            return pastState.position;
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
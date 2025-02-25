package org.texastorque.subsystems;

import org.texastorque.Ports;
import org.texastorque.Subsystems;
import org.texastorque.torquelib.Debug;
import org.texastorque.torquelib.base.TorqueMode;
import org.texastorque.torquelib.base.TorqueState;
import org.texastorque.torquelib.base.TorqueStatorSubsystem;
import org.texastorque.torquelib.motors.TorqueNEO;
import org.texastorque.torquelib.util.TorqueMath;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;

public final class Elevator extends TorqueStatorSubsystem<Elevator.State> implements Subsystems {

    private static volatile Elevator instance;
    private final TorqueNEO elevatorLeft, elevatorRight;
    private final ProfiledPIDController elevatorPID;
    public State pastState;
    private double pastStateTime;
    private final double ELEVATOR_FF = .1;

    public final double MAX_HEIGHT = 228;
    public final double SAFE_HEIGHT = 60;
    public static enum State implements TorqueState {
        ZERO(0), // Not actually a setpoint!! Gets set to whatever we
                          // deployed at so the elevator doesn't try to go to the ZERO position @ startup
        MANUAL(0), // Also not a setpoint!! Gets set to whatever we manually control it to
        STOW(61.2034),
        SCORE_L1(61.2034),
        SCORE_L2(39.952),
        SCORE_L3(87.3845),
        SCORE_L4(222.7442),
        NET(204.144),
        ALGAE_REMOVAL_LOW(134.3316),
        ALGAE_REMOVAL_HIGH(185.2893),
        PROCESSOR(82.4912),
        CORAL_HP(72.7641),
        CLIMB(61.2034);

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
            .idleMode(IdleMode.kBrake)
            .apply();
        
        elevatorRight = new TorqueNEO(Ports.ELEVATOR_RIGHT)
            .idleMode(IdleMode.kBrake)
            .apply();
        
        elevatorPID = new ProfiledPIDController(2, 0, 0,
                new TrapezoidProfile.Constraints(300, 80));
        
        elevatorPID.reset(getElevatorPosition());
    }

    @Override
    public final void initialize(final TorqueMode mode) {
        State.ZERO.position = getElevatorPosition();
        elevatorPID.reset(getElevatorPosition());
    }

    @Override
    public final void update(final TorqueMode mode) {
        Debug.log("Elevator Position", getElevatorPosition());
        Debug.log("Elevator State", desiredState.toString());
        Debug.log("Elevator Setpoint", desiredState.position);
        Debug.log("Elevator At State", isAtState());
        Debug.log("Manual Position", Elevator.State.MANUAL.position);

        final double ELEVATOR_MAX_VOLTS = 12;
        double volts = elevatorPID.calculate(getElevatorPosition(), desiredState.position);
        if (Math.abs(volts) > ELEVATOR_MAX_VOLTS) volts = Math.signum(volts) * ELEVATOR_MAX_VOLTS;

        if (desiredState.position > SAFE_HEIGHT && getElevatorPosition() > SAFE_HEIGHT) {
            elevatorLeft.setVolts(volts + ELEVATOR_FF);
            elevatorRight.setVolts(volts + ELEVATOR_FF);
        } else if (getElevatorPosition() > desiredState.position) {
            if (claw.isAtState()) {
                elevatorLeft.setVolts(volts + ELEVATOR_FF);
                elevatorRight.setVolts(volts + ELEVATOR_FF);
            } else {
                elevatorLeft.setVolts(ELEVATOR_FF);
                elevatorRight.setVolts(ELEVATOR_FF);
            }
        } else if (getElevatorPosition() < desiredState.position) {
            elevatorLeft.setVolts(volts + ELEVATOR_FF);
            elevatorRight.setVolts(volts + ELEVATOR_FF);
        }

        if (desiredState == State.ZERO) {
            elevatorLeft.setVolts(0);
            elevatorRight.setVolts(0);
        }
    }

    @Override
    public final void clean(final TorqueMode mode) {}

    @Override
    public void onStateChange(final State lastState) {
        pastStateTime = Timer.getFPGATimestamp();
        pastState = lastState;
    }

    public final double getElevatorPosition() {
        if (RobotBase.isSimulation()) {
            final double timeToAnimate = Math.abs(desiredState.position - pastState.position) / 114;
            final double animationMultiplier = (Timer.getFPGATimestamp() - pastStateTime) / timeToAnimate;
            final double position = ((desiredState.position - pastState.position) * animationMultiplier) + pastState.position;

            if (desiredState.position > SAFE_HEIGHT && position > SAFE_HEIGHT) {
                if (animationMultiplier >= 1) return desiredState.position;
                return position;
            } else if (position > desiredState.position) {
                if (claw.isAtState()) {
                    if (animationMultiplier >= 1) return desiredState.position;
                    return position;
                }
            } else if (position < desiredState.position) {
                if (animationMultiplier >= 1) return desiredState.position;
                return position;
            }

            pastStateTime = Timer.getFPGATimestamp();
            return pastState.position;
        }
        return (elevatorLeft.getPosition() + elevatorRight.getPosition()) / 2;
    }

    public final boolean isAtState() {
        return TorqueMath.toleranced(getElevatorPosition(), desiredState.position, .25);
    }

    public final boolean isNearState() {
        return TorqueMath.toleranced(getElevatorPosition(), desiredState.position, 2);
    }

    public static final synchronized Elevator getInstance() {
        return instance == null ? instance = new Elevator() : instance;
    }
}
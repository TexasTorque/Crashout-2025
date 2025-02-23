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

    // MAX ELEVATOR: 228
    public static enum State implements TorqueState {
        ZERO(40), // Not actually a setpoint!! Gets set to whatever we
                          // deployed at so the elevator doesn't try to go to the ZERO position @ startup
        MANUAL(0), // Also not a setpoint!! Gets set to whatever we manually control it to
        LOW_STOW(20),
        STOW(40),
        SCORE_L1(150),
        SCORE_L2(0),
        SCORE_L3(0),
        SCORE_L4(0),
        NET(0),
        ALGAE_REMOVAL_LOW(0),
        ALGAE_REMOVAL_HIGH(0),
        PROCESSOR(0),
        CORAL_HP(0);

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

        final double ELEVATOR_MAX_VOLTS = 10;
        double volts = elevatorPID.calculate(getElevatorPosition(), desiredState.position);
        if (Math.abs(volts) > ELEVATOR_MAX_VOLTS) volts = Math.signum(volts) * ELEVATOR_MAX_VOLTS;

        Debug.log("Elevator Volts", volts);

        // if ((desiredState.position > 5 && getElevatorPosition() > 3) || getElevatorPosition() > 5) {
        //     // If we are moving up and high enough or very high enough, move at the same time as claw but not if going to L4 or at L4
        //     elevatorLeft.setVolts(volts + ELEVATOR_FF);
        //     elevatorRight.setVolts(volts + ELEVATOR_FF);
        // } else if (desiredState.position < pastState.position) {
        //     // If we are moving down wait until claw moves first
        //     if (claw.isAtState() || (mode.isAuto() && claw.isNearState())) {
        //         elevatorLeft.setVolts(volts + ELEVATOR_FF);
        //         elevatorRight.setVolts(volts + ELEVATOR_FF);
        //     }
        // } else {
        //     // Otherwise, move elevator first
        //     elevatorLeft.setVolts(volts + ELEVATOR_FF);
        //     elevatorRight.setVolts(volts + ELEVATOR_FF);
        // }

        elevatorLeft.setVolts(volts + .1);
        elevatorRight.setVolts(volts + .1);

        if (desiredState == State.ZERO) {
            elevatorLeft.setVolts(0);
            elevatorRight.setVolts(0);
        }
    }

    @Override
    public final void clean(final TorqueMode mode) {
        
    }

    @Override
    public void onStateChange(final State lastState) {
        pastStateTime = Timer.getFPGATimestamp();
        pastState = lastState;
    }

    public final double getElevatorPosition() {
        if (RobotBase.isSimulation()) {
            final double timeToAnimate = Math.abs(desiredState.position - pastState.position) / 4;
            final double animationMultiplier = (Timer.getFPGATimestamp() - pastStateTime) / timeToAnimate;
            final double position = ((desiredState.position - pastState.position) * animationMultiplier) + pastState.position;

            if (desiredState.position > 5 && position > 3) {
                // If we are moving up and high enough, move at the same time as claw
                if (animationMultiplier >= 1) return desiredState.position;
                return position;
            } else if (desiredState.position < pastState.position) {
                if (claw.isNearState()) {
                    if (animationMultiplier >= 1) return desiredState.position;
                    return position;
                }
            } else {
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
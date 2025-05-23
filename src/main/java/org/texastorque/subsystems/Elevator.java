/**
 * Copyright 2025 Texas Torque.
 *
 * This file is part of Bravo/Charlie/Crashout-2025, which is not licensed for distribution.
 * For more details, see ./license.txt or write <davey.adams.three@gmail.com>.
 */
package org.texastorque.subsystems;

import org.texastorque.Ports;
import org.texastorque.Subsystems;
import org.texastorque.torquelib.Debug;
import org.texastorque.torquelib.base.TorqueMode;
import org.texastorque.torquelib.base.TorqueState;
import org.texastorque.torquelib.base.TorqueStatorSubsystem;
import org.texastorque.torquelib.motors.TorqueKraken;
import org.texastorque.torquelib.util.TorqueMath;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;

public final class Elevator extends TorqueStatorSubsystem<Elevator.State> implements Subsystems {

    private static volatile Elevator instance;
    private final TorqueKraken elevatorLeft, elevatorRight;
    private final ProfiledPIDController elevatorPID;
    private final double ELEVATOR_FF = .35;
    public final double MAX_HEIGHT = 40;
    private State selectedState;
    public State pastState;
    private double pastStateTime;

    public static enum State implements TorqueState {
        ZERO(0), // Not actually a setpoint!! Gets set to whatever we
                          // deployed at so the elevator doesn't try to go to the ZERO position @ startup
        MANUAL(0), // Also not a setpoint!! Gets set to whatever we manually control it to
        STOW(5),
        SCORE_L1(1.044),
        SCORE_L2(0.8044),
        SCORE_L3(4.8713),
        SCORE_L4(39.9693),
        NET(36.7693),
        ALGAE_REMOVAL_LOW(5.2571),
        ALGAE_REMOVAL_HIGH(20.7866),
        ALGAE_GROUND(5),
        PROCESSOR(1.6765),
        LOLLIPOP(2.5144),
        REGRESSION_CORAL_HP(0.8832), // It's a half state, used when not in the HP zone, but when in the zone it uses regression
        CORAL_HP(7),
        CLIMB(0.8044);

        public double position;

        private State(double position) {
            this.position = position;
        }
    }

    private Elevator() {
        super(State.ZERO);
        pastState = State.ZERO;
        pastStateTime = Timer.getFPGATimestamp();

        selectedState = State.ZERO;

        elevatorLeft = new TorqueKraken(Ports.ELEVATOR_LEFT)
            .idleMode(NeutralModeValue.Brake)
            .inverted(true)
            .apply();
        
        elevatorRight = new TorqueKraken(Ports.ELEVATOR_RIGHT)
            .idleMode(NeutralModeValue.Brake)
            .inverted(true)
            .apply();
        
        elevatorPID = new ProfiledPIDController(2, 0, 0,
                new TrapezoidProfile.Constraints(400, 200));
        
        elevatorPID.reset(getElevatorPosition());
    }

    @Override
    public final void initialize(final TorqueMode mode) {
        State.ZERO.position = getElevatorPosition();
        setState(State.ZERO);
        elevatorPID.reset(getElevatorPosition());
    }

    @Override
    public final void update(final TorqueMode mode) {
        final double ELEVATOR_MAX_VOLTS = 10;
        double desiredPosition = desiredState.position;
        if (desiredState == State.REGRESSION_CORAL_HP && perception.inCoralStationZone()) {
            desiredPosition = getCoralStationHeight();
        }
        double volts = elevatorPID.calculate(getElevatorPosition(), desiredPosition);
        if (Math.abs(volts) > ELEVATOR_MAX_VOLTS) volts = Math.signum(volts) * ELEVATOR_MAX_VOLTS;

        if (mode.isDisabled()) {
            State.ZERO.position = getElevatorPosition();
            desiredState = State.ZERO;
            elevatorPID.reset(getElevatorPosition());
            elevatorLeft.setVolts(ELEVATOR_FF);
            elevatorRight.setVolts(ELEVATOR_FF);
        } else if (desiredState == State.ZERO) {
            elevatorLeft.setVolts(ELEVATOR_FF);
            elevatorRight.setVolts(ELEVATOR_FF);
            elevatorPID.reset(getElevatorPosition());
        } else {
            elevatorLeft.setVolts(volts + ELEVATOR_FF);
            elevatorRight.setVolts(volts + ELEVATOR_FF);
        }

        Debug.log("Elevator Volts", volts + ELEVATOR_FF);
        Debug.log("Elevator Position", getElevatorPosition());
        Debug.log("Elevator State", desiredState.toString());
        Debug.log("Elevator Setpoint", desiredState.position);
        Debug.log("Elevator At State", isAtState());
        Debug.log("Manual Position", Elevator.State.MANUAL.position);
        Debug.log("Elevator Left Position", elevatorLeft.getPosition());
        Debug.log("Elevator Right Position", elevatorRight.getPosition());
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
            double desiredPosition = desiredState.position;
            if (desiredState == State.REGRESSION_CORAL_HP && perception.inCoralStationZone()) {
                desiredPosition = getCoralStationHeight();
            }
            final double timeToAnimate = Math.abs(desiredPosition - pastState.position) / 114;
            final double animationMultiplier = (Timer.getFPGATimestamp() - pastStateTime) / timeToAnimate;
            double position = ((desiredPosition - pastState.position) * animationMultiplier) + pastState.position;
            if (animationMultiplier > 1) position = desiredPosition;

            if (desiredState != State.ZERO) {
                return position;
            }

            pastStateTime = Timer.getFPGATimestamp();
            return pastState.position;
        }
        return (elevatorLeft.getPosition() + elevatorRight.getPosition()) / 2;
    }

    public double getCoralStationHeight() {
        double height = -0.15 * perception.getHPDistance() + 10.9;
        if (height > 7) height = 7;
        if (height < 0.8832) height = 0.8832;
        return height;
    }

    public final boolean isAtState() {
        return TorqueMath.toleranced(getElevatorPosition(), desiredState.position, .3);
    }

    public final boolean isNearState() {
        return TorqueMath.toleranced(getElevatorPosition(), desiredState.position, 4);
    }

    public void setSelectedState(State selectedState) {
      	this.selectedState = selectedState;
    }

    public State getSelectedState() {
      	return selectedState;
    }

    public static final synchronized Elevator getInstance() {
        return instance == null ? instance = new Elevator() : instance;
    }
}

package org.texastorque.subsystems;

import org.texastorque.Ports;
import org.texastorque.Subsystems;
import org.texastorque.torquelib.base.TorqueMode;
import org.texastorque.torquelib.base.TorqueState;
import org.texastorque.torquelib.base.TorqueStatorSubsystem;
import org.texastorque.torquelib.motors.TorqueNEO;
import org.texastorque.torquelib.util.TorqueMath;

import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


import org.texastorque.torquelib.auto.TorqueCommand;
import org.texastorque.torquelib.auto.TorqueSequence;
import org.texastorque.torquelib.auto.commands.TorqueContinuous;
import org.texastorque.torquelib.auto.commands.TorqueRun;
import org.texastorque.torquelib.auto.commands.TorqueWaitUntil;


public final class Elevator extends TorqueStatorSubsystem<Elevator.State> implements Subsystems {

    private static volatile Elevator instance;
    private final TorqueNEO elevator;
    public final PIDController elevatorPID;
    public final CANcoder elevatorEncoder;
    private Scoring autoScore;
    private boolean score;

    public static enum State implements TorqueState {
        ZERO(0), SCORE_L1(2000), STOW(3000), SCORE_L2(4000), SCORE_L3(5000), 
        SCORE_L4(6000), NET(7000), SCORE(0);

        public final double position;

        private State(double position) {
            this.position = position;
        }
    }

    public static final class Scoring extends TorqueSequence implements Subsystems {
        public Scoring(State elevatorState, Claw.State clawState) {

            addBlock(new TorqueRun(() -> elevator.setState(elevatorState)));
        
            addBlock(new TorqueWaitUntil(() -> elevator.isAtState()));

            addBlock(new TorqueRun(() -> Claw.getInstance().setState(clawState)));

            addBlock(new TorqueWaitUntil(() -> Claw.getInstance().isAtState()));

        }
    }

    private Elevator() {
        super(State.ZERO);
        elevator = new TorqueNEO(Ports.ELEVATOR);
        elevatorPID = new PIDController(0.005, 0, 0);
        elevatorEncoder = new CANcoder(Ports.ELEVATOR_ENCODER);
        autoScore = null;
    }

    @Override
    public final void initialize(final TorqueMode mode) {}

    @Override
    public final void update(final TorqueMode mode) {
        SmartDashboard.putNumber("Elevator Encoder", getElevatorPosition());

        // if(score){
        //     autoScore = new Scoring(desiredState, claw.getState());
        //     autoScore.run();
        // }else{
        //     autoScore = null;
        // }

        if (autoScore != null) {
            autoScore.run();

            if (autoScore.ended) {
                autoScore = null;
            }
        }
    }

    public final boolean isAtState() {
        return TorqueMath.toleranced(getElevatorPosition(), desiredState.position);
    }

    public final double getElevatorPosition() {
        return elevatorEncoder.getPosition().getValueAsDouble();
    }

    public final void startScoreSequence() {
        autoScore = new Scoring(desiredState, claw.getState());
    }

    public final boolean isScoring() {
        return autoScore != null;
    }

    @Override
    public final void clean(final TorqueMode mode) {
        desiredState = State.STOW;
    }

    public static final synchronized Elevator getInstance() {
        return instance == null ? instance = new Elevator() : instance;
    }
}
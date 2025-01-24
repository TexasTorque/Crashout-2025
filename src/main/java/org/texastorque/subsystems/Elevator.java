package org.texastorque.subsystems;

import org.texastorque.Ports;
import org.texastorque.Subsystems;
import org.texastorque.subsystems.Claw.Gamepiece;
import org.texastorque.torquelib.base.TorqueMode;
import org.texastorque.torquelib.base.TorqueState;
import org.texastorque.torquelib.base.TorqueStatorSubsystem;
import org.texastorque.torquelib.motors.TorqueNEO;
import org.texastorque.torquelib.util.TorqueMath;

import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


// import org.texastorque.torquelib.auto.TorqueSequence;
// import org.texastorque.torquelib.auto.commands.TorqueRun;
// import org.texastorque.torquelib.auto.commands.TorqueWaitUntil;


public final class Elevator extends TorqueStatorSubsystem<Elevator.State> implements Subsystems {

    private static volatile Elevator instance;
    private final TorqueNEO elevatorLeft, elevatorRight;
    public final PIDController elevatorPID;
    public final CANcoder elevatorEncoder;
    // private Scoring autoScore;

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
        CORAL_HP(1000);

        public final double position;

        private State(double position) {
            this.position = position;
        }
    }

    // public static final class Scoring extends TorqueSequence implements Subsystems {
    //     public Scoring(final State elevatorState, final Claw.State clawState, final Gamepiece gamepiece) {

    //         addBlock(new TorqueRun(() -> elevator.setState(elevatorState)));
        
    //         addBlock(new TorqueWaitUntil(() -> elevator.isAtState()));

    //         addBlock(new TorqueRun(() -> Claw.getInstance().setState(clawState)));

    //         addBlock(new TorqueWaitUntil(() -> Claw.getInstance().isAtState()));

    //         // More because actually have to score (set claw gamepiece state to intake/outtake)

    //         addBlock(new TorqueRun(() -> elevator.setState(State.STOW)));
    //         addBlock(new TorqueRun(() -> claw.setState(Claw.State.STOW)));
    //     }
    // }

    private Elevator() {
        super(State.STOW);
        elevatorLeft = new TorqueNEO(Ports.ELEVATOR_LEFT);
        elevatorRight = new TorqueNEO(Ports.ELEVATOR_RIGHT);
        elevatorPID = new PIDController(0.005, 0, 0);
        elevatorEncoder = new CANcoder(Ports.ELEVATOR_ENCODER);
        // autoScore = null;
    }

    @Override
    public final void initialize(final TorqueMode mode) {}

    @Override
    public final void update(final TorqueMode mode) {
        SmartDashboard.putNumber("Elevator Encoder", getElevatorPosition());

        // if (autoScore != null) {
        //     autoScore.run();

        //     if (autoScore.ended) {
        //         autoScore = null;
        //         drivebase.setState(Drivebase.State.FIELD_RELATIVE);
        //     }
        // }

        elevatorLeft.setVolts(elevatorPID.calculate(getElevatorPosition(), desiredState.position));
        elevatorRight.setVolts(elevatorPID.calculate(getElevatorPosition(), desiredState.position));
    }

    @Override
    public final void clean(final TorqueMode mode) {
        desiredState = State.STOW;
    }

    public final boolean isAtState() {
        return TorqueMath.toleranced(getElevatorPosition(), desiredState.position, 5);
    }

    public final double getElevatorPosition() {
        return elevatorEncoder.getPosition().getValueAsDouble();
    }

    // public final void startScoreSequence(final Gamepiece gamepiece) {
    //     autoScore = new Scoring(desiredState, claw.getState(), gamepiece);
    // }

    // public final boolean isScoring() {
    //     return autoScore != null;
    // }

    public static final synchronized Elevator getInstance() {
        return instance == null ? instance = new Elevator() : instance;
    }
}
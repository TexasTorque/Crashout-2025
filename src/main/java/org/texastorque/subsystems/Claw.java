package org.texastorque.subsystems;

import org.texastorque.Ports;
import org.texastorque.Subsystems;
import org.texastorque.torquelib.Debug;
import org.texastorque.torquelib.base.TorqueMode;
import org.texastorque.torquelib.base.TorqueState;
import org.texastorque.torquelib.base.TorqueStatorSubsystem;
import org.texastorque.torquelib.control.TorqueCurrentSpike;
import org.texastorque.torquelib.motors.TorqueNEO;
import org.texastorque.torquelib.util.TorqueMath;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;

public final class Claw extends TorqueStatorSubsystem<Claw.State> implements Subsystems {

    private static volatile Claw instance;
    private final TorqueNEO shoulder, algaeRollers, coralRollers;
    private final ProfiledPIDController shoulderPID;
    private final CANcoder shoulderEncoder;
    private AlgaeState algaeState = AlgaeState.OFF;
    private CoralState coralState = CoralState.OFF;
    public final TorqueCurrentSpike coralSpike;
    private State pastState;
    private double pastStateTime;

    public static enum State implements TorqueState {
        ZERO(0),
        STOW(41.2793),
        SCORE_L1(5),
        SCORE_L2(186.6699),
        SCORE_L3(165),
        SCORE_L4(205.0977),
        NET(156.6797),
        ALGAE_EXTRACTION(277.2656),
        PROCESSOR(70.2832),
        CORAL_HP(20),
        CLIMB(285);

        private double angle;

        private State(double angle) {
            this.angle = angle;
        }

        public double getAngle() {
            return angle;
        }
    }

    public static enum AlgaeState implements TorqueState {
        INTAKE(-10), SHOOT(12), SHOOT_SLOW(4), OFF(0);

        private final double volts;

        private AlgaeState(double volts) {
            this.volts = volts;
        }

        public double getVolts() {
            return volts;
        }
    }

    public static enum CoralState implements TorqueState {
        INTAKE(-4), SHOOT(6), OFF(-1); // normal 3.5 volts

        private final double volts;

        private CoralState(double volts) {
            this.volts = volts;
        }

        public double getVolts() {
            return volts;
        }
    }

    private Claw() {
        super(State.ZERO);
        pastState = State.ZERO;
        pastStateTime = Timer.getFPGATimestamp();

        shoulder = new TorqueNEO(Ports.SHOULDER)
            .idleMode(IdleMode.kBrake)
            .inverted(true)
            .apply();

        shoulderEncoder = new CANcoder(Ports.SHOULDER_ENCODER);
        shoulderPID = new ProfiledPIDController(.3, 0, 0,
                new TrapezoidProfile.Constraints(720, 720));

        algaeRollers = new TorqueNEO(Ports.ROLLERS_ALGAE)
            .apply();

        coralRollers = new TorqueNEO(Ports.ROLLERS_CORAL)
            .inverted(true)
            .currentLimit(10)
            .idleMode(IdleMode.kBrake)
            .apply();

        coralSpike = new TorqueCurrentSpike(18);

        shoulderPID.reset(getShoulderAngle());
    }

    @Override
    public final void initialize(final TorqueMode mode) {
        State.ZERO.angle = getShoulderAngle();
        shoulderPID.reset(getShoulderAngle());
    }

    @Override
    public final void update(final TorqueMode mode) {
        Debug.log("Shoulder Position", getShoulderAngle());
        Debug.log("Claw State", desiredState.toString());
        Debug.log("Has Coral", hasCoral());
        Debug.log("Shoulder At State", isAtState());
        Debug.log("Coral Current", coralRollers.getOutputCurrent());
        Debug.log("Coral State", coralState.toString());
        Debug.log("Algae State", algaeState.toString());

        final double SHOULDER_MAX_VOLTS = 12;
        double volts = shoulderPID.calculate(getShoulderAngle(), desiredState.angle);
        final double ff = .1 * Math.sin(Math.toRadians(getShoulderAngle() + 15)); // Will need tuning
        if (Math.abs(volts) > SHOULDER_MAX_VOLTS) volts = Math.signum(volts) * SHOULDER_MAX_VOLTS;

        if (desiredState == State.ZERO) {
            shoulder.setVolts(ff);
        } else {
            shoulder.setVolts(volts + ff);
        }

        algaeRollers.setVolts(algaeState.getVolts());

        if (coralState == CoralState.SHOOT) 
            coralSpike.reset();
        if (coralState == CoralState.INTAKE && desiredState != State.CORAL_HP)
            coralState = CoralState.OFF;
        
        if (hasCoral() && coralState != CoralState.SHOOT) {
            coralState = CoralState.OFF;
            coralRollers.setVolts(coralState.getVolts());
        } else {
            coralRollers.setVolts(coralState.getVolts());
        }

        Debug.log("Coral Volts", coralState.getVolts());
    }

	@Override
    public final void clean(final TorqueMode mode) {
        if (mode.isTeleop()) {
            algaeState = AlgaeState.OFF;
        }
    }

    @Override
    public void onStateChange(final State lastState) {
        pastStateTime = Timer.getFPGATimestamp();
        pastState = lastState;
    }

    public final double getShoulderAngle() {
        if (RobotBase.isSimulation()) {
            final double timeToAnimate = Math.abs(desiredState.angle - pastState.angle) / 180;
            final double animationMultiplier = (Timer.getFPGATimestamp() - pastStateTime) / timeToAnimate;
            double position = ((desiredState.angle - pastState.angle) * animationMultiplier) + pastState.angle;
            if (animationMultiplier > 1) position = desiredState.angle;

            if (desiredState != State.ZERO) {
                return position;
            }

            pastStateTime = Timer.getFPGATimestamp();
            return pastState.angle;
        }
        return shoulderEncoder.getAbsolutePosition().getValueAsDouble() * 360;
    }

    public final boolean isAtState() {
        return TorqueMath.toleranced(getShoulderAngle(), desiredState.getAngle(), 8);
    }

    public final boolean isNearState() {
        return TorqueMath.toleranced(getShoulderAngle(), desiredState.getAngle(), 20);
    }

    public boolean hasCoral() {
        return coralSpike.calculate(coralRollers.getOutputCurrent());
    }

    public void setAlgaeState(AlgaeState algaeState) {
        this.algaeState = algaeState;
    }

    public void setCoralState(CoralState coralState) {
        this.coralState = coralState;
    }

    public static final synchronized Claw getInstance() {
        return instance == null ? instance = new Claw() : instance;
    }
}
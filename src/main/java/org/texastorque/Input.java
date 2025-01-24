package org.texastorque;

import java.lang.annotation.ElementType;

import org.texastorque.AlignPose2d.Relation;
import org.texastorque.subsystems.Claw;
import org.texastorque.subsystems.Drivebase;
import org.texastorque.subsystems.Elevator;
import org.texastorque.subsystems.Claw.AlgaeState;
import org.texastorque.subsystems.Claw.CoralState;
import org.texastorque.subsystems.Claw.Gamepiece;
import org.texastorque.torquelib.base.TorqueInput;
import org.texastorque.torquelib.control.TorqueBoolSupplier;
import org.texastorque.torquelib.control.TorqueToggleSupplier;
import org.texastorque.torquelib.sensors.TorqueController;
import org.texastorque.torquelib.swerve.TorqueSwerveSpeeds;
import org.texastorque.torquelib.util.TorqueMath;


public final class Input extends TorqueInput<TorqueController> implements Subsystems {
    private static volatile Input instance;
    private final double CONTROLLER_DEADBAND = 0.1;
    private final TorqueBoolSupplier resetGyro, debug, slowMode, L1Mode, L2Mode, L3Mode, L4Mode,
            scoreSequence, scoreSequenceNoAlign, gamepieceMode, leftRelation, rightRelation, intake, outtake, net, processor, algaeHigh,
            algaeLow, algaeGroundIntake;

    private Input() {
        driver = new TorqueController(0, CONTROLLER_DEADBAND);
        operator = new TorqueController(1, CONTROLLER_DEADBAND);

        // Driver
        resetGyro = new TorqueBoolSupplier(driver::isRightCenterButtonDown);
        debug = new TorqueBoolSupplier(driver::isLeftCenterButtonDown);
        slowMode = new TorqueBoolSupplier(driver::isRightBumperPressed);

        // Elevator and Claw
        L1Mode = new TorqueBoolSupplier(operator::isAButtonDown);
        L2Mode = new TorqueBoolSupplier(operator::isXButtonDown);
        L3Mode = new TorqueBoolSupplier(operator::isBButtonDown);
        L4Mode = new TorqueBoolSupplier(operator::isYButtonDown);
        net = new TorqueBoolSupplier(operator::isDPADUpDown);
        processor = new TorqueBoolSupplier(driver::isAButtonDown);

        algaeHigh = new TorqueBoolSupplier(operator::isXButtonDown);
        algaeLow = new TorqueBoolSupplier(operator::isYButtonDown);
        gamepieceMode = new TorqueToggleSupplier(operator::isRightCenterButtonPressed); // Make sure this works (true is algae, false is coral)
        algaeGroundIntake = new TorqueBoolSupplier(driver::isBButtonDown);

        // In progress: Auto scoring
        scoreSequence = new TorqueBoolSupplier(operator::isRightBumperDown);
        scoreSequenceNoAlign = new TorqueBoolSupplier(operator::isLeftBumperDown);

        leftRelation = new TorqueBoolSupplier(operator::isDPADLeftDown);
        rightRelation = new TorqueBoolSupplier(operator::isDPADRightDown);

        // Rollers 
        intake = new TorqueBoolSupplier(driver::isLeftTriggerDown);
        outtake = new TorqueBoolSupplier(driver::isRightTriggerDown);
    }

    @Override
    public final void update() {
        updateDrivebase();
        updateElevator();
        updateClaw();
    }

    public final void updateDrivebase() {
        resetGyro.onTrue(() -> perception.resetHeading());
        slowMode.onTrue(drivebase::toggleSlowMode);

        leftRelation.onTrue(() -> drivebase.setRelation(Relation.LEFT));
        rightRelation.onTrue(() -> drivebase.setRelation(Relation.RIGHT));

        final double xVelocity = TorqueMath.scaledLinearDeadband(-driver.getLeftYAxis(), CONTROLLER_DEADBAND)
                * Drivebase.activeMaxVelocity;
        final double yVelocity = TorqueMath.scaledLinearDeadband(-driver.getLeftXAxis(), CONTROLLER_DEADBAND)
                * Drivebase.activeMaxVelocity;
        final double rotationVelocity = TorqueMath.scaledLinearDeadband(-driver.getRightXAxis(), CONTROLLER_DEADBAND)
                * Drivebase.activeMaxAngularVelocity;

        drivebase.setInputSpeeds(new TorqueSwerveSpeeds(xVelocity, yVelocity, rotationVelocity));
    }

    public final void updateElevator() {
        L1Mode.onTrue(() -> elevator.setState(Elevator.State.SCORE_L1));
        L2Mode.onTrue(() -> elevator.setState(Elevator.State.SCORE_L2));
        L3Mode.onTrue(() -> elevator.setState(Elevator.State.SCORE_L3));
        L4Mode.onTrue(() -> elevator.setState(Elevator.State.SCORE_L4));
        net.onTrue(() -> elevator.setState(Elevator.State.NET));
        processor.onTrue(() -> elevator.setState(Elevator.State.PROCESSOR));
        algaeHigh.onTrue(() -> elevator.setState(Elevator.State.ALGAE_REMOVAL_HIGH));
        algaeLow.onTrue(() -> elevator.setState(Elevator.State.ALGAE_REMOVAL_LOW));
        algaeGroundIntake.onTrue(() -> elevator.setState(Elevator.State.ALGAE_GROUND_INTAKE));

        // scoreSequence.onTrue(() -> {
        //     drivebase.setState(Drivebase.State.ALIGN_TO_APRILTAG);
        //     elevator.startScoreSequence(gamepieceMode.get() ? Gamepiece.ALGAE : Gamepiece.CORAL);
        // });
        // scoreSequenceNoAlign.onTrue(() -> elevator.startScoreSequence(getGamepieceMode()));
    }

    public final void updateClaw() {
        L2Mode.onTrue(() -> claw.setState(Claw.State.MID_SCORE));
        L3Mode.onTrue(() -> claw.setState(Claw.State.MID_SCORE));
        L4Mode.onTrue(() -> claw.setState(Claw.State.L4_SCORE));
        net.onTrue(() -> claw.setState(Claw.State.NET));
        processor.onTrue(() -> claw.setState(Claw.State.PROCESSOR));
        algaeHigh.onTrue(() -> claw.setState(Claw.State.ALGAE_EXTRACTION));
        algaeLow.onTrue(() -> claw.setState(Claw.State.ALGAE_EXTRACTION));
        algaeGroundIntake.onTrue(() -> claw.setState(Claw.State.ALGAE_GROUND_INTAKE));

        intake.onTrue(() -> {
            if (getGamepieceMode() == Gamepiece.ALGAE) {
                claw.setAlgaeState(AlgaeState.INTAKE);
            } else if (getGamepieceMode() == Gamepiece.CORAL) {
                claw.setCoralState(CoralState.INTAKE);
            }
        });
        outtake.onTrue(() -> {
            if (getGamepieceMode() == Gamepiece.ALGAE) {
                claw.setAlgaeState(AlgaeState.SHOOT);
            } else if (getGamepieceMode() == Gamepiece.CORAL) {
                claw.setCoralState(CoralState.SHOOT);
            }
        });
    }

    public Gamepiece getGamepieceMode() {
        return gamepieceMode.get() ? Gamepiece.ALGAE : Gamepiece.CORAL;
    }

    public boolean isDebugMode() {
        return debug.get();
    }

    public static final synchronized Input getInstance() {
        return instance == null ? instance = new Input() : instance;
    }
}

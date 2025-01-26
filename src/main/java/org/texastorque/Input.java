package org.texastorque;

import org.texastorque.AlignPose2d.Relation;
import org.texastorque.subsystems.Claw;
import org.texastorque.subsystems.Drivebase;
import org.texastorque.subsystems.Elevator;
import org.texastorque.torquelib.base.TorqueInput;
import org.texastorque.torquelib.control.TorqueBoolSupplier;
import org.texastorque.torquelib.sensors.TorqueController;
import org.texastorque.torquelib.swerve.TorqueSwerveSpeeds;
import org.texastorque.torquelib.util.TorqueMath;


public final class Input extends TorqueInput<TorqueController> implements Subsystems {
    private static volatile Input instance;
    private final double CONTROLLER_DEADBAND = 0.1;
    private final TorqueBoolSupplier resetGyro, debug, L1Mode, L2Mode, L3Mode, L4Mode, leftRelation,
            rightRelation, intakeCoral, intakeAlgae, outtakeCoral, outtakeAlgae, net, processor,
            algaeHigh, algaeLow, algaeGroundIntake, coralStation;

    private Input() {
        driver = new TorqueController(0, CONTROLLER_DEADBAND);
        operator = new TorqueController(1, CONTROLLER_DEADBAND);

        resetGyro = new TorqueBoolSupplier(driver::isRightCenterButtonDown);
        debug = new TorqueBoolSupplier(driver::isLeftCenterButtonDown);

        L1Mode = new TorqueBoolSupplier(operator::isAButtonDown);
        L2Mode = new TorqueBoolSupplier(operator::isXButtonDown);
        L3Mode = new TorqueBoolSupplier(operator::isBButtonDown);
        L4Mode = new TorqueBoolSupplier(operator::isYButtonDown);
        net = new TorqueBoolSupplier(operator::isDPADUpDown);
        processor = new TorqueBoolSupplier(driver::isAButtonDown);
        coralStation = new TorqueBoolSupplier(driver::isYButtonDown);

        algaeHigh = new TorqueBoolSupplier(operator::isRightBumperDown);
        algaeLow = new TorqueBoolSupplier(operator::isRightTriggerDown);
        algaeGroundIntake = new TorqueBoolSupplier(driver::isBButtonDown);

        leftRelation = new TorqueBoolSupplier(operator::isDPADLeftDown);
        rightRelation = new TorqueBoolSupplier(operator::isDPADRightDown);

        intakeCoral = new TorqueBoolSupplier(driver::isLeftTriggerDown);
        intakeAlgae = new TorqueBoolSupplier(driver::isRightTriggerDown);
        outtakeCoral = new TorqueBoolSupplier(driver::isLeftBumperDown);
        outtakeAlgae = new TorqueBoolSupplier(driver::isRightBumperDown);
    }

    @Override
    public final void update() {
        updateDrivebase();
        updateElevator();
        updateClaw();
    }

    public final void updateDrivebase() {
        resetGyro.onTrue(() -> perception.resetHeading());

        leftRelation.onTrue(() -> drivebase.setRelation(Relation.LEFT));
        rightRelation.onTrue(() -> drivebase.setRelation(Relation.RIGHT));

        final double xVelocity = TorqueMath.scaledLinearDeadband(driver.getLeftYAxis(), CONTROLLER_DEADBAND)
                * Drivebase.activeMaxVelocity;
        final double yVelocity = TorqueMath.scaledLinearDeadband(driver.getLeftXAxis(), CONTROLLER_DEADBAND)
                * Drivebase.activeMaxVelocity;
        final double rotationVelocity = TorqueMath.scaledLinearDeadband(driver.getRightXAxis(), CONTROLLER_DEADBAND)
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
        coralStation.onTrue(() -> {
            elevator.setState(Elevator.State.CORAL_HP);
            claw.setState(Claw.State.CORAL_HP);
            claw.setCoralState(Claw.CoralState.INTAKE);
        });
    }

    public final void updateClaw() {
        L2Mode.onTrue(() -> claw.setState(Claw.State.MID_SCORE));
        L3Mode.onTrue(() -> claw.setState(Claw.State.MID_SCORE));
        L4Mode.onTrue(() -> claw.setState(Claw.State.L4_SCORE));
        net.onTrue(() -> claw.setState(Claw.State.NET));
        processor.onTrue(() -> claw.setState(Claw.State.PROCESSOR));
        algaeHigh.onTrue(() -> claw.setState(Claw.State.ALGAE_EXTRACTION));
        algaeLow.onTrue(() -> claw.setState(Claw.State.ALGAE_EXTRACTION));
        algaeGroundIntake.onTrue(() -> {
            claw.setState(Claw.State.ALGAE_GROUND_INTAKE);
            claw.setAlgaeState(Claw.AlgaeState.INTAKE);
        });

        intakeCoral.onTrue(() -> claw.setCoralState(Claw.CoralState.INTAKE));
        intakeAlgae.onTrue(() -> claw.setAlgaeState(Claw.AlgaeState.INTAKE));
        outtakeCoral.onTrue(() -> claw.setCoralState(Claw.CoralState.SHOOT));
        outtakeAlgae.onTrue(() -> claw.setAlgaeState(Claw.AlgaeState.SHOOT));
    }

    public boolean isDebugMode() {
        return debug.get();
    }

    public static final synchronized Input getInstance() {
        return instance == null ? instance = new Input() : instance;
    }
}

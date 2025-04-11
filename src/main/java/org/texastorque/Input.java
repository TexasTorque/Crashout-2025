/**
 * Copyright 2025 Texas Torque.
 *
 * This file is part of Bravo/Charlie/Crashout-2025, which is not licensed for distribution.
 * For more details, see ./license.txt or write <davey.adams.three@gmail.com>.
 */
package org.texastorque;

import org.texastorque.Field.AlignPosition.AlignableTarget;
import org.texastorque.Field.AlignPosition.Relation;
import org.texastorque.subsystems.Pickup;
import org.texastorque.subsystems.Claw;
import org.texastorque.subsystems.Drivebase;
import org.texastorque.subsystems.Elevator;
import org.texastorque.subsystems.Claw.AlgaeState;
import org.texastorque.subsystems.Intake;
import org.texastorque.torquelib.base.TorqueInput;
import org.texastorque.torquelib.control.TorqueBoolSupplier;
import org.texastorque.torquelib.control.TorqueClickSupplier;
import org.texastorque.torquelib.control.TorqueRequestableTimeout;
import org.texastorque.torquelib.sensors.TorqueController;
import org.texastorque.torquelib.swerve.TorqueSwerveSpeeds;
import org.texastorque.torquelib.util.TorqueMath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public final class Input extends TorqueInput<TorqueController> implements Subsystems {

    private static volatile Input instance;
    private final double CONTROLLER_DEADBAND = 0.1;
    private final TorqueRequestableTimeout driverRumble, operatorRumble;
    private final TorqueClickSupplier slowInitial, endgameClick, manualElevatorInitial;
    private final TorqueBoolSupplier resetGyro, align, slow, stow,
            L1, L2, L3, L4, leftRelation, rightRelation, centerRelation,
            algaeExtractionHigh, algaeExtractionLow, net, processor,
            manualElevatorUp, manualElevatorDown, intakeCoral, intakeAlgae,
            outtakeCoral, outtakeAlgae, goToSelected, groundIntake, backIntake, shootLow;

    private Input() {
        driver = new TorqueController(0, CONTROLLER_DEADBAND);
        operator = new TorqueController(1, CONTROLLER_DEADBAND);

        driverRumble = new TorqueRequestableTimeout();
        operatorRumble = new TorqueRequestableTimeout();

        endgameClick = new TorqueClickSupplier(() -> Timer.getMatchTime() < 30 && Timer.getMatchTime() > 28 && DriverStation.isTeleop());

        resetGyro = new TorqueBoolSupplier(driver::isRightCenterButtonDown);

        intakeCoral = new TorqueBoolSupplier(driver::isLeftBumperDown);
        intakeAlgae = new TorqueBoolSupplier(driver::isYButtonDown);
        align = new TorqueBoolSupplier(() -> driver.isRightTriggerDown() && perception.getCurrentZone() != null);

        goToSelected = new TorqueBoolSupplier(driver::isAButtonDown);

        stow = new TorqueBoolSupplier(() -> driver.isDPADDownDown() || operator.isDPADDownDown());

        slowInitial = new TorqueClickSupplier(driver::isLeftTriggerDown);
        slow = new TorqueBoolSupplier(driver::isLeftTriggerDown);

        L1 = new TorqueBoolSupplier(operator::isAButtonDown);
        L2 = new TorqueBoolSupplier(operator::isXButtonDown);
        L3 = new TorqueBoolSupplier(operator::isBButtonDown);
        L4 = new TorqueBoolSupplier(operator::isYButtonDown);

        algaeExtractionHigh = new TorqueBoolSupplier(operator::isRightBumperDown);
        algaeExtractionLow = new TorqueBoolSupplier(operator::isRightTriggerDown);

        groundIntake = new TorqueBoolSupplier(driver::isRightBumperDown);

        net = new TorqueBoolSupplier(operator::isLeftBumperDown);
        processor = new TorqueBoolSupplier(operator::isLeftTriggerDown);

        leftRelation = new TorqueBoolSupplier(operator::isDPADLeftDown);
        rightRelation = new TorqueBoolSupplier(operator::isDPADRightDown);
        centerRelation = new TorqueBoolSupplier(operator::isDPADUpDown);

        manualElevatorInitial = new TorqueClickSupplier(() -> operator.getRightYAxis() > CONTROLLER_DEADBAND || operator.getRightYAxis() < -CONTROLLER_DEADBAND);
        manualElevatorUp = new TorqueBoolSupplier(() -> operator.getRightYAxis() > CONTROLLER_DEADBAND);
        manualElevatorDown = new TorqueBoolSupplier(() -> operator.getRightYAxis() < -CONTROLLER_DEADBAND);

        outtakeCoral = new TorqueBoolSupplier(driver::isBButtonDown);
        outtakeAlgae = new TorqueBoolSupplier(driver::isXButtonDown);

        backIntake = new TorqueBoolSupplier(driver::isDPADLeftDown);
        shootLow = new TorqueBoolSupplier(driver::isDPADUpDown);
    }

    @Override
    public final void update() {
        updateDrivebase();
        updateSuperstructure();

        endgameClick.onTrue(() -> {
            driverRumble.set(.5);
            operatorRumble.set(.5);
        });

        driver.setRumble(driverRumble.get());
        operator.setRumble(operatorRumble.get());

        final double DELTA = 1;
        manualElevatorInitial.onTrue(() -> {
            Elevator.State.MANUAL.position = elevator.getElevatorPosition();
        });
        manualElevatorUp.onTrue(() -> {
            elevator.setState(Elevator.State.MANUAL);
            Elevator.State.MANUAL.position = -DELTA + elevator.getElevatorPosition();
        });
        manualElevatorDown.onTrue(() -> {
            elevator.setState(Elevator.State.MANUAL);
            Elevator.State.MANUAL.position = DELTA + elevator.getElevatorPosition();
        });

        goToSelected.onTrue(() -> {
            elevator.setState(elevator.getSelectedState());
            claw.setState(claw.getSelectedState());
            perception.setDesiredAlignTarget(AlignableTarget.of(elevator.getSelectedState()));
        });
    }

    public final void updateDrivebase() {
        resetGyro.onTrue(() -> perception.resetHeading());

        leftRelation.onTrue(() -> perception.setRelation(Relation.LEFT));
        rightRelation.onTrue(() -> perception.setRelation(Relation.RIGHT));
        centerRelation.onTrue(() -> perception.setRelation(Relation.CENTER));

        slowInitial.onTrue(() -> drivebase.startSlowMode());
        slow.onTrue(() -> drivebase.setState(Drivebase.State.SLOW));

        align.onTrue(() -> drivebase.setState(Drivebase.State.ALIGN));

        final boolean isRedAlliance = DriverStation.getAlliance().isPresent()
                    ? DriverStation.getAlliance().get() == Alliance.Red
                    : false;
        
        final double xVelocity = TorqueMath.scaledLinearDeadband(-driver.getLeftYAxis(), CONTROLLER_DEADBAND)
                * Drivebase.MAX_VELOCITY * (isRedAlliance ? -1 : 1);
        final double yVelocity = TorqueMath.scaledLinearDeadband(-driver.getLeftXAxis(), CONTROLLER_DEADBAND)
                * Drivebase.MAX_VELOCITY * (isRedAlliance ? -1 : 1);
        final double rotationVelocity = TorqueMath.scaledLinearDeadband(-driver.getRightXAxis(), CONTROLLER_DEADBAND)
                * Drivebase.MAX_ANGULAR_VELOCITY;

        drivebase.setInputSpeeds(new TorqueSwerveSpeeds(xVelocity, yVelocity, rotationVelocity));
    }

    public final void updateSuperstructure() {
        L1.onTrue(() -> {
            elevator.setState(Elevator.State.SCORE_L1);
            claw.setState(Claw.State.SCORE_L1);
            perception.setDesiredAlignTarget(AlignableTarget.L1);
        });
        L2.onTrue(() -> {
            elevator.setSelectedState(Elevator.State.SCORE_L2);
            claw.setSelectedState(Claw.State.SCORE_L2);
            perception.setDesiredAlignTarget(AlignableTarget.L2);
        });
        L3.onTrue(() -> {
            elevator.setSelectedState(Elevator.State.SCORE_L3);
            claw.setSelectedState(Claw.State.SCORE_L3);
            perception.setDesiredAlignTarget(AlignableTarget.L3);
        });
        L4.onTrue(() -> {
            elevator.setSelectedState(Elevator.State.SCORE_L4);
            claw.setSelectedState(Claw.State.SCORE_L4);
            perception.setDesiredAlignTarget(AlignableTarget.L4);
        });
        net.onTrue(() -> {
            elevator.setState(Elevator.State.NET);
            claw.setState(Claw.State.NET);
            perception.setDesiredAlignTarget(AlignableTarget.NET);
        });
        processor.onTrue(() -> {
            elevator.setState(Elevator.State.PROCESSOR);
            claw.setState(Claw.State.PROCESSOR);
            perception.setDesiredAlignTarget(AlignableTarget.PROCESSOR);
        });
        algaeExtractionHigh.onTrue(() -> {
            elevator.setState(Elevator.State.ALGAE_REMOVAL_HIGH);
            elevator.setSelectedState(Elevator.State.ALGAE_REMOVAL_HIGH);
            perception.setDesiredAlignTarget(AlignableTarget.ALGAE_HIGH);
            perception.setRelation(Relation.CENTER);
            claw.setState(Claw.State.ALGAE_EXTRACTION);
            claw.setAlgaeState(Claw.AlgaeState.INTAKE);
        });
        algaeExtractionLow.onTrue(() -> {
            elevator.setState(Elevator.State.ALGAE_REMOVAL_LOW);
            elevator.setSelectedState(Elevator.State.ALGAE_REMOVAL_LOW);
            perception.setDesiredAlignTarget(AlignableTarget.ALGAE_LOW);
            perception.setRelation(Relation.CENTER);
            claw.setState(Claw.State.ALGAE_EXTRACTION);
            claw.setAlgaeState(Claw.AlgaeState.INTAKE);
        });
        intakeCoral.onTrue(() -> {
            elevator.setState(Elevator.State.CORAL_HP);
            claw.setState(Claw.State.CORAL_HP);
            claw.setCoralState(Claw.CoralState.INTAKE);
            claw.coralSpike.reset();
            perception.setDesiredAlignTarget(AlignableTarget.CORAL_STATION);
        });
        intakeAlgae.onTrue(() -> {
            claw.setAlgaeState(AlgaeState.INTAKE);
        });
        stow.onTrue(() -> {
            elevator.setState(Elevator.State.STOW);
            claw.setState(Claw.State.STOW);
            perception.setDesiredAlignTarget(AlignableTarget.NONE);
        });
        outtakeCoral.onTrue(() -> {
            if (claw.getState() == Claw.State.SCORE_L1) claw.setCoralState(Claw.CoralState.SHOOT_SLOW);
            else claw.setCoralState(Claw.CoralState.SHOOT);
            claw.coralSpike.reset();
        });
        outtakeAlgae.onTrue(() -> {
            claw.setAlgaeState(Claw.AlgaeState.SHOOT);
        });
        groundIntake.onTrue(() -> {
            intake.setState(Intake.State.HANDOFF);
            intake.setRollerState(Intake.RollerState.OFF);
            // intake.setRollerState(Intake.RollerState.INTAKE);
            // elevator.setState(Elevator.State.HANDOFF);
            // claw.setState(Claw.State.HANDOFF_INITIAL);
            perception.setDesiredAlignTarget(AlignableTarget.NONE);
        });
        backIntake.onTrue(() -> {
            backPickup.setState(Pickup.State.INTAKE);
            backPickup.setRollersState(Pickup.RollersState.INTAKE);
        });
        shootLow.onTrue(() -> {
            backPickup.setState(Pickup.State.SHOOT);
            backPickup.setRollersState(Pickup.RollersState.SHOOT);
        });
    }

    public final boolean isDebugMode() {
        return false;
    }

    public static final synchronized Input getInstance() {
        return instance == null ? instance = new Input() : instance;
    }
}

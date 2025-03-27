package org.texastorque;

import org.texastorque.Field.AlignPosition.AlignableTarget;
import org.texastorque.Field.AlignPosition.Relation;
import org.texastorque.subsystems.Claw;
import org.texastorque.subsystems.Drivebase;
import org.texastorque.subsystems.Elevator;
import org.texastorque.subsystems.Claw.AlgaeState;
import org.texastorque.subsystems.Climb;
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
    private final TorqueClickSupplier slowInitial, alignInitial, endgameClick, manualElevatorInitial;
    private final TorqueBoolSupplier resetGyro, align, slow, stow,
            L1, L2, L3, L4, leftRelation, rightRelation, centerRelation,
            algaeExtractionHigh, algaeExtractionLow, net, processor,
            climbUp, climbDown, manualElevatorUp, manualElevatorDown,
            intakeCoral, intakeAlgae, outtakeCoral, outtakeAlgae,
            climbMode, intakeCoralShift, goToSelected;

    private Input() {
        driver = new TorqueController(0, CONTROLLER_DEADBAND);
        operator = new TorqueController(1, CONTROLLER_DEADBAND);

        driverRumble = new TorqueRequestableTimeout();
        operatorRumble = new TorqueRequestableTimeout();

        endgameClick = new TorqueClickSupplier(() -> Timer.getMatchTime() < 30 && Timer.getMatchTime() > 28 && DriverStation.isTeleop());

        resetGyro = new TorqueBoolSupplier(driver::isRightCenterButtonDown);

        intakeCoral = new TorqueBoolSupplier(driver::isLeftBumperDown);
        intakeCoralShift = new TorqueBoolSupplier(driver::isDPADUpDown);
        intakeAlgae = new TorqueBoolSupplier(driver::isYButtonDown);
        
        alignInitial = new TorqueClickSupplier(() -> driver.isRightTriggerDown() && perception.getCurrentZone() != null);
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

        net = new TorqueBoolSupplier(operator::isLeftBumperDown);
        processor = new TorqueBoolSupplier(operator::isLeftTriggerDown);

        leftRelation = new TorqueBoolSupplier(operator::isDPADLeftDown);
        rightRelation = new TorqueBoolSupplier(operator::isDPADRightDown);
        centerRelation = new TorqueBoolSupplier(operator::isDPADUpDown);

        climbUp = new TorqueBoolSupplier(() -> operator.getLeftYAxis() < -CONTROLLER_DEADBAND && !operator.isLeftStickClickDown());
        climbDown = new TorqueBoolSupplier(() -> operator.getLeftYAxis() > CONTROLLER_DEADBAND);

        climbMode = new TorqueBoolSupplier(driver::isDPADRightDown);

        manualElevatorInitial = new TorqueClickSupplier(() -> operator.getRightYAxis() > CONTROLLER_DEADBAND || operator.getRightYAxis() < -CONTROLLER_DEADBAND);
        manualElevatorUp = new TorqueBoolSupplier(() -> operator.getRightYAxis() > CONTROLLER_DEADBAND);
        manualElevatorDown = new TorqueBoolSupplier(() -> operator.getRightYAxis() < -CONTROLLER_DEADBAND);

        outtakeCoral = new TorqueBoolSupplier(driver::isBButtonDown);
        outtakeAlgae = new TorqueBoolSupplier(driver::isXButtonDown);
    }

    @Override
    public final void update() {
        updateDrivebase();
        updateSuperstructure();
        updateClimb();

        endgameClick.onTrue(() -> {
            driverRumble.set(.5);
            operatorRumble.set(.5);
        });

        driver.setRumble(driverRumble.get());
        operator.setRumble(operatorRumble.get());

        final double DELTA = .2;
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

        alignInitial.onTrue(() -> drivebase.getAlignController().reset());
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
        intakeCoralShift.onTrue(() -> {
            elevator.setState(Elevator.State.CORAL_HP_SHIFT);
            claw.setState(Claw.State.CORAL_HP_SHIFT);
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
            if (claw.getState() == Claw.State.SCORE_L1) {
                claw.setCoralState(Claw.CoralState.SHOOT_SLOW);
            } else {
                claw.setCoralState(Claw.CoralState.SHOOT);
            }
            claw.coralSpike.reset();
        });
        outtakeAlgae.onTrue(() -> {
            claw.setAlgaeState(Claw.AlgaeState.SHOOT);
        });
        climbMode.onTrue(() -> {
            elevator.setState(Elevator.State.CLIMB);
            claw.setState(Claw.State.CLIMB);
            climb.setState(Climb.State.OUT);
            perception.setDesiredAlignTarget(AlignableTarget.NONE);
        });
    }

    public final void updateClimb() {
        climbUp.onTrue(() -> climb.setState(Climb.State.OUT));
        climbDown.onTrue(() -> climb.setState(Climb.State.IN));
    }

    public final boolean isDebugMode() {
        return false;
    }

    public static final synchronized Input getInstance() {
        return instance == null ? instance = new Input() : instance;
    }
}

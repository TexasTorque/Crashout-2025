package org.texastorque;

import org.texastorque.AlignPose2d.Relation;
import org.texastorque.subsystems.Claw;
import org.texastorque.subsystems.Drivebase;
import org.texastorque.subsystems.Elevator;
import org.texastorque.subsystems.Claw.AlgaeState;
import org.texastorque.subsystems.Climb;
import org.texastorque.torquelib.base.TorqueInput;
import org.texastorque.torquelib.control.TorqueBoolSupplier;
import org.texastorque.torquelib.control.TorqueClickSupplier;
import org.texastorque.torquelib.sensors.TorqueController;
import org.texastorque.torquelib.swerve.TorqueSwerveSpeeds;
import org.texastorque.torquelib.util.TorqueMath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public final class Input extends TorqueInput<TorqueController> implements Subsystems {
    private static volatile Input instance;
    private final double CONTROLLER_DEADBAND = 0.1;
    private final TorqueClickSupplier slowInitial, manualElevatorInitial;
    private final TorqueBoolSupplier resetGyro, align, slow, stow,
            L1, L2, L3, L4, leftRelation, rightRelation, centerRelation,
            algaeExtractionHigh, algaeExtractionLow, net, processor,
            climbUp, climbDown, manualElevatorUp, manualElevatorDown,
            intakeCoral, intakeAlgae, outtakeCoral, outtakeAlgae,
            climbMode, manualClimbInitial, manualClimbUp, manualClimbDown;

    private Input() {
        driver = new TorqueController(0, CONTROLLER_DEADBAND);
        operator = new TorqueController(1, CONTROLLER_DEADBAND);

        resetGyro = new TorqueBoolSupplier(driver::isRightCenterButtonDown);

        intakeCoral = new TorqueBoolSupplier(driver::isLeftBumperDown);
        intakeAlgae = new TorqueBoolSupplier(driver::isYButtonDown);
        
        align = new TorqueBoolSupplier(driver::isRightTriggerDown);

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

        manualClimbInitial = new TorqueClickSupplier(() -> (operator.getLeftYAxis() > CONTROLLER_DEADBAND && operator.isLeftStickClickDown()) || (operator.getLeftYAxis() < -CONTROLLER_DEADBAND && operator.isLeftStickClickDown()));
        manualClimbUp = new TorqueBoolSupplier(() -> (operator.getLeftYAxis() > CONTROLLER_DEADBAND && operator.isLeftStickClickDown()));
        manualClimbDown = new TorqueBoolSupplier(() -> (operator.getLeftYAxis() > CONTROLLER_DEADBAND && operator.isLeftStickClickDown()));

        manualElevatorInitial = new TorqueClickSupplier(() -> operator.getRightXAxis() > CONTROLLER_DEADBAND || operator.getRightYAxis() < -CONTROLLER_DEADBAND);
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
    }

    public final void updateDrivebase() {
        resetGyro.onTrue(() -> perception.resetHeading());

        leftRelation.onTrue(() -> drivebase.setRelation(Relation.LEFT));
        rightRelation.onTrue(() -> drivebase.setRelation(Relation.RIGHT));
        centerRelation.onTrue(() -> drivebase.setRelation(Relation.CENTER));

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
        });
        L2.onTrue(() -> {
            elevator.setState(Elevator.State.SCORE_L2);
            claw.setState(Claw.State.MID_SCORE);
        });
        L3.onTrue(() -> {
            elevator.setState(Elevator.State.SCORE_L3);
            claw.setState(Claw.State.MID_SCORE);
        });
        L4.onTrue(() -> {
            elevator.setState(Elevator.State.SCORE_L4);
            claw.setState(Claw.State.SCORE_L4);
        });
        net.onTrue(() -> {
            elevator.setState(Elevator.State.NET);
            claw.setState(Claw.State.NET);
        });
        processor.onTrue(() -> {
            elevator.setState(Elevator.State.PROCESSOR);
            claw.setState(Claw.State.PROCESSOR);
        });
        algaeExtractionHigh.onTrue(() -> {
            elevator.setState(Elevator.State.ALGAE_REMOVAL_HIGH);
            claw.setState(Claw.State.ALGAE_EXTRACTION);
            claw.setAlgaeState(Claw.AlgaeState.INTAKE);
        });
        algaeExtractionLow.onTrue(() -> {
            elevator.setState(Elevator.State.ALGAE_REMOVAL_LOW);
            claw.setState(Claw.State.ALGAE_EXTRACTION);
            claw.setAlgaeState(Claw.AlgaeState.INTAKE);
        });
        intakeCoral.onTrue(() -> {
            elevator.setState(Elevator.State.CORAL_HP);
            claw.setState(Claw.State.CORAL_HP);
            claw.setCoralState(Claw.CoralState.INTAKE);
            claw.coralSpike.reset();
        });
        intakeAlgae.onTrue(() -> {
            claw.setAlgaeState(AlgaeState.INTAKE);
        });
        stow.onTrue(() -> {
            elevator.setState(Elevator.State.STOW);
            claw.setState(Claw.State.STOW);
        });
        outtakeCoral.onTrue(() -> {
            if (claw.getState() == Claw.State.SCORE_L1) {
                claw.setCoralState(Claw.CoralState.SHOOT_FAST);
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
        });
    }

    public final void updateClimb() {
        climbUp.onTrue(() -> climb.setState(Climb.State.OUT));
        climbDown.onTrue(() -> climb.setState(Climb.State.IN));

        final double DELTA = 10;
        manualClimbInitial.onTrue(() -> {
            Climb.State.MANUAL.position = climb.getClimbPosition();
        });
        manualClimbUp.onTrue(() -> {
            climb.setState(Climb.State.MANUAL);
            Climb.State.MANUAL.position = -DELTA + climb.getClimbPosition();
        });
        manualClimbDown.onTrue(() -> {
            climb.setState(Climb.State.MANUAL);
            Climb.State.MANUAL.position = DELTA + climb.getClimbPosition();
        });
    }

    public final boolean isDebugMode() {
        return false;
    }

    public static final synchronized Input getInstance() {
        return instance == null ? instance = new Input() : instance;
    }
}

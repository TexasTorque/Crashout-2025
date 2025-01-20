package org.texastorque;

import java.lang.annotation.ElementType;

import org.texastorque.AlignPose2d.Relation;
import org.texastorque.subsystems.Claw;
import org.texastorque.subsystems.Drivebase;
import org.texastorque.subsystems.Elevator;
import org.texastorque.torquelib.base.TorqueInput;
import org.texastorque.torquelib.control.TorqueBoolSupplier;
import org.texastorque.torquelib.sensors.TorqueController;
import org.texastorque.torquelib.sensors.TorqueController.DPADState;
import org.texastorque.torquelib.swerve.TorqueSwerveSpeeds;
import org.texastorque.torquelib.util.TorqueMath;


public final class Input extends TorqueInput<TorqueController> implements Subsystems {
    private static volatile Input instance;
    private final double CONTROLLER_DEADBAND = 0.1;
    private final TorqueBoolSupplier resetGyro, apriltagAlign, slowMode, L2Mode, L3Mode, L4Mode, scoreSequence;

    private Input() {
        driver = new TorqueController(0, CONTROLLER_DEADBAND);
        operator = new TorqueController(1, CONTROLLER_DEADBAND);

        // Driver
        resetGyro = new TorqueBoolSupplier(driver::isRightCenterButtonDown);
        apriltagAlign = new TorqueBoolSupplier(driver::isXButtonDown);

        // Drive base slow mode
        slowMode = new TorqueBoolSupplier(driver::isRightBumperPressed);

        // scoring modes
        L2Mode = new TorqueBoolSupplier(operator::isAButtonDown);
        L3Mode = new TorqueBoolSupplier(operator::isBButtonDown);
        L4Mode = new TorqueBoolSupplier(operator::isYButtonDown);

        scoreSequence = new TorqueBoolSupplier(operator::isRightTriggerDown);
    }

    @Override
    public final void update() {
        updateDrivebase();
        updateElevator();
        updateClaw();
    }

    public final void updateDrivebase() {
        resetGyro.onTrue(() -> perception.resetHeading());
        apriltagAlign.onTrue(() -> drivebase.setState(Drivebase.State.ALIGN_TO_APRILTAG));
        slowMode.onTrue(drivebase::toggleSlowMode);

        if (driver.isDPADLeftDown()) {
            drivebase.setRelation(Relation.LEFT);
        } else {
            drivebase.setRelation(null);
        }

        final double xVelocity = TorqueMath.scaledLinearDeadband(-driver.getLeftYAxis(), CONTROLLER_DEADBAND)
                * Drivebase.activeMaxVelocity;
        final double yVelocity = TorqueMath.scaledLinearDeadband(-driver.getLeftXAxis(), CONTROLLER_DEADBAND)
                * Drivebase.activeMaxVelocity;
        final double rotationVelocity = TorqueMath.scaledLinearDeadband(-driver.getRightXAxis(), CONTROLLER_DEADBAND)
                * Drivebase.activeMaxAngularVelocity;

        drivebase.setInputSpeeds(new TorqueSwerveSpeeds(xVelocity, yVelocity, rotationVelocity));
    }

    public final void updateElevator() {
        L2Mode.onTrue(() -> elevator.setState(Elevator.State.SCORE_L2));
        L3Mode.onTrue(() -> elevator.setState(Elevator.State.SCORE_L3));
        L4Mode.onTrue(() -> elevator.setState(Elevator.State.SCORE_L4));
        scoreSequence.onTrue(() -> elevator.startScoreSequence());
    }

    public final void updateClaw() {
        L2Mode.onTrue(() -> claw.setState(Claw.State.SCORE_LOW));
        L3Mode.onTrue(() -> claw.setState(Claw.State.SCORE_LOW));
        L4Mode.onTrue(() -> claw.setState(Claw.State.SCORE_HIGH));
    }

    public boolean isDebugMode() {
        return false;
    }

    public static final synchronized Input getInstance() {
        return instance == null ? instance = new Input() : instance;
    }
}

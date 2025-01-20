package org.texastorque.subsystems;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.texastorque.AlignPose2d.Relation;
import org.texastorque.AprilTagList;
import org.texastorque.Ports;
import org.texastorque.Subsystems;
import org.texastorque.torquelib.auto.commands.TorqueFollowPath.TorquePathingDrivebase;
import org.texastorque.torquelib.base.TorqueMode;
import org.texastorque.torquelib.base.TorqueState;
import org.texastorque.torquelib.base.TorqueStatorSubsystem;
import org.texastorque.torquelib.swerve.TorqueSwerveSpeeds;

import org.texastorque.torquelib.swerve.TorqueSwerveModule2022;
import org.texastorque.torquelib.swerve.TorqueSwerveModuleNEO;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public final class Drivebase extends TorqueStatorSubsystem<Drivebase.State> implements Subsystems, TorquePathingDrivebase {

    public static enum State implements TorqueState {
        FIELD_RELATIVE(null),
        ROBOT_RELATIVE(null),
        ALIGN_TO_APRILTAG(FIELD_RELATIVE),
        PATHING(null);

        public final State parent;

        private State(final State parent) {
            this.parent = parent == null ? this : parent;
        }
    }

    private static volatile Drivebase instance;

    public static final double WIDTH = .552323,
            MAX_VELOCITY = TorqueSwerveModuleNEO.maxVelocity,
            MAX_ANGULAR_VELOCITY = 4 * Math.PI;

    public static double activeMaxVelocity = MAX_VELOCITY;
    public static double activeMaxAngularVelocity = MAX_ANGULAR_VELOCITY;
    public static boolean isSlowMode = false;

    public static final Translation2d LOC_FL = new Translation2d(WIDTH / 2, WIDTH / 2),
            LOC_FR = new Translation2d(WIDTH / 2, -WIDTH / 2),
            LOC_BL = new Translation2d(-WIDTH / 2, WIDTH / 2),
            LOC_BR = new Translation2d(-WIDTH / 2, -WIDTH / 2);

    private final TorqueSwerveModule2022 fl, fr, bl, br;

    public TorqueSwerveSpeeds inputSpeeds;
    public final SwerveDriveKinematics kinematics;
    private SwerveModuleState[] swerveStates;
    private Relation relation;

    private final PIDController apriltagAlignRotationPID;
    private final PIDController apriltagAlignXPID;
    private final PIDController apriltagAlignYPID;

    private Drivebase() {
        super(State.FIELD_RELATIVE);

        fl = new TorqueSwerveModule2022("Front Left", Ports.FL_MOD);
        fr = new TorqueSwerveModule2022("Front Right", Ports.FR_MOD);
        bl = new TorqueSwerveModule2022("Back Left", Ports.BL_MOD);
        br = new TorqueSwerveModule2022("Back Right", Ports.BR_MOD);

        inputSpeeds = new TorqueSwerveSpeeds(0, 0, 0);
        kinematics = new SwerveDriveKinematics(LOC_FL, LOC_FR, LOC_BL, LOC_BR);

        swerveStates = new SwerveModuleState[4];
        for (int i = 0; i < swerveStates.length; i++)
            swerveStates[i] = new SwerveModuleState();

        apriltagAlignXPID = new PIDController(1, 0, 0);
        apriltagAlignYPID = new PIDController(1, 0, 0);
        apriltagAlignRotationPID = new PIDController(.082, 0, 0);
        apriltagAlignRotationPID.enableContinuousInput(0, 360);
    }

    @Override
    public final void initialize(final TorqueMode mode) {
        setInputSpeeds(new TorqueSwerveSpeeds());
    
        SmartDashboard.putData("Swerve",
            builder -> {
                builder.setSmartDashboardType("SwerveDrive");
                builder.addDoubleProperty(
                    "Front Left Angle", () -> fl.getPosition().angle.getRadians(), null);
                builder.addDoubleProperty(
                    "Front Left Velocity", () -> fl.getState().speedMetersPerSecond, null);
                builder.addDoubleProperty(
                    "Front Right Angle", () -> fr.getPosition().angle.getRadians(), null);
                builder.addDoubleProperty(
                    "Front Right Velocity", () -> fr.getState().speedMetersPerSecond, null);
                builder.addDoubleProperty(
                    "Back Left Angle", () -> bl.getPosition().angle.getRadians(), null);
                builder.addDoubleProperty(
                    "Back Left Velocity", () -> bl.getState().speedMetersPerSecond, null);
                builder.addDoubleProperty(
                    "Back Right Angle", () -> br.getPosition().angle.getRadians(), null);
                builder.addDoubleProperty(
                    "Back Right Velocity", () -> br.getState().speedMetersPerSecond, null);
                builder.addDoubleProperty(
                    "Robot Angle", () -> perception.getHeading().getRadians(), null);
            }
        );
    }

    final Pose2d targetPose = new Pose2d(2.6576, 4.02659, Rotation2d.fromDegrees(270));

    @Override
    public final void update(final TorqueMode mode) {
        if (wantsState(State.FIELD_RELATIVE)) {
            inputSpeeds = inputSpeeds.toFieldRelativeSpeeds(perception.getHeading());
        }
        
        if (wantsState(State.ALIGN_TO_APRILTAG)) {
            // final Optional<Pose2d> alignPose = perception.getAlignPose(relation);
            // if (alignPose.isPresent() && relation != null && ) {
            //     SmartDashboard.putString("Align Target Pose", alignPose.get().toString());
            //     final Pose2d targetPose = alignPose.get();

            final double xPower = apriltagAlignXPID.calculate(perception.getPose().getX(), targetPose.getX());
            final double yPower = apriltagAlignYPID.calculate(perception.getPose().getY(), targetPose.getY());
            final double omegaPower = apriltagAlignRotationPID.calculate(perception.getHeading().getDegrees(), targetPose.getRotation().getDegrees());

            inputSpeeds.vxMetersPerSecond = yPower;
            inputSpeeds.vyMetersPerSecond = -xPower;
            inputSpeeds.omegaRadiansPerSecond = omegaPower;
        }
        SmartDashboard.putString("Drivebase State", desiredState.toString());
        swerveStates = kinematics.toSwerveModuleStates(inputSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveStates, MAX_VELOCITY);

        if (inputSpeeds.hasZeroVelocity()) {
            manuallySetModuleAngles(
                    swerveStates[0].angle,
                    swerveStates[1].angle,
                    swerveStates[2].angle,
                    swerveStates[3].angle);
        } else {
            fl.setDesiredState(swerveStates[0]);
            fr.setDesiredState(swerveStates[1]);
            bl.setDesiredState(swerveStates[2]);
            br.setDesiredState(swerveStates[3]);
        }

        SmartDashboard.putNumber("Robot Velocity", inputSpeeds.getVelocityMagnitude());
        Logger.recordOutput("Swerve States", swerveStates);
        Logger.recordOutput("Gyro Angle", perception.getHeading());
    }

    @Override
    public void clean(TorqueMode mode) {
        if (mode.isTeleop()) {
            desiredState = desiredState.parent;
        }
    }

    public void toggleSlowMode() {
        isSlowMode = !isSlowMode;
        activeMaxVelocity = isSlowMode ? MAX_VELOCITY / 10 : MAX_VELOCITY;
        activeMaxAngularVelocity = isSlowMode ? MAX_ANGULAR_VELOCITY / 10 : MAX_ANGULAR_VELOCITY;
    }

    public void setInputSpeeds(TorqueSwerveSpeeds inputSpeeds) {
        this.inputSpeeds = inputSpeeds;
    }

    public void setRelation(final Relation relation) {
      this.relation = relation;
    }

    private void manuallySetModuleAngles(final Rotation2d flAngle, final Rotation2d frAngle, final Rotation2d blAngle, final Rotation2d brAngle) {
        fl.setDesiredState(new SwerveModuleState(0, flAngle));
        fr.setDesiredState(new SwerveModuleState(0, frAngle));
        bl.setDesiredState(new SwerveModuleState(0, blAngle));
        br.setDesiredState(new SwerveModuleState(0, brAngle));
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            fl.getPosition(), fr.getPosition(),
            bl.getPosition(), br.getPosition()
        };
    }

    public double getRadius() {
        return WIDTH * Math.sqrt(2);
    }

    @Override
    public double getMaxPathingVelocity() {
        return MAX_VELOCITY;
    }

    @Override
    public ChassisSpeeds getActualChassisSpeeds() {
        return kinematics.toChassisSpeeds(swerveStates);
    }

    @Override
    public void onBeginPathing() {
        setState(State.PATHING);
    }

    @Override
    public void onEndPathing() {
        setState(State.FIELD_RELATIVE);
    }

    @Override
    public Pose2d getPose() {
        return perception.getPose();
    }

    @Override
    public void setPose(Pose2d pose) {
        perception.setPose(pose);
    }

    public static synchronized final Drivebase getInstance() {
        return instance == null ? instance = new Drivebase() : instance;
    }
}

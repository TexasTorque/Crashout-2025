/**
 * Copyright 2025 Texas Torque.
 *
 * This file is part of Bravo/Charlie/Crashout-2025, which is not licensed for distribution.
 * For more details, see ./license.txt or write <davey.adams.three@gmail.com>.
 */
package org.texastorque.subsystems;

import org.littletonrobotics.junction.Logger;
import org.texastorque.Field;
import org.texastorque.Ports;
import org.texastorque.Subsystems;
import org.texastorque.Field.AlignPosition.AlignableTarget;
import org.texastorque.torquelib.Debug;
import org.texastorque.torquelib.auto.commands.TorqueFollowPath.TorquePathingDrivebase;
import org.texastorque.torquelib.base.TorqueMode;
import org.texastorque.torquelib.base.TorqueState;
import org.texastorque.torquelib.base.TorqueStatorSubsystem;
import org.texastorque.torquelib.control.TorqueAlignController;
import org.texastorque.torquelib.swerve.TorqueSwerveSpeeds;
import org.texastorque.torquelib.util.TorqueMath;
import org.texastorque.torquelib.swerve.TorqueSwerveModuleKraken;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

public final class Drivebase extends TorqueStatorSubsystem<Drivebase.State> implements Subsystems, TorquePathingDrivebase {

    public static enum State implements TorqueState {
        FIELD_RELATIVE(null),
        ROBOT_RELATIVE(null),
        ALIGN(FIELD_RELATIVE),
        SLOW(FIELD_RELATIVE),
        PATHING(null),
        HP_ALIGN(FIELD_RELATIVE);

        public final State parent;

        private State(final State parent) {
            this.parent = parent == null ? this : parent;
        }
    }

    private static volatile Drivebase instance;
    private final TorqueSwerveModuleKraken fl, fr, bl, br;
    public final SwerveDriveKinematics kinematics;
    private final TorqueAlignController alignController;
    private final PIDController headingLockPID;
    private final PIDController coralStationPID;
    public TorqueSwerveSpeeds inputSpeeds;
    private SwerveModuleState[] swerveStates;
    private double slowStartTimestamp;
    private Pose2d alignPoseOverride;
    public double lastRotationTarget;

    public static final double WIDTH = 0.417581911,
            MAX_VELOCITY = TorqueSwerveModuleKraken.maxVelocity,
            MAX_ANGULAR_VELOCITY = 4 * Math.PI;
    public static final Translation2d LOC_FL = new Translation2d(WIDTH / 2, WIDTH / 2),
            LOC_FR = new Translation2d(WIDTH / 2, -WIDTH / 2),
            LOC_BL = new Translation2d(-WIDTH / 2, WIDTH / 2),
            LOC_BR = new Translation2d(-WIDTH / 2, -WIDTH / 2);
    final double MAX_ALIGN_VELOCITY = 1;
    final double MAX_ALIGN_OMEGA = 2 * Math.PI;
    final double MAX_ALIGN_VELOCITY_SLOW = .5;
    final double MAX_ALIGN_OMEGA_SLOW = Math.PI;

    final double IDEAL_HP_DISTANCE = 20;

    private Drivebase() {
        super(State.FIELD_RELATIVE);

        fl = new TorqueSwerveModuleKraken("Front Left", Ports.FL_MOD);
        fr = new TorqueSwerveModuleKraken("Front Right", Ports.FR_MOD);
        bl = new TorqueSwerveModuleKraken("Back Left", Ports.BL_MOD);
        br = new TorqueSwerveModuleKraken("Back Right", Ports.BR_MOD);

        inputSpeeds = new TorqueSwerveSpeeds(0, 0, 0);
        kinematics = new SwerveDriveKinematics(LOC_FL, LOC_FR, LOC_BL, LOC_BR);

        swerveStates = new SwerveModuleState[4];
        for (int i = 0; i < swerveStates.length; i++)
            swerveStates[i] = new SwerveModuleState();

        alignController = new TorqueAlignController(
            new PIDConstants(10.75, 0, 0),
            new PIDConstants(Math.PI * 2, 0, 0)
        );

        headingLockPID = new PIDController(.08, 0, 0);
        headingLockPID.enableContinuousInput(0, 360);

        coralStationPID = new PIDController(.01, 0, 0);
    }

    @Override
    public final void initialize(final TorqueMode mode) {
        setInputSpeeds(new TorqueSwerveSpeeds());
        if (mode.isTeleop()) {
            setState(State.FIELD_RELATIVE);
        }

        lastRotationTarget = getPose().getRotation().getDegrees();
    }

    @Override
    public final void update(final TorqueMode mode) {
        if (alignPoseOverride != null && wantsState(State.ALIGN)) {
            runAlignment(alignPoseOverride, false);
        } else if (wantsState(State.ALIGN)) {
            final Pose2d alignPose = perception.getAlignPose();
            if (alignPose != null) {
                runAlignment(alignPose, true);
            }
        }

        if (!inputSpeeds.hasRotationalVelocity() && mode.isTeleop() && desiredState != State.ALIGN)
            inputSpeeds.omegaRadiansPerSecond = headingLockPID.calculate(perception.getHeading().getDegrees(), lastRotationTarget);
        if (inputSpeeds.hasRotationalVelocity())
            lastRotationTarget = getPose().getRotation().getDegrees();
        if (wantsState(State.HP_ALIGN) && perception.getCurrentZone() != null)
            runHPAlignment(); 
        if (wantsState(State.FIELD_RELATIVE) || wantsState(State.ALIGN) || wantsState(State.SLOW) || alignPoseOverride != null)
            inputSpeeds = inputSpeeds.toFieldRelativeSpeeds(perception.getHeading());
        
        swerveStates = kinematics.toSwerveModuleStates(inputSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveStates, MAX_VELOCITY);

        if (wantsState(State.SLOW))
            for (SwerveModuleState state : swerveStates)
                state.speedMetersPerSecond *= getSlowMultiplier();

        if (inputSpeeds.hasZeroVelocity()) {
            manuallySetModuleAngles(
                swerveStates[0].angle,
                swerveStates[1].angle,
                swerveStates[2].angle,
                swerveStates[3].angle
            );
        } else {
            fl.setDesiredState(swerveStates[0]);
            fr.setDesiredState(swerveStates[1]);
            bl.setDesiredState(swerveStates[2]);
            br.setDesiredState(swerveStates[3]);
        }

        if (perception.getCurrentZone() != null) {
            final Pose2d tagPose = Field.AprilTagList.values()[perception.getCurrentZone().getID() - 1].pose;
            final Pair<Double, Double> offsets = perception.getOffsets(tagPose, getPose());
            Debug.log("AprilTag Forward Offset", -offsets.getFirst() + "");
            Debug.log("AprilTag Right Offset", -offsets.getSecond() + "");
        } else {
            Debug.log("AprilTag Forward Offset", "None");
            Debug.log("AprilTag Right Offset", "None");
        }

        Debug.log("Drivebase State", desiredState.toString());
        Debug.log("Is Aligned", isAligned());
        Debug.log("In Zone", perception.getCurrentZone() != null);
        Debug.log("Robot Velocity", inputSpeeds.getVelocityMagnitude());
        Logger.recordOutput("Gyro Angle", perception.getHeading());
        Logger.recordOutput("Swerve States", swerveStates);
    }

    @Override
    public void clean(TorqueMode mode) {
        if (mode.isTeleop()) {
            desiredState = desiredState.parent;
        }
    }

    public void runAlignment(final Pose2d pose, final boolean sidewaysFirst) {
        if (isAligned()) {
            setInputSpeeds(new TorqueSwerveSpeeds());
            return;
        }

        inputSpeeds = alignController.calculate(getPose(), pose, sidewaysFirst);

        if (perception.getDesiredAlignTarget() == AlignableTarget.L4) {
            inputSpeeds.vxMetersPerSecond = TorqueMath.constrain(inputSpeeds.vxMetersPerSecond, MAX_ALIGN_VELOCITY_SLOW);
            inputSpeeds.vyMetersPerSecond = TorqueMath.constrain(inputSpeeds.vyMetersPerSecond, MAX_ALIGN_VELOCITY_SLOW);
            inputSpeeds.omegaRadiansPerSecond = TorqueMath.constrain(inputSpeeds.omegaRadiansPerSecond, MAX_ALIGN_OMEGA_SLOW);
        } else {
            inputSpeeds.vxMetersPerSecond = TorqueMath.constrain(inputSpeeds.vxMetersPerSecond, MAX_ALIGN_VELOCITY);
            inputSpeeds.vyMetersPerSecond = TorqueMath.constrain(inputSpeeds.vyMetersPerSecond, MAX_ALIGN_VELOCITY);
            inputSpeeds.omegaRadiansPerSecond = TorqueMath.constrain(inputSpeeds.omegaRadiansPerSecond, MAX_ALIGN_OMEGA);
        }

        Debug.log("Distance to Align", pose.getTranslation().getDistance(perception.getPose().getTranslation()));
        Logger.recordOutput("Desired Component Poses", perception.getDesiredComponentPoses());
    }

    public boolean isAligned(final double translationTolerance, final double rotationTolerance) {
        Pose2d alignPose = perception.getAlignPose();

        if (alignPoseOverride != null) alignPose = alignPoseOverride;
        if (alignPose == null) return false;
        final double distance = alignPose.getTranslation().getDistance(perception.getPose().getTranslation());
        final boolean translationAligned = distance < translationTolerance;
        final double rotation = (alignPose.getRotation().getDegrees() - perception.getPose().getRotation().getDegrees() + 360) % 360;
        final boolean rotationAligned = rotation < rotationTolerance;

        Debug.log("Rotation Aligned", rotationAligned);
        Debug.log("Translation Aligned", translationAligned);

        if (translationAligned && rotationAligned && desiredState == State.ALIGN) {
            if (DriverStation.isAutonomous()) {
                setInputSpeeds(new TorqueSwerveSpeeds());
            }
            return true;
        }
        return false;
    }

    public void runHPAlignment() {
        if (isAtHP()) {
            setInputSpeeds(new TorqueSwerveSpeeds());
            return;
        }

        final Rotation2d target = perception.currentTagPose.getRotation();
        double speed = coralStationPID.calculate(perception.getHPDistance(), IDEAL_HP_DISTANCE);
        inputSpeeds.omegaRadiansPerSecond = headingLockPID.calculate(perception.getHeading().getDegrees(), target.getDegrees());
        inputSpeeds.vxMetersPerSecond = speed;
    }

    public boolean isAtHP() {;
        final double HP_DISTANCE_TOLERANCE = 0.01;
        return perception.getHPDistance() - IDEAL_HP_DISTANCE < HP_DISTANCE_TOLERANCE;
    }

    public boolean isAligned() {
        return isAligned(.005, 1);
    }

    public boolean isNearAligned() {
        return isAligned(.05, 2);
    }

    public double getSlowMultiplier() {
        final double MIN_MULT = .5;
        final double TIME_TO_MIN = .5;
        final double timeDelta = Timer.getFPGATimestamp() - slowStartTimestamp;

        if (timeDelta <= 0) return 1;
        if (timeDelta > TIME_TO_MIN) return MIN_MULT;
        return -1.5 * timeDelta + 1;
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

    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
            fl.getState(), fr.getState(),
            bl.getState(), br.getState()
        };
    }

    public void startSlowMode() {
        setState(State.SLOW);
        slowStartTimestamp = Timer.getFPGATimestamp();
    }

    public void setAlignPoseOverride(final Pose2d alignPoseOverride) {
        this.alignPoseOverride = alignPoseOverride;
    }

    public void setInputSpeeds(final TorqueSwerveSpeeds inputSpeeds) {
        this.inputSpeeds = inputSpeeds;
    }

    public Pose2d getAlignOverride() {
        return alignPoseOverride;
    }

    public TorqueAlignController getAlignController() {
        return alignController;
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

    @Override
    public void setCurrentTrajectory(Trajectory trajectory) {
        perception.setCurrentTrajectory(trajectory);
    }

    public static synchronized final Drivebase getInstance() {
        return instance == null ? instance = new Drivebase() : instance;
    }
}

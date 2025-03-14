package org.texastorque.subsystems;

import org.littletonrobotics.junction.Logger;
import org.texastorque.Field;
import org.texastorque.Ports;
import org.texastorque.Subsystems;
import org.texastorque.torquelib.Debug;
import org.texastorque.torquelib.auto.commands.TorqueFollowPath.TorquePathingDrivebase;
import org.texastorque.torquelib.base.TorqueMode;
import org.texastorque.torquelib.base.TorqueState;
import org.texastorque.torquelib.base.TorqueStatorSubsystem;
import org.texastorque.torquelib.control.TorqueFieldZone;
import org.texastorque.torquelib.swerve.TorqueSwerveSpeeds;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.util.PIDConstants;

import org.texastorque.torquelib.swerve.TorqueSwerveModuleKraken;

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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public final class Drivebase extends TorqueStatorSubsystem<Drivebase.State> implements Subsystems, TorquePathingDrivebase {

    public static enum State implements TorqueState {
        FIELD_RELATIVE(null),
        ROBOT_RELATIVE(null),
        ALIGN(FIELD_RELATIVE),
        SLOW(FIELD_RELATIVE),
        PATHING(null);

        public final State parent;

        private State(final State parent) {
            this.parent = parent == null ? this : parent;
        }
    }

    private static volatile Drivebase instance;

    public static final double WIDTH = 0.417581911,
            MAX_VELOCITY = TorqueSwerveModuleKraken.maxVelocity,
            MAX_ANGULAR_VELOCITY = 4 * Math.PI;

    public static final Translation2d LOC_FL = new Translation2d(WIDTH / 2, WIDTH / 2),
            LOC_FR = new Translation2d(WIDTH / 2, -WIDTH / 2),
            LOC_BL = new Translation2d(-WIDTH / 2, WIDTH / 2),
            LOC_BR = new Translation2d(-WIDTH / 2, -WIDTH / 2);

    private final TorqueSwerveModuleKraken fl, fr, bl, br;

    public TorqueSwerveSpeeds inputSpeeds;
    public final SwerveDriveKinematics kinematics;
    private SwerveModuleState[] swerveStates;
    private final PPHolonomicDriveController driveController;
    private double slowStartTimestamp;
    private Pose2d alignPoseOverride;

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

        driveController = new PPHolonomicDriveController(
                new PIDConstants(2.5, 0, 0),
                new PIDConstants(2 * Math.PI, 0, 0),
                getMaxPathingVelocity(), getRadius());
    }

    @Override
    public final void initialize(final TorqueMode mode) {
        setInputSpeeds(new TorqueSwerveSpeeds());
        if (mode.isTeleop()) {
            setState(State.FIELD_RELATIVE);
        }

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

    @Override
    public final void update(final TorqueMode mode) {
        Debug.log("Drivebase State", desiredState.toString());
        Debug.log("Robot Velocity", inputSpeeds.getVelocityMagnitude());
        Debug.log("Is Aligned", isAligned());
        Debug.log("In Zone", perception.getCurrentZone() != null);
        Logger.recordOutput("Gyro Angle", perception.getHeading());

        if (perception.getCurrentZone() != null) {
            final TorqueFieldZone zone = perception.getCurrentZone();
            final Pose2d tagPose = Field.AprilTagList.values()[zone.getID() - 1].pose;
            final Pose2d currentPose = perception.getPose();

            final double dx = tagPose.getX() - currentPose.getX();
            final double dy = tagPose.getY() - currentPose.getY();

            final double forward = dx * Math.cos(tagPose.getRotation().getRadians()) + dy * Math.sin(tagPose.getRotation().getRadians());
            final double right = -dx * Math.sin(tagPose.getRotation().getRadians()) + dy * Math.cos(tagPose.getRotation().getRadians());

            Debug.log("AprilTag Forward Offset", -forward + "");
            Debug.log("AprilTag Right Offset", -right + "");
        } else {
            Debug.log("AprilTag Forward Offset", "None");
            Debug.log("AprilTag Right Offset", "None");
        }

        if (alignPoseOverride != null && wantsState(State.ALIGN)) {
            runAlignment(alignPoseOverride);
        } else if (wantsState(State.ALIGN)) {
            final Pose2d alignPose = perception.getAlignPose();
            Debug.log("Align Target Pose", alignPose == null ? "None" : alignPose.toString());
            if (alignPose != null) {
                runAlignment(alignPose);
            }
        }

        if (wantsState(State.FIELD_RELATIVE) || wantsState(State.ALIGN) || wantsState(State.SLOW) || alignPoseOverride != null) {
            inputSpeeds = inputSpeeds.toFieldRelativeSpeeds(perception.getHeading());
        }
        
        swerveStates = kinematics.toSwerveModuleStates(inputSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveStates, MAX_VELOCITY);

        if (wantsState(State.SLOW)) {
            for (SwerveModuleState state : swerveStates) {
                state.speedMetersPerSecond *= getSlowMultiplier();
            }
        }

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

        Logger.recordOutput("Swerve States", swerveStates);
    }

    @Override
    public void clean(TorqueMode mode) {
        if (mode.isTeleop()) {
            desiredState = desiredState.parent;
        }
    }

    public void runAlignment(final Pose2d pose) {
        if (isAligned()) {
            setInputSpeeds(new TorqueSwerveSpeeds());
            return;
        }

        PathPlannerTrajectory.State state = new PathPlannerTrajectory.State();
        state.positionMeters = pose.getTranslation();
        state.targetHolonomicRotation = pose.getRotation();
        state.constraints = new PathConstraints(1, .25, 2 * Math.PI, 4 * Math.PI);

        inputSpeeds = TorqueSwerveSpeeds.fromChassisSpeeds(
            ChassisSpeeds.fromRobotRelativeSpeeds(
                driveController.calculateRobotRelativeSpeeds(new Pose2d(perception.getPose().getTranslation(), perception.getPose().getRotation()), state),
                perception.getPose().getRotation()
            )
        );
    }

    public boolean isAligned() {
        Pose2d alignPose = perception.getAlignPose();
        final double TRANSLATION_TOLERANCE = .01;
        final double ROTATION_TOLERANCE = 1;

        if (alignPoseOverride != null) alignPose = alignPoseOverride;
        if (alignPose == null) return false;
        final double distance = alignPose.getTranslation().getDistance(perception.getPose().getTranslation());
        final boolean translationAligned = distance < TRANSLATION_TOLERANCE;
        final double rotation = (alignPose.getRotation().getDegrees() - perception.getPose().getRotation().getDegrees() + 360) % 360;
        final boolean rotationAligned = rotation < ROTATION_TOLERANCE;

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

    public boolean isNearAligned() {
        Pose2d alignPose = perception.getAlignPose();
        final double TRANSLATION_TOLERANCE = .05;
        final double ROTATION_TOLERANCE = 2;

        if (alignPoseOverride != null) alignPose = alignPoseOverride;
        if (alignPose == null) return false;
        final double distance = alignPose.getTranslation().getDistance(perception.getPose().getTranslation());
        final boolean translationAligned = distance < TRANSLATION_TOLERANCE;
        final double rotation = (alignPose.getRotation().getDegrees() - perception.getPose().getRotation().getDegrees() + 360) % 360;
        final boolean rotationAligned = rotation < ROTATION_TOLERANCE;

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

    public void resetAlign() {
        driveController.reset(perception.getPose(), inputSpeeds);
    }

    public void startSlowMode() {
        setState(State.SLOW);
        slowStartTimestamp = Timer.getFPGATimestamp();
    }

    public void setAlignPoseOverride(Pose2d alignPoseOverride) {
        this.alignPoseOverride = alignPoseOverride;
    }

    public void setInputSpeeds(TorqueSwerveSpeeds inputSpeeds) {
        this.inputSpeeds = inputSpeeds;
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

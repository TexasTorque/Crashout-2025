package org.texastorque.auto;

import org.texastorque.AlignPose2d.Relation;
import org.texastorque.subsystems.Claw;
import org.texastorque.subsystems.Elevator;

public class ReefSequence {
    public static enum Location {
        LEFT("LFT"),
        CENTER("CTR"),
        RIGHT("RGT"),
        FAR_LEFT("FL"),
        FAR("FF"),
        FAR_RIGHT("FR"),
        CLOSE_LEFT("CL"),
        CLOSE("CC"),
        CLOSE_RIGHT("CR"),
        CORAL_STATION_LEFT("CSL"),
        CORAL_STATION_RIGHT("CSR"),
        PROCESSOR("PSR");

        public final String code;

        Location(final String code) {
            this.code = code;
        }
    }

    public static enum EndAction {
        L1_L(Elevator.State.SCORE_L1, Claw.State.L1_SCORE, Relation.LEFT, Claw.CoralState.SHOOT),
        L2_L(Elevator.State.SCORE_L2, Claw.State.MID_SCORE, Relation.LEFT, Claw.CoralState.SHOOT),
        L3_L(Elevator.State.SCORE_L3, Claw.State.MID_SCORE, Relation.LEFT, Claw.CoralState.SHOOT),
        L4_L(Elevator.State.SCORE_L4, Claw.State.L4_SCORE, Relation.LEFT, Claw.CoralState.SHOOT),
        L1_R(Elevator.State.SCORE_L1, Claw.State.L1_SCORE, Relation.RIGHT, Claw.CoralState.SHOOT),
        L2_R(Elevator.State.SCORE_L2, Claw.State.MID_SCORE, Relation.RIGHT, Claw.CoralState.SHOOT),
        L3_R(Elevator.State.SCORE_L3, Claw.State.MID_SCORE, Relation.RIGHT, Claw.CoralState.SHOOT),
        L4_R(Elevator.State.SCORE_L4, Claw.State.L4_SCORE, Relation.RIGHT, Claw.CoralState.SHOOT),
        CORAL_PICKUP(Elevator.State.CORAL_HP, Claw.State.CORAL_HP, Relation.CENTER, Claw.CoralState.INTAKE),
        ALGAE_EXTRACTION_HIGH(Elevator.State.ALGAE_REMOVAL_HIGH, Claw.State.ALGAE_EXTRACTION, Relation.CENTER, Claw.AlgaeState.INTAKE),
        ALGAE_EXTRACTION_LOW(Elevator.State.ALGAE_REMOVAL_LOW, Claw.State.ALGAE_EXTRACTION, Relation.CENTER, Claw.AlgaeState.INTAKE),
        PROCESSOR(Elevator.State.PROCESSOR, Claw.State.PROCESSOR, Relation.CENTER, Claw.AlgaeState.SHOOT);

        public final Elevator.State elevatorState;
        public final Claw.State clawState;
        public final Relation relation;
        public final Claw.AlgaeState algaeState;
        public final Claw.CoralState coralState;

        EndAction(final Elevator.State elevatorState, final Claw.State clawState, final Relation relation, final Claw.AlgaeState algaeState) {
            this.elevatorState = elevatorState;
            this.clawState = clawState;
            this.relation = relation;
            this.algaeState = algaeState;
            this.coralState = Claw.CoralState.OFF;
        }

        EndAction(final Elevator.State elevatorState, final Claw.State clawState, final Relation relation, final Claw.CoralState coralState) {
            this.elevatorState = elevatorState;
            this.clawState = clawState;
            this.relation = relation;
            this.algaeState = Claw.AlgaeState.OFF;
            this.coralState = coralState;
        }
    }

    private final Location start, end;
    private final EndAction[] actions;

    public ReefSequence(final Location start, final Location end, final EndAction ...actions) {
        this.start = start;
        this.end = end;
        this.actions = actions;
    }

    public EndAction[] getActions() {
        return actions;
    }

    public String getPathName() {
        return start.code + "_" + end.code;
    }
}

package org.texastorque.auto;

import org.texastorque.torquelib.auto.marker.Marker;

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

    private final Location start, end;
    private final Marker[] markers;

    public ReefSequence(final Location start, final Location end, final Marker ...markers) {
        this.start = start;
        this.end = end;
        this.markers = markers;
    }

    public Marker[] getMarkers() {
        return markers;
    }

    public String getPathName() {
        return start.code + "_" + end.code;
    }
}

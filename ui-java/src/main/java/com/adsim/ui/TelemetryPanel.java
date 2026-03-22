package com.adsim.ui;

import com.adsim.model.TelemetryFrame;
import com.adsim.network.ConnectionState;
import javafx.geometry.Insets;
import javafx.scene.control.Label;
import javafx.scene.layout.GridPane;
import javafx.scene.layout.VBox;

import java.util.LinkedHashMap;
import java.util.Map;

public class TelemetryPanel extends VBox {

    private final Label connectionLabel = new Label("DISCONNECTED");

    private final Label simTime    = makeValue();
    private final Label posNorth   = makeValue();
    private final Label posEast    = makeValue();
    private final Label posAlt     = makeValue();
    private final Label velMag     = makeValue();
    private final Label velN       = makeValue();
    private final Label velE       = makeValue();
    private final Label velU       = makeValue();
    private final Label altRate    = makeValue();
    private final Label roll       = makeValue();
    private final Label pitch      = makeValue();
    private final Label yaw        = makeValue();
    private final Label angVelX    = makeValue();
    private final Label angVelY    = makeValue();
    private final Label angVelZ    = makeValue();
    private final Label waypoint   = makeValue();
    private final Label failsafe   = makeValue();
    private final Label missionSt  = makeValue();

    public TelemetryPanel() {
        setSpacing(0);
        setPadding(new Insets(8, 10, 8, 10));
        setPrefWidth(220);
        setMinWidth(200);
        setStyle("-fx-background-color: #161616;");

        connectionLabel.getStyleClass().add("disconnected-indicator");
        connectionLabel.setStyle("-fx-font-size: 11px; -fx-font-weight: bold; -fx-padding: 0 0 8 0;");

        getChildren().add(connectionLabel);
        getChildren().add(sectionGrid("Navigation", new String[]{
                "Sim Time",   "s",
                "North",      "m",
                "East",       "m",
                "Altitude",   "m",
                "Speed",      "m/s",
                "Vel N",      "m/s",
                "Vel E",      "m/s",
                "Vel U",      "m/s",
                "Alt Rate",   "m/s"
        }, new Label[]{simTime, posNorth, posEast, posAlt, velMag, velN, velE, velU, altRate}));

        getChildren().add(sectionGrid("Attitude", new String[]{
                "Roll",       "°",
                "Pitch",      "°",
                "Yaw",        "°",
                "Rate X",     "°/s",
                "Rate Y",     "°/s",
                "Rate Z",     "°/s"
        }, new Label[]{roll, pitch, yaw, angVelX, angVelY, angVelZ}));

        getChildren().add(sectionGrid("System", new String[]{
                "Waypoint",   "",
                "Failsafe",   "",
                "Mission",    ""
        }, new Label[]{waypoint, failsafe, missionSt}));
    }

    private GridPane sectionGrid(String title, String[] labelUnits, Label[] values) {
        GridPane grid = new GridPane();
        grid.setHgap(6);
        grid.setVgap(2);
        grid.setPadding(new Insets(6, 0, 4, 0));

        Label header = new Label(title.toUpperCase());
        header.getStyleClass().add("telem-section");
        grid.add(header, 0, 0, 3, 1);

        for (int i = 0; i < values.length; i++) {
            Label lbl = new Label(labelUnits[i * 2]);
            lbl.getStyleClass().add("telem-label");
            lbl.setMinWidth(64);

            Label unit = new Label(labelUnits[i * 2 + 1]);
            unit.getStyleClass().add("telem-label");
            unit.setMinWidth(20);

            values[i].getStyleClass().add("telem-value");
            values[i].setMinWidth(70);

            grid.add(lbl,      0, i + 1);
            grid.add(values[i], 1, i + 1);
            grid.add(unit,     2, i + 1);
        }
        return grid;
    }

    private static Label makeValue() {
        Label l = new Label("—");
        return l;
    }

    public void update(TelemetryFrame frame, ConnectionState state) {
        switch (state) {
            case CONNECTED -> {
                connectionLabel.setText("CONNECTED");
                connectionLabel.getStyleClass().setAll("connected-indicator");
            }
            case CONNECTING -> {
                connectionLabel.setText("CONNECTING...");
                connectionLabel.getStyleClass().setAll("connecting-indicator");
            }
            default -> {
                connectionLabel.setText("DISCONNECTED");
                connectionLabel.getStyleClass().setAll("disconnected-indicator");
            }
        }

        if (frame == null) return;

        simTime.setText(String.format("%.3f", frame.t));
        posNorth.setText(String.format("%.2f", frame.pos.x));
        posEast.setText(String.format("%.2f", frame.pos.y));
        posAlt.setText(String.format("%.2f", frame.pos.z));

        double speed = Math.sqrt(frame.vel.x * frame.vel.x + frame.vel.y * frame.vel.y + frame.vel.z * frame.vel.z);
        velMag.setText(String.format("%.2f", speed));
        velN.setText(String.format("%.2f", frame.vel.x));
        velE.setText(String.format("%.2f", frame.vel.y));
        velU.setText(String.format("%.2f", frame.vel.z));
        altRate.setText(String.format("%.2f", frame.vel.z));

        double toDeg = 180.0 / Math.PI;
        roll.setText(String.format("%.1f", frame.euler.x * toDeg));
        pitch.setText(String.format("%.1f", frame.euler.y * toDeg));
        yaw.setText(String.format("%.1f", frame.euler.z * toDeg));
        angVelX.setText(String.format("%.1f", frame.angvel.x * toDeg));
        angVelY.setText(String.format("%.1f", frame.angvel.y * toDeg));
        angVelZ.setText(String.format("%.1f", frame.angvel.z * toDeg));

        waypoint.setText((frame.wpIdx + 1) + " / " + frame.wpTotal);
        failsafe.setText(frame.failsafe);
        missionSt.setText(frame.missionComplete ? "Complete" : "Active");
    }
}

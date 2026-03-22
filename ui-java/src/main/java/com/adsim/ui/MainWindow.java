package com.adsim.ui;

import com.adsim.model.TelemetryFrame;
import com.adsim.model.Vec3d;
import com.adsim.network.ConnectionState;
import com.adsim.network.TelemetryClient;
import javafx.animation.AnimationTimer;
import javafx.geometry.Insets;
import javafx.scene.Scene;
import javafx.scene.control.Label;
import javafx.scene.control.SplitPane;
import javafx.scene.control.Tab;
import javafx.scene.control.TabPane;
import javafx.scene.layout.BorderPane;
import javafx.scene.layout.HBox;
import javafx.stage.Stage;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Deque;
import java.util.List;
import java.util.TreeMap;
import java.util.concurrent.atomic.AtomicReference;

public class MainWindow {

    private static final int MAX_HISTORY = 1500;

    private final String host;
    private final int    port;

    private final AtomicReference<TelemetryFrame>   latestFrame      = new AtomicReference<>();
    private final AtomicReference<ConnectionState>  connectionState  = new AtomicReference<>(ConnectionState.DISCONNECTED);

    private final Deque<Vec3d>       positionHistory  = new ArrayDeque<>();
    private final TreeMap<Integer, Vec3d> knownWaypoints = new TreeMap<>();

    private TelemetryFrame lastProcessedFrame = null;

    public MainWindow(Stage stage, String host, int port) {
        this.host = host;
        this.port = port;

        View2D        view2D     = new View2D();
        View3D        view3D     = new View3D();
        TelemetryPanel telemPanel = new TelemetryPanel();

        Tab tab2D = new Tab("2D View", view2D);
        tab2D.setClosable(false);
        Tab tab3D = new Tab("3D View", view3D);
        tab3D.setClosable(false);

        TabPane tabPane = new TabPane(tab2D, tab3D);

        SplitPane splitPane = new SplitPane(tabPane, telemPanel);
        splitPane.setDividerPositions(0.82);
        SplitPane.setResizableWithParent(telemPanel, Boolean.FALSE);

        Label statusLabel = new Label("Disconnected — reconnecting...");
        statusLabel.getStyleClass().add("disconnected-indicator");

        HBox statusBar = new HBox(statusLabel);
        statusBar.getStyleClass().add("status-bar");
        statusBar.setPadding(new Insets(4, 8, 4, 8));

        BorderPane root = new BorderPane();
        root.setCenter(splitPane);
        root.setBottom(statusBar);

        Scene scene = new Scene(root, 1380, 860);
        scene.getStylesheets().add(
                getClass().getResource("/com/adsim/style.css").toExternalForm()
        );

        stage.setTitle("ADSIM Telemetry Monitor");
        stage.setScene(scene);
        stage.show();

        TelemetryClient client = new TelemetryClient(host, port, latestFrame, connectionState);
        Thread clientThread = new Thread(client, "telemetry-client");
        clientThread.setDaemon(true);
        clientThread.start();

        new AnimationTimer() {
            @Override
            public void handle(long now) {
                ConnectionState state = connectionState.get();
                TelemetryFrame frame  = latestFrame.get();

                boolean newData = (frame != null && frame != lastProcessedFrame);

                if (newData) {
                    lastProcessedFrame = frame;

                    Vec3d posCopy = frame.pos.copy();
                    positionHistory.addLast(posCopy);
                    while (positionHistory.size() > MAX_HISTORY) {
                        positionHistory.pollFirst();
                    }

                    knownWaypoints.put(frame.wpIdx, frame.wpTarget.copy());

                    List<Vec3d> histSnapshot = new ArrayList<>(positionHistory);
                    TreeMap<Integer, Vec3d> wpSnapshot = new TreeMap<>(knownWaypoints);

                    view2D.update(frame, histSnapshot, wpSnapshot);
                    view3D.update(frame, histSnapshot, wpSnapshot);
                    telemPanel.update(frame, state);

                    updateStatusBar(statusLabel, state);
                } else {
                    boolean stateChanged = updateStatusBar(statusLabel, state);
                    if (stateChanged) {
                        telemPanel.update(frame, state);
                    }
                }
            }

            private ConnectionState lastStatusState = null;

            private boolean updateStatusBar(Label label, ConnectionState state) {
                if (state == lastStatusState) return false;
                lastStatusState = state;
                switch (state) {
                    case CONNECTED -> {
                        label.setText("Connected to " + host + ":" + port);
                        label.getStyleClass().setAll("connected-indicator");
                    }
                    case CONNECTING -> {
                        label.setText("Connecting to " + host + ":" + port + "...");
                        label.getStyleClass().setAll("connecting-indicator");
                    }
                    default -> {
                        label.setText("Disconnected — reconnecting...");
                        label.getStyleClass().setAll("disconnected-indicator");
                    }
                }
                return true;
            }
        }.start();
    }
}

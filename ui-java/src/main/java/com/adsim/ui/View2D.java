package com.adsim.ui;

import com.adsim.model.TelemetryFrame;
import com.adsim.model.Vec3d;
import javafx.scene.canvas.Canvas;
import javafx.scene.canvas.GraphicsContext;
import javafx.scene.layout.Pane;
import javafx.scene.paint.Color;
import javafx.scene.text.Font;

import java.util.List;
import java.util.Map;

public class View2D extends Pane {

    private static final Color BG         = Color.web("#1a1a1a");
    private static final Color GRID       = Color.web("#252525");
    private static final Color TRAIL      = Color.web("#3d6070");
    private static final Color DRONE      = Color.web("#c8c8c8");
    private static final Color WP_CUR     = Color.web("#c8a040");
    private static final Color WP_OTHER   = Color.web("#706030");
    private static final Color TEXT       = Color.web("#888888");

    private final Canvas canvas = new Canvas();

    private double scale = 6.0;
    private double panX  = 0.0;
    private double panY  = 0.0;

    private double dragStartX;
    private double dragStartY;
    private double dragStartPanX;
    private double dragStartPanY;

    private TelemetryFrame lastFrame;
    private List<Vec3d> lastHistory;
    private Map<Integer, Vec3d> lastWaypoints;

    public View2D() {
        getChildren().add(canvas);
        canvas.widthProperty().bind(widthProperty());
        canvas.heightProperty().bind(heightProperty());

        widthProperty().addListener((obs, o, n) -> redraw());
        heightProperty().addListener((obs, o, n) -> redraw());

        setOnScroll(e -> {
            double factor = e.getDeltaY() > 0 ? 1.1 : (1.0 / 1.1);
            scale = Math.max(0.5, Math.min(scale * factor, 200.0));
            redraw();
        });

        setOnMousePressed(e -> {
            dragStartX    = e.getX();
            dragStartY    = e.getY();
            dragStartPanX = panX;
            dragStartPanY = panY;
        });

        setOnMouseDragged(e -> {
            double dx = e.getX() - dragStartX;
            double dy = e.getY() - dragStartY;
            panX = dragStartPanX - dx / scale;
            panY = dragStartPanY + dy / scale;
            redraw();
        });
    }

    public void update(TelemetryFrame frame, List<Vec3d> history, Map<Integer, Vec3d> knownWaypoints) {
        lastFrame     = frame;
        lastHistory   = history;
        lastWaypoints = knownWaypoints;
        redraw();
    }

    private double toScreenX(double worldX, double worldY, double w) {
        return w / 2.0 + (worldY - panY) * scale;
    }

    private double toScreenY(double worldX, double worldY, double h) {
        return h / 2.0 - (worldX - panX) * scale;
    }

    private void redraw() {
        double w = canvas.getWidth();
        double h = canvas.getHeight();
        if (w <= 0 || h <= 0) return;

        GraphicsContext gc = canvas.getGraphicsContext2D();
        gc.setFill(BG);
        gc.fillRect(0, 0, w, h);

        drawGrid(gc, w, h);
        if (lastWaypoints != null && lastFrame != null) drawWaypoints(gc, w, h);
        if (lastHistory != null) drawTrail(gc, w, h);
        if (lastFrame != null) drawDrone(gc, w, h);
    }

    private void drawGrid(GraphicsContext gc, double w, double h) {
        gc.setStroke(GRID);
        gc.setLineWidth(1.0);

        double gridSpacing = chooseGridSpacing();
        double left   = panX - w / (2.0 * scale);
        double right  = panX + w / (2.0 * scale);
        double bottom = panY - h / (2.0 * scale);
        double top    = panY + h / (2.0 * scale);

        double startX = Math.floor(left  / gridSpacing) * gridSpacing;
        double startY = Math.floor(bottom / gridSpacing) * gridSpacing;

        for (double wx = startX; wx <= right; wx += gridSpacing) {
            double sx = toScreenX(wx, 0, w);
            gc.strokeLine(sx, 0, sx, h);
        }
        for (double wy = startY; wy <= top; wy += gridSpacing) {
            double sy = toScreenY(0, wy, h);
            gc.strokeLine(0, sy, w, sy);
        }

        gc.setStroke(GRID.brighter());
        double ox = toScreenX(0, 0, w);
        double oy = toScreenY(0, 0, h);
        gc.strokeLine(ox, 0, ox, h);
        gc.strokeLine(0, oy, w, oy);

        gc.setFill(TEXT);
        gc.setFont(Font.font("Monospace", 10));
        gc.fillText("N", toScreenX(20, 0, w) - 4, toScreenY(20, 0, h) + 4);
        gc.fillText("E", toScreenX(0, 20, w) - 4, toScreenY(0, 20, h) + 4);
    }

    private double chooseGridSpacing() {
        double worldWidth = canvas.getWidth() / scale;
        if      (worldWidth < 20)   return 1;
        else if (worldWidth < 100)  return 5;
        else if (worldWidth < 500)  return 20;
        else if (worldWidth < 2000) return 100;
        else                        return 500;
    }

    private void drawWaypoints(GraphicsContext gc, double w, double h) {
        double r = 6.0;
        for (Map.Entry<Integer, Vec3d> entry : lastWaypoints.entrySet()) {
            int idx   = entry.getKey();
            Vec3d wp  = entry.getValue();
            double sx = toScreenX(wp.x, wp.y, w);
            double sy = toScreenY(wp.x, wp.y, h);

            boolean isCurrent = (idx == lastFrame.wpIdx);
            gc.setStroke(isCurrent ? WP_CUR : WP_OTHER);
            gc.setLineWidth(isCurrent ? 1.5 : 1.0);
            gc.strokeOval(sx - r, sy - r, r * 2, r * 2);

            gc.setFill(isCurrent ? WP_CUR : WP_OTHER);
            gc.setFont(Font.font("Monospace", 10));
            gc.fillText(String.valueOf(idx), sx + r + 2, sy + 4);
        }

        if (lastWaypoints.size() > 1) {
            gc.setStroke(WP_OTHER.deriveColor(0, 1, 1, 0.5));
            gc.setLineWidth(1.0);
            Integer[] keys = lastWaypoints.keySet().stream().sorted().toArray(Integer[]::new);
            for (int i = 0; i < keys.length - 1; i++) {
                Vec3d a = lastWaypoints.get(keys[i]);
                Vec3d b = lastWaypoints.get(keys[i + 1]);
                gc.strokeLine(
                        toScreenX(a.x, a.y, w), toScreenY(a.x, a.y, h),
                        toScreenX(b.x, b.y, w), toScreenY(b.x, b.y, h)
                );
            }
        }
    }

    private void drawTrail(GraphicsContext gc, double w, double h) {
        if (lastHistory.isEmpty()) return;
        gc.setStroke(TRAIL);
        gc.setLineWidth(1.5);
        gc.beginPath();
        boolean first = true;
        for (Vec3d p : lastHistory) {
            double sx = toScreenX(p.x, p.y, w);
            double sy = toScreenY(p.x, p.y, h);
            if (first) { gc.moveTo(sx, sy); first = false; }
            else        gc.lineTo(sx, sy);
        }
        gc.stroke();
    }

    private void drawDrone(GraphicsContext gc, double w, double h) {
        double sx = toScreenX(lastFrame.pos.x, lastFrame.pos.y, w);
        double sy = toScreenY(lastFrame.pos.x, lastFrame.pos.y, h);

        gc.setFill(DRONE);
        double r = 5.0;
        gc.fillOval(sx - r, sy - r, r * 2, r * 2);

        double yaw = lastFrame.euler.z;
        double arrowLen = 14.0;
        double ax = sx + Math.sin(yaw) * arrowLen;
        double ay = sy - Math.cos(yaw) * arrowLen;
        gc.setStroke(DRONE);
        gc.setLineWidth(2.0);
        gc.strokeLine(sx, sy, ax, ay);
    }
}

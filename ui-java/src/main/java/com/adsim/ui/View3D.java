package com.adsim.ui;

import com.adsim.model.TelemetryFrame;
import com.adsim.model.Vec3d;
import javafx.scene.*;
import javafx.scene.layout.Pane;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.Box;
import javafx.scene.shape.Cylinder;
import javafx.scene.shape.Sphere;
import javafx.scene.transform.Affine;
import javafx.scene.transform.Rotate;
import javafx.scene.transform.Translate;

import java.util.ArrayDeque;
import java.util.Deque;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class View3D extends Pane {

    private static final int MAX_TRAIL_BOXES = 400;

    private final SubScene subScene;
    private final Group    root3D  = new Group();
    private final Group    droneGroup   = new Group();
    private final Group    trailGroup   = new Group();
    private final Group    waypointGroup = new Group();
    private final PerspectiveCamera camera = new PerspectiveCamera(true);

    private double camAzimuth   = -45.0;
    private double camElevation =  30.0;
    private double camDistance  =  80.0;

    private double dragStartX;
    private double dragStartY;
    private double dragStartAz;
    private double dragStartEl;

    private final Affine droneAffine = new Affine();

    private final PhongMaterial trailMat  = new PhongMaterial(Color.web("#3d5f75"));
    private final PhongMaterial droneMat  = new PhongMaterial(Color.web("#787878"));
    private final PhongMaterial wpMat     = new PhongMaterial(Color.web("#706030"));
    private final PhongMaterial wpCurMat  = new PhongMaterial(Color.web("#c8a040"));
    private final PhongMaterial postMat   = new PhongMaterial(Color.web("#404040"));

    private final Deque<Box> trailBoxes = new ArrayDeque<>();
    private final Map<Integer, Group> waypointNodes = new HashMap<>();

    private Vec3d lastDronePos = new Vec3d();

    public View3D() {
        subScene = new SubScene(root3D, 100, 100, true, SceneAntialiasing.BALANCED);
        subScene.setFill(Color.web("#1a1a1a"));
        subScene.widthProperty().bind(widthProperty());
        subScene.heightProperty().bind(heightProperty());

        setupCamera();
        setupLighting();
        setupGround();
        setupAxes();
        setupDroneModel();

        root3D.getChildren().addAll(trailGroup, waypointGroup, droneGroup);
        subScene.setCamera(camera);
        getChildren().add(subScene);

        setOnMousePressed(e -> {
            if (e.isPrimaryButtonDown()) {
                dragStartX  = e.getX();
                dragStartY  = e.getY();
                dragStartAz = camAzimuth;
                dragStartEl = camElevation;
            }
        });

        setOnMouseDragged(e -> {
            if (e.isPrimaryButtonDown()) {
                double dx = e.getX() - dragStartX;
                double dy = e.getY() - dragStartY;
                camAzimuth   = dragStartAz + dx * 0.4;
                camElevation = Math.max(-89, Math.min(89, dragStartEl - dy * 0.4));
                updateCamera();
            }
        });

        setOnScroll(e -> {
            double factor = e.getDeltaY() > 0 ? 0.92 : (1.0 / 0.92);
            camDistance = Math.max(5, Math.min(camDistance * factor, 1000));
            updateCamera();
        });
    }

    private void setupCamera() {
        camera.setNearClip(0.1);
        camera.setFarClip(10000.0);
        camera.setFieldOfView(45);
        updateCamera();
    }

    private void updateCamera() {
        double azRad = Math.toRadians(camAzimuth);
        double elRad = Math.toRadians(camElevation);
        double cx = camDistance * Math.cos(elRad) * Math.sin(azRad);
        double cy = -camDistance * Math.sin(elRad);
        double cz = camDistance * Math.cos(elRad) * Math.cos(azRad);

        camera.getTransforms().setAll(
                new Translate(lastDronePos.y + cx, -lastDronePos.z + cy, lastDronePos.x + cz),
                new Rotate(-camElevation, new javafx.geometry.Point3D(1, 0, 0)),
                new Rotate(camAzimuth,   new javafx.geometry.Point3D(0, 1, 0))
        );
    }

    private void setupLighting() {
        AmbientLight ambient = new AmbientLight(Color.web("#404040"));
        PointLight point = new PointLight(Color.web("#d0d0d0"));
        point.setTranslateX(-30);
        point.setTranslateY(-50);
        point.setTranslateZ(-30);
        root3D.getChildren().addAll(ambient, point);
    }

    private void setupGround() {
        Box ground = new Box(2000, 0.1, 2000);
        PhongMaterial mat = new PhongMaterial(Color.web("#202020"));
        ground.setMaterial(mat);
        ground.setTranslateY(0);
        root3D.getChildren().add(ground);
    }

    private void setupAxes() {
        double len  = 5.0;
        double thick = 0.08;

        Box xAxis = new Box(len, thick, thick);
        xAxis.setTranslateX(len / 2);
        xAxis.setMaterial(new PhongMaterial(Color.web("#903030")));

        Box yAxis = new Box(thick, len, thick);
        yAxis.setTranslateY(-len / 2);
        yAxis.setMaterial(new PhongMaterial(Color.web("#309030")));

        Box zAxis = new Box(thick, thick, len);
        zAxis.setTranslateZ(len / 2);
        zAxis.setMaterial(new PhongMaterial(Color.web("#304090")));

        root3D.getChildren().addAll(xAxis, yAxis, zAxis);
    }

    private void setupDroneModel() {
        double armLen   = 1.4;
        double armThick = 0.12;
        double bodySize = 0.25;

        Box armX = new Box(armLen * 2, armThick, armThick);
        Box armZ = new Box(armThick, armThick, armLen * 2);
        Box body = new Box(bodySize, bodySize * 0.6, bodySize);

        armX.setMaterial(droneMat);
        armZ.setMaterial(droneMat);
        body.setMaterial(droneMat);

        droneGroup.getTransforms().add(droneAffine);
        droneGroup.getChildren().addAll(armX, armZ, body);
    }

    private Vec3d toJfx(Vec3d sim) {
        return new Vec3d(sim.y, -sim.z, sim.x);
    }

    public void update(TelemetryFrame frame, List<Vec3d> history, Map<Integer, Vec3d> knownWaypoints) {
        Vec3d jfxPos = toJfx(frame.pos);
        lastDronePos = frame.pos;

        applyDroneTransform(frame, jfxPos);
        updateTrail(history);
        updateWaypoints(knownWaypoints, frame.wpIdx);
        updateCamera();
    }

    private void applyDroneTransform(TelemetryFrame frame, Vec3d jfxPos) {
        double w = frame.att.w;
        double qx = frame.att.x;
        double qy = frame.att.y;
        double qz = frame.att.z;

        double r00 = 1 - 2*(qy*qy + qz*qz);
        double r01 = 2*(qx*qy - qz*w);
        double r02 = 2*(qx*qz + qy*w);
        double r10 = 2*(qx*qy + qz*w);
        double r11 = 1 - 2*(qx*qx + qz*qz);
        double r12 = 2*(qy*qz - qx*w);
        double r20 = 2*(qx*qz - qy*w);
        double r21 = 2*(qy*qz + qx*w);
        double r22 = 1 - 2*(qx*qx + qy*qy);

        droneAffine.setToIdentity();
        droneAffine.setMxx(r11);   droneAffine.setMxy(-r12);  droneAffine.setMxz(r10);
        droneAffine.setMyx(-r21);  droneAffine.setMyy(r22);   droneAffine.setMyz(-r20);
        droneAffine.setMzx(r01);   droneAffine.setMzy(-r02);  droneAffine.setMzz(r00);
        droneAffine.setTx(jfxPos.x);
        droneAffine.setTy(jfxPos.y);
        droneAffine.setTz(jfxPos.z);
    }

    private void updateTrail(List<Vec3d> history) {
        int target = history.size();
        int current = trailBoxes.size();

        if (target > current) {
            List<Vec3d> newPoints = history.subList(current, target);
            for (Vec3d p : newPoints) {
                if (trailBoxes.size() >= MAX_TRAIL_BOXES) {
                    Box old = trailBoxes.pollFirst();
                    trailGroup.getChildren().remove(old);
                }
                Vec3d jp = toJfx(p);
                Box b = new Box(0.15, 0.15, 0.15);
                b.setMaterial(trailMat);
                b.setTranslateX(jp.x);
                b.setTranslateY(jp.y);
                b.setTranslateZ(jp.z);
                trailBoxes.addLast(b);
                trailGroup.getChildren().add(b);
            }
        } else if (target < current) {
            trailGroup.getChildren().clear();
            trailBoxes.clear();
        }
    }

    private void updateWaypoints(Map<Integer, Vec3d> knownWaypoints, int currentIdx) {
        for (Map.Entry<Integer, Vec3d> entry : knownWaypoints.entrySet()) {
            int idx   = entry.getKey();
            Vec3d sim = entry.getValue();
            Vec3d jp  = toJfx(sim);

            Group existing = waypointNodes.get(idx);
            if (existing == null) {
                Group g = buildWaypointMarker(jp, idx == currentIdx);
                waypointNodes.put(idx, g);
                waypointGroup.getChildren().add(g);
            } else {
                PhongMaterial mat = idx == currentIdx ? wpCurMat : wpMat;
                for (Node n : existing.getChildren()) {
                    if (n instanceof Sphere s) s.setMaterial(mat);
                }
            }
        }
    }

    private Group buildWaypointMarker(Vec3d jp, boolean isCurrent) {
        Group g = new Group();

        double postH = Math.max(0.2, jp.y < 0 ? -jp.y : 0.2);
        Cylinder post = new Cylinder(0.05, postH);
        post.setMaterial(postMat);
        post.setTranslateX(jp.x);
        post.setTranslateY(jp.y + postH / 2.0);
        post.setTranslateZ(jp.z);

        Sphere sphere = new Sphere(0.4);
        sphere.setMaterial(isCurrent ? wpCurMat : wpMat);
        sphere.setTranslateX(jp.x);
        sphere.setTranslateY(jp.y);
        sphere.setTranslateZ(jp.z);

        g.getChildren().addAll(post, sphere);
        return g;
    }
}

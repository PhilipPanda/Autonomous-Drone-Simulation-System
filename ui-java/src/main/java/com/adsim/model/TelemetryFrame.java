package com.adsim.model;

import com.fasterxml.jackson.annotation.JsonIgnoreProperties;
import com.fasterxml.jackson.annotation.JsonProperty;

@JsonIgnoreProperties(ignoreUnknown = true)
public class TelemetryFrame {

    @JsonProperty("t")
    public double t = 0.0;

    @JsonProperty("pos")
    public Vec3d pos = new Vec3d();

    @JsonProperty("vel")
    public Vec3d vel = new Vec3d();

    @JsonProperty("accel")
    public Vec3d accel = new Vec3d();

    @JsonProperty("att")
    public Attitude att = new Attitude();

    @JsonProperty("euler")
    public Vec3d euler = new Vec3d();

    @JsonProperty("angvel")
    public Vec3d angvel = new Vec3d();

    @JsonProperty("wp_idx")
    public int wpIdx = 0;

    @JsonProperty("wp_total")
    public int wpTotal = 0;

    @JsonProperty("wp_target")
    public Vec3d wpTarget = new Vec3d();

    @JsonProperty("mission_complete")
    public boolean missionComplete = false;

    @JsonProperty("failsafe")
    public String failsafe = "nominal";

    @JsonIgnoreProperties(ignoreUnknown = true)
    public static class Attitude {

        @JsonProperty("w")
        public double w = 1.0;

        @JsonProperty("x")
        public double x = 0.0;

        @JsonProperty("y")
        public double y = 0.0;

        @JsonProperty("z")
        public double z = 0.0;

        public Attitude() {
        }
    }
}

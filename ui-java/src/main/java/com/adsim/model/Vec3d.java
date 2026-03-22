package com.adsim.model;

import com.fasterxml.jackson.annotation.JsonProperty;

public class Vec3d {

    @JsonProperty("x")
    public double x = 0.0;

    @JsonProperty("y")
    public double y = 0.0;

    @JsonProperty("z")
    public double z = 0.0;

    public Vec3d() {
    }

    public Vec3d(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public Vec3d copy() {
        return new Vec3d(x, y, z);
    }
}

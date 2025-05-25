package com.example.tracking;
import org.osmdroid.util.GeoPoint;
import java.util.ArrayList;
import java.util.List;

public class Obstacle {
    private double latitude;
    private double longitude;
    private double radius;
    private String type; // может быть, например, "barrier"

    // Конструкторы
    public Obstacle(double latitude, double longitude, double radius, String type) {
        this.latitude = latitude;
        this.longitude = longitude;
        this.radius = radius;
        this.type = type;
    }

    // Геттеры и сеттеры
    public double getLatitude() {
        return latitude;
    }

    public void setLatitude(double latitude) {
        this.latitude = latitude;
    }

    public double getLongitude() {
        return longitude;
    }

    public void setLongitude(double longitude) {
        this.longitude = longitude;
    }

    public String getType() {
        return type;
    }

    public void setType(String type) {
        this.type = type;
    }

    public  void setRadius(double radius) { this.radius = radius; }
    public  double getRadius() { return  radius; }
}
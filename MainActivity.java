package com.example.tracking;


import com.example.tracking.Node;
import android.graphics.Color;
import android.os.Bundle;
import androidx.appcompat.app.AppCompatActivity;
import org.osmdroid.config.Configuration;
import org.osmdroid.views.MapView;

import org.osmdroid.views.overlay.Polyline;
import org.osmdroid.util.GeoPoint;
import org.osmdroid.views.overlay.Marker;
import org.osmdroid.views.overlay.ScaleBarOverlay;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import android.util.Log;

public class MainActivity extends AppCompatActivity {


    private MapView mapView;

    private static final String TAG = "OSM";



    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        Configuration.getInstance().load(this, getSharedPreferences("osmdroid", MODE_PRIVATE));
        setContentView(R.layout.activity_main);

        mapView = findViewById(R.id.map);
        mapView.setTileSource(org.osmdroid.tileprovider.tilesource.TileSourceFactory.MAPNIK);
        mapView.setMultiTouchControls(true);

        mapView.getController().setZoom(20.0);
        mapView.getController().setCenter(new GeoPoint(61.7849, 34.3469));

        ScaleBarOverlay scaleBarOverlay = new ScaleBarOverlay(mapView);
        mapView.getOverlays().add(scaleBarOverlay);

// Пример координат
        double startLat = 61.7846; // latitude start
        double startLon = 34.3465; // longitude start
        double endLat = 61.7849; // latitude end
        double endLon = 34.347; // longitude end
        double postendLat = 61.7849;
        double postendLon = 34.3473;
        //1 четверть
//        double startLat = 61.7849; // latitude start
//        double startLon = 34.3469; // longitude start
//        double endLat = 61.7855; // latitude end
//        double endLon = 34.3475; // longitude end
//        double postendLat = 61.7860;
//        double postendLon = 34.3470;
//        //2 четверть
//        double startLat = 61.7849; // latitude start
//        double startLon = 34.3469; // longitude start
//        double endLat = 61.7855; // latitude end
//        double endLon = 34.3465; // longitude end
//        double postendLat = 61.7855;
//        double postendLon = 34.3467;
//        //3 четверть
//        double startLat = 61.7849; // latitude start
//        double startLon = 34.3469; // longitude start
//        double endLat = 61.7845; // latitude end
//        double endLon = 34.3465; // longitude end
//        double postendLat = 61.7845;
//        double postendLon = 34.3472;
//////        //4 четверть
//        double startLat = 61.7849; // latitude start
//        double startLon = 34.3469; // longitude start
//        double endLat = 61.7845; // latitude end
//        double endLon = 34.3475; // longitude end
//        double postendLat = 61.7847;
//        double postendLon = 34.3475;
        //Горизонт
//        double startLat = 61.7847; // latitude start
//        double startLon = 34.3460; // longitude start
//        double endLat = 61.7847; // latitude end
//        double endLon = 34.3477;
//        double postendLat = 61.7845;
//        double postendLon = 34.3475;
        //Вертикаль
//        double startLat = 61.7849; // latitude start
//        double startLon = 34.3469; // longitude start
//        double endLat = 61.7845; // latitude end
//        double endLon = 34.3469;
//        double postendLat = 61.7845;
//        double postendLon = 34.3471;


        GeoPoint start = new GeoPoint(startLat, startLon);
        GeoPoint finish = new GeoPoint(endLat, endLon);
        GeoPoint afterFinish = new GeoPoint(postendLat, postendLon);

        double angle = 0;
        // Азимут движения
        double radius = 10; // Радиус поворота
        List<List<GeoPoint>> obstacles = new ArrayList<>();
        List<GeoPoint> obstacle = new ArrayList<>();
        obstacle.add(new GeoPoint(61.7853, 34.3470));
        obstacle.add(new GeoPoint(61.7852, 34.3472));
        obstacle.add(new GeoPoint(61.7850, 34.3473));
        obstacle.add(new GeoPoint(61.7851, 34.3471));
        List<GeoPoint> lastSide = new ArrayList<>();
        lastSide.add(obstacle.get(0));
        lastSide.add(obstacle.get(obstacle.size() - 1));
        drawPath(obstacle, 0xFFFF0000);
        drawPath(lastSide, 0xFFFF0000);
        obstacles.add(obstacle);
        List<GeoPoint> triangle = new ArrayList<>();
        triangle.add(new GeoPoint(61.7856, 34.3473));
        triangle.add(new GeoPoint(61.7852, 34.34745));
        triangle.add(new GeoPoint(61.7854, 34.3476));
        lastSide = new ArrayList<>();
        lastSide.add(triangle.get(0));
        lastSide.add(triangle.get(triangle.size() - 1));
        drawPath(triangle, 0xFFFF0000);
        drawPath(lastSide, 0xFFFF0000);
        obstacles.add(triangle);
        obstacle = new ArrayList<>();
        obstacle.add(new GeoPoint(61.7846, 34.3473));
        obstacle.add(new GeoPoint(61.7846, 34.3467));
        obstacle.add(new GeoPoint(61.7848, 34.3467));
        obstacle.add(new GeoPoint(61.7848, 34.3473));


        lastSide = new ArrayList<>();
        lastSide.add(obstacle.get(0));
        lastSide.add(obstacle.get(obstacle.size() - 1));
        drawPath(obstacle, 0xFFFF0000);
        drawPath(lastSide, 0xFFFF0000);
        obstacles.add(obstacle);

//        addMarker(start);
//        addMarker(finish);
        Node startNode = new Node(start);
        Node finishNode = new Node(finish);
        DeviationCorrector corrector = new DeviationCorrector(angle, start, finish, afterFinish, obstacles, mapView);
        long startTime = System.currentTimeMillis();
        List<GeoPoint> pathPoints = corrector.findPath();
        long endTime = System.currentTimeMillis();
        long timeElapsed = endTime - startTime;
        Log.d(TAG, (double)timeElapsed / 1000 + " sec");

//        for (GeoPoint point : pathPoints)
//            addMarker(point);
        drawPath(pathPoints, 0xFF008000);

    }

    private void drawPath(List<GeoPoint> path, int colour) {
        Polyline polyline = new Polyline();
        polyline.setPoints(path);
        polyline.setColor(colour); // Цвет линии (синий)
        polyline.setWidth(5.0f); // Ширина линии
        mapView.getOverlays().add(polyline);
        mapView.invalidate(); // Обновляем отображение карты
    }

    private void addMarker(GeoPoint point) {
        Marker marker = new Marker(mapView);
        marker.setPosition(point);
        marker.setPanToView(true);
        marker.setAnchor(Marker.ANCHOR_CENTER, Marker.ANCHOR_BOTTOM);
        mapView.getOverlays().add(marker);
        mapView.invalidate();
    }

    @Override
    protected void onPause() {
        super.onPause();
        mapView.onPause(); // Приостановить отображение карты
    }

    @Override
    protected void onResume() {
        super.onResume();
        mapView.onResume(); // Возобновить отображение карты
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        mapView.onDetach();
    }
}
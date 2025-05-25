package com.example.tracking;

import android.util.Log;

import org.osmdroid.util.GeoPoint;
import org.osmdroid.views.MapView;
import org.osmdroid.views.overlay.Marker;
import org.osmdroid.views.overlay.Polygon;
import org.osmdroid.views.overlay.Polyline;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

public class DeviationCorrector {
    private static final String TAG = "OSM";
    final double Meters_per_lon_degree = 111134.855555555555555555556;
    final double Meters_per_lat_degree = 111319.444444444444444444444;
    final double Radius = 6371000;
    private List<List<GeoPoint>> obstacles = new ArrayList<>();
    List<Node> gridPoints = new ArrayList<>();
    HashMap<GeoPoint, List<Node>> gridPointsNeighbors = new HashMap<GeoPoint, List<Node>>();
    private final double startAngle;
    private final double finishAngle;
    private Node start;
    private Node preStart;
    private Node afterStart;
    private Node finish;
    private Node preFinish;
    private Node afterFinish;
    GeoPoint botLeft;
    GeoPoint topRight;
    private MapView mapView;
    private int extraBounds = 5;
    private double gridSize = 5;
    private double r = 8.3;
    private int turnAccuracy = 25;
    private final double criticalTurn = Math.PI / 4;
    private List<Double> directions = new ArrayList<>();

    private long startTime;
    private double findPathTimer = 0;
    private double drawGridTimer = 0;
    private double writeNeighborsTimer = 0;
    private double getNeighborsTimer = 0;
    private double createRotationTimer = 0;

    public DeviationCorrector(double angle, GeoPoint start, GeoPoint finish, GeoPoint afterFinish, List<List<GeoPoint>> obstacles, MapView mapView) {
        this.startAngle = angle;
        this.start = new Node(start);
        this.preStart = new Node(new GeoPoint(start.getLatitude() - Math.sin(Math.toRadians(angle)) * metersToLatDeg(gridSize), start.getLongitude() - Math.cos(Math.toRadians(angle)) * metersToLonDeg(gridSize, start.getLatitude())));
        this.finish = new Node(finish);
        this.afterFinish = new Node(afterFinish);
        this.obstacles = obstacles;
        this.mapView = mapView;
        this.finishAngle = Math.atan2(afterFinish.getLatitude() - finish.getLatitude(), afterFinish.getLongitude() - finish.getLongitude());
        startTime = System.currentTimeMillis();
    }

    public List<GeoPoint> findPath() {



        preFinish = new Node(new GeoPoint(finish.position.getLatitude() - 2 * Math.sin(finishAngle) * metersToLatDeg(r),
                finish.position.getLongitude() - 2 * Math.cos(finishAngle) * metersToLonDeg(r, finish.position.getLatitude())));
        afterStart = new Node(new GeoPoint(start.position.getLatitude() + 2 * Math.sin(startAngle) * metersToLatDeg(r),
                start.position.getLongitude() + 2 * Math.cos(startAngle) * metersToLonDeg(r, start.position.getLatitude())));


        List<GeoPoint> finishTrajectory = new ArrayList<>();
        finishTrajectory.add(finish.position);
        finishTrajectory.add(afterFinish.position);
        drawPath(finishTrajectory, 0xFFfc03f4);
        List<GeoPoint> startTrajectory = new ArrayList<>();
        startTrajectory.add(start.position);
        startTrajectory.add(preStart.position);
        drawPath(startTrajectory, 0xFFfc03f4);

//
        addMarker(start.position);
        addMarker(finish.position);

        double startLat = start.position.getLatitude();
        double startLon = start.position.getLongitude();
        double endLat = finish.position.getLatitude();
        double endLon = finish.position.getLongitude();

        if (startLat < endLat) {
            if (startLon < endLon) {
                botLeft = new GeoPoint(startLat, startLon);
                topRight = new GeoPoint(endLat, endLon);
            } else {
                botLeft = new GeoPoint(startLat, endLon);
                topRight = new GeoPoint(endLat, startLon);
            }
        } else {
            if (startLon < endLon) {
                botLeft = new GeoPoint(endLat, startLon);
                topRight = new GeoPoint(startLat, endLon);
            } else {
                botLeft = new GeoPoint(endLat, endLon);
                topRight = new GeoPoint(startLat, startLon);
            }
        }

        drawGrid();

        List<Node> openList = new ArrayList<>();
        Set<Node> closedList = new HashSet<>();
        openList.add(start);

        if (start.hCost == Double.POSITIVE_INFINITY ||
                start.gCost == Double.POSITIVE_INFINITY ||
                finish.hCost == Double.POSITIVE_INFINITY ||
                finish.gCost == Double.POSITIVE_INFINITY)
            return Collections.emptyList();

        while (!openList.isEmpty()) {
            long endTime = System.currentTimeMillis();
            long timeElapsed = endTime - startTime;
            if((double)timeElapsed / 1000 >= 20)
                return Collections.emptyList();
            Node currentNode = getLowestFCostNode(openList);
            openList.remove(currentNode);
            closedList.add(currentNode);
            double angleFromParent = 0;
            if (currentNode.parent != null) {
                angleFromParent = Math.atan2(currentNode.position.getLatitude() - currentNode.parent.position.getLatitude(),
                        currentNode.position.getLongitude() - currentNode.parent.position.getLongitude());
            }

            if (equalWithTolerance(currentNode.position, finish.position)
                    || equalWithTolerance(currentNode.position, afterFinish.position)
                    || isNodeOnSegment(currentNode, finish.position, afterFinish.position)) {
                Log.d(TAG, "Draw grid get " + drawGridTimer + " sec");
                Log.d(TAG, "writeNeighbors get " + writeNeighborsTimer + " sec");
                Log.d(TAG, "getNeighbors get " + getNeighborsTimer + " sec");
                Log.d(TAG, "createRotation get " + createRotationTimer + " sec");
                return reconstructPath(currentNode, true);
            }

            for (Node neighbor : getNeighbors(currentNode)) {
                if (closedList.contains(neighbor) ||
                        neighbor.gCost == Double.POSITIVE_INFINITY ||
                        neighbor.hCost == Double.POSITIVE_INFINITY) continue;

                if (isPathInsideObstacle(currentNode.position, neighbor.position))
                    continue;

                double dx = neighbor.position.getLongitude() - currentNode.position.getLongitude();
                double dy = neighbor.position.getLatitude() - currentNode.position.getLatitude();
                double angleToNeighbor = Math.atan2(dy, dx);

                if (!isDirectionAllowed(angleToNeighbor, angleFromParent, Math.PI / 2) && currentNode.parent != null)
                    continue;

                if (equalWithTolerance(currentNode.position, start.position)) {
                    if (!isDirectionAllowed(angleToNeighbor, Math.toRadians(startAngle), Math.PI / 4))
                        continue;
                }

                if (equalWithTolerance(neighbor.position, finish.position)
                        || equalWithTolerance(neighbor.position, afterFinish.position)
                        || isNodeOnSegment(neighbor, finish.position, afterFinish.position)) {
                    if (!isDirectionAllowed(angleToNeighbor, finishAngle, Math.PI / 4))
                        continue;
                }

                double tempGCost = currentNode.gCost + getLength(currentNode.position, neighbor.position);
                if (tempGCost < neighbor.gCost || !openList.contains(neighbor)) {
                    neighbor.gCost = tempGCost;
                    neighbor.hCost = heuristic(neighbor, finish);
                    neighbor.parent = currentNode;

                    if (!openList.contains(neighbor)
                            && reconstructPath(neighbor, false) != null
                    ) {
                        openList.add(neighbor);
                    }
                }
            }


        }

        extraBounds+=1;
        Log.d(TAG, "New attempt..");
        return  findPath();

    }

    private boolean isDirectionAllowed(double fromAlpha, double toBeta, double tolerance) {
        double angleDifference = Math.abs(toBeta - fromAlpha);
        // Нормализуем разницу углов в диапазон [0, 2*PI]
        if (angleDifference > Math.PI) {
            angleDifference = 2 * Math.PI - angleDifference;
        }
        return angleDifference < tolerance;
    }

    private Node getLowestFCostNode(List<Node> openList) {
        double infty = Double.POSITIVE_INFINITY;
        Node infNode = new Node(new GeoPoint(0,0));
        infNode.hCost = infty;
        Node lowestFCostNode = infNode;
        for (Node node : openList) {
            if (node.getFCost() < lowestFCostNode.getFCost() ||
                    (node.getFCost() == lowestFCostNode.getFCost() &&
                            node.gCost < lowestFCostNode.gCost)) {
                lowestFCostNode = node;
            }
        }
        return lowestFCostNode;
    }

    private List<GeoPoint> nodesToPoints(List<Node> nodesPath) {
        List<GeoPoint> pointsPath = new ArrayList<>();
        for (Node node : nodesPath) {
            pointsPath.add(node.position);
        }

        return pointsPath;
    }

    private int compareWithTolerance(double a, double b) {
        a = a * Math.pow(10, 6);
        b = b * Math.pow(10, 6);
        double c = Math.round(a);
        double d = Math.round(b);
        return Double.compare(c, d);
    }

    private boolean equalWithTolerance(Object a, Object b) {
        double tolerance = 0.000000001;
        if (a instanceof GeoPoint && b instanceof GeoPoint) {
            GeoPoint c = (GeoPoint)a;
            GeoPoint d = (GeoPoint)b;
            if (Math.abs(c.getLatitude() - d.getLatitude()) <= tolerance && Math.abs(c.getLongitude() - d.getLongitude()) <= tolerance)
                return true;
        } else if (a instanceof Double && b instanceof Double) {
            double c = (Double)a;
            double d = (Double)b;
            if (Math.abs(c - d) <= tolerance)
                return true;
        }

        return false;
    }

    private List<Node> getNeighbors(Node node) {
        long startTime = System.currentTimeMillis();
        for (GeoPoint key : gridPointsNeighbors.keySet()) {
//            Log.d(TAG, "" + getLength(key, node.position));
            if (equalWithTolerance(key, node.position) || getLength(key, node.position) < gridSize / 2)
                return gridPointsNeighbors.get(key);
        }
        long endTime = System.currentTimeMillis();
        long timeElapsed = endTime - startTime;
        getNeighborsTimer += (double)timeElapsed / 1000;

        return new ArrayList<>();
    }

    private double heuristic(Node a, Node b) {
        return getLength(a.position, b.position);
    }

    private List<GeoPoint> reconstructPath(Node node, boolean isFinish) {
        List<Node> path = new ArrayList<>();

        while (node != null) {
            path.add(node);
            node = node.parent;
        }

//        return smoothPath(nodesToPoints(path));
        List<GeoPoint> pointPath = smoothPath(nodesToPoints(path));

        Collections.reverse(pointPath);

        if (isFinish) {
            drawPath(pointPath, 0xFF07fae6);
//            for (GeoPoint point : pointPath)
////                Log.d(TAG, point.getLongitude() + " " + point.getLatitude());
        }

//        return pointPath;
        return createRotation(pointPath, isFinish);
    }

    private List<GeoPoint> smoothPath(List<GeoPoint> path) {
        for (int i = 1; i < path.size() - 4; i++) {
            GeoPoint a = path.get(i);
            GeoPoint b = path.get(i + 1);
            GeoPoint c = path.get(i + 2);
            GeoPoint d = path.get(i + 3);
            GeoPoint d1 = compareGeoPoints(a, b);
            GeoPoint d2 = compareGeoPoints(c,d);
            if (equalWithTolerance(d1, d2) && !isPathInsideObstacle(a, d)) {
                path.remove(i + 1);
                path.remove(i + 2);
            }

        }
        for (int i = 0; i < path.size() - 2; i++) {
            GeoPoint a = path.get(i);
            GeoPoint b = path.get(i + 1);
            GeoPoint c = path.get(i + 2);
            GeoPoint d1 = compareGeoPoints(a, b);
            GeoPoint d2 = compareGeoPoints(b,c);
            if (equalWithTolerance(d1, d2) && !isPathInsideObstacle(a, c)) {
                path.remove(i + 1);
            }
            if (equalWithTolerance(d1, new GeoPoint(-d2.getLatitude(), -d2.getLongitude())))
                path.remove(i);
        }
        return path;
    }

    private List<GeoPoint> createRotation(List<GeoPoint> path, boolean isFinish) {
        long startTime = System.currentTimeMillis();
        if (path.size() < 3) return path;
        List<GeoPoint> chargePath = new ArrayList<>();

//        path.add(0, preStart.position);
//        path.add(path.size() - 1, afterFinish.position);
        chargePath.add(path.get(0));
        GeoPoint a;
        GeoPoint b;
        GeoPoint c;
        GeoPoint lastPathPoint = path.get(0);
        for (int i = 1; i < path.size() - 1; i++) {
            a = path.get(i-1);
            b = path.get(i);
            c = path.get(i + 1);
            if (chargePath.size() > 1)
                lastPathPoint = chargePath.get(chargePath.size() - 1);
            List<GeoPoint> arcPoints = getArc(a, b, c);
            if (arcPoints == null) {
//                Log.d(TAG, "Returning null on " + i);
                return null;
            }
            GeoPoint first = arcPoints.get(0);
            GeoPoint second = arcPoints.get(1);
            double dx1 = first.getLongitude() - lastPathPoint.getLongitude();
            double dy1 = first.getLatitude() - lastPathPoint.getLatitude();
            double dx2 = second.getLongitude() - first.getLongitude();
            double dy2 = second.getLatitude() - first.getLatitude();
            double alpha = Math.atan2(dy1, dx1);
            double beta = Math.atan2(dy2, dx2);
            if (Math.abs(alpha + Math.PI - (beta + Math.PI)) > criticalTurn && !equalWithTolerance(lastPathPoint, first)) {
//                Log.d(TAG, "Returning null on " + i);
                return null;
            }
//            GeoPoint lastArcPoint = arcPoints.get(arcPoints.size() - 1);
//            path.set(i, lastArcPoint);
//            dx1 = b.getLongitude() - c.getLongitude();
//            dy1 = b.getLatitude() - c.getLatitude();
//            dx2 = lastArcPoint.getLongitude() - c.getLongitude();
//            dy2 = lastArcPoint.getLatitude() - c.getLatitude();
//            alpha = Math.atan2(dy1, dx1);
//            beta = Math.atan2(dy2, dx2);
//            if (!equalWithTolerance(alpha, beta))
//                return null;

            chargePath.addAll(arcPoints);
        }
        lastPathPoint = path.get(path.size()-1);
        GeoPoint lastChargePathPoint = chargePath.get(chargePath.size()-1);
        if (!equalWithTolerance(lastPathPoint, lastChargePathPoint))
            chargePath.add(lastPathPoint);
        long endTime = System.currentTimeMillis();
        long timeElapsed = endTime - startTime;
        createRotationTimer += (double)timeElapsed / 1000;
        return chargePath;
    }

    private List<GeoPoint> getArc (GeoPoint a, GeoPoint b, GeoPoint c) {
        double ry = metersToLatDeg(r);
        double rx = metersToLonDeg(r, b.getLatitude());
        GeoPoint[] tangentsAndCenter = calcCenterAndTangents(a, b, c, rx * rx, ry * ry);
        if (tangentsAndCenter == null)
            return null;
        GeoPoint t1 = tangentsAndCenter[0];
        GeoPoint t2 = tangentsAndCenter[1];
        GeoPoint center = tangentsAndCenter[2];
        int or = (int) center.getAltitude();
        if (Double.isInfinite(t1.getLatitude()) || Double.isInfinite(t1.getLongitude())
                || Double.isNaN(t2.getLatitude()) || Double.isNaN(t2.getLongitude())
                || Double.isNaN(center.getLatitude()) || Double.isNaN(center.getLongitude()))
            Log.d(TAG, "NaN on" + b.getLatitude() + " " + b.getLongitude());
        return getArcPoints(t1, t2, or, center, rx, ry);
    }

    private List<GeoPoint> getArcPoints (GeoPoint p1, GeoPoint p2, int or, GeoPoint center, double a, double b) {
//        addMarker(p1);
//        addMarker(p2);
        GeoPoint lastPoint = p1;
        List<GeoPoint> arcPoints = new ArrayList<>();

        double alpha = Math.atan2(center.getLatitude()-p1.getLatitude(), center.getLongitude() - p1.getLongitude());
        double beta = Math.atan2(center.getLatitude()-p2.getLatitude(), center.getLongitude() - p2.getLongitude());
        double x0 = center.getLongitude() + a * Math.cos(alpha);
        double y0 = center.getLatitude() + b * Math.sin(alpha);
        if (getLength(new GeoPoint(y0, x0), p1) > (a + b) / 2)
            alpha += Math.PI;
        double x1 = center.getLongitude() + a * Math.cos(beta);
        double y1 = center.getLatitude() + b * Math.sin(beta);
        if (getLength(new GeoPoint(y1, x1), p2) > (a + b) / 2)
            beta += Math.PI;

        if (or == 1) {
            while (beta > alpha)
                beta -= 2 * Math.PI;
        } else {
            while (alpha > beta)
                beta += 2 * Math.PI;
        }
        while (beta - alpha > 2 * Math.PI) beta -= 2 * Math.PI;
        while (beta - alpha < - 2 * Math.PI) beta += 2 * Math.PI;
        double step = (beta - alpha) / turnAccuracy;
        double curSinSign;
        for (int i = 0; i <= turnAccuracy; i++) {
            double angle = alpha + step * i;
            double sinSign = Math.sin(angle) > 0 ? 1 : -1;
            double t = Math.atan(a * Math.tan(angle) / b);
            int f = 1;
            if (Math.floor((Math.abs(angle) + Math.PI / 2) / Math.PI) % 2 == 1)
                f = -1;
            double x = center.getLongitude() + f * a * Math.cos(t);
            double y = center.getLatitude() + f * b * Math.sin(t);
            curSinSign = f * Math.sin(t) > 0 ? 1 : -1;
            if (isNumberMultipleOf(angle + Math.PI / 2, Math.PI) && sinSign != curSinSign)
                y = center.getLatitude() - f * b * Math.sin(t);
            GeoPoint arcPoint = new GeoPoint(y, x);

            if (isNodeInsideObstacle(new Node(arcPoint)) || isPathInsideObstacle(lastPoint, arcPoint)) {
//                Log.d(TAG, "Returning null cause arc is in obstacle");
                return null;
            }

            lastPoint = arcPoint;
            arcPoints.add(arcPoint);
//            Log.d(TAG, "" + getLength(arcPoint, center));

        }

//        mapView.setMapOrientation(90f);
        return arcPoints;
    }

    private GeoPoint[] calcCenterAndTangents(GeoPoint p1, GeoPoint p2, GeoPoint p3, double a, double b) {
        double alpha = Math.atan2(p1.getLatitude()-p2.getLatitude(), p1.getLongitude() - p2.getLongitude());
        double angle1 = Math.atan2(p2.getLatitude()-p1.getLatitude(), p2.getLongitude() - p1.getLongitude());
        double angle2 = Math.atan2(p2.getLatitude()-p3.getLatitude(), p2.getLongitude() - p3.getLongitude());
        double k1 = Math.tan(angle1);
        double k2 = Math.tan(angle2);
        GeoPoint[] k1Tangents = getTangents(k1, a, b);
        GeoPoint[] k2Tangents = getTangents(k2, a, b);
        GeoPoint[] three = getTargetThree(k1Tangents, k2Tangents, k1, k2, p1, p2, p3);
        if (three == null) {
//            return goBypass(p1, p2, p3);
            return null;
        }
        double dxt1 = three[1].getLongitude() - three[0].getLongitude();
        double dyt1 = three[1].getLatitude() - three[0].getLatitude();
        double dxt2 = three[1].getLongitude() - three[2].getLongitude();
        double dyt2 = three[1].getLatitude() - three[2].getLatitude();
        GeoPoint t1 = new GeoPoint(p2.getLatitude() - dyt1, p2.getLongitude() - dxt1);
        GeoPoint t2 = new GeoPoint(p2.getLatitude() - dyt2, p2.getLongitude() - dxt2);
        GeoPoint center = new GeoPoint(p2.getLatitude() - three[1].getLatitude(), p2.getLongitude() - three[1].getLongitude());
//        addMarker(center);
        double beta = Math.atan2(p1.getLatitude()-t1.getLatitude(), p1.getLongitude() - t1.getLongitude());
//        if (!equalWithTolerance(alpha, beta)) {
//            return goBypass(p1, p2, p3);
//        }
        int or = orientation(p1, p2, p3);
        center.setAltitude(or);

        return new GeoPoint[] {t1, t2, center};
    }

//    private GeoPoint[] goBypass(GeoPoint p1, GeoPoint p2, GeoPoint p3) {
//        int or;
//        List<GeoPoint> centers = calcCenters(p1, p2);
//        GeoPoint center1 = centers.get(0);
//        GeoPoint center2 = centers.get(1);
//        double b = metersToLatDeg(r);
//        double a = metersToLonDeg(r, p1.getLatitude());
//        int or1 = orientation(center1, p1, p2);
//        int or2 = orientation(center2, p1, p2);
//        GeoPoint tan1 = calcTangent(p3, center1, or1);
//        GeoPoint tan2 = calcTangent(p3, center2, or2);
//        if (tan1 != null && getArcPoints(p1, tan1, or1, center1, a, b) != null) {
//            or = orientation(p1, p2, center1);
//            center1.setAltitude(or);
//            return new GeoPoint[] {p1, tan1, center1};
//        } else if (tan2 != null && getArcPoints(p1, tan2, or2, center2, a, b) != null) {
//            or = orientation(p1, p2, center2);
//            center2.setAltitude(or);
//            return new GeoPoint[] {p1, tan2, center2};
//        }
////        Log.d(TAG, "Returning null cause cant create bypass");
//        return null;
//    }

//    private GeoPoint calcTangent(GeoPoint point, GeoPoint center, int or) {
//        GeoPoint tan1;
//        GeoPoint tan2;
//        GeoPoint baseTan;
//        GeoPoint tangent;
//        double b = metersToLatDeg(r);
//        double a = metersToLonDeg(r, center.getLatitude());
//        GeoPoint base = new GeoPoint(point.getLatitude() - center.getLatitude(), point.getLongitude() - center.getLongitude());
//        double c = base.getLongitude();
//        double d = base.getLatitude();
//        if (d * d / (b * b) + c * c / (a * a) < 1) {
////            Log.d(TAG, "Returning null cause point is inside ellipse");
//            return null;
//        }
//        double tanX1 = (a * a * (b * b * c + d * Math.sqrt(a * a * d * d - a * a * b * b + b * b * c * c))) / (b * b * c * c + a * a * d * d);
//        double tanX2 = (a * a * (b * b * c - d * Math.sqrt(a * a * d * d - a * a * b * b + b * b * c * c))) / (b * b * c * c + a * a * d * d);
//        double tanY11;
//        if (equalWithTolerance(b * b, b * b * tanX1 * tanX1 / (a * a)))
//            tanY11 = 0;
//        else
//            tanY11 = Math.sqrt(b * b - b * b * tanX1 * tanX1 / (a * a));
//        double tanY12 = -tanY11;
//        double k1 = (tanY11 - d) / (tanX1 - c);
//        if (isZeroDiscriminant(tanX1, tanY11, k1, a * a, b * b)) {
//            tan1 = new GeoPoint(tanY11, tanX1);
//        } else
//            tan1 = new GeoPoint(tanY12, tanX1);
//        double tanY21 = Math.sqrt(b * b - b * b * tanX2 * tanX2 / (a * a));
//        double tanY22 = -tanY11;
//        double k2 = (tanY21 - d) / (tanX2 - c);
//        if (isZeroDiscriminant(tanX2, tanY21, k2, a * a, b * b)) {
//            tan2 = new GeoPoint(tanY21, tanX2);
//        } else
//            tan2 = new GeoPoint(tanY22, tanX2);
//        int curOr = orientation(new GeoPoint(0,0), tan1, base);
//        baseTan = curOr == or ? tan1 : tan2;
//        tangent = new GeoPoint(center.getLatitude() + baseTan.getLatitude(), center.getLongitude() + baseTan.getLongitude());
//        return tangent;
//    }

    //    private List<GeoPoint> calcCenters(GeoPoint p1, GeoPoint p2) {
//        List<GeoPoint> centers = new ArrayList<>();
//        double b = metersToLatDeg(r);
//        double a = metersToLonDeg(r, p1.getLatitude());
//        double k = Math.tan(Math.atan2(p2.getLatitude()-p1.getLatitude(), p2.getLongitude() - p1.getLongitude()));
//        GeoPoint[] tangents = getTangents(k, a * a, b * b);
//        GeoPoint center1 = new GeoPoint(p1.getLatitude() + tangents[0].getLatitude(), p1.getLongitude() + tangents[0].getLongitude());
//        GeoPoint center2 = new GeoPoint(p1.getLatitude() + tangents[1].getLatitude(), p1.getLongitude() + tangents[1].getLongitude());
//        centers.add(center1);
//        centers.add(center2);
//        return centers;
//    }

    private GeoPoint[] getTargetThree (GeoPoint[] t1, GeoPoint[] t2, double k1, double k2, GeoPoint p1, GeoPoint p2, GeoPoint p3) {
        double alpha = Math.atan(k1);
        double beta = Math.atan(k2);
        GeoPoint v1 = new GeoPoint(compareWithTolerance(p2.getLatitude(), p1.getLatitude()), compareWithTolerance(p2.getLongitude(),p1.getLongitude()));
        GeoPoint v2 = new GeoPoint(compareWithTolerance(p2.getLatitude(), p3.getLatitude()),compareWithTolerance(p2.getLongitude(),p3.getLongitude()));
        double curAlpha;
        double curBeta;
        for (GeoPoint point1 : t1) {
            for (GeoPoint point2 : t2) {
                GeoPoint intersection = getIntersection(point1, k1, point2, k2);
                curAlpha = Math.atan2(intersection.getLatitude()-point1.getLatitude(), intersection.getLongitude() - point1.getLongitude());
                curBeta = Math.atan2(intersection.getLatitude()-point2.getLatitude(), intersection.getLongitude() - point2.getLongitude());
                double delta1 = curAlpha - alpha;
                double delta2 = curBeta - beta;
                GeoPoint curV1 = new GeoPoint(compareWithTolerance(intersection.getLatitude(), point1.getLatitude()),
                        compareWithTolerance(intersection.getLongitude(),point1.getLongitude()));
                GeoPoint curV2 = new GeoPoint(compareWithTolerance(intersection.getLatitude(), point2.getLatitude()),
                        compareWithTolerance(intersection.getLongitude(),point2.getLongitude()));
                if (isNumberMultipleOf(delta1, Math.PI) && isNumberMultipleOf(delta2, Math.PI)  && equalWithTolerance(curV1, v1) && equalWithTolerance(curV2, v2))
                    return new GeoPoint[] {point1, intersection, point2};
                delta1 = curAlpha - beta;
                delta2 = curBeta - alpha;
                if (isNumberMultipleOf(delta1, Math.PI) && isNumberMultipleOf(delta2, Math.PI) && equalWithTolerance(curV1, v2) && equalWithTolerance(curV2, v1))
                    return new GeoPoint[] {point2, intersection, point1};
            }
        }

//        Log.d(TAG, "Returning null attempt to find tangents and center");
//        Log.d(TAG, "p1:" + p1.getLatitude() + " " + p1.getLongitude());
//        Log.d(TAG, "p2:" + p2.getLatitude() + " " + p2.getLongitude());
//        Log.d(TAG, "p3:" + p3.getLatitude() + " " + p3.getLongitude());
        return null;
    }

    private boolean isNumberMultipleOf(double number, double multiplier) {
        return Math.abs(number) - (multiplier * Math.round(Math.abs(number) / multiplier)) < 0.1;
    }

    private GeoPoint getIntersection (GeoPoint p1, double k1, GeoPoint p2, double k2) {
        double x1 = p1.getLongitude();
        double y1 = p1.getLatitude();
        double x2 = p2.getLongitude();
        double y2 = p2.getLatitude();
        double x = (k1 * x1 - y1 - k2 * x2 + y2) / (k1 - k2);
        double y = x * k1 - k1 * x1 + y1;
        return  new GeoPoint(y, x);
    }

    private GeoPoint[] getTangents(double k, double a, double b) {
        double x1 = Math.sqrt(k * k * a * a / (k * k * a + b));
        double x2 = -x1;
        double y1;
        if (equalWithTolerance(b, b * x1 * x1 / a)) {
            y1 = 0;
        } else {
            y1 = Math.sqrt(b - b * x1 * x1 / a);
        }
        double y2 = -y1;
        GeoPoint p1;
        GeoPoint p2;
        if (isZeroDiscriminant(x1, y1, k, a, b)) {
            p1 = new GeoPoint(y1, x1);
            p2 = new GeoPoint(y2, x2);
        } else {
            p1 = new GeoPoint(y2, x1);
            p2 = new GeoPoint(y1, x2);
        }

        return new GeoPoint[] {p1, p2};
    }

    private boolean isZeroDiscriminant(double x, double y, double k, double a, double b) {
        double A = k * k + b / a;
        double B = -2 * k * k * x + 2 * k * y;
        double C = k * k * x * x - 2 * k * y * x + y * y - b;
        return equalWithTolerance(B * B, 4 * A * C);
    }

    private GeoPoint compareGeoPoints(GeoPoint a, GeoPoint b) {
        double dLat = compareWithTolerance(a.getLatitude(), b.getLatitude());
        double dLon = compareWithTolerance(a.getLongitude(), b.getLongitude());
        return new GeoPoint(dLat, dLon);
    }

    private double getLength(GeoPoint aDeg, GeoPoint bDeg) {
        GeoPoint a = new GeoPoint(aDeg.getLatitude() * Math.PI / 180, aDeg.getLongitude() * Math.PI / 180);
        GeoPoint b = new GeoPoint(bDeg.getLatitude() * Math.PI / 180, bDeg.getLongitude() * Math.PI / 180);
        return 2 * Radius * Math.asin(Math.sqrt(Math.pow(Math.sin((b.getLongitude() - a.getLongitude()) / 2), 2) +
                Math.cos(a.getLongitude()) * Math.cos(b.getLongitude()) *
                        Math.pow(Math.sin((b.getLatitude() - a.getLatitude()) / 2), 2)));
    }

    private boolean isNodeInsideObstacle(Node node) {
        for (List<GeoPoint> obstacle : obstacles) {
            int intersectCount = 0;
            for (int i = 0; i < obstacle.size(); i++) {
                GeoPoint v1 = obstacle.get(i);
                GeoPoint v2 = obstacle.get((i + 1) % obstacle.size());

                if (isNodeOnSegment(node, v1, v2)) {
//                    addMarker(node.position);
                    return true;
                }

                if (isRayIntersecting(node, v1, v2)) {
                    intersectCount++;
                }
            }

            if (intersectCount % 2 == 1) {
//                addMarker(node.position);
                return true;
            }

        }
        return false;
    }

    private boolean isRayIntersecting(Node node, GeoPoint v1, GeoPoint v2) {
        if (v1.getLatitude() > v2.getLatitude()) {
            GeoPoint temp = v1;
            v1 = v2;
            v2 = temp;
        }

        return node.position.getLatitude() >= v1.getLatitude() && node.position.getLatitude() <= v2.getLatitude() &&
                (node.position.getLatitude() - v1.getLatitude()) * (v2.getLongitude() - v1.getLongitude()) /
                        (v2.getLatitude() - v1.getLatitude()) + v1.getLongitude() > node.position.getLongitude();
    }

    private boolean isNodeOnSegment(Node node, GeoPoint v1, GeoPoint v2) {
        double minX = Math.min(v1.getLongitude(), v2.getLongitude());
        double maxX = Math.max(v1.getLongitude(), v2.getLongitude());
        double minY = Math.min(v1.getLatitude(), v2.getLatitude());
        double maxY = Math.max(v1.getLatitude(), v2.getLatitude());

        // Проверяем, находится ли узел в пределах границ отрезка
        if (node.position.getLongitude() > minX && node.position.getLongitude() < maxX &&
                node.position.getLatitude() > minY && node.position.getLatitude() < maxY || equalWithTolerance(node.position, v1) || equalWithTolerance(node.position, v2)) {

            // Проверяем, является ли отрезок вертикальным или горизонтальным

            if (equalWithTolerance(v1.getLongitude(), v2.getLongitude())) {
                // Вертикальный отрезок
                return equalWithTolerance(node.position.getLongitude(), v1.getLongitude());
            } else if (equalWithTolerance(v1.getLatitude(), v2.getLatitude())) { // Горизонтальный отрезок
                return equalWithTolerance(node.position.getLatitude(), v1.getLatitude());
            } else { // Обычный отрезок
                // Проверка принадлежности с использованием уравнения прямой

                return equalWithTolerance((node.position.getLongitude() - v1.getLongitude()) * (v2.getLatitude() - v1.getLatitude()),
                        (node.position.getLatitude() - v1.getLatitude()) * (v2.getLongitude() - v1.getLongitude()));
            }
        }

        return false; // За пределами отрезка
    }

    private boolean isPathInsideObstacle(GeoPoint a, GeoPoint b) {
        for (List<GeoPoint> obstacle : obstacles) {
            for (int i = 0; i < obstacle.size(); i++) {
                GeoPoint c = obstacle.get(i);
                GeoPoint d = obstacle.get((i + 1) % obstacle.size());
                int o1 = orientation(a, b, c);
                int o2 = orientation(a, b, d);
                int o3 = orientation(c, d, a);
                int o4 = orientation(c, d, b);
                // Общий случай
                if (o1 != o2 && o3 != o4) {
                    return true;
                }
                //Специальные случаи
                // a, b и c коллинеарны и c находится на отрезке ab
                if (o1 == 0 && isNodeOnSegment(new Node(c), a, b)) return true;
                // a, b и d коллинеарны и d находится на отрезке ab
                if (o2 == 0 && isNodeOnSegment(new Node(d), a, b)) return true;
                // c, d и a коллинеарны и a находится на отрезке cd
                if (o3 == 0 && isNodeOnSegment(new Node(a), c, d)) return true;
                // c, d и b коллинеарны и b находится на отрезке cd
                if (o4 == 0 && isNodeOnSegment(new Node(b), c, d)) return true;
            }
        }
        return false; // не пересекаются
    }
    // метод для определения направления обхода трех точек
    private int orientation(GeoPoint a, GeoPoint b, GeoPoint c) {
        double val = (b.getLatitude() - a.getLatitude()) * (c.getLongitude() - b.getLongitude()) -
                (b.getLongitude() - a.getLongitude()) * (c.getLatitude() - b.getLatitude());
        if (equalWithTolerance(val, 0)) return 0;// точки коллинеарны
        return (val > 0) ? 1 : 2;// 1 - по часовой стрелке; 2 - против часовой;
    }

    private void drawGrid() {
        long startTime = System.currentTimeMillis();

        double dLat = topRight.getLatitude() - botLeft.getLatitude();
        double dLon = topRight.getLongitude() - botLeft.getLongitude();
        double latSquareSize;
        double lonSquareSize;
        double startLat;
        double startLon;
        double endLat;
        double endLon;
        double latLimiter = metersToLatDeg(gridSize);
        if (dLat == 0) {
            startLat = botLeft.getLatitude() - latLimiter;
            latSquareSize = latLimiter;
            endLat = botLeft.getLatitude() + latLimiter;
        } else {
            latSquareSize = dLat / Math.round(dLat / latLimiter);
            startLat = botLeft.getLatitude();
            endLat = topRight.getLatitude();
        }

        for (double lat = startLat - latSquareSize * extraBounds;
             lat < endLat + latSquareSize * extraBounds; lat += latSquareSize) {
            double lonLimiter = metersToLonDeg(gridSize, lat);
            if (dLon == 0) {
                startLon = botLeft.getLongitude() - lonLimiter;
                lonSquareSize = lonLimiter;
                endLon = botLeft.getLongitude() + lonLimiter;
            } else {
                lonSquareSize = dLon / Math.round(dLon / lonLimiter);
                startLon = botLeft.getLongitude();
                endLon = topRight.getLongitude();
            }

            for (double lon = startLon - lonSquareSize * extraBounds;
                 lon < endLon + lonSquareSize * extraBounds; lon += lonSquareSize) {
                List<GeoPoint> square = new ArrayList<>();
                GeoPoint a = new GeoPoint(lat, lon);
                GeoPoint b = new GeoPoint(lat + latSquareSize, lon);
                GeoPoint c = new GeoPoint(lat + latSquareSize, lon + lonSquareSize);
                GeoPoint d = new GeoPoint(lat, lon + lonSquareSize);
                square.add(a);
                square.add(b);
                square.add(c);
                square.add(d);
                Polygon polygon = new Polygon();
                polygon.addPoint(a);
                polygon.addPoint(b);
                polygon.addPoint(c);
                polygon.addPoint(d);
                polygon.setStrokeColor(0xFF0000CD);
                polygon.setStrokeWidth(1);
                mapView.getOverlays().add(polygon);
                mapView.invalidate();

                for (int i = 0; i < square.size(); i++) {
                    if (!gridPoints.contains(new Node(square.get(i)))) {
                        gridPoints.add(new Node(square.get(i)));
                    }
                }

            }
        }

        long endTime = System.currentTimeMillis();
        long timeElapsed = endTime - startTime;
        drawGridTimer += (double)timeElapsed / 1000;

        writeNeighbors(dLat, dLon);
    }

    private void writeNeighbors(double dLat, double dLon) {
        long startTime = System.currentTimeMillis();
        double latSquareSize;
        double lonSquareSize;
        boolean notExist;
        for (Node node : gridPoints) {
            List<Node> neighbors = new ArrayList<>();
            if (!gridPointsNeighbors.containsKey(node.position)) {
                double latLimiter = metersToLatDeg(gridSize);
                if (dLat == 0) {
                    latSquareSize = latLimiter;
                } else {
                    latSquareSize = dLat / Math.round(dLat / latLimiter);
                }
                for (double lat = node.position.getLatitude() - latSquareSize; lat <= node.position.getLatitude() + latSquareSize; lat += latSquareSize) {
                    double lonLimiter = metersToLonDeg(gridSize, lat);
                    if (dLon == 0) {
                        lonSquareSize = lonLimiter;
                    } else {
                        lonSquareSize = dLon / Math.round(dLon / lonLimiter);
                    }
                    for (double lon = node.position.getLongitude() - lonSquareSize; lon <= node.position.getLongitude() + lonSquareSize; lon += lonSquareSize) {
                        GeoPoint point = new GeoPoint(lat, lon);
                        Node currentNode = new Node(point);
                        if (equalWithTolerance(currentNode, node)) continue;
                        double dx = node.position.getLongitude() - point.getLongitude();
                        double dy = node.position.getLatitude() - point.getLatitude();
                        double angle = Math.atan2(dy, dx);
                        if (!directions.contains(angle)) {
                            notExist = true;
                            for (double direction : directions) {
                                if (equalWithTolerance(direction, angle))
                                    notExist = false;
                            }
                            if (notExist)
                                directions.add(angle);
                        }

                        if (isNodeInsideObstacle(currentNode) || isPathInsideObstacle(currentNode.position, node.position)) {
                            currentNode.hCost = Double.POSITIVE_INFINITY;
                            currentNode.gCost = Double.POSITIVE_INFINITY;
//                            addMarker(currentNode.position);
                        }  else {
                            currentNode.hCost = getLength(point, finish.position);
                            currentNode.gCost = getLength(point, start.position);
                        }
                        neighbors.add(currentNode);
                    }
                }

                gridPointsNeighbors.put(node.position, neighbors);
            }
        }

        long endTime = System.currentTimeMillis();
        long timeElapsed = endTime - startTime;
        writeNeighborsTimer+= (double)timeElapsed / 1000;

    }

    private double metersToLatDeg(double meters) {
        return meters / Meters_per_lat_degree;
    }

    private double metersToLonDeg(double meters, double lattitude) {
        return meters / (Meters_per_lon_degree * Math.cos(lattitude * Math.PI / 180));
    }

    private void addMarker(GeoPoint point) {
        Marker marker = new Marker(mapView);
        marker.setPosition(point);
        marker.setPanToView(true);
        marker.setAnchor(Marker.ANCHOR_CENTER, Marker.ANCHOR_BOTTOM);
        mapView.getOverlays().add(marker);
        mapView.invalidate();
    }

    private void drawPath(List<GeoPoint> path, int colour) {
        Polyline polyline = new Polyline();
        polyline.setPoints(path);
        polyline.setColor(colour); // Цвет линии (синий)
        polyline.setWidth(5.0f); // Ширина линии
        mapView.getOverlays().add(polyline);
        mapView.invalidate(); // Обновляем отображение карты
    }
}

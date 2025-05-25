package com.example.tracking;

import org.osmdroid.util.GeoPoint;

public class Node {
    public final GeoPoint position;
    public Node parent;
    public double gCost; // cost from start to node
    public double hCost; // heuristic cost to goal
    public double fCost; // total cost

    public Node(GeoPoint position) {
        this.position = position;
    }

    public double getFCost() {
        return gCost + hCost;
    }

    @Override
    public boolean equals(Object obj) {
        if (this == obj) {
            return true; // Проверка на ссылку на самих себя
        }
        if (obj instanceof Node) {
            Node other = (Node) obj;
            // Используем метод equals у GeoPoint для сравнения координат
            return this.position.equals(other.position);
        }
        return false;
    }

    @Override
    public int hashCode() {
        return position.hashCode(); // Генерация хэш-кода на основе координат GeoPoint
    }
}

/*
 * Copyright (c) 2021. Meng Lian and Yushen Zhang
 *
 *   CONFIDENTIAL
 *   __________________
 *
 *   Meng Lian and Yushen Zhang
 *   All Rights Reserved.
 *
 *   NOTICE:  All information contained herein is, and remains
 *   the property of Meng Lian, Yushen Zhang and the Technical University
 *   of Munich, if applies. The intellectual and technical concepts contained
 *   herein are proprietary to Meng Lian, Yushen Zhang and/or the Technical University
 *   of Munich and may be covered by European and Foreign Patents,
 *   patents in process, and are protected by trade secret or copyright law.
 *   Dissemination of this information or reproduction of this material
 *   is strictly forbidden unless prior written permission is obtained
 *   from Meng Lian and Yushen Zhang.
 */

package shapes;


import java.util.Objects;

public class Point {
    protected double x;
    protected double y;
    protected double length_I;
    protected double length_S;

    public Slave slave;

    /**
     * @param x X coordinate of the point
     * @param y Y coordinate of the point
     */
    public Point(double x, double y) {
        this();
        this.x = x;
        this.y = y;
    }

    public Point() {
        this.length_I = 0;
    }


    public Slave getSlave() {
        return slave;
    }

    public void setSlave(Slave slave) {
        this.slave = slave;
    }

    public void setLength_I(double length_I) {
        this.length_I = length_I;
    }

    public double getLength_S() {
        return length_S;
    }

    public void setLength_S(double length_S) {
        this.length_S = length_S;
    }

    public double getLength_I() {
        return length_I;
    }

    public double getX_exact() {
        return x;
    }

    public void setX(double x) {
        this.x = x;
    }

    public double getY_exact() {
        return y;
    }

    public void setY(double y) {
        this.y = y;
    }


    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        Point point = (Point) o;
        return Double.compare(point.x, x) == 0 && Double.compare(point.y, y) == 0;
    }

    @Override
    public int hashCode() {
        return Objects.hash(x, y);
    }

    @Override
    public String toString() {

        return "Point{" +
                "x=" + x +
                ", y=" + y +
                ", length_I=" + length_I +
                ", length_S=" + length_S +
                ", slave=" + slave +
                '}';

    }
}

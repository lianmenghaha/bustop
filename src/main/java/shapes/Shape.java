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

public abstract class Shape {

    protected double x_ct;
    protected double y_ct;

    protected Point center;//center point

    protected String name;

    public ShapeType type = ShapeType.UNKNOWN;



    public Shape() {

    }

    public double getX_ct_exact() {
        return x_ct;
    }

    public int getX_ct(){
        return (int)Math.round(x_ct);
    }

    public void setX_ct(int x_ct) {
        this.x_ct = x_ct;
    }

    public double getY_ct_exact() {
        return y_ct;
    }

    public int getY_ct(){
        return (int)Math.round(y_ct);
    }

    public Point getCenter() {
        return center;
    }

    public void setCenter(Point center) {
        this.center = center;
    }


    public String getName() {
        return name;
    }

    public void setName(String name) {
        this.name = name;
    }

    @Override
    public String toString() {
        return "Shape{" +
                "x_ct=" + x_ct +
                ", y_ct=" + y_ct +
                ", name='" + name + '\'' +
                '}';
    }
}

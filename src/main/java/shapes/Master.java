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

public class Master extends Shape {
    public ShapeType type = ShapeType.Master;


    /**
     * @param x_ct center x
     * @param y_ct center y
     */
    public Master(double x_ct, double y_ct) {
        super();
        this.x_ct = x_ct;
        this.y_ct = y_ct;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        Master cmp = (Master) o;
        return Double.compare(cmp.x_ct, x_ct) == 0 && Double.compare(cmp.y_ct, y_ct) == 0;
    }

    @Override
    public String toString() {
        return "Component{" +
                "type=" + type +
                ", x_ct=" + x_ct +
                ", y_ct" + y_ct +
                ", name='" + name + '\'' +
                '}';
    }
}

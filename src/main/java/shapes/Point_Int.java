package shapes;

import java.util.Objects;

public class Point_Int {
    protected int x;
    protected int y;
    protected int length_I;
    protected int length_S;

    public Slave slave;

    /**
     * @param x X coordinate of the point
     * @param y Y coordinate of the point
     */
    public Point_Int(int x, int y) {
        this();
        this.x = x;
        this.y = y;
    }

    public Point_Int() {
        this.length_I = 0;
    }


    public Slave getSlave() {
        return slave;
    }

    public void setSlave(Slave slave) {
        this.slave = slave;
    }

    public void setLength_I(int length_I) {
        this.length_I = length_I;
    }

    public double getLength_S() {
        return length_S;
    }

    public void setLength_S(int length_S) {
        this.length_S = length_S;
    }

    public double getLength_I() {
        return length_I;
    }

    public double getX() {
        return x;
    }

    public void setX(int x) {
        this.x = x;
    }

    public double getY() {
        return y;
    }

    public void setY(int y) {
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

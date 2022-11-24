package shapes;

import java.util.ArrayList;

/**
 * A representation of a keepout on the pcb.
 */
public class Keepout {
    private String name;

    private int minX, maxX, minY, maxY;
    private int inter_x1, inter_x2, inter_y1, inter_y2;

    private final ArrayList<Keepout> left_os;
    private final ArrayList<Keepout> right_os;
    private final ArrayList<Keepout> above_os;
    private final ArrayList<Keepout> below_os;

    private Keepout left_miny_o;
    private Keepout left_maxy_o;

    private Keepout right_miny_o;
    private Keepout right_maxy_o;

    private Keepout above_minx_o;
    private Keepout above_maxx_o;

    private Keepout below_minx_o;
    private Keepout below_maxx_o;

    public int[] dist;

    public Keepout(int minX, int maxX, int minY, int maxY) {
        this.minX = minX;
        this.minY = minY;
        this.maxX = maxX;
        this.maxY = maxY;
        this.left_os = new ArrayList<>();
        this.right_os = new ArrayList<>();
        this.above_os = new ArrayList<>();
        this.below_os = new ArrayList<>();
    }

    public String getName() {
        return name;
    }

    public void setName(String name) {
        this.name = name;
    }

    public ArrayList<Keepout> getLeft_os() {
        return left_os;
    }

    public void addToLeft_os(Keepout left_o) {
        this.left_os.add(left_o);
    }

    public ArrayList<Keepout> getRight_os() {
        return right_os;
    }

    public void addToRight_os(Keepout right_os) {
        this.right_os.add(right_os);
    }

    public ArrayList<Keepout> getAbove_os() {
        return above_os;
    }

    public void addToAbove_os(Keepout above_os) {
        this.above_os.add(above_os);
    }

    public ArrayList<Keepout> getBelow_os() {
        return below_os;
    }

    public void addToBelow_os(Keepout below_os) {
        this.below_os.add(below_os);
    }

    public Keepout getLeft_maxy_o() {
        return left_maxy_o;
    }

    public void setLeft_maxy_o(Keepout left_maxy_o) {
        this.left_maxy_o = left_maxy_o;
    }

    public Keepout getRight_maxy_o() {
        return right_maxy_o;
    }

    public void setRight_maxy_o(Keepout right_maxy_o) {
        this.right_maxy_o = right_maxy_o;
    }

    public Keepout getAbove_maxx_o() {
        return above_maxx_o;
    }

    public void setAbove_maxx_o(Keepout above_maxx_o) {
        this.above_maxx_o = above_maxx_o;
    }

    public Keepout getBelow_maxx_o() {
        return below_maxx_o;
    }

    public void setBelow_maxx_o(Keepout below_maxx_o) {
        this.below_maxx_o = below_maxx_o;
    }

    public Keepout getLeft_miny_o() {
        return left_miny_o;
    }

    public void setLeft_miny_o(Keepout left_miny_o) {
        this.left_miny_o = left_miny_o;
    }

    public Keepout getRight_miny_o() {
        return right_miny_o;
    }

    public void setRight_miny_o(Keepout right_miny_o) {
        this.right_miny_o = right_miny_o;
    }

    public Keepout getAbove_minx_o() {
        return above_minx_o;
    }

    public void setAbove_minx_o(Keepout above_minx_o) {
        this.above_minx_o = above_minx_o;
    }

    public Keepout getBelow_minx_o() {
        return below_minx_o;
    }

    public void setBelow_minx_o(Keepout below_minx_o) {
        this.below_minx_o = below_minx_o;
    }

    public int getInter_x1() {
        return inter_x1;
    }

    public void setInter_x1(int inter_x1) {
        this.inter_x1 = inter_x1;
    }

    public int getInter_x2() {
        return inter_x2;
    }

    public void setInter_x2(int inter_x2) {
        this.inter_x2 = inter_x2;
    }

    public int getInter_y1() {
        return inter_y1;
    }

    public void setInter_y1(int inter_y1) {
        this.inter_y1 = inter_y1;
    }

    public int getInter_y2() {
        return inter_y2;
    }

    public void setInter_y2(int inter_y2) {
        this.inter_y2 = inter_y2;
    }

    public int getMinX() {
        return minX;
    }

    public void setMinX(int minX) {
        this.minX = minX;
    }

    public int getMinY() {
        return minY;
    }

    public void setMinY(int minY) {
        this.minY = minY;
    }

    public int getMaxX() {
        return maxX;
    }

    public void setMaxX(int maxX) {
        this.maxX = maxX;
    }

    public int getMaxY() {
        return maxY;
    }

    public void setMaxY(int maxY) {
        this.maxY = maxY;
    }

    @Override
    public String toString() {
        return "Keepout{" +
                "name='" + name + '\'' +
                ", minX=" + minX +
                ", maxX=" + maxX +
                ", minY=" + minY +
                ", maxY=" + maxY +
                '}';
    }
}
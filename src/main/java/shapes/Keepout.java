package shapes;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

/**
 * A representation of a keepout on the pcb.
 */
public class Keepout {
    private String name;

    private final int minX, maxX, minY, maxY;

    private final ArrayList<Keepout> left_os, right_os, above_os, below_os;

    private final Map<Keepout, int[]> map_oo_dist;

    public Keepout(String name, int minX, int maxX, int minY, int maxY) {
        this.name = name;
        this.minX = minX;
        this.minY = minY;
        this.maxX = maxX;
        this.maxY = maxY;
        this.left_os = new ArrayList<>();
        this.right_os = new ArrayList<>();
        this.above_os = new ArrayList<>();
        this.below_os = new ArrayList<>();
        this.map_oo_dist = new HashMap<>();

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

    public Map<Keepout, int[]> getMap_oo_dist() {
        return map_oo_dist;
    }

    public void addToMap_oo_dist(Keepout o, int[] dist) {
        this.map_oo_dist.put(o, dist);
    }

    public int getMinX() {
        return minX;
    }

    public int getMinY() {
        return minY;
    }

    public int getMaxX() {
        return maxX;
    }

    public int getMaxY() {
        return maxY;
    }

    @Override
    public String toString() {

        ArrayList<String> oLnames = new ArrayList<>();
        if (this.getLeft_os().size() != 0 ){
            for (Keepout oL : this.getLeft_os()){
                oLnames.add(oL.name);
            }
        }
        ArrayList<String> oRnames = new ArrayList<>();
        if (this.getRight_os().size() != 0 ){
            for (Keepout oR : this.getRight_os()){
                oRnames.add(oR.name);
            }
        }
        ArrayList<String> oAnames = new ArrayList<>();
        if (this.getAbove_os().size() != 0 ){
            for (Keepout oA : this.getAbove_os()){
                oAnames.add(oA.name);
            }
        }
        ArrayList<String> oBnames = new ArrayList<>();
        if (this.getBelow_os().size() != 0 ){
            for (Keepout oB : this.getBelow_os()){
                oBnames.add(oB.name);
            }
        }





        return "Keepout{" +
                "name='" + name + '\'' +
                ", minX=" + minX +
                ", maxX=" + maxX +
                ", minY=" + minY +
                ", maxY=" + maxY +
                ", oLnames = " + oLnames +
                ", oRnames = " + oRnames +
                ", oAnames = " + oAnames +
                ", oBnames = " + oBnames +
                '}';
    }
}
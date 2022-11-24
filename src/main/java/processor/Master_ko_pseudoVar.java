package processor;

import shapes.Keepout;
import shapes.Master;

import java.util.HashMap;
import java.util.Map;

public class Master_ko_pseudoVar extends Master {



    /*
     * 0: Left
     * 1: Right
     * 2: Above
     * 3: Below
     */
    public Map<Keepout, int[]> pseudo_bVars; //Left, Right, Above, Below

    /*
     * 0: |ms.x - c^k_1.x|
     * 1: |ms.y - c^k_1.y|
     * 2: |ms.x - c^k_2.x|
     * 3: |ms.y - c^k_2.y|
     */
    public Map<Keepout, int[]> pseudo_iVars;



    /**
     * @param x_ct center x
     * @param y_ct center y
     */
    public Master_ko_pseudoVar(double x_ct, double y_ct) {
        super(x_ct, y_ct);
        this.pseudo_bVars = new HashMap<>();
        this.pseudo_iVars = new HashMap<>();
    }



    public void addToPseudo_bVars(Keepout k, int[] ints) {
        this.pseudo_bVars.put(k,ints);
    }

    public void addToPsedo_iVars(Keepout k, int[] ints) {
        this.pseudo_iVars.put(k,ints);
    }
}

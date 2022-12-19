package shapeVar;

import shapes.Keepout;
import shapes.Master;

import java.util.HashMap;
import java.util.Map;

public class Master_var extends Master {



    /*
     * binary pseudo variables:
     * 0: L
     * 1: R
     * 2: A
     * 3: B
     * 4: Ld
     * 5: Rd
     * 6: Ad
     * 7: Bd
     */
    private Map<Keepout, int[]> pseudo_bVars; //Left, Right, Above, Below

    /*
     * 0: |ms.x - c^k_1.x|
     * 1: |ms.y - c^k_1.y|
     * 2: |ms.x - c^k_2.x|
     * 3: |ms.y - c^k_2.y|
     */
    private Map<Keepout, int[]> pseudo_iVars;



    /**
     * @param x_ct center x
     * @param y_ct center y
     */
    public Master_var(double x_ct, double y_ct) {
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

    public Map<Keepout, int[]> getPseudo_bVars() {
        return pseudo_bVars;
    }

    public Map<Keepout, int[]> getPseudo_iVars() {
        return pseudo_iVars;
    }
}

package shapeVar;

import grb.GurobiVariable;
import shapes.Keepout;

import java.util.HashMap;
import java.util.Map;

public class PointVar_ko extends PointVar {

    /*
    ONLY for v1 <-> ms
     * 0: Left (q^L_1_ms)
     * 1: Right (q^R_1_ms)
     * 2: Up (q^U_1_ms)
     * 3: Down (q^D_1_ms)
     * 4: detour trigger with masterIC w.r.t. all keepouts (dq_1_ms)
     */
    public GurobiVariable[] ms_sp_bVars;

    /*
     * ONLY for v1 <-> ms
     * 0: d(v_1, ms): >= d(v_1, o, ms) - oq^d_i_ms * M, for each o
     * 1: |v_1.x - ms.x|
     * 2: |v_1.y - ms.y|
     * 3: v_1.x - ms.x
     * 4: v_1.y - ms.y
     */
    public GurobiVariable[] ms_sp_iVars;

    /*
     * ONLY for v1 <-> ms
     * 10: left-right opposite with masterIC (oq^LR_1_ms)
     * 11: right-left  ----------||--------- (oq^RL_1_ms)
     * 12: above-below ----------||--------- (oq^AB_1_ms)
     * 13: below-above ----------||--------- (oq^BA_1_ms)
     * 14: for each o: detour trigger with masterIC (oq^d_1_ms)
     * 15: choose path for d(v_1, ms) (aux_oq_1_ms)
     */
    public Map<Keepout, GurobiVariable[]> ms_ko_sp_bVars;


    /*
     * ONLY for v1 <-> ms
     * d(v_1, o, ms)
     * d(v_1, o^LL, ms)
     * d(v_1, o^UR, ms)
     */
    public Map<Keepout, GurobiVariable[]> ms_ko_sp_iVars;

    /*
     * sp_bVars
     * Binary variables:
     * orientation with next steiner point:
     * 0: Left (q^L_ij)
     * 1: Right (q^R_ij)
     * 2: Up (q^U_ij)
     * 3: Down (q^D_ij)
     * 4: detour trigger with next steiner point w.r.t. all keepouts (dq_ij)
     */
    public GurobiVariable[] sp_bVars;

    /*
     * sp_iVars
     * Integer variables:
     * 0: d(v_i, v_i+1): >= d(v_i, o, v_i+1) - oq^d_ij * M, for each o
     * 1: d(v_i, corr.S) <=/>= d(v_i, s_j) +/- (1 - q_i_sj) * M, for each slave
     * 2: |v_i.x - v_i+1.x|
     * 3: |v_i.y - v_i+1.y|
     */
    public GurobiVariable[] sp_iVars;

    /*
     * ko_sp_bVars
     * Binary variables:
     * Non-overlapping with keepouts
     * 0: Left (nonL)
     * 1: Right (nonR)
     * 2: Above (nonA)
     * 3: Below (nonB)
     * 4: left-right opposite with next steiner point (oq^LR_ij)
     * 5: right-left  --------------||--------------- (oq^RL_ij)
     * 6: above-below --------------||--------------- (oq^AB_ij)
     * 7: below-above --------------||--------------- (oq^BA_ij)
     * 8: for each o: detour trigger with next steiner point (oq^d_ij)
     * 9: choose path for d(vi, o, v_i+1) (aux_oq_ij)
     *
     */
    public Map<Keepout, GurobiVariable[]> ko_sp_bVars;

    /*
     * ko_sp_iVars
     * Integer variables:
     * 0: |v_i.x - c^k_LL.x| (vo^LL_i.x)
     * 1: |v_i.y - c^k_LL.y| (vo^LL_i.y)
     * 2: |v_i.x - c^k_UR.x| (vo^UR_i.x)
     * 3: |v_i.y - c^k_UR.y| (vo^UR_i.y)
     * (0. 1. 2. 3) also for connecting with next steiner point, slaves, and master
     * //
     * sum of i.(0. 1.) + i+1.(0. 1.): d(v_i, o.LL, v_i+1)
     * sum of i.(2. 3.) + i+1.(2. 3.): d(v_i, o.UR, v_i+1)
     * 4: d(v_i, o, v_i+1)
     * 5: v_i.x - c^k_LL.x
     * 6: v_i.y - c^k_LL.y
     * 7: v_i.x - c^k_UR.x
     * 8: v_i.y - c^k_UR.y
     *
     */
    public Map<Keepout, GurobiVariable[]> ko_sp_iVars;

    /*
     * orientation with slaves
     * sl_bVars
     * Binary variables:
     * 0: Left (qL_i_sj)
     * 1: Right (qR_i_sj)
     * 2: Up (qU_i_sj)
     * 3: Down (qD_i_sj)
     * 4: for fixed slave, detour trigger with all keepouts (dq_i_sj)
     * 5: connection binary var (q_i_sj)
     */
    public Map<Slave_var, GurobiVariable[]> sl_bVars;

    /*
     * sl_iVars
     * Integer variables (5):
     * 0: d(v_i, s_j): >= d (v_i, o, s_j) - oq^d_i_sj * M
     * 1: |v_i.x - s_j.x|
     * 2: |v_i.y - s_j.y|
     * 3: v_i.x - s_j.x
     * 4: v_i.y - s_j.y
     */
    public Map<Slave_var, GurobiVariable[]> sl_iVars;

    /*
     * ko_sl_bVars
     * Binary variables (6):
     * 0: left-right opposite with each slave with each ko (oq^LR_i_sj)
     * 1: right-left  -----------------||----------------- (oq^RL_i_sj)
     * 2: above-below -----------------||----------------- (oq^AB_i_sj)
     * 3: below-above -----------------||----------------- (oq^BA_i_sj)
     * 4: detour trigger with each slave with each ko (oq^d_i_sj)
     * 5: choose path for d(v_i, o, s_j) (aux_oq_i_sj)
     */
    public Map<Keepout, Map<Slave_var, GurobiVariable[]>> ko_sl_bVars;

    /*
     * ko_sl_iVars
     * Integer varibales (1):
     * 0: d(v_i, o, s_j) = d(v_i, o.LL, s_j) * aux_oq_i_sj + d(v_i, o.UR, s_j) * (1 - aux_oq_i_sj)
     * 1 : d(v_i, o.LL, s_j)
     * 2: d(v_i, o.UR, s_j)
     */
    public Map<Keepout, Map<Slave_var, GurobiVariable[]>> ko_sl_iVars;






    public PointVar_ko() {
        this.ms_ko_sp_bVars = new HashMap<>();
        this.ms_ko_sp_iVars = new HashMap<>();


        this.ko_sp_bVars = new HashMap<>();
        this.ko_sp_iVars = new HashMap<>();

        this.ko_sl_bVars = new HashMap<>();
        this.ko_sl_iVars = new HashMap<>();

        this.sl_bVars = new HashMap<>();
        this.sl_iVars = new HashMap<>();




    }


}

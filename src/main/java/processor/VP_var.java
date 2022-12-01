package processor;

import grb.GurobiVariable;
import shapes.Keepout;

import java.util.Map;

public class VP_var {

    public GurobiVariable x;//x-coordinate of v_point
    public GurobiVariable y;//y-coordinate of v_point

    /*
     * ONLY for v1 <-> ms
     */
    public GurobiVariable[] ms_vp_bVars;

    /*
     * ONLY for v1 <-> ms
     */
    public GurobiVariable[] ms_vp_iVars;

    /*
     * ONLY for v1 <-> ms
     */
    public Map<Keepout, GurobiVariable[]> ms_ko_vp_bVars;

    /*
     * ONLY for v1 <-> ms
     *
     */
    public Map<Keepout, GurobiVariable[]> ms_ko_vp_iVars;


    /*
     * vp_bVars regarding next vp
     * (0) q(RL)
     * (1) q(LR)
     * (2) q(AB)
     * (3) q(BA)
     * (4) q(d): detour trigger
     */
    public GurobiVariable[] vp_bVars;

    /*
     * vp_iVars_abs
     * (0) d_vv
     * (1) |v_i.x - v_i+1.x|
     * (2) |v_i.y - v_i+1.y|
     * (3) d_vv(d)
     * (4) d_v_S(d)
     *
     */
    public GurobiVariable[] vp_iVars_abs;

    /*
     * vp_iVars
     * (0) v_i.x - v_i+1.x
     * (1) v_i.y - v_i+1.y
     */
    public GurobiVariable[] vp_iVars;


    /*
     * ko_sp_bVars
     * (0) oqL
     * (1) oqR
     * (2) oqA
     * (3) oqB
     * (4) oqLd
     * (5) oqRd
     * (6) oqAd
     * (7) oqBd
     * (8) oq(L): oq_ij^L = 1
     * (9) oq(R)
     * (10) oq(A)
     * (11) oq(B)
     * (12) oq(RL)
     * (13) oq(LR)
     * (14) oq(AB)
     * (15) oq(BA)
     * (16) oq(d): indicate relative obstacle regarding next vp
     * (17) oq(d,ll): indicate route pass by lower-left corner
     * (18) oq(d,ur)
     * (19) oq(d,ll,out)
     * (20) oq(d,ur,out)
     * (21) oq(d,ll,in)
     * (22) oq(d,ur,in)
     */
    public Map<Keepout, GurobiVariable[]> ko_vp_bVars;

    /*
     * ko_vp_iVars_abs
     * (0) vo_x_ll
     * (1) vo_y_ll
     * (2) vo_x_ur
     * (3) vo_y_ur
     *
     */
    public Map<Keepout, GurobiVariable[]> ko_vp_iVars_abs;


    /*
     * ko_vp_iVars
     * (0) vx - ox_ll
     * (1) vy - oy_ll
     * (2) vx - ox_ur
     * (3) vy - oy_ur
     */
    public Map<Keepout, GurobiVariable[]> ko_vp_iVars;




    /*
     * sl_bVars (vp <-> slaves)
     * (0) q_vs(RL)
     * (1) q_vs(LR)
     * (2) q_vs(AB)
     * (3) q_vs(BA)
     * (4) q_vs(d)
     * (5) q_vs: connection binary var
     */
    public Map<Slave_var, GurobiVariable[]> sl_bVars;

    /*
     * sl_iVars_abs
     * (0) d_vs(d)
     * (1) |v_i.x - s_j.x|
     * (2) |v_i.y - s_j.y|
     */
    public Map<Slave_var, GurobiVariable[]> sl_iVars_abs;

    /*
     * sl_iVars
     * (0) v_i.x - s_j.x
     * (1) v_i.y - s_j.y
     */
    public Map<Slave_var, GurobiVariable[]> sl_iVars;



    /*
     * ko_sl_bVars (vp <-> slaves)
     * (0) oq(L): oq_i_sj^L = 1
     * (1) oq(R)
     * (2) oq(A)
     * (3) oq(B)
     * (4) oq_vs(RL)
     * (5) oq_vs(LR)
     * (6) oq_vs(AB)
     * (7) oq_vs(BA)
     * (8) oq_vs(d)
     * (9) oq_vs(d,ll,in)
     * (10) oq_vs(d,ur,in)
     */
    public Map<Keepout, Map<Slave_var, GurobiVariable[]>> ko_sl_bVars;




    /*
     * oo_vp_bVars
     * (0) ooq(d)
     * (1) ooq(d,ll)
     * (2) ooq(d,ll,ur)
     * (3) ooq(d,ur,ll)
     * (4) ooq(d,ur)
     */
    public Map<Keepout, Map<Keepout, GurobiVariable[]>> oo_vp_bVars;
}

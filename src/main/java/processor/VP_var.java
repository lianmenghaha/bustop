package processor;

import grb.GurobiVariable;
import shapes.Keepout;

import java.util.HashMap;
import java.util.Map;

public class VP_var {

    public GurobiVariable x;//x-coordinate of v_point
    public GurobiVariable y;//y-coordinate of v_point


    /*
     * ONLY for v1 <-> ms
     * mvp_bVars
     * (0) q_mv(RL)
     * (1) q_mv(LR)
     * (2) q_mv(AB)
     * (3) q_mv(BA)
     * (4) q_mv(d)
     */
    public GurobiVariable[] mvp_bVars;

    /*
     * ONLY for v1 <-> ms
     * mvp_iVars_abs
     * (0) d_mv
     * (1) |v_1.x - ms.x|
     * (2) |v_1.y - ms.y|
     * (3) d_mv(d)
     */
    public GurobiVariable[] mvp_iVars_abs;

    /*
     * ONLY for v1 <-> ms
     * mvp_iVars
     * (0) v_1.x - ms.x
     * (1) v_1.y - ms.y
     */
    public GurobiVariable[] mvp_iVars;

    /*
     * ONLY for v1 <-> ms
     * ko_mvp_bVars_aux
     * (0) oq_mv(RL.R): oq_i_sj^L = 1
     * (1) oq_mv(RL.L)
     * (2) oq_mv(LR.L)
     * (3) oq_mv(LR.R)
     * (4) oq_mv(AB.A)
     * (5) oq_mv(AB.B)
     * (6) oq_mv(BA.B)
     * (7) oq_mv(BA.A)
     *
     */
    public Map<Keepout, GurobiVariable[]> ko_mvp_bVars_aux;


    /*
     * ONLY for v1 <-> ms
     * ko_mvp_bVars_dt
     * (0) oq_mv(RL)
     * (1) oq_mv(LR)
     * (2) oq_mv(AB)
     * (3) oq_mv(BA)
     * (4) oq_mv(d)
     * (5) oq_mv(d,ll): indicate route pass by lower-left corner
     * (6) oq_mv(d,ur)
     * (7) oq_mv(d,ll,in)
     * (8) oq_mv(d,ur,in)
     * (9) oq_mv(d,ll,out): if vp -> slave must detour, this indicate that vp -> obstacle's lower-left corner
     * (10) oq_mv(d,ur,out)
     */
    public Map<Keepout, GurobiVariable[]> ko_mvp_bVars_dt;

    /*
     * ONLY for v1 <-> ms
     * ko_mvp_bVars_dtAus
     * (0) _oq_vs(ll,out)
     * (1) _oq_vs(ur,out)
     * (2) _oq_vs(ll,in)
     * (3) _oq_vs(ur,in)
     * (4) _oq_vs(ur,ll)
     * (5) _oq_vs(ur)
     * (6) _oq_vs(ll)
     * (7) _oq_vs(ll,ur)
     */
    public Map<Keepout, GurobiVariable[]> ko_mvp_bVars_dtAux;

    /*
     * ONLY for v1 <-> ms
     * ko_mvp_iVars_dtAux
     * (0) _vso_ll_in
     * (1) _vso_ur_in
     */
    public Map<Keepout, GurobiVariable[]> ko_mvp_iVars_dtAux;


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
     * (8) oq(RL.R): oq_ij^L = 1
     * (9) oq(RL.L)
     * (10) oq(LR.L)
     * (11) oq(LR.R)
     * (12) oq(AB.A)
     * (13) oq(AB.B)
     * (14) oq(BA.B)
     * (15) oq(BA.A)
     *
     */
    public Map<Keepout, GurobiVariable[]> ko_vp_bVars_aux;

    /*
     * ko_vp_bVars_dt
     * (0) oq(RL)
     * (1) oq(LR)
     * (2) oq(AB)
     * (3) oq(BA)
     * (4) oq(d): indicate relative obstacle regarding next vp
     * (5) oq(d,ll): indicate route pass by lower-left corner
     * (6) oq(d,ur)
     * (7) oq(d,ll,out)
     * (8) oq(d,ur,out)
     * (9) oq(d,ll,in)
     * (10) oq(d,ur,in)
     */
    public Map<Keepout, GurobiVariable[]> ko_vp_bVars_dt;

    /*
     * ko_vp_bVars_dtAux
     * (0) _oq(ll,out)
     * (1) _oq(ur,out)
     * (2) _oq(ll,in)
     * (3) _oq(ur,in)
     * (4) _oq(ur,ll)
     * (5) _oq(ur)
     * (6) _oq(ll)
     * (7) _oq(ll,ur)
     */

    public Map<Keepout, GurobiVariable[]> ko_vp_bVars_dtAux;

    /*
     * ko_vp_iVars_abs
     * (0) vo_x_ll
     * (1) vo_y_ll
     * (2) vo_x_ur
     * (3) vo_y_ur
     * (4) vo_ll
     * (5) vo_ur
     * (6) _vo_ll_out
     * (7) _vo_ur_out
     * (8) _vo_ll_in
     * (9) _vo_ur_in
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
     * (0) d_vs
     * (1) |v_i.x - s_j.x|
     * (2) |v_i.y - s_j.y|
     * (3) d_vs(d)
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
     * (0) oq_vs(RL.R): oq_i_sj^L = 1
     * (1) oq_vs(RL.L)
     * (2) oq_vs(LR.L)
     * (3) oq_vs(LR.R)
     * (4) oq_vs(AB.A)
     * (5) oq_vs(AB.B)
     * (6) oq_vs(BA.B)
     * (7) oq_vs(BA.A)
     */
    public Map<Keepout, Map<Slave_var, GurobiVariable[]>> ko_sl_bVars_aux;

    /*
     * ko_sl_bVars_dt
     * (0) oq_vs(RL)
     * (1) oq_vs(LR)
     * (2) oq_vs(AB)
     * (3) oq_vs(BA)
     * (4) oq_vs(d)
     * (5) oq_vs(d,ll): indicate route pass by lower-left corner
     * (6) oq_vs(d,ur)
     * (7) oq_vs(d,ll,in)
     * (8) oq_vs(d,ur,in)
     * (9) oq_vs(d,ll,out): if vp -> slave must detour, this indicate that vp -> obstacle's lower-left corner
     * (10) oq_vs(d,ur,out)
     */
    public Map<Keepout, Map<Slave_var, GurobiVariable[]>> ko_sl_bVars_dt;

    /*
     * ko_sl_bVars_dtAux
     * (0) _oq_vs(ll,out)
     * (1) _oq_vs(ur,out)
     * (2) _oq_vs(ll,in)
     * (3) _oq_vs(ur,in)
     * (4) _oq_vs(ur,ll)
     * (5) _oq_vs(ur)
     * (6) _oq_vs(ll)
     * (7) _oq_vs(ll,ur)
     */

    public Map<Keepout, Map<Slave_var, GurobiVariable[]>> ko_sl_bVars_dtAux;

    /*
     * ko_sl_iVars_dtAux
     * (0) _vso_ll_in
     * (1) _vso_ur_in
     */
    public Map<Keepout, Map<Slave_var, GurobiVariable[]>> ko_sl_iVars_dtAux;




    /*
     * oo_vp_bVars vp <-> vp
     * (0) ooq(d)
     * (1) ooq(d,ll)
     * (2) ooq(d,ll,ur)
     * (3) ooq(d,ur,ll)
     * (4) ooq(d,ur)
     */
    public Map<Keepout, Map<Keepout, GurobiVariable[]>> oo_vp_bVars;

    /*
     * oo_vp_bVars vp <-> slave
     * (0) ooq_vs(d)
     * (1) ooq_vs(d,ll)
     * (2) ooq_vs(d,ll,ur)
     * (3) ooq_vs(d,ur,ll)
     * (4) ooq_vs(d,ur)
     */
    public Map<Slave_var, Map<Keepout, Map<Keepout, GurobiVariable[]>>> oo_sl_bVars;

    /*
     * oo_mvp_bVars vp1 <-> master
     * (0) ooq_mv(d)
     * (1) ooq_mv(d,ll)
     * (2) ooq_mv(d,ll,ur)
     * (3) ooq_mv(d,ur,ll)
     * (4) ooq_mv(d,ur)
     */
    public Map<Keepout, Map<Keepout, GurobiVariable[]>> oo_mvp_bVars;

    public VP_var(){

        this.ko_mvp_bVars_aux = new HashMap<>();
        this.ko_mvp_bVars_dt = new HashMap<>();
        this.ko_mvp_bVars_dtAux = new HashMap<>();
        this.ko_mvp_iVars_dtAux = new HashMap<>();



        this.ko_vp_bVars_aux = new HashMap<>();
        this.ko_vp_bVars_dt = new HashMap<>();
        this.ko_vp_bVars_dtAux = new HashMap<>();
        this.ko_vp_iVars_abs = new HashMap<>();
        this.ko_vp_iVars = new HashMap<>();

        this.sl_bVars = new HashMap<>();
        this.sl_iVars_abs = new HashMap<>();
        this.sl_iVars = new HashMap<>();

        this.ko_sl_bVars_aux = new HashMap<>();
        this.ko_sl_bVars_dt = new HashMap<>();
        this.ko_sl_bVars_dtAux =new HashMap<>();
        this.ko_sl_iVars_dtAux = new HashMap<>();

        this.oo_vp_bVars = new HashMap<>();
        this.oo_sl_bVars = new HashMap<>();
        this.oo_mvp_bVars = new HashMap<>();




    }
}

package processor;

import com.itextpdf.kernel.colors.DeviceCmyk;
import com.itextpdf.kernel.colors.DeviceRgb;
import com.itextpdf.kernel.geom.PageSize;
import com.itextpdf.kernel.pdf.PdfDocument;
import com.itextpdf.kernel.pdf.PdfWriter;
import com.itextpdf.kernel.pdf.canvas.PdfCanvas;
import com.itextpdf.kernel.colors.Color;
import com.itextpdf.kernel.geom.Rectangle;
import com.itextpdf.layout.Document;
import com.itextpdf.layout.element.Paragraph;
import parser.OutputDoc;
import shapes.*;

import java.awt.*;
import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.Collections;

import static com.itextpdf.kernel.colors.Color.convertRgbToCmyk;

public class DrawPDF {
    private OutputDoc outputDoc;
    private final String path;

    public DrawPDF(OutputDoc outputDoc, String path) {
        this.outputDoc = outputDoc;
        this.path = path;
    }

    public void outputToPdf() throws FileNotFoundException {


        ArrayList<VirtualPoint> virtualPoints = outputDoc.getVirtualPoints();
        ArrayList<Slave> slaves = outputDoc.getSlaves();
        ArrayList<Keepout> uni_keepouts = outputDoc.getKeepouts();
        Master master = outputDoc.getMaster();
        /*
        Process the PDFsize
         */
        ArrayList<Integer> xs = new ArrayList<>();
        ArrayList<Integer> ys = new ArrayList<>();
        xs.add(master.getX_ct());
        ys.add(master.getY_ct());
        for (VirtualPoint vp : virtualPoints){
            xs.add(vp.getX_ct());
            ys.add(vp.getY_ct());
        }
        for (Slave sv : slaves){
            xs.add(sv.getX_ct());
            ys.add(sv.getY_ct());
        }
        for (Keepout o : uni_keepouts){
            xs.add(o.getMinX());
            xs.add(o.getMaxX());
            ys.add(o.getMinY());
            ys.add(o.getMaxY());
        }
        int lb_x = Collections.min(xs);
        int ub_x = Collections.max(xs);
        int lb_y = Collections.min(ys);
        int ub_y = Collections.max(ys);
        lb_x -= 100;
        ub_x += 100;
        lb_y -= 100;
        ub_y += 100;
        int width = ub_x - lb_x;
        int height = ub_y - lb_y;
        int sum_wh = width + height;

        double master_r = sum_wh/200;
        double vp_r = sum_wh/300;
        double sl_r = sum_wh/350;
        float lineWidth = sum_wh/1000;



        String pdfPath = this.path + "_" + outputDoc.getI2Cname();
        Rectangle rectangle = new Rectangle(lb_x, lb_y, width, height);
        PdfDocument pdfDoc = new PdfDocument(new PdfWriter(pdfPath));
        Document document = new Document(pdfDoc, new PageSize(rectangle));


        PdfCanvas canvas = new PdfCanvas(pdfDoc.addNewPage());
        canvas.setLineWidth(lineWidth);

        /*
        draw Keepouts
         */
        float p_bias = 2;
        for (Keepout o : uni_keepouts){
            Rectangle rect = new Rectangle(o.getMinX(), o.getMinY(), o.getMaxX() - o.getMinX(), o.getMaxY() - o.getMinY());
            Color LightBlue = convertRgbToCmyk(new DeviceRgb(173,216,230));
            canvas.setColor(LightBlue, true)
                    .rectangle(rect)
                    .fill()
                    .stroke();
            Color BLACK = convertRgbToCmyk(new DeviceRgb(0,0,0));
            canvas.setColor(BLACK, false)
                    .rectangle(rect)
                    .stroke();
            Paragraph pO = new Paragraph("[" + o.getMinX() + ", " + o.getMaxX() + "]*[" + o.getMinY() + ", " + o.getMaxY() + "]").setFontSize(20).setFontColor(convertRgbToCmyk(new DeviceRgb(0,0,0)));
            pO.setFixedPosition(o.getMinX() + p_bias, (float) ((o.getMinY() + o.getMaxY())*0.5), 2000);
            document.add(pO);
        }
        /*
        draw Master
         */

        Color PINK = convertRgbToCmyk(new DeviceRgb(255,182,193));
        canvas.setColor(PINK, true)
                .circle(master.getX_ct(), master.getY_ct(), master_r)
                .fill()
                .stroke();
        Paragraph pMaster = new Paragraph(master.getName() + " (" + master.getX_ct() + ", " + master.getY_ct() + ")").setFontSize(20).setFontColor(convertRgbToCmyk(new DeviceRgb(0,0,0)));
        pMaster.setFixedPosition(master.getX_ct() , master.getY_ct() + p_bias, 200);
        document.add(pMaster);

        int vp_cnt = 0;
        for (VirtualPoint vp : virtualPoints){
            /*
            draw Vp
             */
            vp_cnt++;
            Color CYAN = convertRgbToCmyk(new DeviceRgb(0,255,255));
            canvas.setColor(CYAN, false)
                    .circle(vp.getX_ct(), vp.getY_ct(), vp_r)
                    .stroke();
            //Paragraph pVP = new Paragraph("VP" + vp_cnt +" (" + vp.getX_ct() + ", " + vp.getY_ct() + ")").setFontSize(20).setFontColor(convertRgbToCmyk(new DeviceRgb(0,0,0)));
            Paragraph pVP = new Paragraph("VP" + vp_cnt).setFontSize(20).setFontColor(convertRgbToCmyk(new DeviceRgb(0,0,0)));
            pVP.setFixedPosition(vp.getX_ct(), vp.getY_ct() + 4 * p_bias, 2000);
            document.add(pVP);
            ArrayList<ObObC> rel_mv_obs = vp.getRel_mv_Obs();
            ArrayList<ObObC> rel_vp_obs = vp.getRel_vp_Obs();
            ArrayList<ObObC> rel_sl_obs = vp.getRel_sl_Obs();
            if (rel_mv_obs.size() != 0){
                Color LIME = convertRgbToCmyk(new DeviceRgb(0, 255,0));
                canvas.setColor(LIME, false);
                for (ObObC connect : rel_mv_obs){
                    if (connect.type == ConnectionType.URtoLL){

                        canvas.moveTo(((Keepout)connect.start).getMaxX(), ((Keepout)connect.start).getMaxY());
                        canvas.lineTo(((Keepout)connect.end).getMinX(), ((Keepout)connect.end).getMinY());

                    }
                    if (connect.type == ConnectionType.URtoUR){

                        canvas.moveTo(((Keepout)connect.start).getMaxX(), ((Keepout)connect.start).getMaxY());
                        canvas.lineTo(((Keepout)connect.end).getMaxX(), ((Keepout)connect.end).getMaxY());

                    }
                    if (connect.type == ConnectionType.LLtoUR){

                        canvas.moveTo(((Keepout)connect.start).getMinX(), ((Keepout)connect.start).getMinY());
                        canvas.lineTo(((Keepout)connect.end).getMaxX(), ((Keepout)connect.end).getMaxY());

                    }
                    if (connect.type == ConnectionType.LLtoLL){

                        canvas.moveTo(((Keepout)connect.start).getMinX(), ((Keepout)connect.start).getMinY());
                        canvas.lineTo(((Keepout)connect.end).getMinX(), ((Keepout)connect.end).getMinY());

                    }
                    if (connect.type == ConnectionType.VPtoLL){

                        canvas.moveTo(connect.start.getX_ct(), connect.start.getY_ct());
                        canvas.lineTo(((Keepout)connect.end).getMinX(), ((Keepout)connect.end).getMinY());

                    }
                    if (connect.type == ConnectionType.VPtoUR){

                        canvas.moveTo(connect.start.getX_ct(), connect.start.getY_ct());
                        canvas.lineTo(((Keepout)connect.end).getMaxX(), ((Keepout)connect.end).getMaxY());

                    }
                    if (connect.type == ConnectionType.URtoMS){
                        canvas.moveTo(((Keepout)connect.start).getMaxX(), ((Keepout)connect.start).getMaxY());
                        canvas.lineTo(master.getX_ct(), master.getY_ct());

                    }
                    if (connect.type == ConnectionType.LLtoMS){

                        canvas.moveTo(((Keepout)connect.start).getMinX(), ((Keepout)connect.start).getMinY());
                        canvas.lineTo(master.getX_ct(), master.getY_ct());
                    }
                    if (connect.type == ConnectionType.NoDetour){
                        canvas.moveTo(connect.start.getX_ct(), connect.start.getY_ct());
                        canvas.lineTo(connect.end.getX_ct(), connect.end.getY_ct());
                    }
                    canvas.closePathStroke();


                }
            }

            if (rel_vp_obs.size() != 0){
                Color LIME = convertRgbToCmyk(new DeviceRgb(0, 255,0));
                canvas.setColor(LIME, false);
                for (ObObC connect : rel_vp_obs){
                    if (connect.type == ConnectionType.URtoLL){

                        canvas.moveTo(((Keepout)connect.start).getMaxX(), ((Keepout)connect.start).getMaxY());
                        canvas.lineTo(((Keepout)connect.end).getMinX(), ((Keepout)connect.end).getMinY());

                    }
                    if (connect.type == ConnectionType.URtoUR){

                        canvas.moveTo(((Keepout)connect.start).getMaxX(), ((Keepout)connect.start).getMaxY());
                        canvas.lineTo(((Keepout)connect.end).getMaxX(), ((Keepout)connect.end).getMaxY());

                    }
                    if (connect.type == ConnectionType.LLtoUR){

                        canvas.moveTo(((Keepout)connect.start).getMinX(), ((Keepout)connect.start).getMinY());
                        canvas.lineTo(((Keepout)connect.end).getMaxX(), ((Keepout)connect.end).getMaxY());

                    }
                    if (connect.type == ConnectionType.LLtoLL){

                        canvas.moveTo(((Keepout)connect.start).getMinX(), ((Keepout)connect.start).getMinY());
                        canvas.lineTo(((Keepout)connect.end).getMinX(), ((Keepout)connect.end).getMinY());

                    }
                    if (connect.type == ConnectionType.VPtoLL){

                        canvas.moveTo(connect.start.getX_ct(), connect.start.getY_ct());
                        canvas.lineTo(((Keepout)connect.end).getMinX(), ((Keepout)connect.end).getMinY());

                    }
                    if (connect.type == ConnectionType.VPtoUR){

                        canvas.moveTo(connect.start.getX_ct(), connect.start.getY_ct());
                        canvas.lineTo(((Keepout)connect.end).getMaxX(), ((Keepout)connect.end).getMaxY());

                    }
                    if (connect.type == ConnectionType.LLtoVP){
                        canvas.moveTo(((Keepout)connect.start).getMaxX(), ((Keepout)connect.start).getMaxY());
                        canvas.lineTo(connect.end.getX_ct(), connect.end.getY_ct());

                    }
                    if (connect.type == ConnectionType.URtoVP){

                        canvas.moveTo(((Keepout)connect.start).getMinX(), ((Keepout)connect.start).getMinY());
                        canvas.lineTo(connect.end.getX_ct(), connect.end.getY_ct());
                    }
                    if (connect.type == ConnectionType.NoDetour){
                        canvas.moveTo(connect.start.getX_ct(), connect.start.getY_ct());
                        canvas.lineTo(connect.end.getX_ct(), connect.end.getY_ct());
                    }

                    canvas.closePathStroke();


                }
            }

            if (rel_sl_obs.size() != 0){
                Color ORANGE = convertRgbToCmyk(new DeviceRgb(255,165,0));
                canvas.setColor(ORANGE, false);
                for (ObObC connect : rel_sl_obs){
                    if (connect.type == ConnectionType.URtoLL){

                        canvas.moveTo(((Keepout)connect.start).getMaxX(), ((Keepout)connect.start).getMaxY());
                        canvas.lineTo(((Keepout)connect.end).getMinX(), ((Keepout)connect.end).getMinY());

                    }
                    if (connect.type == ConnectionType.URtoUR){

                        canvas.moveTo(((Keepout)connect.start).getMaxX(), ((Keepout)connect.start).getMaxY());
                        canvas.lineTo(((Keepout)connect.end).getMaxX(), ((Keepout)connect.end).getMaxY());

                    }
                    if (connect.type == ConnectionType.LLtoUR){

                        canvas.moveTo(((Keepout)connect.start).getMinX(), ((Keepout)connect.start).getMinY());
                        canvas.lineTo(((Keepout)connect.end).getMaxX(), ((Keepout)connect.end).getMaxY());

                    }
                    if (connect.type == ConnectionType.LLtoLL){

                        canvas.moveTo(((Keepout)connect.start).getMinX(), ((Keepout)connect.start).getMinY());
                        canvas.lineTo(((Keepout)connect.end).getMinX(), ((Keepout)connect.end).getMinY());

                    }
                    if (connect.type == ConnectionType.VPtoLL){

                        canvas.moveTo(connect.start.getX_ct(), connect.start.getY_ct());
                        canvas.lineTo(((Keepout)connect.end).getMinX(), ((Keepout)connect.end).getMinY());

                    }
                    if (connect.type == ConnectionType.VPtoUR){

                        canvas.moveTo(connect.start.getX_ct(), connect.start.getY_ct());
                        canvas.lineTo(((Keepout)connect.end).getMaxX(), ((Keepout)connect.end).getMaxY());

                    }
                    if (connect.type == ConnectionType.URtoSL){
                        canvas.moveTo(((Keepout)connect.start).getMaxX(), ((Keepout)connect.start).getMaxY());
                        canvas.lineTo(connect.end.getX_ct(), connect.end.getY_ct());

                    }
                    if (connect.type == ConnectionType.LLtoSL){

                        canvas.moveTo(((Keepout)connect.start).getMinX(), ((Keepout)connect.start).getMinY());
                        canvas.lineTo(connect.end.getX_ct(), connect.end.getY_ct());
                    }
                    if (connect.type == ConnectionType.NoDetour){
                        canvas.moveTo(connect.start.getX_ct(), connect.start.getY_ct());
                        canvas.lineTo(connect.end.getX_ct(), connect.end.getY_ct());
                    }

                    canvas.closePathStroke();


                }
            }


        }

        /*
        draw Slaves
         */

        for (Slave sv : slaves){
            Color RED = convertRgbToCmyk(new DeviceRgb(255,0,0));
            canvas.setColor(RED, true)
                    .circle(sv.getX_ct(), sv.getY_ct(), sl_r)
                    .fill()
                    .stroke();

            //Paragraph pSV = new Paragraph(sv.getName() +" (" + sv.getX_ct() + ", " + sv.getY_ct() + ")").setFontSize(20).setFontColor(convertRgbToCmyk(new DeviceRgb(0,0,0)));
            Paragraph pSV = new Paragraph(sv.getName()).setFontSize(20).setFontColor(convertRgbToCmyk(new DeviceRgb(0,0,0)));
            pSV.setFixedPosition(sv.getX_ct() , sv.getY_ct() - p_bias, 2000);
            document.add(pSV);
        }



        pdfDoc.close();




    }







}

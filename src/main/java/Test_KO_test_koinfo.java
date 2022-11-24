import processor.Processor;
import shapes.Keepout;

import java.time.Duration;
import java.time.LocalDateTime;
import java.time.ZoneId;
import java.time.format.DateTimeFormatter;
import java.util.ArrayList;

public class Test_KO_test_koinfo {

    public static void main(String[] args) {
        LocalDateTime start = LocalDateTime.now();
        System.out.println("Program Starts at: " + DateTimeFormatter.ofPattern("yyyy/MM/dd HH:mm:ss.SSS").format(start));

        Processor processor = new Processor();


        ArrayList<Keepout> uni_keepouts = new ArrayList<>();
        Keepout o1 = new Keepout(0, 1, 7, 12);
        o1.setName("o1");
        Keepout o2 = new Keepout(3, 4, 0, 5);
        o2.setName("o2");
        Keepout o3 = new Keepout(6, 7, 3, 9);
        o3.setName("o3");
        Keepout o4 = new Keepout(8, 9, 7, 12);
        o4.setName("o4");
        Keepout o5 = new Keepout(3, 4, 8, 10);
        o5.setName("o5");
        Keepout o6 = new Keepout(3, 4, 11, 12);
        o6.setName("o6");

        uni_keepouts.add(o1);
        uni_keepouts.add(o2);
        uni_keepouts.add(o3);
        uni_keepouts.add(o4);
        uni_keepouts.add(o5);
        uni_keepouts.add(o6);


        //processor.integrated_information_keepouts(uni_keepouts);
        //debug
        /*for (Keepout k : uni_keepouts){
            System.out.println(k);
            System.out.println(k.getName());
            if (k.getLeft_os().size() !=  0 ){
                if (k.getLeft_miny_o() != null) {
                    System.out.println("k.left_o_min: " + k.getLeft_miny_o().getName());
                }
                if (k.getLeft_maxy_o() != null) {
                    System.out.println("k.left_o_max: " + k.getLeft_maxy_o().getName());
                }
            }

            if (k.getRight_os().size() != 0){
                if (k.getRight_miny_o() != null) {
                    System.out.println("k.right_o_min: " + k.getRight_miny_o().getName());
                }
                if (k.getRight_maxy_o() != null) {
                    System.out.println("k.right_o_max: " + k.getRight_maxy_o().getName());
                }
            }

            if (k.getAbove_os().size() != 0){
                if (k.getAbove_minx_o() != null) {
                    System.out.println("k.above_o_min: " + k.getAbove_minx_o().getName());
                }
                if (k.getAbove_maxx_o() != null) {
                    System.out.println("k.above_o_max: " + k.getAbove_maxx_o().getName());
                }
            }

            if (k.getBelow_os().size() != 0){
                if (k.getBelow_minx_o() != null) {
                    System.out.println("k.below_o_min: " + k.getBelow_minx_o().getName());
                }
                if (k.getBelow_minx_o() != null) {
                    System.out.println("k.below_o_max: " + k.getBelow_minx_o().getName());
                }
            }

            for (int i : k.dist){
                System.out.print(i + " , ");
            }
            System.out.println(" ");
        }*/

        LocalDateTime end = LocalDateTime.now();
        Duration duration = Duration.between(start, end);
        LocalDateTime duration_formated = LocalDateTime.ofInstant(java.time.Instant.ofEpochMilli(duration.toMillis()), ZoneId.of("UTC"));
        System.out.println("Program Ends at: " + DateTimeFormatter.ofPattern("yyyy/MM/dd HH:mm:ss.SSS").format(end));
        System.out.println("Program Run Time is: " + DateTimeFormatter.ofPattern("HH:mm:ss.SSS").format(duration_formated));

    }
}

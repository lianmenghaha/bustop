import gurobi.GRBException;
import processor.Processor;
import shapes.Keepout;
import shapes.Master;
import shapes.Poly_Keepout;
import shapes.Slave;

import java.time.Duration;
import java.time.LocalDateTime;
import java.time.ZoneId;
import java.time.format.DateTimeFormatter;
import java.util.ArrayList;

public class Test_KO_test {

    public static void main(String[] args) {
        LocalDateTime start = LocalDateTime.now();
        System.out.println("Program Starts at: " + DateTimeFormatter.ofPattern("yyyy/MM/dd HH:mm:ss.SSS").format(start));

        Processor processor = new Processor();


        ArrayList<Keepout> uni_keepouts = new ArrayList<>();
        Keepout o1 = new Keepout(2,3,3,7);
        o1.setName("o1");
        Keepout o2 = new Keepout(5,6,5,9);
        o2.setName("o2");
        Keepout o3 = new Keepout(8,9,1,5);
        o3.setName("o3");

        uni_keepouts.add(o1);
        uni_keepouts.add(o2);
        uni_keepouts.add(o3);

        ArrayList<Slave> slaves = new ArrayList<>();
        Slave s1 = new Slave(1,5);
        s1.setName("s1");
        Slave s2 = new Slave(4,6);
        s2.setName("s2");
        Slave s3 = new Slave(8,7);
        s3.setName("s3");
        slaves.add(s1);
        slaves.add(s2);
        slaves.add(s3);

        Master master = new Master(0,0);
        ArrayList<Poly_Keepout> poly_keepouts = new ArrayList<>();




        try {
            processor.processToOutput_w_KO(master, slaves, uni_keepouts, poly_keepouts,1, 1);
            LocalDateTime end = LocalDateTime.now();
            Duration duration = Duration.between(start,end);
            LocalDateTime duration_formated = LocalDateTime.ofInstant(java.time.Instant.ofEpochMilli(duration.toMillis()), ZoneId.of("UTC"));
            System.out.println("Program Ends at: "+ DateTimeFormatter.ofPattern("yyyy/MM/dd HH:mm:ss.SSS").format(end));
            System.out.println("Program Run Time is: "+ DateTimeFormatter.ofPattern("HH:mm:ss.SSS").format(duration_formated));
        } catch (GRBException e) {
            e.printStackTrace();
        }

    }
}

import gurobi.GRBException;
import parser.OutputDoc;
import parser.OutputDocProcess;
import processor.Processor;

import java.io.FileNotFoundException;
import java.io.IOException;
import java.time.Duration;
import java.time.LocalDateTime;
import java.time.ZoneId;
import java.time.format.DateTimeFormatter;

public class Test_KO_hw_test_PATH {

    public static void main(String[] args) throws IOException, GRBException {
        LocalDateTime start = LocalDateTime.now();
        System.out.println("Program Starts at: " + DateTimeFormatter.ofPattern("yyyy/MM/dd HH:mm:ss.SSS").format(start));
        Processor processor = new Processor();

//        OutputDoc outputDoc = processor.processToOutput_w_multiKO("input_busTop/case1/case1_I2C4_SCL");
        OutputDoc outputDoc = processor.processToOutput_w_multiKO("input_busTop/test");
        LocalDateTime end = LocalDateTime.now();
//        OutputDocProcess finOutput = new OutputDocProcess(outputDoc, "result_busTop/case1_I2C4_SCL");
        OutputDocProcess finOutput = new OutputDocProcess(outputDoc, "result_busTop/test");
        Duration duration = Duration.between(start, end);
        LocalDateTime duration_formated = LocalDateTime.ofInstant(java.time.Instant.ofEpochMilli(duration.toMillis()), ZoneId.of("UTC"));
        System.out.println("Program Ends at: " + DateTimeFormatter.ofPattern("yyyy/MM/dd HH:mm:ss.SSS").format(end));
        System.out.println("Program Run Time is: " + DateTimeFormatter.ofPattern("HH:mm:ss.SSS").format(duration_formated));

    }
}

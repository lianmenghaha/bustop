package parser;

import processor.DrawPDF;

import java.io.FileNotFoundException;
import java.time.LocalDateTime;

public class OutputDocProcess {

    private OutputDoc outputDoc;
    private final String path;
    private final LocalDateTime beginTime;

    public OutputDocProcess(OutputDoc outputDoc, String path, LocalDateTime beginTime) throws FileNotFoundException {
        this.outputDoc = outputDoc;
        this.path = path;
        this.beginTime = beginTime;
        new DrawPDF(outputDoc, path).outputToPdf();
    }
}

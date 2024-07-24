package org.firstinspires.ftc.teamcode.Files.FileUtils;
import   org.firstinspires.ftc.teamcode.zLibraries.Files.FileUtils.FileLogReader ;

public class FileDataHandler {

    FileLogReader reader;
    FileLogWriter writer;

    public FileDataHandler(String filename){
        reader = new FileLogReader(filename);
        writer = new FileLogWriter(filename);
    }

    public void writeData(){

    }

}

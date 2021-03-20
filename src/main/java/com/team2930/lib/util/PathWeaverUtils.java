package com.team2930.lib.util;

import java.nio.file.DirectoryStream;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;

public class PathWeaverUtils {

    private Path m_pathsDirectory;
    private List<String> m_filenameList = new ArrayList<String>();
    private List<String> m_chooserName = new ArrayList<String>();

    // This filter will only include files ending with .json
    private DirectoryStream.Filter<Path> jsonFilter = file -> {
        return Files.isRegularFile(file) && file.toString().endsWith(".json");
    };

    public PathWeaverUtils(String pathDirectoryString) {
        m_pathsDirectory = Filesystem.getDeployDirectory().toPath().resolve(pathDirectoryString);
        scanPathDirectory();
    }

    public PathWeaverUtils() {
        this("paths");
    }

    private String chooserNameFromString(String f) {
        String cn = new String(f);
        int index = cn.indexOf('.');
        if (index > 0) {
            cn = f.substring(0, index);
        }
        return cn;
    }


    private void scanPathDirectory() {

        try (DirectoryStream<Path> ds =  Files.newDirectoryStream(m_pathsDirectory, jsonFilter))  {
    
            ds.forEach(path -> System.out.println(path));
            ds.forEach(path -> m_filenameList.add(path.toString()));
            ds.forEach(path -> m_chooserName.add(chooserNameFromString(path.toString())));
            ds.forEach(path -> System.out.println(chooserNameFromString(path.toString())));

        }
        catch (Exception e)
        {
            System.out.println("Error: " + e.getMessage());
            DriverStation.reportError("Scan for PathWeaver files failed: " + e.getMessage(), true);
        }
    }
    
}

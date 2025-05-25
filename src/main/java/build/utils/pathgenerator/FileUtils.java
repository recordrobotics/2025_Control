package build.utils.pathgenerator;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.SerializationFeature;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;

public final class FileUtils {
  private FileUtils() {}

  public static File getLaunchDirectory() {
    // workaround for
    // https://www.chiefdelphi.com/t/filesystem-getdeploydirectory-returning-wrong-location-how-to-fix/427292
    String path =
        System.getProperty("user.dir")
            .replace(
                File.separator + "build" + File.separator + "jni" + File.separator + "release", "");
    return new File(path).getAbsoluteFile();
  }

  private static final ObjectMapper mapper =
      new ObjectMapper().enable(SerializationFeature.INDENT_OUTPUT);

  public static JsonNode readJson(File file) {
    if (!file.exists()) throw new RuntimeException("File not found: " + file.getAbsolutePath());

    JsonNode obj;
    try {
      FileReader reader = new FileReader(file);
      obj = mapper.readTree(reader);
      reader.close();
    } catch (IOException e) {
      throw new RuntimeException("Error reading JSON file: " + file.getAbsolutePath(), e);
    }

    return obj;
  }

  public static void writeJson(File file, JsonNode obj) {
    try (FileWriter writer = new FileWriter(file)) {
      mapper.writer(new PathPlannerPrettyPrinter()).writeValue(writer, obj);
    } catch (IOException e) {
      throw new RuntimeException("Error writing JSON file: " + file.getAbsolutePath(), e);
    }
  }
}

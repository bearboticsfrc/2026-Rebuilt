package bearlib.util;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.io.Serializable;
import java.text.SimpleDateFormat;
import java.util.Date;

/** GitVersion class borrowed from team 2832 */
public class GitVersion implements Serializable {

  /** Formatter for build dates, used to format the build date into a readable string. */
  private final SimpleDateFormat BUILD_DATE_FORMATTER = new SimpleDateFormat("MM/dd/yyyy HH:mm:ss");

  /** The hash of the last commit in the repository. */
  private String lastCommit;

  /** Indicates whether the source code has been modified since the last commit. */
  private boolean isModified;

  /** The date and time when the build was created. */
  private Date buildDate;

  /** The name of the author who created the build. */
  private String buildAuthor;

  /**
   * Creates a GitVersion instance using metadata.
   *
   * @param lastCommit The hash of the last commit.
   * @param isModified Whether the source was modified since the last commit.
   * @param buildDate The date of the build.
   * @param buildAuthor The name of the build author.
   * @return A new instance of GitVersion populated with the provided metadata.
   */
  public static GitVersion fromMetadata(
      String lastCommit, boolean isModified, Date buildDate, String buildAuthor) {
    GitVersion version = new GitVersion();

    version.lastCommit = lastCommit;
    version.isModified = isModified;
    version.buildDate = buildDate;
    version.buildAuthor = buildAuthor;

    return version;
  }

  /**
   * Gets the latest commit hash.
   *
   * @return The hash of the last commit.
   */
  public String getLastCommit() {
    return lastCommit;
  }

  /**
   * Indicates whether the source was modified from the most recent local release.
   *
   * @return {@code true} if the source was modified, {@code false} otherwise.
   */
  public boolean isModified() {
    return isModified;
  }

  /**
   * Gets the date of the latest build.
   *
   * @return The build date.
   */
  public Date getBuildDate() {
    return buildDate;
  }

  /**
   * Gets the name of the author of the latest build.
   *
   * @return The build author.
   */
  public String getBuildAuthor() {
    return buildAuthor;
  }

  /**
   * Publishes the Git version information to a specified network table.
   *
   * @param tableName The name of the network table to publish to.
   */
  public void publishVersions(String tableName) {
    NetworkTable table = NetworkTableInstance.getDefault().getTable(tableName);

    if (table == null) {
      DriverStation.reportError("Could not find NT Table \"" + tableName + "\"", null);
      return;
    }

    table.getEntry("Build Date").setString(BUILD_DATE_FORMATTER.format(buildDate));
    table.getEntry("Build Author").setString(buildAuthor);
    table.getEntry("Current Commit").setString(lastCommit);
    table.getEntry("Modified").setBoolean(isModified);
  }

  /**
   * Loads the Git version information from a file.
   *
   * @param filename The name of the file containing the Git version information.
   * @return A GitVersion instance with the loaded data, or default values if loading fails.
   */
  public static GitVersion loadVersion(String filename) {
    String path = Filesystem.getDeployDirectory() + "/" + filename;
    GitVersion version;

    try {
      FileInputStream fileInputStream = new FileInputStream(path);
      ObjectInputStream objectInputStream = new ObjectInputStream(fileInputStream);
      version = (GitVersion) objectInputStream.readObject();
      objectInputStream.close();
    } catch (Exception ignored) {
      version = GitVersion.fromMetadata("Unknown", false, new Date(0), "Unknown");
    }

    return version;
  }

  /**
   * Executes a shell command and returns its output as a string.
   *
   * @param command The shell command to execute.
   * @param runtime The runtime environment to execute the command in.
   * @return The output of the command as a string.
   */
  private static String executeCommand(String command, Runtime runtime) {
    Process process;

    try {
      process = runtime.exec(command);
      process.waitFor();

      return new String(process.getInputStream().readAllBytes());
    } catch (InterruptedException | IOException exc) {
      System.out.println(exc);
      System.exit(1);

      return ""; // Practically will never be reached, compiler doesn't know this though
    }
  }

  /**
   * Writes an object to a specified file location.
   *
   * @param location The file location to write to.
   * @param object The object to write.
   */
  private static void writeObject(String location, Object object) {
    try {
      FileOutputStream file = new FileOutputStream(location);
      ObjectOutputStream objectStream = new ObjectOutputStream(file);

      objectStream.writeObject(object);
      objectStream.close();
    } catch (IOException exc) {
      System.exit(1);
    }
  }

  /**
   * The main method, designed to be called from Gradle, for creating and saving Git version
   * metadata.
   *
   * @param args Command-line arguments (unused).
   */
  public static void main(String[] args) {
    Runtime runtime = Runtime.getRuntime();
    Date buildDate = new Date();

    String buildAuthor = executeCommand("git config user.name", runtime).replace("\n", "");
    String lastCommit = executeCommand("git rev-parse --short HEAD", runtime).replace("\n", "");
    boolean isModified = executeCommand("git status -s", runtime).length() > 0;

    GitVersion version = GitVersion.fromMetadata(lastCommit, isModified, buildDate, buildAuthor);
    writeObject("src/main/deploy/gitinfo.obj", version);

    System.exit(0);
  }
}

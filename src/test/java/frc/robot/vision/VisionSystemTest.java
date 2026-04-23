package frc.robot.vision;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.ArgumentMatchers.*;
import static org.mockito.Mockito.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Robot;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import org.junit.jupiter.api.*;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.ArgumentCaptor;
import org.mockito.Mock;
import org.mockito.MockedStatic;
import org.mockito.Mockito;
import org.mockito.junit.jupiter.MockitoExtension;

@ExtendWith(MockitoExtension.class)
class VisionSystemTest {

  private static final Matrix<N3, N1> UNIT_STD_DEVS = VecBuilder.fill(1.0, 1.0, 1.0);
  private static final Matrix<N3, N1> LARGE_STD_DEVS = VecBuilder.fill(10.0, 10.0, 10.0);

  @Mock CommandSwerveDrivetrain mockDrivetrain;

  // isSimulation() is declared on RobotBase; Robot.get() is declared on Robot
  private MockedStatic<RobotBase> robotBaseMock;
  private MockedStatic<Robot> robotStaticMock;
  private Robot mockRobotInstance;
  private VisionSystem visionSystem;

  @BeforeAll
  static void initHal() {
    HAL.initialize(500, 0);
  }

  @BeforeEach
  void setUp() {
    robotBaseMock = Mockito.mockStatic(RobotBase.class);
    robotBaseMock.when(RobotBase::isSimulation).thenReturn(false);

    robotStaticMock = Mockito.mockStatic(Robot.class);
    mockRobotInstance = mock(Robot.class);
    robotStaticMock.when(Robot::get).thenReturn(mockRobotInstance);
    lenient().when(mockRobotInstance.getSwerve()).thenReturn(mockDrivetrain);

    visionSystem =
        new VisionSystem(Collections.emptyList(), false, mockDrivetrain, () -> 0.0, () -> 0.0);
  }

  @AfterEach
  void tearDown() {
    robotStaticMock.close();
    robotBaseMock.close();
  }

  // -------------------------------------------------------------------------
  // VisionEstimate record
  // -------------------------------------------------------------------------

  @Test
  void isAccepted_nullRejectionReason_returnsTrue() {
    var estimate = makeAcceptedEstimate(new Pose3d(), 1.0, UNIT_STD_DEVS);
    assertTrue(estimate.isAccepted());
  }

  @Test
  void isAccepted_nonNullRejectionReason_returnsFalse() {
    for (VisionSystem.RejectionReason reason : VisionSystem.RejectionReason.values()) {
      var estimate =
          new VisionSystem.VisionEstimate(new Pose3d(), 1.0, UNIT_STD_DEVS, 1, 0.0, 1.0, reason);
      assertFalse(estimate.isAccepted(), "Expected false for reason: " + reason);
    }
  }

  @Test
  void visionEstimate_recordAccessors_returnProvidedValues() {
    var pose = new Pose3d(new Pose2d(1.0, 2.0, Rotation2d.fromDegrees(45)));
    var stdDevs = VecBuilder.fill(0.1, 0.2, 0.3);
    var estimate =
        new VisionSystem.VisionEstimate(
            pose, 3.14, stdDevs, 2, 0.05, 2.5, VisionSystem.RejectionReason.HIGH_AMBIGUITY);

    assertEquals(pose, estimate.pose());
    assertEquals(3.14, estimate.timestampSeconds(), 1e-9);
    assertEquals(stdDevs, estimate.stdDevs());
    assertEquals(2, estimate.numTags());
    assertEquals(0.05, estimate.ambiguity(), 1e-9);
    assertEquals(2.5, estimate.distanceToTag(), 1e-9);
    assertEquals(VisionSystem.RejectionReason.HIGH_AMBIGUITY, estimate.rejectionReason());
  }

  // -------------------------------------------------------------------------
  // integrateMultipleEstimates
  // -------------------------------------------------------------------------

  @Test
  void integrateMultipleEstimates_emptyList_noDrivetrainCalls() {
    visionSystem.integrateMultipleEstimates(new ArrayList<>());
    verifyNoInteractions(mockDrivetrain);
  }

  @Test
  void integrateMultipleEstimates_singleRejectedEstimate_noDrivetrainCalls() {
    var rejected =
        new VisionSystem.VisionEstimate(
            new Pose3d(),
            1.0,
            UNIT_STD_DEVS,
            1,
            0.0,
            1.0,
            VisionSystem.RejectionReason.HIGH_AMBIGUITY);

    visionSystem.integrateMultipleEstimates(new ArrayList<>(List.of(rejected)));
    verifyNoInteractions(mockDrivetrain);
  }

  @Test
  void integrateMultipleEstimates_singleAcceptedEstimate_addsOneMeasurement() {
    var estimate =
        makeAcceptedEstimate(
            new Pose3d(new Pose2d(3.0, 4.0, new Rotation2d())), 1.0, UNIT_STD_DEVS);

    visionSystem.integrateMultipleEstimates(new ArrayList<>(List.of(estimate)));

    ArgumentCaptor<Pose2d> poseCaptor = ArgumentCaptor.forClass(Pose2d.class);
    verify(mockDrivetrain).addVisionMeasurement(poseCaptor.capture(), eq(1.0), any());
    assertEquals(3.0, poseCaptor.getValue().getX(), 1e-6);
    assertEquals(4.0, poseCaptor.getValue().getY(), 1e-6);
  }

  @Test
  void integrateMultipleEstimates_twoAcceptedFarApart_addsTwoMeasurements() {
    // timeDelta = 0.5 >= MAX_TIME_DELTA_SECONDS (0.1), so no fusing
    var a =
        makeAcceptedEstimate(
            new Pose3d(new Pose2d(1.0, 0.0, new Rotation2d())), 1.0, UNIT_STD_DEVS);
    var b =
        makeAcceptedEstimate(
            new Pose3d(new Pose2d(2.0, 0.0, new Rotation2d())), 1.5, UNIT_STD_DEVS);

    visionSystem.integrateMultipleEstimates(new ArrayList<>(List.of(a, b)));

    verify(mockDrivetrain, times(2)).addVisionMeasurement(any(), anyDouble(), any());
  }

  @Test
  void integrateMultipleEstimates_twoAcceptedCloseInTime_fusesIntoOneMeasurement() {
    // timeDelta = 0.05 < MAX_TIME_DELTA_SECONDS (0.1), triggers fuseEstimates
    when(mockDrivetrain.getPoseAtTimestamp(anyDouble())).thenReturn(new Pose2d());

    var a =
        makeAcceptedEstimate(
            new Pose3d(new Pose2d(0.0, 0.0, new Rotation2d())), 1.0, UNIT_STD_DEVS);
    var b =
        makeAcceptedEstimate(
            new Pose3d(new Pose2d(2.0, 0.0, new Rotation2d())), 1.05, UNIT_STD_DEVS);

    visionSystem.integrateMultipleEstimates(new ArrayList<>(List.of(a, b)));

    ArgumentCaptor<Pose2d> poseCaptor = ArgumentCaptor.forClass(Pose2d.class);
    verify(mockDrivetrain, times(1)).addVisionMeasurement(poseCaptor.capture(), anyDouble(), any());

    // With equal unit variances, fused X = (0 + 2) / 2 = 1.0
    assertEquals(1.0, poseCaptor.getValue().getX(), 1e-6);
  }

  @Test
  void integrateMultipleEstimates_sortsOutOfOrderByTimestamp() {
    // Provide out-of-order, far-apart estimates; verify all three are integrated
    var t3 =
        makeAcceptedEstimate(
            new Pose3d(new Pose2d(3.0, 0.0, new Rotation2d())), 3.0, UNIT_STD_DEVS);
    var t1 =
        makeAcceptedEstimate(
            new Pose3d(new Pose2d(1.0, 0.0, new Rotation2d())), 1.0, UNIT_STD_DEVS);
    var t2 =
        makeAcceptedEstimate(
            new Pose3d(new Pose2d(2.0, 0.0, new Rotation2d())), 2.0, UNIT_STD_DEVS);

    List<VisionSystem.VisionEstimate> list = new ArrayList<>(List.of(t3, t1, t2));
    visionSystem.integrateMultipleEstimates(list);

    ArgumentCaptor<Double> timestampCaptor = ArgumentCaptor.forClass(Double.class);
    verify(mockDrivetrain, times(3)).addVisionMeasurement(any(), timestampCaptor.capture(), any());
    List<Double> timestamps = timestampCaptor.getAllValues();
    assertEquals(1.0, timestamps.get(0), 1e-9);
    assertEquals(2.0, timestamps.get(1), 1e-9);
    assertEquals(3.0, timestamps.get(2), 1e-9);
  }

  @Test
  void integrateMultipleEstimates_skipsRejectedInMiddle() {
    // accepted, rejected, accepted — only the two accepted should be integrated
    var a = makeAcceptedEstimate(new Pose3d(), 1.0, UNIT_STD_DEVS);
    var rejected =
        new VisionSystem.VisionEstimate(
            new Pose3d(),
            1.5,
            UNIT_STD_DEVS,
            1,
            0.0,
            1.0,
            VisionSystem.RejectionReason.POSE_OUT_OF_FIELD);
    var b = makeAcceptedEstimate(new Pose3d(), 2.5, UNIT_STD_DEVS);

    visionSystem.integrateMultipleEstimates(new ArrayList<>(List.of(a, rejected, b)));

    verify(mockDrivetrain, times(2)).addVisionMeasurement(any(), anyDouble(), any());
  }

  @Test
  void integrateMultipleEstimates_allRejected_noDrivetrainCalls() {
    var r1 =
        new VisionSystem.VisionEstimate(
            new Pose3d(), 1.0, UNIT_STD_DEVS, 1, 0.0, 1.0, VisionSystem.RejectionReason.NO_DATA);
    var r2 =
        new VisionSystem.VisionEstimate(
            new Pose3d(),
            2.0,
            UNIT_STD_DEVS,
            1,
            0.0,
            1.0,
            VisionSystem.RejectionReason.DISTANCE_OUT_OF_RANGE);

    visionSystem.integrateMultipleEstimates(new ArrayList<>(List.of(r1, r2)));
    verifyNoInteractions(mockDrivetrain);
  }

  // -------------------------------------------------------------------------
  // fuseEstimates (tested indirectly via integrateMultipleEstimates)
  // -------------------------------------------------------------------------

  @Test
  void fuseEstimates_unequalVariances_weightsTowardLowerVariance() {
    when(mockDrivetrain.getPoseAtTimestamp(anyDouble())).thenReturn(new Pose2d());

    // A at x=0 with low variance (tight), B at x=4 with high variance (loose)
    // Weight of A = 1/1 = 1, weight of B = 1/100 = 0.01
    // fusedX = (0*1 + 4*0.01) / (1 + 0.01) = 0.04 / 1.01 ≈ 0.0396
    var tightDev = VecBuilder.fill(1.0, 1.0, 1.0);
    var looseDev = VecBuilder.fill(10.0, 10.0, 10.0);
    var a = makeAcceptedEstimate(new Pose3d(new Pose2d(0.0, 0.0, new Rotation2d())), 1.0, tightDev);
    var b =
        makeAcceptedEstimate(new Pose3d(new Pose2d(4.0, 0.0, new Rotation2d())), 1.05, looseDev);

    visionSystem.integrateMultipleEstimates(new ArrayList<>(List.of(a, b)));

    ArgumentCaptor<Pose2d> poseCaptor = ArgumentCaptor.forClass(Pose2d.class);
    verify(mockDrivetrain).addVisionMeasurement(poseCaptor.capture(), anyDouble(), any());

    double fusedX = poseCaptor.getValue().getX();
    // Fused result should be much closer to A (x=0) than B (x=4)
    assertTrue(
        fusedX < 1.0, "Fused X should be pulled toward lower-variance estimate, was: " + fusedX);
  }

  @Test
  void fuseEstimates_reverseOrder_producesIdenticalResult() {
    when(mockDrivetrain.getPoseAtTimestamp(anyDouble())).thenReturn(new Pose2d());

    var a =
        makeAcceptedEstimate(
            new Pose3d(new Pose2d(0.0, 0.0, new Rotation2d())), 1.0, UNIT_STD_DEVS);
    var b =
        makeAcceptedEstimate(
            new Pose3d(new Pose2d(2.0, 0.0, new Rotation2d())), 1.05, UNIT_STD_DEVS);

    // Forward order
    visionSystem.integrateMultipleEstimates(new ArrayList<>(List.of(a, b)));
    ArgumentCaptor<Pose2d> captor1 = ArgumentCaptor.forClass(Pose2d.class);
    verify(mockDrivetrain, times(1)).addVisionMeasurement(captor1.capture(), anyDouble(), any());
    double forwardX = captor1.getValue().getX();

    reset(mockDrivetrain);
    when(mockDrivetrain.getPoseAtTimestamp(anyDouble())).thenReturn(new Pose2d());

    // Reverse order: fuseEstimates should swap a and b internally
    visionSystem.integrateMultipleEstimates(new ArrayList<>(List.of(b, a)));
    ArgumentCaptor<Pose2d> captor2 = ArgumentCaptor.forClass(Pose2d.class);
    verify(mockDrivetrain, times(1)).addVisionMeasurement(captor2.capture(), anyDouble(), any());
    double reverseX = captor2.getValue().getX();

    assertEquals(forwardX, reverseX, 1e-6);
  }

  // -------------------------------------------------------------------------
  // fuseEstimates — NaN bug triggered by MAX_STD_DEVS accepted estimates
  // -------------------------------------------------------------------------

  /**
   * Production bug: calculateStdDevs() returns MAX_STD_DEVS=[Double.MAX_VALUE,...] for ACCEPTED
   * estimates when no targets are found in the field layout (numTags==0) or a single tag is too far
   * from the estimated pose (numTags==1 && avgDist > 4m). These estimates reach fuseEstimates with
   * a null rejection reason.
   *
   * <p>When two such estimates are fused: Double.MAX_VALUE * Double.MAX_VALUE overflows to
   * Infinity, so variance = Infinity, weight = 1/Infinity = 0. Then (x*0 + y*0) / (0+0) = 0/0 =
   * NaN.
   *
   * <p>With three simultaneous camera estimates this becomes a cascade: fuse(MAX_A, MAX_B) → NaN
   * pose, then fuse(NaN, GOOD_C) → NaN propagates because NaN*0 = NaN in IEEE 754, poisoning the
   * valid estimate.
   */
  @Test
  void fuseEstimates_twoMaxStdDevsAccepted_producesFiniteAverage() {
    when(mockDrivetrain.getPoseAtTimestamp(anyDouble())).thenReturn(new Pose2d());

    // MAX_STD_DEVS is now 1e9 (not Double.MAX_VALUE). (1e9)^2 = 1e18 is finite, so
    // weight = 1/1e18 is tiny but non-zero → equal weights → arithmetic average, no NaN.
    var maxDevs = VecBuilder.fill(1e9, 1e9, 1e9);
    var a =
        new VisionSystem.VisionEstimate(
            new Pose3d(new Pose2d(1.0, 0.0, new Rotation2d())), 1.0, maxDevs, 1, 0.0, 1.0, null);
    var b =
        new VisionSystem.VisionEstimate(
            new Pose3d(new Pose2d(2.0, 0.0, new Rotation2d())), 1.05, maxDevs, 1, 0.0, 1.0, null);

    visionSystem.integrateMultipleEstimates(new ArrayList<>(List.of(a, b)));

    ArgumentCaptor<Pose2d> poseCaptor = ArgumentCaptor.forClass(Pose2d.class);
    verify(mockDrivetrain).addVisionMeasurement(poseCaptor.capture(), anyDouble(), any());

    assertFalse(Double.isNaN(poseCaptor.getValue().getX()), "Should not be NaN with 1e9 stdDevs");
    // Equal weights → fused x = (1.0 + 2.0) / 2 = 1.5
    assertEquals(1.5, poseCaptor.getValue().getX(), 1e-6);
  }

  @Test
  void fuseEstimates_threeEstimates_twoMaxStdDevs_validThirdDominates() {
    when(mockDrivetrain.getPoseAtTimestamp(anyDouble())).thenReturn(new Pose2d());

    // Scenario: 3 cameras fire simultaneously. Two return MAX_STD_DEVS (1e9) accepted estimates;
    // the third is a good estimate with stdDev=1.0.
    // fuse(A,B) → x=1.5, stdDev≈707M.  Then fuse(AB, C): weight_C = 1.0, weight_AB ≈ 2e-18.
    // C dominates completely; fused x ≈ 4.0. No NaN.
    var maxDevs = VecBuilder.fill(1e9, 1e9, 1e9);
    var a =
        new VisionSystem.VisionEstimate(
            new Pose3d(new Pose2d(1.0, 0.0, new Rotation2d())), 1.0, maxDevs, 1, 0.0, 1.0, null);
    var b =
        new VisionSystem.VisionEstimate(
            new Pose3d(new Pose2d(2.0, 0.0, new Rotation2d())), 1.02, maxDevs, 1, 0.0, 1.0, null);
    var c =
        makeAcceptedEstimate(
            new Pose3d(new Pose2d(4.0, 0.0, new Rotation2d())), 1.04, UNIT_STD_DEVS);

    visionSystem.integrateMultipleEstimates(new ArrayList<>(List.of(a, b, c)));

    ArgumentCaptor<Pose2d> poseCaptor = ArgumentCaptor.forClass(Pose2d.class);
    verify(mockDrivetrain).addVisionMeasurement(poseCaptor.capture(), anyDouble(), any());

    assertFalse(Double.isNaN(poseCaptor.getValue().getX()), "Should not be NaN after fix");
    // C's weight (1.0) overwhelms A+B's combined weight (≈2e-18); result is effectively x=4.0
    assertEquals(4.0, poseCaptor.getValue().getX(), 1e-3);
  }

  // -------------------------------------------------------------------------
  // getEstimationStdDevs
  // -------------------------------------------------------------------------

  @Test
  void getEstimationStdDevs_initiallyNull() {
    assertNull(visionSystem.getEstimationStdDevs());
  }

  // -------------------------------------------------------------------------
  // updateCameraSettings / resetPose — no limelight, should be no-ops
  // -------------------------------------------------------------------------

  @Test
  void updateCameraSettings_noLimelight_doesNotThrow() {
    assertDoesNotThrow(() -> visionSystem.updateCameraSettings());
  }

  @Test
  void resetPose_noLimelight_doesNotThrow() {
    assertDoesNotThrow(() -> visionSystem.resetPose());
  }

  // -------------------------------------------------------------------------
  // Geometry getters
  // -------------------------------------------------------------------------

  @Test
  void robotToTurretCenter_hasExpectedValues() {
    Translation2d t = visionSystem.getRobotToTurretCenter();
    assertEquals(Units.inchesToMeters(-6.25), t.getX(), 1e-6);
    assertEquals(Units.inchesToMeters(-6.25), t.getY(), 1e-6);
  }

  @Test
  void turretCenterToCamera_hasExpectedValues() {
    Translation2d t = visionSystem.getTurretCenterToCamera();
    assertEquals(Units.inchesToMeters(6.877436), t.getX(), 1e-6);
    assertEquals(0.0, t.getY(), 1e-6);
  }

  // -------------------------------------------------------------------------
  // Helper
  // -------------------------------------------------------------------------

  private static VisionSystem.VisionEstimate makeAcceptedEstimate(
      Pose3d pose, double timestamp, Matrix<N3, N1> stdDevs) {
    return new VisionSystem.VisionEstimate(pose, timestamp, stdDevs, 1, 0.0, 1.0, null);
  }

  private static VisionSystem.VisionEstimate makeAcceptedEstimate(Pose3d pose, double timestamp) {
    return makeAcceptedEstimate(pose, timestamp, UNIT_STD_DEVS);
  }
}

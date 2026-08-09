#include "../../Robot.h"
#include "FrontMatrixPolicy.h"

namespace {

struct MatrixTrackState {
  bool active;
  uint32_t id;
  uint32_t lastFrameSequence;
  unsigned long lastAcquiredMs;
  float lastWorldX;
  float lastWorldY;
  uint8_t staticFrames;
  uint8_t movingFrames;
  uint8_t lossFrames;
};

struct ColumnProfile {
  bool weightStep;
  bool wallLike;
  bool rampLike;
  float nearestLowMm;
  float nearestRobotXmm;
  float nearestRobotYmm;
  float minimumHeightMm;
  float maximumHeightMm;
  uint64_t lowMask;
};

struct MatrixPointMm {
  float x;
  float y;
  float z;
  float horizontalRange;
};

struct MatrixCandidate {
  bool valid;
  uint8_t firstColumn;
  uint8_t lastColumn;
  uint8_t supportColumns;
  uint64_t sourceMask;
  float weightedColumn;
  float closestHorizontalMm;
  float minimumHeightMm;
  float maximumHeightMm;
  float robotXmm;
  float robotYmm;
  float worldX;
  float worldY;
  float widthMm;
  float heightMm;
};

MatrixTrackState matrixTrack = {};
uint32_t nextMatrixTrackId = 1;
uint32_t activePickupTrackId = 0;

void invalidateTarget(MatrixEvidenceKind evidence, uint32_t frameSequence) {
  matrixTargetObservation.valid = false;
  matrixTargetObservation.confirmedStatic = false;
  matrixTargetObservation.evidence = evidence;
  matrixTargetObservation.frameSequence = frameSequence;
}

float continuousColumnAngleDeg(float columnCoordinate) {
  return FRONT_MATRIX_CONFIG.horizontalFovDeg * 0.5f -
    (columnCoordinate + 0.5f) *
      (FRONT_MATRIX_CONFIG.horizontalFovDeg / FRONT_MATRIX_CONFIG.columns);
}

void rotateRollPitch(float rollDeg, float pitchDeg,
                     float &x, float &y, float &z) {
  const float rollRad = rollDeg * DEG_TO_RAD;
  const float pitchRad = pitchDeg * DEG_TO_RAD;
  const float rolledY = cosf(rollRad) * y - sinf(rollRad) * z;
  const float rolledZ = sinf(rollRad) * y + cosf(rollRad) * z;
  y = rolledY;
  z = rolledZ;
  const float pitchedX = cosf(pitchRad) * x - sinf(pitchRad) * z;
  const float pitchedZ = sinf(pitchRad) * x + cosf(pitchRad) * z;
  x = pitchedX;
  z = pitchedZ;
}

void rotateYaw(float yawDeg, float &x, float &y) {
  const float yawRad = yawDeg * DEG_TO_RAD;
  const float rotatedX = cosf(yawRad) * x - sinf(yawRad) * y;
  const float rotatedY = sinf(yawRad) * x + cosf(yawRad) * y;
  x = rotatedX;
  y = rotatedY;
}

MatrixPointMm reconstructMatrixPoint(const FrontMatrixFrame &frame,
                                     uint8_t row, uint8_t column,
                                     uint16_t distanceMm) {
  const float horizontalRad = continuousColumnAngleDeg(column) * DEG_TO_RAD;
  const float verticalRad = frontMatrixVerticalAngleDeg(
    row, FRONT_MATRIX_CONFIG.verticalFovDeg) * DEG_TO_RAD;
  float rayX = cosf(verticalRad) * cosf(horizontalRad);
  float rayY = cosf(verticalRad) * sinf(horizontalRad);
  float rayZ = sinf(verticalRad);
  rotateRollPitch(FRONT_MATRIX_CONFIG.mount.rollDeg,
                  FRONT_MATRIX_CONFIG.mount.pitchDeg,
                  rayX, rayY, rayZ);
  rotateYaw(FRONT_MATRIX_CONFIG.mount.yawDeg, rayX, rayY);

  float mountX = FRONT_MATRIX_CONFIG.mount.xMm;
  float mountY = FRONT_MATRIX_CONFIG.mount.yMm;
  float mountZ = FRONT_MATRIX_CONFIG.mount.zMm;
  rotateRollPitch(frame.rollDeg, frame.pitchDeg,
                  mountX, mountY, mountZ);
  rotateRollPitch(frame.rollDeg, frame.pitchDeg,
                  rayX, rayY, rayZ);
  return {
    mountX + distanceMm * rayX,
    mountY + distanceMm * rayY,
    mountZ + distanceMm * rayZ,
    distanceMm * hypotf(rayX, rayY)
  };
}

void transformCandidate(const FrontMatrixFrame &frame,
                        MatrixCandidate &candidate) {
  const float headingRad = frame.robotHeadingDeg * DEG_TO_RAD;
  candidate.worldX = frame.robotX +
    (candidate.robotXmm * cosf(headingRad) -
     candidate.robotYmm * sinf(headingRad)) / 1000.0f;
  candidate.worldY = frame.robotY +
    (candidate.robotXmm * sinf(headingRad) +
     candidate.robotYmm * cosf(headingRad)) / 1000.0f;
}

bool candidateAssociated(const MatrixCandidate &candidate) {
  return matrixTrack.active &&
    hypotf((candidate.worldX - matrixTrack.lastWorldX) * 1000.0f,
           (candidate.worldY - matrixTrack.lastWorldY) * 1000.0f) <=
      MATRIX_TRACK_ASSOCIATION_DISTANCE_MM;
}

}  // namespace

void setMatrixPickupTrackId(uint32_t trackId) {
  activePickupTrackId = trackId;
}

bool matrixCellBelongsToActivePickup(uint8_t cellIndex) {
  if (activePickupTrackId == 0 || cellIndex >= 64 ||
      matrixTargetObservation.trackId != activePickupTrackId) return false;
  const int row = cellIndex / 8;
  const int column = cellIndex % 8;
  for (uint8_t source = 0; source < 64; source++) {
    if ((matrixTargetObservation.sourceCellMask &
         (UINT64_C(1) << source)) == 0) continue;
    if (abs(row - (int)(source / 8)) <= 1 &&
        abs(column - (int)(source % 8)) <= 1) return true;
  }
  return false;
}

const char* matrixEvidenceKindName(MatrixEvidenceKind kind) {
  switch (kind) {
    case MATRIX_EVIDENCE_NONE: return "none";
    case MATRIX_EVIDENCE_WEIGHT_CANDIDATE: return "weight_candidate";
    case MATRIX_EVIDENCE_RAMP_LIKE: return "ramp_like";
    case MATRIX_EVIDENCE_WALL_LIKE: return "wall_like";
    case MATRIX_EVIDENCE_DYNAMIC_LOW_OBJECT: return "dynamic_low_object";
    case MATRIX_EVIDENCE_MIXED_OR_OCCLUDED: return "mixed_or_occluded";
    case MATRIX_EVIDENCE_UNKNOWN: return "unknown";
  }
  return "unknown";
}

void updateMatrixWeightDetection() {
  FrontMatrixFrame frame;
  if (!getFrontMatrixFrame(frame) ||
      frame.sequence == matrixTargetObservation.frameSequence) return;

  ColumnProfile profiles[8] = {};
  bool anyLowEvidence = false;
  bool anyWallEvidence = false;
  bool anyRampEvidence = false;
  for (uint8_t column = 0; column < 8; column++) {
    ColumnProfile &profile = profiles[column];
    profile.nearestLowMm = 1000000.0f;
    profile.minimumHeightMm = 1000000.0f;
    profile.maximumHeightMm = -1000000.0f;
    float nearestUpperMm = 1000000.0f;
    float minimumRowRangeMm = 1000000.0f;
    float maximumRowRangeMm = 0.0f;
    uint8_t validProfilePoints = 0;
    for (uint8_t row = 0; row < 8; row++) {
      const uint8_t index = row * 8U + column;
      if ((FRONT_MATRIX_CONFIG.perceptionCellMask &
           (UINT64_C(1) << index)) == 0 ||
          frame.cellState[index] != FRONT_MATRIX_CELL_VALID) continue;
      const MatrixPointMm point = reconstructMatrixPoint(
        frame, row, column, frame.distanceMm[index]);
      const float horizontalMm = point.horizontalRange;
      const float hitHeightMm = point.z;
      minimumRowRangeMm = min(minimumRowRangeMm, horizontalMm);
      maximumRowRangeMm = max(maximumRowRangeMm, horizontalMm);
      validProfilePoints++;
      if (hitHeightMm >= MATRIX_WEIGHT_MIN_HEIGHT_MM * 0.5f &&
          hitHeightMm <= MATRIX_WEIGHT_MAX_HEIGHT_MM) {
        anyLowEvidence = true;
        if (horizontalMm < profile.nearestLowMm) {
          profile.nearestLowMm = horizontalMm;
          profile.nearestRobotXmm = point.x;
          profile.nearestRobotYmm = point.y;
        }
        profile.minimumHeightMm = min(profile.minimumHeightMm, hitHeightMm);
        profile.maximumHeightMm = max(profile.maximumHeightMm, hitHeightMm);
        profile.lowMask |= UINT64_C(1) << index;
      } else if (hitHeightMm > MATRIX_WEIGHT_MAX_HEIGHT_MM) {
        nearestUpperMm = min(nearestUpperMm, horizontalMm);
      }
    }
    if (profile.lowMask == 0) continue;
    profile.weightStep = nearestUpperMm < 1000000.0f &&
      nearestUpperMm - profile.nearestLowMm >= MATRIX_WEIGHT_MIN_DEPTH_STEP_MM;
    profile.wallLike = nearestUpperMm < 1000000.0f &&
      fabsf(nearestUpperMm - profile.nearestLowMm) <
        MATRIX_WEIGHT_MIN_DEPTH_STEP_MM;
    // A smooth depth sweep through several height samples is ramp evidence;
    // unlike a weight, it has no abrupt low-to-upper depth discontinuity.
    profile.rampLike = validProfilePoints >= 3 && !profile.weightStep &&
      maximumRowRangeMm - minimumRowRangeMm >=
        MATRIX_WEIGHT_MIN_DEPTH_STEP_MM;
    anyWallEvidence = anyWallEvidence || profile.wallLike;
    anyRampEvidence = anyRampEvidence || profile.rampLike;
  }

  MatrixCandidate candidates[4] = {};
  uint8_t candidateCount = 0;
  uint8_t column = 0;
  while (column < 8 && candidateCount < 4) {
    while (column < 8 && !profiles[column].weightStep) column++;
    if (column >= 8) break;
    MatrixCandidate &candidate = candidates[candidateCount];
    candidate.valid = true;
    candidate.firstColumn = column;
    candidate.closestHorizontalMm = 1000000.0f;
    candidate.minimumHeightMm = 1000000.0f;
    candidate.maximumHeightMm = -1000000.0f;
    float weightedSum = 0.0f;
    float totalWeight = 0.0f;
    float weightedRobotXmm = 0.0f;
    float weightedRobotYmm = 0.0f;
    while (column < 8 && profiles[column].weightStep) {
      const ColumnProfile &profile = profiles[column];
      const float weight = 1.0f / max(1.0f, profile.nearestLowMm);
      weightedSum += column * weight;
      totalWeight += weight;
      weightedRobotXmm += profile.nearestRobotXmm * weight;
      weightedRobotYmm += profile.nearestRobotYmm * weight;
      candidate.supportColumns++;
      candidate.lastColumn = column;
      candidate.sourceMask |= profile.lowMask;
      candidate.closestHorizontalMm = min(candidate.closestHorizontalMm,
                                           profile.nearestLowMm);
      candidate.minimumHeightMm = min(candidate.minimumHeightMm,
                                       profile.minimumHeightMm);
      candidate.maximumHeightMm = max(candidate.maximumHeightMm,
                                       profile.maximumHeightMm);
      column++;
    }
    candidate.weightedColumn = weightedSum / max(0.000001f, totalWeight);
    candidate.robotXmm = weightedRobotXmm / max(0.000001f, totalWeight);
    candidate.robotYmm = weightedRobotYmm / max(0.000001f, totalWeight);
    const float halfWidthAngleRad =
      candidate.supportColumns * FRONT_MATRIX_CONFIG.horizontalFovDeg /
      FRONT_MATRIX_CONFIG.columns * MATRIX_CELL_EFFECTIVE_WIDTH_FRACTION *
      0.5f * DEG_TO_RAD;
    candidate.widthMm = 2.0f * candidate.closestHorizontalMm *
                        tanf(halfWidthAngleRad);
    // Sparse vertical zones usually see only part of a short cylinder. Treat
    // the observed span as a lower bound and retain the nominal 70 mm class
    // height until physical calibration provides a zone-footprint estimator.
    candidate.heightMm = max(
      70.0f, candidate.maximumHeightMm - candidate.minimumHeightMm);
    candidate.valid = candidate.widthMm >= MATRIX_WEIGHT_MIN_WIDTH_MM &&
      candidate.widthMm <= MATRIX_WEIGHT_MAX_WIDTH_MM &&
      candidate.heightMm >= MATRIX_WEIGHT_MIN_HEIGHT_MM &&
      candidate.heightMm <= MATRIX_WEIGHT_MAX_HEIGHT_MM;
    transformCandidate(frame, candidate);
    candidateCount++;
  }

  int8_t selected = -1;
  float bestScore = 1000000.0f;
  for (uint8_t i = 0; i < candidateCount; i++) {
    if (!candidates[i].valid) continue;
    const bool associated = candidateAssociated(candidates[i]);
    if (activePickupTrackId != 0 && matrixTrack.active && !associated) continue;
    const float score = activePickupTrackId != 0
      ? hypotf((candidates[i].worldX - matrixTrack.lastWorldX) * 1000.0f,
               (candidates[i].worldY - matrixTrack.lastWorldY) * 1000.0f)
      : candidates[i].robotXmm;
    if (score < bestScore) {
      bestScore = score;
      selected = (int8_t)i;
    }
  }

  if (selected < 0) {
    if (matrixTrack.active && frame.acquiredMs - matrixTrack.lastAcquiredMs <=
          MATRIX_TRACK_STALE_TIMEOUT_MS) {
      if (matrixTrack.lossFrames < 255) matrixTrack.lossFrames++;
      if (matrixTrack.lossFrames < MATRIX_TARGET_LOSS_CONFIRM_FRAMES) {
        matrixTargetObservation.frameSequence = frame.sequence;
        matrixTargetObservation.acquiredMs = frame.acquiredMs;
        matrixTargetObservation.evidence = MATRIX_EVIDENCE_MIXED_OR_OCCLUDED;
      } else {
        invalidateTarget(MATRIX_EVIDENCE_UNKNOWN, frame.sequence);
      }
    } else {
      matrixTrack = {};
      invalidateTarget(anyRampEvidence ? MATRIX_EVIDENCE_RAMP_LIKE :
        (anyWallEvidence ? MATRIX_EVIDENCE_WALL_LIKE :
         (anyLowEvidence ? MATRIX_EVIDENCE_MIXED_OR_OCCLUDED :
                           MATRIX_EVIDENCE_NONE)), frame.sequence);
    }
    return;
  }

  MatrixCandidate &candidate = candidates[selected];
  matrixTrack.lossFrames = 0;
  float apparentSpeedMps = 0.0f;
  const bool associated = candidateAssociated(candidate);
  if (!associated) {
    // Once a hunt is latched, disappearance or a different candidate is loss,
    // never permission to switch targets.
    if (activePickupTrackId != 0) {
      invalidateTarget(MATRIX_EVIDENCE_UNKNOWN, frame.sequence);
      return;
    }
    matrixTrack = {};
    matrixTrack.active = true;
    matrixTrack.id = nextMatrixTrackId++;
    if (nextMatrixTrackId == 0) nextMatrixTrackId = 1;
  } else {
    const unsigned long dtMs = frame.acquiredMs - matrixTrack.lastAcquiredMs;
    if (dtMs > 0) {
      const float displacementMm = hypotf(
        (candidate.worldX - matrixTrack.lastWorldX) * 1000.0f,
        (candidate.worldY - matrixTrack.lastWorldY) * 1000.0f);
      apparentSpeedMps = max(0.0f,
        displacementMm - MATRIX_TRACK_POSITION_NOISE_MM) / dtMs;
    }
  }
  if (apparentSpeedMps > MATRIX_MAX_STATIC_TARGET_SPEED_MPS) {
    if (matrixTrack.movingFrames < 255) matrixTrack.movingFrames++;
    matrixTrack.staticFrames = 0;
  } else {
    if (matrixTrack.staticFrames < 255) matrixTrack.staticFrames++;
    matrixTrack.movingFrames = 0;
  }
  matrixTrack.lastFrameSequence = frame.sequence;
  matrixTrack.lastAcquiredMs = frame.acquiredMs;
  matrixTrack.lastWorldX = candidate.worldX;
  matrixTrack.lastWorldY = candidate.worldY;

  const bool dynamic = matrixTrack.movingFrames >= MATRIX_MOVING_CONFIRM_FRAMES;
  matrixTargetObservation = {
    !dynamic, matrixTrack.staticFrames >= MATRIX_STATIC_CONFIRM_FRAMES,
    matrixTrack.id, frame.sequence, frame.acquiredMs, candidate.sourceMask,
    dynamic ? MATRIX_EVIDENCE_DYNAMIC_LOW_OBJECT
            : MATRIX_EVIDENCE_WEIGHT_CANDIDATE,
    candidate.robotXmm, candidate.robotYmm,
    candidate.worldX, candidate.worldY,
    candidate.widthMm, candidate.heightMm,
    candidate.weightedColumn, candidate.weightedColumn - 3.5f,
    max(0.0f, candidate.robotXmm -
      ROBOT_FOOTPRINT_GEOMETRY.frontExtentMm),
    apparentSpeedMps
  };
}

bool getMatrixTargetObservation(MatrixTargetObservation &observation) {
  if (!matrixTargetObservation.valid ||
      !matrixTargetObservation.confirmedStatic ||
      millis() - matrixTargetObservation.acquiredMs >
        MATRIX_TRACK_STALE_TIMEOUT_MS) return false;
  observation = matrixTargetObservation;
  return true;
}

void printMatrixTelemetry() {
  FrontMatrixFrame frame;
  if (!getFrontMatrixFrame(frame)) {
    Serial2.println("front_matrix,valid=0");
    return;
  }
  Serial2.print("front_matrix,valid=1,sequence=");
  Serial2.print(frame.sequence);
  Serial2.print(",age_ms=");
  Serial2.print(millis() - frame.acquiredMs);
  Serial2.print(",evidence=");
  Serial2.print(matrixEvidenceKindName(matrixTargetObservation.evidence));
  Serial2.print(",track=");
  Serial2.print(matrixTargetObservation.trackId);
  Serial2.print(",confirmed=");
  Serial2.print(matrixTargetObservation.confirmedStatic ? 1 : 0);
  Serial2.print(",gap_mm=");
  Serial2.print(matrixTargetObservation.directGapMm, 1);
  Serial2.print(",column=");
  Serial2.println(matrixTargetObservation.column, 2);
  Serial2.println("matrix_row,d0,d1,d2,d3,d4,d5,d6,d7");
  for (uint8_t row = 0; row < 8; row++) {
    Serial2.print(row);
    for (uint8_t columnIndex = 0; columnIndex < 8; columnIndex++) {
      const uint8_t index = row * 8U + columnIndex;
      Serial2.print(",");
      if (frame.cellState[index] == FRONT_MATRIX_CELL_VALID) {
        Serial2.print(frame.distanceMm[index]);
      } else {
        Serial2.print("?");
      }
    }
    Serial2.println();
  }
}

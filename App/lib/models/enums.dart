enum RiskCategory {
  normal(0, 'NORMAL'),
  moderate(1, 'MODERATE'),
  high(2, 'HIGH'),
  veryHigh(3, 'VERY_HIGH'),
  unknown(255, 'UNKNOWN');

  const RiskCategory(this.value, this.label);
  final int value;
  final String label;

  static RiskCategory fromValue(int value) {
    for (final category in RiskCategory.values) {
      if (category.value == value) {
        return category;
      }
    }
    return RiskCategory.unknown;
  }
}

enum BuzzerState {
  off(0, 'OFF'),
  longBeep(1, 'LONG_BEEP'),
  shortBeep(2, 'SHORT_BEEP'),
  unknown(255, 'UNKNOWN');

  const BuzzerState(this.value, this.label);
  final int value;
  final String label;

  static BuzzerState fromValue(int value) {
    for (final state in BuzzerState.values) {
      if (state.value == value) {
        return state;
      }
    }
    return BuzzerState.unknown;
  }
}

enum PpgContactState {
  unknown(0, 'UNKNOWN'),
  none(1, 'NONE'),
  weak(2, 'WEAK'),
  valid(3, 'VALID');

  const PpgContactState(this.value, this.label);
  final int value;
  final String label;

  static PpgContactState fromValue(int value) {
    for (final state in PpgContactState.values) {
      if (state.value == value) {
        return state;
      }
    }
    return PpgContactState.unknown;
  }
}

enum InvalidReason {
  none(0, 'NONE'),
  noContact(1, 'NO_CONTACT'),
  weakContact(2, 'WEAK_CONTACT'),
  insufficientSamples(3, 'INSUFFICIENT_SAMPLES'),
  signalWeak(4, 'SIGNAL_WEAK'),
  signalSaturated(5, 'SIGNAL_SATURATED'),
  signalNoisy(6, 'SIGNAL_NOISY'),
  unstableBeats(7, 'UNSTABLE_BEATS'),
  invalidRatio(8, 'INVALID_RATIO'),
  unknown(255, 'UNKNOWN');

  const InvalidReason(this.value, this.label);
  final int value;
  final String label;

  static InvalidReason fromValue(int value) {
    for (final reason in InvalidReason.values) {
      if (reason.value == value) {
        return reason;
      }
    }
    return InvalidReason.unknown;
  }
}

class ValidityFlags {
  static const int rawBodyTemp = 1 << 0;
  static const int filteredBodyTemp = 1 << 1;
  static const int estimatedBodyTemp = 1 << 2;
  static const int ambientTemp = 1 << 3;
  static const int humidity = 1 << 4;
  static const int pressure = 1 << 5;
  static const int heatIndex = 1 << 6;
  static const int heartRate = 1 << 7;
  static const int spo2 = 1 << 8;
  static const int hrConfidence = 1 << 9;
  static const int spo2Confidence = 1 << 10;
}

extension ValidityFlagHelpers on int {
  bool hasFlag(int flag) => (this & flag) != 0;
}

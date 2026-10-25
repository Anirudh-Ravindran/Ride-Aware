// Driver safety score. Same structure as Tesla's published Safety Score:
//   PCF   = base x m1^x1 x m2^x2 x ...      (x = each factor as a percentage)
//   Score = 115.382324 - 22.526504 x PCF    (clamped 0..100)
// base, hardBrake and turn multipliers are Tesla's published values. The g thresholds are
// scooter-adapted starting points, and the proximity multiplier is our own. None of this is
// validated on scooter data yet; calibrate once real rides are logged.
//
// Each factor = % of ride time past the "hard" threshold, relative to time past a lower "base"
// threshold (this is how Tesla defines it: >0.3g relative to >0.1g for braking, >0.4g vs >0.2g
// for turning), so a gentle ride with little braking isn't unfairly dominated by one jolt.
const CFG = {
  base: 0.682854,
  mult: { hardBrake: 1.127294, turn: 1.019630, prox: 1.03 },
  a: 115.382324, b: 22.526504,
  // Used by the device/phone when it builds the metrics (documented here as the single source of truth)
  thresholds: {
    brakeG: { hard: 0.25, base: 0.10 },   // backward acceleration
    turnG:  { hard: 0.30, base: 0.15 },   // left/right acceleration
    proxLevel: { hard: 7, base: 4 },      // ultrasonic ThreatMeter level
  },
};
const clamp = (x) => Math.max(0, Math.min(100, x));
const PRIOR_S = 20;   // pseudo-seconds added to the base time so a few seconds of data can't swing the score
const pct = (n, d) => (100 * n) / (d + PRIOR_S);

const METRIC_KEYS = ['brakeBase_s', 'brakeHard_s', 'turnBase_s', 'turnHard_s', 'proxBase_s', 'proxHigh_s'];

function factors(m) {
  return {
    hardBrake: pct(m.brakeHard_s, m.brakeBase_s),
    turn: pct(m.turnHard_s, m.turnBase_s),
    prox: pct(m.proxHigh_s, m.proxBase_s),
  };
}

function scoreMetrics(m) {
  const f = factors(m);
  let pcf = CFG.base;
  const lost = {};  // points each factor costs on its own
  for (const k in f) {
    const mk = Math.pow(CFG.mult[k], f[k]);
    pcf *= mk;
    lost[k] = Math.round(100 - clamp(CFG.a - CFG.b * CFG.base * mk));
  }
  return { score: Math.round(clamp(CFG.a - CFG.b * pcf)), pcf, factors: f, lost };
}

module.exports = { CFG, METRIC_KEYS, scoreMetrics };

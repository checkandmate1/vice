package wx

import (
	"testing"
)

func TestEstimateDBZ(t *testing.T) {
	root := makeRadarKdTree()

	// Test that the NWS standard colors at known dBZ values map back correctly (±2 dBZ).
	tests := []struct {
		r, g, b byte
		wantDBZ float32
	}{
		{0, 255, 0, 20},    // bright green
		{0, 200, 0, 25},    // green
		{0, 144, 0, 30},    // dark green
		{255, 255, 0, 35},  // yellow
		{231, 192, 0, 40},  // gold
		{255, 144, 0, 45},  // orange
		{255, 0, 0, 50},    // red
		{214, 0, 0, 55},    // dark red
		{0, 236, 236, 5},   // light cyan
		{1, 160, 246, 10},  // blue
		{0, 0, 246, 15},    // dark blue
		{192, 0, 0, 60},    // darker red
		{255, 0, 255, 65},  // magenta
		{153, 85, 201, 70}, // purple
	}

	for _, tt := range tests {
		got := estimateDBZ(root, [3]byte{tt.r, tt.g, tt.b})
		if got < tt.wantDBZ-2 || got > tt.wantDBZ+2 {
			t.Errorf("estimateDBZ(%d,%d,%d) = %.1f, want %.0f (±2)",
				tt.r, tt.g, tt.b, got, tt.wantDBZ)
		}
	}
}

// dbzToLevel mirrors radar.dbzToLevel for testing purposes.
func dbzToLevel(dbz byte) int {
	if dbz > 55 {
		return 6
	} else if dbz > 50 {
		return 5
	} else if dbz > 45 {
		return 4
	} else if dbz > 40 {
		return 3
	} else if dbz > 30 {
		return 2
	} else if dbz > 20 {
		return 1
	}
	return 0
}

func TestAllLevelsReachable(t *testing.T) {
	root := makeRadarKdTree()

	// Test representative colors for each level and verify all 6 levels are reachable.
	// Colors are the NWS standard colors at dBZ values that should map to each level.
	levelColors := []struct {
		r, g, b     byte
		description string
		wantLevel   int
	}{
		// Level 1: dBZ 21-30
		{0, 200, 0, "green (25 dBZ)", 1},
		{0, 144, 0, "dark green (30 dBZ)", 1},
		// Level 2: dBZ 31-40
		{255, 255, 0, "yellow (35 dBZ)", 2},
		{231, 192, 0, "gold (40 dBZ)", 2},
		// Level 3: dBZ 41-45
		{255, 144, 0, "orange (45 dBZ)", 3},
		// Level 4: dBZ 46-50
		{255, 0, 0, "red (50 dBZ)", 4},
		// Level 5: dBZ 51-55
		{214, 0, 0, "dark red (55 dBZ)", 5},
		// Level 6: dBZ > 55
		{192, 0, 0, "darker red (60 dBZ)", 6},
	}

	levelsHit := make(map[int]bool)
	for _, lc := range levelColors {
		dbz := estimateDBZ(root, [3]byte{lc.r, lc.g, lc.b})
		level := dbzToLevel(byte(max(0, min(255, dbz))))
		levelsHit[level] = true
		if level != lc.wantLevel {
			t.Errorf("%s: estimateDBZ=%0.1f → level %d, want level %d",
				lc.description, dbz, level, lc.wantLevel)
		}
	}

	for level := 1; level <= 6; level++ {
		if !levelsHit[level] {
			t.Errorf("level %d was never reached by any test color", level)
		}
	}
}

func TestInterpolatedColorsMapCorrectly(t *testing.T) {
	root := makeRadarKdTree()

	// Test colors that are interpolated between NWS anchor points (not exact matches).
	// These simulate what the NOAA GeoServer actually renders.
	interpolated := []struct {
		r, g, b byte
		minDBZ  float32
		maxDBZ  float32
	}{
		// Between gold (40) and orange (45): should be level 3 territory
		{243, 168, 0, 40, 46},
		{237, 180, 0, 39, 46},
		// Between yellow (35) and gold (40): should be level 2 territory
		{243, 224, 0, 34, 41},
		// Between red (50) and dark red (55): should be level 4-5 territory
		{235, 0, 0, 49, 56},
	}

	for _, tt := range interpolated {
		got := estimateDBZ(root, [3]byte{tt.r, tt.g, tt.b})
		if got < tt.minDBZ || got > tt.maxDBZ {
			t.Errorf("estimateDBZ(%d,%d,%d) = %.1f, want [%.0f, %.0f]",
				tt.r, tt.g, tt.b, got, tt.minDBZ, tt.maxDBZ)
		}
	}
}

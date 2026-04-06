// nav/approach_test.go
// Copyright(c) 2022-2025 vice contributors, licensed under the GNU Public License, Version 3.
// SPDX: GPL-3.0-only

package nav

import (
	gomath "math"
	"testing"

	av "github.com/mmp/vice/aviation"
	"github.com/mmp/vice/math"
)

// TestDirectToApproachFixNoDescentWithoutClearance verifies that going
// direct to a fix on the approach does NOT cause descent unless the
// approach has been cleared (regression test for 36b2bd31).
//
// ROSLY is on the I22L approach. When the aircraft's route does not
// include ROSLY, DirectFix finds it via the approach waypoints (source =
// waypointSourceApproach) and sets InterceptState = OnApproachCourse
// without enabling altitude restrictions.
func TestDirectToApproachFixNoDescentWithoutClearance(t *testing.T) {
	// Route does NOT include ROSLY so that DirectFix will find it on the approach.
	f := NewArrivalFlight(t, ArrivalConfig{
		Waypoints:        "HAUPT/a6000 LEFER/a4000",
		DepartureAirport: "KMCO",
		ArrivalAirport:   "KJFK",
		AircraftType:     "A320",
		InitialAltitude:  5000,
		InitialSpeed:     210,
		AssignedAltitude: 5000, // hold at 5000
	})

	f.ExpectApproach("I22L")
	// Direct to ROSLY — which is on the I22L approach but not in our route
	f.DirectFix("ROSLY")

	f.AfterTicks(20, func(f *FlightTest) {
		// InterceptState should be OnApproachCourse (tracking laterally)
		if f.nav.Approach.InterceptState != OnApproachCourse {
			t.Errorf("expected OnApproachCourse, got %d", f.nav.Approach.InterceptState)
		}
		// Should NOT be cleared
		if f.nav.Approach.Cleared {
			t.Errorf("approach should not be cleared")
		}
	})

	// After 100 ticks without clearance, altitude should be roughly unchanged
	f.AfterTicks(100, func(f *FlightTest) {
		f.AssertAltitudeNear(5000, 200)
	})

	// Now clear the approach
	f.AfterTicks(101, func(f *FlightTest) {
		f.ClearedApproach("I22L")
	})

	// Give enough time for the descent to start after clearance at tick 101.
	// At tick 200 the aircraft should be descending.
	f.AfterTicks(200, func(f *FlightTest) {
		f.AssertAltitudeBelow(5000)
	})

	f.AtFix("ROSLY", func(f *FlightTest) {
		// After clearance and descent, should be below 5000
	})

	f.Run()
}

// TestAtFixClearedApproach verifies that "at fix, cleared approach"
// activates the approach clearance when the aircraft passes the named fix
// (regression test for 61db003f).
func TestAtFixClearedApproach(t *testing.T) {
	f := NewArrivalFlight(t, ArrivalConfig{
		Waypoints:        "HAUPT/a6000 LEFER/a4000 ROSLY/a3000",
		DepartureAirport: "KMCO",
		ArrivalAirport:   "KJFK",
		AircraftType:     "A320",
		InitialAltitude:  7000,
		InitialSpeed:     210,
	})

	f.ExpectApproach("I22L")
	f.AtFixCleared("ROSLY", "I22L")

	// Before ROSLY: approach should NOT be cleared
	f.BeforeFix("ROSLY", func(f *FlightTest) {
		if f.nav.Approach.Cleared {
			t.Errorf("tick %d: approach should not be cleared before ROSLY", f.tick)
		}
	})

	// After ROSLY: the at-fix clearance should have fired
	f.AtFix("ROSLY", func(f *FlightTest) {
		if !f.nav.Approach.Cleared {
			t.Errorf("approach should be cleared after passing ROSLY")
		}
	})

	f.Run()
}

// TestApproachWindCorrectionOnLocalizer verifies that an aircraft tracks
// the extended centerline closely despite crosswind. The bug (216f11fa)
// caused the aircraft to drift off the localizer because TurningToJoin
// flew the raw runway heading without crabbing into the wind.
func TestApproachWindCorrectionOnLocalizer(t *testing.T) {
	f := NewArrivalFlight(t, ArrivalConfig{
		Waypoints:        "HAUPT/a6000/star LEFER/a4000/star ROSLY/a3000/star",
		DepartureAirport: "KMCO",
		ArrivalAirport:   "KJFK",
		AircraftType:     "A320",
		InitialAltitude:  3000,
		InitialSpeed:     180,
	})

	// Strong crosswind from the west — 40kt creates a crab angle (~11°)
	// that exceeds the old heading tolerance of 10° in shouldTurnToIntercept.
	f.SetWind(270, 40)

	f.AfterTicks(1, func(f *FlightTest) {
		f.ExpectApproach("I22L")
		f.AssignHeading(180, av.TurnClosest)
		f.ClearedApproach("I22L")

		// Position the aircraft northwest of the localizer (perpendicular
		// left of the outbound course). The westerly wind pushes it east
		// toward the centerline, creating a natural intercept.
		ap := f.nav.Approach.Assigned
		nmPerLong := f.nav.FlightState.NmPerLongitude
		rwyHdg := ap.RunwayHeading(nmPerLong)
		outbound := math.OppositeHeading(rwyHdg)
		onCourse := math.Offset2LL(ap.Threshold, outbound, 12, nmPerLong)
		perpLeft := math.OffsetHeading(outbound, -90)
		f.nav.FlightState.Position = math.Offset2LL(onCourse, perpLeft, 3, nmPerLong)
	})

	// Once established on the approach (OnApproachCourse), the aircraft
	// should stay within 0.15nm of the extended centerline despite
	// the crosswind. Without the wind correction fix, it drifts to
	// ~1.9nm before slowly oscillating back.
	for tick := 200; tick <= 800; tick += 10 {
		f.AfterTicks(tick, func(f *FlightTest) {
			if f.nav.Approach.InterceptState == OnApproachCourse {
				f.AssertOnExtendedCenterline(0.15)
			}
		})
	}

	f.Run()
}

// TestDirectFixRevokesApproachClearance verifies that giving a "direct fix"
// to an aircraft that has been cleared for an approach implicitly revokes
// the approach clearance: the aircraft continues to fly the approach
// waypoints laterally but does not descend via approach altitude
// restrictions.
func TestDirectFixRevokesApproachClearance(t *testing.T) {
	f := NewArrivalFlight(t, ArrivalConfig{
		Waypoints:        "HAUPT/a6000 LEFER/a4000",
		DepartureAirport: "KMCO",
		ArrivalAirport:   "KJFK",
		AircraftType:     "A320",
		InitialAltitude:  5000,
		InitialSpeed:     210,
		AssignedAltitude: 5000,
	})

	// This sequence must run after the first tick so the random state
	// and deferred-heading timing match the intended scenario.
	f.AfterTicks(1, func(f *FlightTest) {
		f.ExpectApproach("I22L")
		// 1. Direct to ROSLY (on the I22L approach)
		f.DirectFix("ROSLY")
		// 2. Clear the approach
		f.ClearedApproach("I22L")
		// 3. Direct to ROSLY again — revokes the approach clearance
		f.DirectFix("ROSLY")
	})

	// InterceptState should be NotIntercepting (approach revoked)
	f.AfterTicks(20, func(f *FlightTest) {
		if f.nav.Approach.InterceptState != NotIntercepting {
			t.Errorf("expected NotIntercepting after second direct fix, got %d",
				f.nav.Approach.InterceptState)
		}
	})

	// Altitude should remain near 5000 — no approach descent
	f.AfterTicks(150, func(f *FlightTest) {
		f.AssertAltitudeNear(5000, 200)
	})

	f.Run()
}

// TestThresholdOffset verifies that ThresholdOffset places points at the
// correct distance and lateral offset from the runway threshold.
func TestThresholdOffset(t *testing.T) {
	apg := LookupApproachGeometry(t, "KJFK", "I22L")

	// On centerline: distance from threshold should match distNM.
	for _, dist := range []float32{1, 5, 10, 20} {
		pos := apg.ThresholdOffset(dist, 0)
		got := math.NMDistance2LLFast(apg.Threshold, pos, apg.NmPerLongitude)
		if gomath.Abs(float64(got-dist)) > 0.05 {
			t.Errorf("ThresholdOffset(%.0f, 0): distance from threshold = %.3f nm, want ~%.0f",
				dist, got, dist)
		}
	}

	// On centerline: point should lie on the extended centerline.
	for _, dist := range []float32{5, 10} {
		pos := apg.ThresholdOffset(dist, 0)
		posNM := math.LL2NM(pos, apg.NmPerLongitude)
		threshNM := math.LL2NM(apg.Threshold, apg.NmPerLongitude)
		// Use a second point on the centerline for the line.
		pos2 := apg.ThresholdOffset(dist+1, 0)
		pos2NM := math.LL2NM(pos2, apg.NmPerLongitude)
		deviation := math.PointLineDistance(posNM, threshNM, pos2NM)
		if deviation > 0.001 {
			t.Errorf("ThresholdOffset(%.0f, 0): %.4f nm off centerline", dist, deviation)
		}
	}

	// Lateral offset: distance from the on-course point should match |lateralNM|.
	for _, tc := range []struct {
		dist, lateral float32
	}{
		{10, 2},
		{10, -2},
		{5, 0.5},
		{5, -0.5},
	} {
		onCourse := apg.ThresholdOffset(tc.dist, 0)
		offset := apg.ThresholdOffset(tc.dist, tc.lateral)
		lateralDist := math.NMDistance2LLFast(onCourse, offset, apg.NmPerLongitude)
		want := float32(gomath.Abs(float64(tc.lateral)))
		if gomath.Abs(float64(lateralDist-want)) > 0.05 {
			t.Errorf("ThresholdOffset(%.0f, %.1f): lateral distance = %.3f nm, want ~%.1f",
				tc.dist, tc.lateral, lateralDist, want)
		}
	}

	// Sign convention: negative lateral = left of outbound.
	// For KJFK 22L (runway heading ~224° true, outbound ~044°),
	// left of outbound points roughly NW, right points SE.
	left := apg.ThresholdOffset(10, -1)
	right := apg.ThresholdOffset(10, 1)
	onCourse := apg.ThresholdOffset(10, 0)

	// Left and right should be on opposite sides of the centerline.
	leftNM := math.LL2NM(left, apg.NmPerLongitude)
	rightNM := math.LL2NM(right, apg.NmPerLongitude)
	courseNM := math.LL2NM(onCourse, apg.NmPerLongitude)
	threshNM := math.LL2NM(apg.Threshold, apg.NmPerLongitude)
	sdLeft := math.SignedPointLineDistance(leftNM, threshNM, courseNM)
	sdRight := math.SignedPointLineDistance(rightNM, threshNM, courseNM)
	if math.Sign(sdLeft) == math.Sign(sdRight) {
		t.Errorf("left (sd=%.3f) and right (sd=%.3f) should be on opposite sides of centerline",
			sdLeft, sdRight)
	}

	// Perpendicularity: the on-course → offset vector should be ~90° from the course.
	outbound := math.OppositeHeading(apg.RunwayHeading)
	offsetHdg := math.Heading2LL(onCourse, right, apg.NmPerLongitude)
	angleDiff := math.HeadingDifference(outbound, offsetHdg)
	if gomath.Abs(float64(angleDiff)-90) > 1 {
		t.Errorf("offset direction %.1f° differs from course %.1f° by %.1f° (want ~90°)",
			float32(offsetHdg), float32(outbound), angleDiff)
	}
}

// TestFAFOffset verifies that FAFOffset places points at the correct
// distance and lateral offset from the FAF.
func TestFAFOffset(t *testing.T) {
	apg := LookupApproachGeometry(t, "KJFK", "I22L")

	// On centerline: distance from FAF should match distNM.
	for _, dist := range []float32{1, 3, 5} {
		pos := apg.FAFOffset(dist, 0)
		got := math.NMDistance2LLFast(apg.FAFLocation, pos, apg.NmPerLongitude)
		if gomath.Abs(float64(got-dist)) > 0.05 {
			t.Errorf("FAFOffset(%.0f, 0): distance from FAF = %.3f nm, want ~%.0f",
				dist, got, dist)
		}
	}

	// Lateral offset magnitude.
	for _, tc := range []struct {
		dist, lateral float32
	}{
		{1, 0.3},
		{1, -0.3},
		{3, 1},
	} {
		onCourse := apg.FAFOffset(tc.dist, 0)
		offset := apg.FAFOffset(tc.dist, tc.lateral)
		lateralDist := math.NMDistance2LLFast(onCourse, offset, apg.NmPerLongitude)
		want := float32(gomath.Abs(float64(tc.lateral)))
		if gomath.Abs(float64(lateralDist-want)) > 0.05 {
			t.Errorf("FAFOffset(%.0f, %.1f): lateral distance = %.3f nm, want ~%.1f",
				tc.dist, tc.lateral, lateralDist, want)
		}
	}

	// FAFOffset should use the same course as ThresholdOffset (outbound heading).
	// A point at FAFOffset(0, 0) should be at the FAF location.
	atFAF := apg.FAFOffset(0, 0)
	fafDist := math.NMDistance2LLFast(apg.FAFLocation, atFAF, apg.NmPerLongitude)
	if fafDist > 0.01 {
		t.Errorf("FAFOffset(0, 0) is %.4f nm from FAF, want ~0", fafDist)
	}

	// FAF should lie on the extended centerline (same outbound course as threshold).
	fafNM := math.LL2NM(apg.FAFLocation, apg.NmPerLongitude)
	threshNM := math.LL2NM(apg.Threshold, apg.NmPerLongitude)
	farNM := math.LL2NM(apg.ThresholdOffset(20, 0), apg.NmPerLongitude)
	fafDeviation := math.PointLineDistance(fafNM, threshNM, farNM)
	if fafDeviation > 0.1 {
		t.Errorf("FAF is %.3f nm off extended centerline (want <0.1)", fafDeviation)
	}
}

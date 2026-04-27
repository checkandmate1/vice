// sim/aircraft_test.go
// Copyright(c) 2022-2026 vice contributors, licensed under the GNU Public License, Version 3.
// SPDX: GPL-3.0-only

package sim

import (
	"testing"

	av "github.com/mmp/vice/aviation"
	"github.com/mmp/vice/math"
	"github.com/mmp/vice/nav"
)

// 1 degree of latitude is ~60 NM, so waypoints are placed along the meridian
// at known distances from (0, 0).
func wp(fix string, latDeg float32) av.Waypoint {
	return av.Waypoint{Fix: fix, Location: math.Point2LL{0, latDeg}}
}

func makeAircraftForSTTFixes(wps []av.Waypoint) *Aircraft {
	return &Aircraft{
		FlightPlan: av.FlightPlan{
			DepartureAirport: "KJFK",
			ArrivalAirport:   "KBOS",
		},
		Nav: nav.Nav{
			FlightState: nav.FlightState{Position: math.Point2LL{0, 0}},
			Waypoints:   wps,
		},
	}
}

func TestGetSTTFixes_STARS_CullsBeyond75NM(t *testing.T) {
	wps := []av.Waypoint{
		wp("CLOSE", 0.5), // ~30 NM
		wp("NEAR", 1.0),  // ~60 NM
		wp("FAR", 1.5),   // ~90 NM — should stop loop here
		wp("FARTHER", 3.0),
	}
	ac := makeAircraftForSTTFixes(wps)

	got := ac.GetSTTFixes(false)
	want := []string{"KBOS", "KJFK", "CLOSE", "NEAR"}
	if !equalStrings(got, want) {
		t.Errorf("STARS: got %v, want %v", got, want)
	}
}

func TestGetSTTFixes_ERAM_Allows300NMAndCapsAt5(t *testing.T) {
	wps := []av.Waypoint{
		wp("ALPHA", 0.5), // 30 NM
		wp("BRAVO", 1.0), // 60 NM
		wp("CHARL", 1.5), // 90 NM
		wp("DELTA", 2.5), // 150 NM
		wp("ECHO", 3.5),  // 210 NM
		wp("FOXTR", 4.0), // 240 NM — in range but exceeds count
		wp("GOLF", 6.0),  // 360 NM — beyond range
	}
	ac := makeAircraftForSTTFixes(wps)

	got := ac.GetSTTFixes(true)
	want := []string{"KBOS", "KJFK", "ALPHA", "BRAVO", "CHARL", "DELTA", "ECHO"}
	if !equalStrings(got, want) {
		t.Errorf("ERAM: got %v, want %v", got, want)
	}
}

func TestGetSTTFixes_ERAM_CullsBeyond300NM(t *testing.T) {
	wps := []av.Waypoint{
		wp("NEAR", 2.0), // 120 NM
		wp("MID", 4.0),  // 240 NM
		wp("FAR", 6.0),  // 360 NM — beyond 300
		wp("FARTHER", 7.0),
	}
	ac := makeAircraftForSTTFixes(wps)

	got := ac.GetSTTFixes(true)
	want := []string{"KBOS", "KJFK", "NEAR", "MID"}
	if !equalStrings(got, want) {
		t.Errorf("ERAM: got %v, want %v", got, want)
	}
}

func TestGetSTTFixes_SkipsInternalAndShortFixes(t *testing.T) {
	wps := []av.Waypoint{
		wp("_INT", 0.2),   // internal, underscore-prefixed
		wp("AB", 0.3),     // too short
		wp("OK", 0.4),     // too short (len 2)
		wp("GOOD", 0.5),   // valid
		wp("LONGER", 1.0), // length 6, too long
	}
	ac := makeAircraftForSTTFixes(wps)

	got := ac.GetSTTFixes(false)
	want := []string{"KBOS", "KJFK", "GOOD"}
	if !equalStrings(got, want) {
		t.Errorf("got %v, want %v", got, want)
	}
}

func equalStrings(a, b []string) bool {
	if len(a) != len(b) {
		return false
	}
	for i := range a {
		if a[i] != b[i] {
			return false
		}
	}
	return true
}

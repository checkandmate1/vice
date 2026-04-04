// nav/alt_test.go
// Copyright(c) 2022-2025 vice contributors, licensed under the GNU Public License, Version 3.
// SPDX: GPL-3.0-only

package nav

import (
	"testing"

	av "github.com/mmp/vice/aviation"
)

// TestSTARDescentMeetsRestrictions verifies that an aircraft descending
// via a STAR meets altitude restrictions at each fix.
func TestSTARDescentMeetsRestrictions(t *testing.T) {
	f := NewArrivalFlight(t, ArrivalConfig{
		Waypoints:        "SAJUL/a10000/star DETGY/a7000/star HAUPT/a6000/star LEFER/a4000/star ROSLY/a3000/star",
		DepartureAirport: "KMCO",
		ArrivalAirport:   "KJFK",
		AircraftType:     "A320",
		InitialAltitude:  11000,
		InitialSpeed:     250,
	})

	f.AtFix("DETGY", func(f *FlightTest) {
		f.AssertAltitudeNear(7000, 100)
	})
	f.AtFix("HAUPT", func(f *FlightTest) {
		// HAUPT is only 5.6nm from DETGY. The descent algorithm's
		// ramp-up period (~13s) consumes nearly 1nm of that distance,
		// limiting achievable precision at closely-spaced fixes.
		f.AssertAltitudeNear(6000, 200)
	})
	f.AtFix("LEFER", func(f *FlightTest) {
		f.AssertAltitudeNear(4000, 100)
	})
	f.AtFix("ROSLY", func(f *FlightTest) {
		f.AssertAltitudeNear(3000, 100)
	})

	f.Run()
}

// TestDescentPreservedOnApproachTransition verifies that a controller-
// assigned descent continues through the transition to approach mode
// (regression test for ecfa0ce7).
//
// Scenario: aircraft is descending via STAR, controller assigns "descend
// to 3000" and clears the approach. The aircraft must not level off when
// the approach waypoints are spliced in — it should continue descending.
func TestDescentPreservedOnApproachTransition(t *testing.T) {
	f := NewArrivalFlight(t, ArrivalConfig{
		Waypoints:        "HAUPT/a6000/star LEFER/a4000/star ROSLY/a3000/star",
		DepartureAirport: "KMCO",
		ArrivalAirport:   "KJFK",
		AircraftType:     "A320",
		InitialAltitude:  7000,
		InitialSpeed:     210,
	})

	f.AtFix("HAUPT", func(f *FlightTest) {
		f.ExpectApproach("I22L")
		f.AssignAltitude(3000)
		f.ClearedApproach("I22L")
	})

	// After being assigned 3000 and cleared approach at HAUPT, the
	// aircraft must continue descending toward 3000 through LEFER.
	// LEFER is 6.5nm from HAUPT; at 210kt that's ~112s. Descending
	// from 7000 toward 3000 the aircraft should be well below 3800
	// by LEFER — not stalled at the STAR restriction of 4000.
	f.AtFix("LEFER", func(f *FlightTest) {
		f.AssertAltitudeBelow(3800)
		f.AssertDescending()
	})

	f.Run()
}

// TestApproachAtOrAboveDescentTarget verifies that "at or above"
// restrictions on approach waypoints are treated as descent targets,
// not as already-satisfied constraints (regression test for eb46d623).
//
// Scenario: aircraft is cleared for I22L approach at 5000 ft. The first
// approach fix has "at or above 3000". Without the fix, the aircraft
// would stay at 5000 because the restriction is "satisfied" — but it
// should descend toward 3000 to maintain a normal descent profile.
func TestApproachAtOrAboveDescentTarget(t *testing.T) {
	f := NewArrivalFlight(t, ArrivalConfig{
		Waypoints:        "LEFER/a4000/star ROSLY/a3000/star",
		DepartureAirport: "KMCO",
		ArrivalAirport:   "KJFK",
		AircraftType:     "A320",
		InitialAltitude:  5000,
		InitialSpeed:     210,
	})

	f.AfterTicks(1, func(f *FlightTest) {
		f.ExpectApproach("I22L")
		f.AssignAltitude(3000)
		f.ClearedApproach("I22L")
	})

	// The aircraft should be descending toward 3000, not leveling
	// off at 5000 because the "at or above 3000" is "satisfied".
	f.BeforeFix("LEFER", func(f *FlightTest) {
		if f.tick > 30 {
			f.AssertDescending()
		}
	})

	f.Run()
}

// TestAssignedAltitudeOverridesSTAR verifies that a controller-assigned
// altitude takes priority over STAR restrictions.
func TestAssignedAltitudeOverridesSTAR(t *testing.T) {
	f := NewArrivalFlight(t, ArrivalConfig{
		Waypoints:        "SAJUL/a10000/star DETGY/a7000/star HAUPT/a6000/star",
		DepartureAirport: "KMCO",
		ArrivalAirport:   "KJFK",
		AircraftType:     "A320",
		InitialAltitude:  11000,
		InitialSpeed:     250,
	})

	// Assign 5000 immediately — should override the 10000/7000 restrictions.
	f.AfterTicks(1, func(f *FlightTest) {
		f.AssignAltitude(5000)
	})

	// At DETGY (charted restriction 7000), the aircraft should be at
	// 5000 — the assigned altitude — not leveled at 7000.
	f.AtFix("DETGY", func(f *FlightTest) {
		f.AssertAltitudeNear(5000, 100)
	})

	f.Run()
}

// TestCrossFixAtAltitude verifies that "cross fix at altitude"
// assignments are respected when the restriction differs from charted.
func TestCrossFixAtAltitude(t *testing.T) {
	f := NewArrivalFlight(t, ArrivalConfig{
		Waypoints:        "SAJUL/a10000/star DETGY/a7000/star HAUPT/a6000/star",
		DepartureAirport: "KMCO",
		ArrivalAirport:   "KJFK",
		AircraftType:     "A320",
		InitialAltitude:  10000,
		InitialSpeed:     250,
	})

	// "Cross DETGY at 8000" — controller raises the charted 7000
	// restriction to 8000. The aircraft must level at 8000, not
	// descend through to the charted 7000.
	f.AfterTicks(1, func(f *FlightTest) {
		ar := av.MakeAtAltitudeRestriction(8000)
		f.nav.CrossFixAt("DETGY", &ar, nil, 0)
	})

	f.AtFix("DETGY", func(f *FlightTest) {
		f.AssertAltitudeNear(8000, 100)
	})

	f.Run()
}

// TestDescendViaSTAR verifies that DescendViaSTAR clears assigned
// altitude and lets the aircraft follow charted restrictions.
func TestDescendViaSTAR(t *testing.T) {
	f := NewArrivalFlight(t, ArrivalConfig{
		Waypoints:        "SAJUL/a10000/star DETGY/a7000/star HAUPT/a6000/star",
		DepartureAirport: "KMCO",
		ArrivalAirport:   "KJFK",
		AircraftType:     "A320",
		InitialAltitude:  11000,
		InitialSpeed:     250,
		AssignedAltitude: 11000, // held at 11000
	})

	// Verify the aircraft is held at 11000 before DVS.
	f.AfterTicks(30, func(f *FlightTest) {
		f.AssertAltitudeNear(11000, 50)
		f.nav.DescendViaSTAR(f.simTime)
	})

	// After DVS, aircraft should descend to meet DETGY's 7000 restriction.
	f.AtFix("DETGY", func(f *FlightTest) {
		f.AssertAltitudeNear(7000, 100)
	})

	f.Run()
}

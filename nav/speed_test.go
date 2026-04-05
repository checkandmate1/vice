// nav/speed_test.go
// Copyright(c) 2022-2025 vice contributors, licensed under the GNU Public License, Version 3.
// SPDX: GPL-3.0-only

package nav

import (
	"testing"

	av "github.com/mmp/vice/aviation"
)

// TestSTARSpeedRestrictions verifies that STAR speed restrictions are
// respected at each fix (regression test for 9ae3110c).
func TestSTARSpeedRestrictions(t *testing.T) {
	f := NewArrivalFlight(t, ArrivalConfig{
		Waypoints:        "SAJUL/a10000/s250/star DETGY/a7000/s210/star HAUPT/a6000/star",
		DepartureAirport: "KMCO",
		ArrivalAirport:   "KJFK",
		AircraftType:     "A320",
		InitialAltitude:  11000,
		InitialSpeed:     280,
	})

	// Before DETGY: the SAJUL/s250 restriction should slow the aircraft.
	f.BeforeFix("DETGY", func(f *FlightTest) {
		if f.tick > 30 {
			f.AssertSpeedBelow(255)
		}
	})

	f.AtFix("DETGY", func(f *FlightTest) {
		f.AssertSpeedNear(210, 15)
	})

	f.Run()
}

// TestSpeed250Below10000 verifies that aircraft decelerate to 250kt or
// below when descending through 10000 ft.
func TestSpeed250Below10000(t *testing.T) {
	f := NewArrivalFlight(t, ArrivalConfig{
		Waypoints:        "SAJUL/a10000/star DETGY/a7000/star HAUPT/a6000/star",
		DepartureAirport: "KMCO",
		ArrivalAirport:   "KJFK",
		AircraftType:     "A320",
		InitialAltitude:  12000,
		InitialSpeed:     280,
	})

	// At DETGY (7000), the aircraft has descended through 10000 and
	// should have slowed to 250 or below.
	f.AtFix("DETGY", func(f *FlightTest) {
		f.AssertSpeedBelow(255)
	})

	f.Run()
}

// TestAfterFixSpeed verifies that "after fix, maintain speed" fires
// when the aircraft passes the named fix (regression test for 3155bf14).
func TestAfterFixSpeed(t *testing.T) {
	f := NewArrivalFlight(t, ArrivalConfig{
		Waypoints:        "SAJUL/star DETGY/star HAUPT/star LEFER/star",
		DepartureAirport: "KMCO",
		ArrivalAirport:   "KJFK",
		AircraftType:     "A320",
		InitialAltitude:  11000,
		InitialSpeed:     250,
		OnSTAR:           true,
	})

	f.AfterTicks(1, func(f *FlightTest) {
		f.AfterFixSpeed("DETGY", 210)
	})

	// Before DETGY: speed should stay near 250
	f.BeforeFix("DETGY", func(f *FlightTest) {
		if f.tick > 5 {
			f.AssertSpeedAbove(240)
		}
	})

	// After DETGY the after-fix speed fires and the aircraft decelerates
	f.AtFix("HAUPT", func(f *FlightTest) {
		f.AssertSpeedNear(210, 15)
	})

	f.Run()
}

// TestDirectSpeedCancelsAfterFixSpeed verifies that a direct speed
// assignment clears any pending after-fix speed (regression test for 3155bf14).
func TestDirectSpeedCancelsAfterFixSpeed(t *testing.T) {
	f := NewArrivalFlight(t, ArrivalConfig{
		Waypoints:        "SAJUL/star DETGY/star HAUPT/star LEFER/star",
		DepartureAirport: "KMCO",
		ArrivalAirport:   "KJFK",
		AircraftType:     "A320",
		InitialAltitude:  11000,
		InitialSpeed:     250,
		OnSTAR:           true,
	})

	f.AfterTicks(1, func(f *FlightTest) {
		f.AfterFixSpeed("DETGY", 210)
	})

	// After setting after-fix, assign a direct speed — should clear it
	f.AfterTicks(30, func(f *FlightTest) {
		f.AssignSpeed(230)
		// The after-fix speed for DETGY should be cleared
		nfa := f.nav.FixAssignments["DETGY"]
		if nfa.Depart.Speed != nil {
			t.Errorf("after-fix speed for DETGY should be nil after direct speed assignment, got %v", nfa.Depart.Speed)
		}
	})

	f.AtFix("DETGY", func(f *FlightTest) {
		// Should be near 230 (the direct assignment), not 210
		f.AssertSpeedNear(230, 15)
	})

	f.Run()
}

// TestCompoundSpeed verifies multi-segment speed assignments
// (regression test for fa6cd545).
func TestCompoundSpeed(t *testing.T) {
	f := NewArrivalFlight(t, ArrivalConfig{
		Waypoints:        "SAJUL/star DETGY/star HAUPT/star LEFER/star ROSLY/star",
		DepartureAirport: "KMCO",
		ArrivalAirport:   "KJFK",
		AircraftType:     "A320",
		InitialAltitude:  11000,
		InitialSpeed:     280,
		OnSTAR:           true,
	})

	f.AfterTicks(1, func(f *FlightTest) {
		sr250 := av.MakeAtSpeedRestriction(250)
		sr210 := av.MakeAtSpeedRestriction(210)
		sr180 := av.MakeAtSpeedRestriction(180)
		f.CompoundSpeed([]av.CompoundSpeedSegment{
			{Speed: &sr250, UntilFix: "DETGY"},
			{Speed: &sr210, UntilFix: "HAUPT"},
			{Speed: &sr180},
		})
	})

	// Before DETGY: speed should be decelerating toward 250
	f.BeforeFix("DETGY", func(f *FlightTest) {
		if f.tick > 30 {
			f.AssertSpeedBelow(255)
		}
	})

	// At HAUPT: the 210 segment has been active since DETGY
	f.AtFix("HAUPT", func(f *FlightTest) {
		f.AssertSpeedNear(210, 15)
	})

	// After HAUPT: the 180 segment kicks in
	f.AtFix("LEFER", func(f *FlightTest) {
		f.AssertSpeedNear(180, 15)
	})

	f.Run()
}

// TestAfterFixDescendAltitude verifies "after fix, descend and maintain"
// (regression test for 9274cf11).
func TestAfterFixDescendAltitude(t *testing.T) {
	f := NewArrivalFlight(t, ArrivalConfig{
		Waypoints:        "SAJUL/a10000/star DETGY/a7000/star HAUPT/a6000/star LEFER/a4000/star",
		DepartureAirport: "KMCO",
		ArrivalAirport:   "KJFK",
		AircraftType:     "A320",
		InitialAltitude:  11000,
		InitialSpeed:     250,
	})

	f.AfterTicks(1, func(f *FlightTest) {
		f.AfterFixAltitude("DETGY", 5000)
	})

	// At DETGY: the after-fix altitude should be assigned
	f.AtFix("DETGY", func(f *FlightTest) {
		if f.nav.Altitude.Assigned == nil || *f.nav.Altitude.Assigned != 5000 {
			t.Errorf("at DETGY: expected assigned altitude 5000, got %v", f.nav.Altitude.Assigned)
		}
	})

	// At HAUPT: aircraft should have descended past the charted 6000
	// toward the assigned 5000
	f.AtFix("HAUPT", func(f *FlightTest) {
		f.AssertAltitudeNear(5000, 200)
	})

	f.Run()
}

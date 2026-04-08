// nav/commands_test.go
// Copyright(c) 2022-2025 vice contributors, licensed under the GNU Public License, Version 3.
// SPDX: GPL-3.0-only

package nav

import (
	"testing"

	av "github.com/mmp/vice/aviation"
	"github.com/mmp/vice/math"
)

// TestCommandValidation verifies that invalid commands produce UnableIntents.
func TestCommandValidation(t *testing.T) {
	makeNav := func(t *testing.T) *FlightTest {
		t.Helper()
		return NewArrivalFlight(t, ArrivalConfig{
			Waypoints:        "SAJUL/a10000/star DETGY/a7000/star HAUPT/a6000/star",
			DepartureAirport: "KMCO",
			ArrivalAirport:   "KJFK",
			AircraftType:     "A320",
			InitialAltitude:  11000,
			InitialSpeed:     250,
		})
	}

	t.Run("AltitudeAboveCeiling", func(t *testing.T) {
		f := makeNav(t)
		intent := f.nav.AssignAltitude(99000, false)
		AssertUnable(t, intent)
	})

	t.Run("SpeedBelowLanding", func(t *testing.T) {
		f := makeNav(t)
		sr := av.MakeAtSpeedRestriction(80)
		intent := f.nav.AssignSpeed(&sr, false)
		AssertUnable(t, intent)
	})

	t.Run("DirectFixInvalid", func(t *testing.T) {
		f := makeNav(t)
		intent := f.nav.DirectFix("ZZZZZZ", av.TurnClosest, f.simTime)
		AssertUnable(t, intent)
	})

	t.Run("CrossFixNotInRoute", func(t *testing.T) {
		f := makeNav(t)
		ar := av.MakeAtAltitudeRestriction(5000)
		intent := f.nav.CrossFixAt("NOTINROUTE", &ar, nil)
		AssertUnable(t, intent)
	})

	t.Run("ClearedApproachWithoutExpect", func(t *testing.T) {
		f := makeNav(t)
		intent, _ := f.nav.ClearedApproach(f.fp.ArrivalAirport, "I22L", false, f.simTime)
		AssertUnable(t, intent)
	})

	t.Run("DescendViaSTARNotOnSTAR", func(t *testing.T) {
		// Create a nav without OnSTAR waypoints
		f := NewArrivalFlight(t, ArrivalConfig{
			Waypoints:        "SAJUL DETGY HAUPT",
			DepartureAirport: "KMCO",
			ArrivalAirport:   "KJFK",
			AircraftType:     "A320",
			InitialAltitude:  11000,
			InitialSpeed:     250,
		})
		intent := f.nav.DescendViaSTAR(f.simTime)
		AssertUnable(t, intent)
	})

	t.Run("ExpediteDescentWhenLevel", func(t *testing.T) {
		// Aircraft at 11000, no assigned descent — level flight
		f := NewArrivalFlight(t, ArrivalConfig{
			Waypoints:        "SAJUL DETGY HAUPT",
			DepartureAirport: "KMCO",
			ArrivalAirport:   "KJFK",
			AircraftType:     "A320",
			InitialAltitude:  11000,
			InitialSpeed:     250,
			AssignedAltitude: 11000,
		})
		intent := f.nav.ExpediteDescent()
		AssertUnable(t, intent)
	})
}

// TestLeftDirectFix verifies that DirectFix with TurnLeft turns the long way.
func TestLeftDirectFix(t *testing.T) {
	// Aircraft heading north (360). Fix is DETGY which is roughly east
	// of SAJUL. By requesting TurnLeft, the aircraft should turn counter-
	// clockwise (heading decreasing from 360 toward 270).
	f := NewArrivalFlight(t, ArrivalConfig{
		Waypoints:        "SAJUL/star DETGY/star HAUPT/star",
		DepartureAirport: "KMCO",
		ArrivalAirport:   "KJFK",
		AircraftType:     "A320",
		InitialAltitude:  11000,
		InitialSpeed:     250,
		OnSTAR:           true,
	})

	// Assign heading north first, then direct to DETGY with left turn
	f.AssignHeading(360, av.TurnClosest)

	f.AfterTicks(30, func(f *FlightTest) {
		f.DirectFixWithTurn("DETGY", av.TurnLeft)
	})

	// Check after some time that the deferred heading includes a left turn
	f.AfterTicks(60, func(f *FlightTest) {
		if dh := f.nav.DeferredNavHeading; dh != nil && dh.Turn != nil {
			if *dh.Turn != av.TurnLeft {
				t.Errorf("expected TurnLeft in deferred heading, got %v", *dh.Turn)
			}
		}
	})

	f.AtFix("DETGY", func(f *FlightTest) {
		// Just verify the fix was reached
	})

	f.Run()
}

// TestRightDirectFix verifies that DirectFix with TurnRight turns the short way.
func TestRightDirectFix(t *testing.T) {
	f := NewArrivalFlight(t, ArrivalConfig{
		Waypoints:        "SAJUL/star DETGY/star HAUPT/star",
		DepartureAirport: "KMCO",
		ArrivalAirport:   "KJFK",
		AircraftType:     "A320",
		InitialAltitude:  11000,
		InitialSpeed:     250,
		OnSTAR:           true,
	})

	f.AssignHeading(360, av.TurnClosest)

	f.AfterTicks(30, func(f *FlightTest) {
		f.DirectFixWithTurn("DETGY", av.TurnRight)
	})

	f.AfterTicks(60, func(f *FlightTest) {
		if dh := f.nav.DeferredNavHeading; dh != nil && dh.Turn != nil {
			if *dh.Turn != av.TurnRight {
				t.Errorf("expected TurnRight in deferred heading, got %v", *dh.Turn)
			}
		}
	})

	f.AtFix("DETGY", func(f *FlightTest) {
		// Verify fix reached
	})

	f.Run()
}

// TestExpectDirectReducesDelay verifies that calling ExpectDirect before
// DirectFix results in a shorter pilot delay.
func TestExpectDirectReducesDelay(t *testing.T) {
	makeFlight := func(t *testing.T) *FlightTest {
		return NewArrivalFlight(t, ArrivalConfig{
			Waypoints:        "SAJUL/star DETGY/star HAUPT/star",
			DepartureAirport: "KMCO",
			ArrivalAirport:   "KJFK",
			AircraftType:     "A320",
			InitialAltitude:  11000,
			InitialSpeed:     250,
			OnSTAR:           true,
		})
	}

	// Without ExpectDirect
	fNoExpect := makeFlight(t)
	fNoExpect.nav.AssignHeading(math.MagneticHeading(360), av.TurnClosest, fNoExpect.simTime)
	// Wait for heading to take effect
	for i := 0; i < 10; i++ {
		wxs := fNoExpect.weather(fNoExpect.nav.FlightState.Altitude)
		fNoExpect.nav.UpdateWithWeather(fNoExpect.callsign, wxs, &fNoExpect.fp, fNoExpect.simTime, nil)
		fNoExpect.simTime = fNoExpect.simTime.Add(1e9) // 1 second
	}
	fNoExpect.nav.DirectFix("DETGY", av.TurnClosest, fNoExpect.simTime)
	noExpectDelay := fNoExpect.nav.DeferredNavHeading.Time

	// With ExpectDirect
	fExpect := makeFlight(t)
	fExpect.nav.AssignHeading(math.MagneticHeading(360), av.TurnClosest, fExpect.simTime)
	for i := 0; i < 10; i++ {
		wxs := fExpect.weather(fExpect.nav.FlightState.Altitude)
		fExpect.nav.UpdateWithWeather(fExpect.callsign, wxs, &fExpect.fp, fExpect.simTime, nil)
		fExpect.simTime = fExpect.simTime.Add(1e9)
	}
	fExpect.nav.ExpectDirect("DETGY")
	fExpect.nav.DirectFix("DETGY", av.TurnClosest, fExpect.simTime)
	expectDelay := fExpect.nav.DeferredNavHeading.Time

	// With expect, the delay should be shorter (earlier time)
	if !expectDelay.Before(noExpectDelay) {
		t.Errorf("ExpectDirect did not reduce delay: expect=%v noExpect=%v", expectDelay, noExpectDelay)
	}
}

// TestAtFixClearedInvalidFix verifies that AtFixCleared with a fix not on
// the approach returns an UnableIntent.
func TestAtFixClearedInvalidFix(t *testing.T) {
	f := NewArrivalFlight(t, ArrivalConfig{
		Waypoints:        "HAUPT/a6000/star LEFER/a4000/star ROSLY/a3000/star",
		DepartureAirport: "KMCO",
		ArrivalAirport:   "KJFK",
		AircraftType:     "A320",
		InitialAltitude:  7000,
		InitialSpeed:     210,
	})

	f.ExpectApproach("I22L")
	intent := f.nav.AtFixCleared("BOGUS", "I22L")
	AssertUnable(t, intent)
}

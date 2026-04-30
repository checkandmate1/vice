// nav/procedures.go
// Copyright(c) 2022-2025 vice contributors, licensed under the GNU Public License, Version 3.
// SPDX: GPL-3.0-only

package nav

import (
	"fmt"
	"time"

	av "github.com/mmp/vice/aviation"
	"github.com/mmp/vice/math"
	"github.com/mmp/vice/util"
	"github.com/mmp/vice/wx"
)

///////////////////////////////////////////////////////////////////////////
// Procedure turns

// ManeuverCompleteType discriminates the four completion conditions.
type ManeuverCompleteType int

const (
	UntilHeading                ManeuverCompleteType = iota // done when aircraft heading ≈ Until.Heading
	UntilTime                                               // done after Until.Seconds elapsed (lazy start)
	UntilDist                                               // done after Until.Dist nm flown (lazy start)
	UntilFix                                                // done when ETA to Until.Fix < 2s
	UntilIntercept                                          // done when shouldTurnToIntercept fires for course through Fix
	UntilControllerIntervention                             // never completes; lasts until controller issues a new instruction
	UntilAltitude                                           // done when reaching Until.Altitude
	UntilDME                                                // done when crossing Until.DMEDistance from Until.DMEFix
)

// ManeuverComplete encapsulates the completion condition for a lateral
// maneuver. Type selects the condition; the relevant field(s) provide
// its parameters. Time and distance conditions capture their start
// state lazily on first check.
type ManeuverComplete struct {
	Type     ManeuverCompleteType
	Heading  math.MagneticHeading // target heading (UntilHeading)
	Seconds  float32              // duration in seconds (UntilTime)
	Dist     float32              // distance in nm (UntilDist)
	Fix      math.Point2LL        // target fix (UntilFix, UntilIntercept)
	Altitude int                  // target altitude (UntilAltitude)

	// UntilIntercept: inbound course to intercept and turn direction for the intercept turn.
	InterceptCourse math.MagneticHeading
	InterceptTurn   av.TurnDirection

	// UntilDME: DME distance from a fix.
	DMEDistance     float32
	DMEFix          math.Point2LL
	DMEFixElevation int

	// Lazy-init start state for time/distance conditions.
	Start    Time          // captured on first Done() call (UntilTime)
	StartPos math.Point2LL // captured on first Done() call (UntilDist)
}

func (mc *ManeuverComplete) Done(nav *Nav, simTime Time, wxs wx.Sample, targetHdg math.MagneticHeading) bool {
	switch mc.Type {
	case UntilHeading:
		return math.HeadingDifference(nav.FlightState.Heading, targetHdg) < 1
	case UntilTime:
		if mc.Start.IsZero() {
			mc.Start = simTime
		}
		return float32(simTime.Sub(mc.Start).Seconds()) >= mc.Seconds
	case UntilDist:
		if mc.StartPos.IsZero() {
			mc.StartPos = nav.FlightState.Position
		}
		return math.NMDistance2LL(mc.StartPos, nav.FlightState.Position) >= mc.Dist
	case UntilFix:
		return nav.ETA(mc.Fix) < 2
	case UntilIntercept:
		return nav.shouldTurnToIntercept(mc.Fix, mc.InterceptCourse, mc.InterceptTurn, wxs) == turnToInterceptTurn
	case UntilControllerIntervention:
		return false
	case UntilAltitude:
		return nav.FlightState.Altitude >= float32(mc.Altitude)
	case UntilDME:
		dist := math.DMEDistance(nav.FlightState.Position, nav.FlightState.Altitude,
			mc.DMEFix, float32(mc.DMEFixElevation))
		return dist >= mc.DMEDistance
	default:
		panic(fmt.Sprintf("unhandled ManeuverCompleteType: %d", mc.Type))
	}
}

// LateralManeuver describes a single phase of flight: fly a heading until
// a condition is met. A sequence of LateralManeuvers forms a procedure
// turn, hold circuit, or ordered heading-leg instruction.
type LateralManeuver struct {
	Heading              math.MagneticHeading // heading to fly
	Track                math.MagneticHeading // if non-zero, wind-corrected heading via headingForTrack
	FlyToward            math.Point2LL        // if non-zero, heading = bearing to this point each tick
	Turn                 av.TurnDirection
	Until                ManeuverComplete
	AssignAltitude       *float32 // if non-nil, set nav.Altitude when this maneuver becomes active
	ClearAltitudeOnFinal bool
	Fix                  string
	Actions              av.WaypointActions
}

func (m *LateralManeuver) String() string {
	var action string
	if !m.FlyToward.IsZero() {
		action = "fly toward fix"
	} else if m.Track != 0 {
		action = fmt.Sprintf("fly track %03d", int(m.Track))
	} else {
		action = fmt.Sprintf("fly heading %03d", int(m.Heading))
	}

	var until string
	switch m.Until.Type {
	case UntilHeading:
		until = fmt.Sprintf("until heading %03d", int(m.Until.Heading))
	case UntilTime:
		until = fmt.Sprintf("for %.0fs", m.Until.Seconds)
	case UntilDist:
		until = fmt.Sprintf("for %.1fnm", m.Until.Dist)
	case UntilFix:
		until = "until fix"
	case UntilIntercept:
		until = fmt.Sprintf("until intercept %03d", int(m.Until.InterceptCourse))
	case UntilControllerIntervention:
		until = "until controller intervention"
	case UntilAltitude:
		until = fmt.Sprintf("until altitude %d", m.Until.Altitude)
	case UntilDME:
		until = fmt.Sprintf("until DME %.1f", m.Until.DMEDistance)
	}

	if until != "" {
		return action + " " + until
	}
	return action
}

// maneuverHeading computes the heading for a lateral maneuver, considering
// FlyToward, Track, and fixed Heading in priority order.
func (nav *Nav) maneuverHeading(m *LateralManeuver, wxs wx.Sample) math.MagneticHeading {
	if !m.FlyToward.IsZero() {
		return math.TrueToMagnetic(
			math.Heading2LL(nav.FlightState.Position, m.FlyToward, nav.FlightState.NmPerLongitude),
			nav.FlightState.MagneticVariation)
	} else if m.Track != 0 {
		return nav.headingForTrack(m.Track, wxs)
	}
	return m.Heading
}

// maneuverGetHeading returns the heading, turn direction, and turn rate
// for the current maneuver. It advances to the next maneuver when the
// completion condition is met. When the last maneuver completes, it clears
// the maneuver slice so normal waypoint following resumes.
func (nav *Nav) maneuverGetHeading(wxs wx.Sample, simTime Time) (math.MagneticHeading, av.TurnDirection, float32) {
	m := &nav.Heading.Maneuvers[0]
	heading := nav.maneuverHeading(m, wxs)

	if m.Until.Done(nav, simTime, wxs, heading) {
		nav.Heading.Maneuvers = nav.Heading.Maneuvers[1:]
		if len(nav.Heading.Maneuvers) == 0 {
			if m.ClearAltitudeOnFinal {
				nav.Altitude = NavAltitude{}
			}
			return heading, m.Turn, StandardTurnRate
		}
		// Recompute heading for the new maneuver
		m = &nav.Heading.Maneuvers[0]
		if m.AssignAltitude != nil {
			nav.setAssignedAltitude(*m.AssignAltitude)
		}
		if event := nav.activateWaypointActions(m.Fix, m.Actions); event != nil {
			nav.PendingWaypointActionEvents = append(nav.PendingWaypointActionEvents, *event)
		}
		heading = nav.maneuverHeading(m, wxs)
	}

	return heading, m.Turn, StandardTurnRate
}

func (nav *Nav) flyProcedureTurnIfNecessary() {
	wp := nav.AssignedWaypoints()
	if !nav.Approach.Cleared || len(wp) < 2 || wp[0].ProcedureTurn() == nil || nav.Approach.NoPT {
		return
	}

	if wp[0].ProcedureTurn().Entry180NoPT {
		inboundHeading := math.Heading2LL(wp[0].Location, wp[1].Location, nav.FlightState.NmPerLongitude)

		acFixHeading := math.Heading2LL(nav.FlightState.Position, wp[0].Location,
			nav.FlightState.NmPerLongitude)

		if math.HeadingDifference(acFixHeading, inboundHeading) < 90 {
			return
		}
	}

	// Ensure the approach waypoints are in nav.Waypoints (not just in
	// DeferredNavHeading) so they're available after the PT completes.
	nav.Waypoints = wp
	nav.DeferredNavHeading = nil

	pt := wp[0].ProcedureTurn()

	var exitAlt *float32
	if pt.ExitAltitude != 0 {
		alt := float32(pt.ExitAltitude)
		exitAlt = &alt
	}

	var maneuvers []LateralManeuver
	switch pt.Type {
	case av.PTRacetrack:
		maneuvers = makeRacetrackManeuver(nav, wp, exitAlt)
	case av.PTStandard45:
		maneuvers = makeStandard45Maneuver(nav, wp, exitAlt)
	default:
		panic("Unhandled procedure turn type")
	}

	nav.Heading = NavHeading{Maneuvers: maneuvers}
	if len(nav.Heading.Maneuvers) > 0 {
		nav.Heading.Maneuvers[len(nav.Heading.Maneuvers)-1].ClearAltitudeOnFinal = true
	}
}

func makeStandard45Maneuver(nav *Nav, wp []av.Waypoint, exitAlt *float32) []LateralManeuver {
	pt := wp[0].ProcedureTurn()
	fixLoc := wp[0].Location
	nmPerLong := nav.FlightState.NmPerLongitude
	magVar := nav.FlightState.MagneticVariation

	inboundHdg := math.TrueToMagnetic(math.Heading2LL(wp[0].Location, wp[1].Location, nmPerLong), magVar)
	outboundHdg := math.OppositeHeading(inboundHdg)
	awayHdg := math.OffsetHeading(outboundHdg, float32(util.Select(pt.RightTurns, -45, 45)))
	reverseHdg := math.OppositeHeading(awayHdg)
	turn := av.TurnDirection(util.Select(pt.RightTurns, av.TurnRight, av.TurnLeft))

	return []LateralManeuver{
		{FlyToward: fixLoc, Until: ManeuverComplete{Type: UntilFix, Fix: fixLoc}},                           // approach fix
		{Heading: outboundHdg, Until: ManeuverComplete{Type: UntilHeading, Heading: outboundHdg}},           // turn outbound
		outboundLeg(nav, pt, outboundHdg, 1.2),                                                              // fly outbound
		{Heading: awayHdg, Until: ManeuverComplete{Type: UntilHeading, Heading: awayHdg}},                   // turn 45° away
		outboundLeg(nav, pt, awayHdg, 1),                                                                    // fly away
		{Heading: reverseHdg, Turn: turn, Until: ManeuverComplete{Type: UntilHeading, Heading: reverseHdg}}, // 180° reversal
		{Heading: reverseHdg, AssignAltitude: exitAlt, Until: ManeuverComplete{ // fly 45° intercept, descend to exit alt
			Type: UntilIntercept, Fix: fixLoc, InterceptCourse: inboundHdg}},
	}
}

func makeRacetrackManeuver(nav *Nav, wp []av.Waypoint, exitAlt *float32) []LateralManeuver {
	pt := wp[0].ProcedureTurn()
	fixLoc := wp[0].Location
	nmPerLong := nav.FlightState.NmPerLongitude
	magVar := nav.FlightState.MagneticVariation

	inboundHdg := math.TrueToMagnetic(math.Heading2LL(wp[0].Location, wp[1].Location, nmPerLong), magVar)
	outboundHdg := math.OppositeHeading(inboundHdg)

	acFixHdg := math.TrueToMagnetic(math.Heading2LL(nav.FlightState.Position, fixLoc, nmPerLong), magVar)
	entry := pt.SelectRacetrackEntry(inboundHdg, acFixHdg)

	ptTurn := av.TurnDirection(util.Select(pt.RightTurns, av.TurnRight, av.TurnLeft))
	antiTurn := av.TurnDirection(util.Select(pt.RightTurns, av.TurnLeft, av.TurnRight))

	maneuvers := []LateralManeuver{
		// All entries start by approaching the fix
		{FlyToward: fixLoc, Until: ManeuverComplete{Type: UntilFix, Fix: fixLoc}},
	}

	switch entry {
	case av.DirectEntryShortTurn, av.DirectEntryLongTurn:
		// no entry

	case av.TeardropEntry:
		tearHdg := math.OffsetHeading(outboundHdg, float32(util.Select(pt.RightTurns, -30, 30)))
		baseHdg := math.OffsetHeading(inboundHdg, float32(util.Select(pt.RightTurns, -90, 90)))
		maneuvers = append(maneuvers,
			// turn to teardrop
			LateralManeuver{Heading: tearHdg, Until: ManeuverComplete{Type: UntilHeading, Heading: tearHdg}},
			// fly teardrop leg
			outboundLeg(nav, pt, tearHdg, 1.2),
			// turn to base heading
			LateralManeuver{Heading: baseHdg, Until: ManeuverComplete{Type: UntilHeading, Heading: baseHdg}},
			// fly base until time to turn onto inbound course
			LateralManeuver{Heading: baseHdg,
				Until: ManeuverComplete{Type: UntilIntercept, Fix: fixLoc, InterceptCourse: inboundHdg}},
			// fly to fix
			LateralManeuver{FlyToward: fixLoc, Until: ManeuverComplete{Type: UntilFix, Fix: fixLoc}},
		)

	case av.ParallelEntry:
		// After the outbound leg, turn anti-PT to a 30° intercept heading,
		// then fly it until shouldTurnToIntercept fires, then fly to fix.
		interceptHdg := math.OffsetHeading(inboundHdg, float32(util.Select(pt.RightTurns, -30, 30)))
		maneuvers = append(maneuvers,
			// turn outbound
			LateralManeuver{Heading: outboundHdg, Until: ManeuverComplete{Type: UntilHeading, Heading: outboundHdg}},
			// fly outbound leg
			outboundLeg(nav, pt, outboundHdg, 1),
			// turn anti-PT to intercept heading
			LateralManeuver{Heading: interceptHdg, Turn: antiTurn,
				Until: ManeuverComplete{Type: UntilHeading, Heading: interceptHdg}},
			// fly intercept heading until time to turn onto inbound course
			LateralManeuver{Heading: interceptHdg,
				Until: ManeuverComplete{Type: UntilIntercept, Fix: fixLoc, InterceptCourse: inboundHdg}},
			// fly to fix
			LateralManeuver{FlyToward: fixLoc, Until: ManeuverComplete{Type: UntilFix, Fix: fixLoc}},
		)

	default:
		panic(fmt.Sprintf("unhandled racetrack entry type: %d", entry))
	}

	// Standard racetrack circuit; descend to exit altitude here.
	maneuvers = append(maneuvers,
		// turn outbound
		LateralManeuver{Heading: outboundHdg, Turn: ptTurn, AssignAltitude: exitAlt,
			Until: ManeuverComplete{Type: UntilHeading, Heading: outboundHdg}},
		// fly outbound leg
		outboundLeg(nav, pt, outboundHdg, 1),
		// turn inbound
		LateralManeuver{Heading: inboundHdg, Turn: ptTurn, Until: ManeuverComplete{Type: UntilHeading, Heading: inboundHdg}})

	return maneuvers
}

// outboundLeg returns a LateralManeuver for the outbound leg of a
// procedure turn, using the correct completion condition based on what was
// specified: UntilDist if a distance was given, UntilTime if a time was
// given. scale multiplies the extent (e.g. 1.5 for teardrop legs).
func outboundLeg(nav *Nav, pt *av.ProcedureTurn, heading math.MagneticHeading, scale float32) LateralManeuver {
	step := LateralManeuver{Heading: heading, Turn: av.TurnClosest}
	if pt.NmLimit > 0 {
		step.Until = ManeuverComplete{Type: UntilDist, Dist: pt.NmLimit * scale}
	} else if pt.MinuteLimit > 0 {
		step.Until = ManeuverComplete{Type: UntilTime, Seconds: pt.MinuteLimit * 60 * scale}
	} else {
		// Default: 1 minute for ILS/LOC/VOR, 4nm for RNAV
		switch nav.Approach.Assigned.Type {
		case av.ILSApproach, av.LocalizerApproach, av.VORApproach:
			step.Until = ManeuverComplete{Type: UntilTime, Seconds: 60 * scale}
		case av.RNAVApproach:
			step.Until = ManeuverComplete{Type: UntilDist, Dist: 4 * scale}
		default:
			panic(fmt.Sprintf("unhandled approach type: %s", nav.Approach.Assigned.Type))
		}
	}
	return step
}

///////////////////////////////////////////////////////////////////////////
// Holds

type FlyHold struct {
	Hold         av.Hold
	FixLocation  math.Point2LL
	State        HoldState
	LegStartTime Time
	LegStartPos  math.Point2LL
	Entry        av.HoldEntry
	Cancel       bool // when set, we end the hold after the last leg
}

type HoldState int

const (
	// Everyone starts here and then transitions to one of the next three groups depending on their entry method.
	HoldStateApproaching HoldState = iota

	HoldStateDirectTurningInitialOutbound

	HoldStateTurningForParallelEntry
	HoldStateFlyingParallelOutbound
	HoldStateTurningParallelInbound

	HoldStateFlyingTeardropOutbound
	HoldStateTurningForTeardropEntry

	// All holds cycle through these once after entry.
	HoldStateTurningOutbound
	HoldStateFlyingOutbound
	HoldStateTurningInbound
	HoldStateFlyingInbound
)

func (s HoldState) String() string {
	return []string{"Approaching", "DirectTurningInitialOutbound", "TurningForParallelEntry", "FlyingParallelOutbound",
		"TurningParallelInbound", "FlyingTeardropOutbound", "TurningForTeardropEntry", "TurningOutbound",
		"FlyingOutbound", "TurningInbound", "FlyingInbound"}[int(s)]
}

// Holds are implemented using a simple state machine where each state is handled by a function with
// this signature.  Return values: heading to fly, which direction to turn, and which state to be in
// for the next step.
type HoldStateFunc func(nav *Nav, fh *FlyHold, wxs wx.Sample, simTime Time) (math.MagneticHeading, av.TurnDirection, HoldState)

var holdStateMachine map[HoldState]HoldStateFunc

func init() {
	holdStateMachine = map[HoldState]HoldStateFunc{
		HoldStateApproaching: func(nav *Nav, fh *FlyHold, wxs wx.Sample, simTime Time) (math.MagneticHeading, av.TurnDirection, HoldState) {
			switch fh.Entry {
			case av.HoldEntryDirect:
				// Overfly the fix before starting to turn
				if nav.ETA(fh.FixLocation) < 2 {
					return nav.FlightState.Heading, av.TurnClosest, HoldStateDirectTurningInitialOutbound
				}

			case av.HoldEntryParallel:
				outbound := math.OppositeHeading(fh.Hold.InboundCourse)

				if nav.shouldTurnForOutbound(fh.FixLocation, outbound, av.TurnClosest, wxs) {
					return outbound, av.TurnClosest, HoldStateTurningForParallelEntry
				}

			case av.HoldEntryTeardrop:
				if nav.ETA(fh.FixLocation) < 2 {
					// For teardrop, we want to overfly the fix before we start the entry procedure
					return nav.FlightState.Heading, av.TurnClosest, HoldStateFlyingTeardropOutbound
				}
			}

			magHdg := math.TrueToMagnetic(math.Heading2LL(nav.FlightState.Position, fh.FixLocation,
				nav.FlightState.NmPerLongitude), nav.FlightState.MagneticVariation)
			hdg := nav.headingForTrack(magHdg, wxs)
			return hdg, av.TurnClosest, HoldStateApproaching
		},

		HoldStateDirectTurningInitialOutbound: func(nav *Nav, fh *FlyHold, wxs wx.Sample, simTime Time) (math.MagneticHeading, av.TurnDirection, HoldState) {
			// Direct entry: turn to be on the inbound course before starting the outbound turn
			// (i.e. don't cut the corner if we're not entering more or less already along the
			// inbound course.)
			inbound := nav.headingForTrack(fh.Hold.InboundCourse, wxs)
			if math.HeadingDifference(nav.FlightState.Heading, inbound) < 1 {
				return inbound, av.TurnClosest, HoldStateTurningOutbound
			}
			return inbound, av.TurnClosest, HoldStateDirectTurningInitialOutbound

		},

		HoldStateTurningForParallelEntry: func(nav *Nav, fh *FlyHold, wxs wx.Sample, simTime Time) (math.MagneticHeading, av.TurnDirection, HoldState) {
			outbound := nav.headingForTrack(math.OppositeHeading(fh.Hold.InboundCourse), wxs)
			if math.HeadingDifference(nav.FlightState.Heading, outbound) < 1 {
				return outbound, av.TurnClosest, HoldStateFlyingParallelOutbound
			}
			return outbound, av.TurnClosest, HoldStateTurningForParallelEntry
		},

		HoldStateFlyingParallelOutbound: func(nav *Nav, fh *FlyHold, wxs wx.Sample, simTime Time) (math.MagneticHeading, av.TurnDirection, HoldState) {
			outbound := nav.headingForTrack(math.OppositeHeading(fh.Hold.InboundCourse), wxs)

			sec := 70 + wxs.Component(float32(fh.Hold.InboundCourse))
			if simTime.Sub(fh.LegStartTime) < time.Duration(sec)*time.Second {
				return outbound, av.TurnClosest, HoldStateFlyingParallelOutbound
			}
			return outbound, av.TurnClosest, HoldStateTurningParallelInbound
		},

		HoldStateTurningParallelInbound: func(nav *Nav, fh *FlyHold, wxs wx.Sample, simTime Time) (math.MagneticHeading, av.TurnDirection, HoldState) {
			offset := float32(util.Select(fh.Hold.TurnDirection == av.TurnRight, -40, 40))
			intercept := nav.headingForTrack(math.OffsetHeading(fh.Hold.InboundCourse, offset), wxs)

			if math.HeadingDifference(nav.FlightState.Heading, intercept) < 1 {
				turn := util.Select(fh.Hold.TurnDirection == av.TurnRight, av.TurnRight, av.TurnLeft)
				if nav.shouldTurnToIntercept(fh.FixLocation, fh.Hold.InboundCourse, turn, wxs) == turnToInterceptTurn {
					return intercept, turn, HoldStateFlyingInbound
				}
			}

			// Note: intentionally flipped!
			turn := util.Select(fh.Hold.TurnDirection == av.TurnRight, av.TurnLeft, av.TurnRight)
			return intercept, turn, HoldStateTurningParallelInbound
		},

		HoldStateFlyingTeardropOutbound: func(nav *Nav, fh *FlyHold, wxs wx.Sample, simTime Time) (math.MagneticHeading, av.TurnDirection, HoldState) {
			offset := float32(util.Select(fh.Hold.TurnDirection == av.TurnRight, 150, -150))
			hdg := nav.headingForTrack(math.OffsetHeading(fh.Hold.InboundCourse, offset), wxs)

			if math.HeadingDifference(nav.FlightState.Heading, hdg) < 1 {
				sec := 70 + wxs.Component(float32(fh.Hold.InboundCourse))
				if simTime.Sub(fh.LegStartTime) > time.Duration(sec)*time.Second {
					return hdg, av.TurnClosest, HoldStateTurningForTeardropEntry
				}
			}
			return hdg, av.TurnClosest, HoldStateFlyingTeardropOutbound
		},

		HoldStateTurningForTeardropEntry: func(nav *Nav, fh *FlyHold, wxs wx.Sample, simTime Time) (math.MagneticHeading, av.TurnDirection, HoldState) {
			offset := float32(util.Select(fh.Hold.TurnDirection == av.TurnRight, 150, -150))
			hdg := nav.headingForTrack(math.OppositeHeading(math.OffsetHeading(fh.Hold.InboundCourse, offset)), wxs)
			turn := util.Select(fh.Hold.TurnDirection == av.TurnRight, av.TurnRight, av.TurnLeft)

			if nav.shouldTurnToIntercept(fh.FixLocation, fh.Hold.InboundCourse, turn, wxs) == turnToInterceptTurn {
				return fh.Hold.InboundCourse, turn, HoldStateFlyingInbound
			}

			return hdg, turn, HoldStateTurningForTeardropEntry
		},

		HoldStateTurningOutbound: func(nav *Nav, fh *FlyHold, wxs wx.Sample, simTime Time) (math.MagneticHeading, av.TurnDirection, HoldState) {
			outbound := nav.headingForTrack(math.OppositeHeading(fh.Hold.InboundCourse), wxs)
			if math.HeadingDifference(nav.FlightState.Heading, outbound) < 1 {
				return outbound, av.TurnClosest, HoldStateFlyingOutbound
			}
			turn := util.Select(fh.Hold.TurnDirection == av.TurnRight, av.TurnRight, av.TurnLeft)
			return outbound, turn, HoldStateTurningOutbound
		},

		HoldStateFlyingOutbound: func(nav *Nav, fh *FlyHold, wxs wx.Sample, simTime Time) (math.MagneticHeading, av.TurnDirection, HoldState) {
			done := func() bool {
				if fh.Hold.LegLengthNM > 0 {
					dist := math.NMDistance2LL(fh.LegStartPos, nav.FlightState.Position)
					return dist >= fh.Hold.LegLengthNM
				} else {
					mins := fh.Hold.LegMinutes
					if mins == 0 {
						mins = float32(util.Select(nav.FlightState.Altitude < 14000, 1.0, 1.5))
					}

					windComp := wxs.Component(float32(fh.Hold.InboundCourse))
					adjSeconds := mins*60 + windComp*mins
					return simTime.Sub(fh.LegStartTime) >= time.Duration(adjSeconds)*time.Second
				}
			}()

			outbound := nav.headingForTrack(math.OppositeHeading(fh.Hold.InboundCourse), wxs)
			if done {
				return outbound, av.TurnClosest, HoldStateTurningInbound
			}
			return outbound, av.TurnClosest, HoldStateFlyingOutbound
		},

		HoldStateTurningInbound: func(nav *Nav, fh *FlyHold, wxs wx.Sample, simTime Time) (math.MagneticHeading, av.TurnDirection, HoldState) {
			turn := util.Select(fh.Hold.TurnDirection == av.TurnRight, av.TurnRight, av.TurnLeft)

			if math.HeadingDifference(nav.FlightState.Heading, fh.Hold.InboundCourse) > 5 {
				return fh.Hold.InboundCourse, turn, HoldStateTurningInbound
			}

			return fh.Hold.InboundCourse, av.TurnClosest, HoldStateFlyingInbound
		},

		HoldStateFlyingInbound: func(nav *Nav, fh *FlyHold, wxs wx.Sample, simTime Time) (math.MagneticHeading, av.TurnDirection, HoldState) {
			// Fly direct to the fix; hopefully this will be close to the inbound heading.
			magHdg := math.TrueToMagnetic(math.Heading2LL(nav.FlightState.Position, fh.FixLocation,
				nav.FlightState.NmPerLongitude), nav.FlightState.MagneticVariation)
			hdg := nav.headingForTrack(magHdg, wxs)

			if nav.ETA(fh.FixLocation) < 2 {
				if fh.Cancel {
					nav.Heading = NavHeading{} // switch to straight-up direct to the next fix
				} else {
					return hdg, av.TurnClosest, HoldStateTurningOutbound
				}
			}
			return hdg, av.TurnClosest, HoldStateFlyingInbound
		},
	}
}

func (fh *FlyHold) GetHeading(callsign string, nav *Nav, wxs wx.Sample, simTime Time) (math.MagneticHeading, av.TurnDirection, float32) {
	hdg, turn, newState := holdStateMachine[fh.State](nav, fh, wxs, simTime)

	if newState != fh.State {
		NavLog(callsign, simTime, NavLogHold, "STATE CHANGE: %s -> %s (fix=%s entry=%s)",
			fh.State.String(), newState.String(), fh.Hold.Fix, fh.Entry.String())
		fh.State = newState
		fh.LegStartPos = nav.FlightState.Position
		fh.LegStartTime = simTime
	}

	dist := math.NMDistance2LL(nav.FlightState.Position, fh.FixLocation)
	NavLog(callsign, simTime, NavLogHold, "state=%s acHdg=%.1f targetHdg=%.1f turn=%v dist=%.1fnm timer=%s",
		fh.State.String(), nav.FlightState.Heading, hdg, turn, dist, simTime.Sub(fh.LegStartTime))

	return hdg, turn, StandardTurnRate
}

func StartAirwork(wp av.Waypoint, nav Nav) *NavAirwork {
	a := &NavAirwork{
		Radius:         float32(wp.AirworkRadius()),
		Center:         wp.Location,
		AltRange:       wp.AltitudeRestriction().Range,
		RemainingSteps: wp.AirworkMinutes() * 60, // sim ticks are 1 second.
		Altitude:       nav.FlightState.Altitude,
	}

	a.Start360(nav)

	return a
}

func (aw *NavAirwork) Update(nav *Nav) bool {
	// Tick down the number of seconds we're doing this.
	aw.RemainingSteps--
	if aw.RemainingSteps == 0 {
		// Direct to the next waypoint in the route
		nav.Heading = NavHeading{}
		return false
	}

	// If we're getting close to the maximum distance from the center
	// point, turn back toward it.
	d := math.NMDistance2LL(nav.FlightState.Position, aw.Center)
	if aw.ToCenter && d < 1 {
		// Close enough
		aw.ToCenter = false
	} else if float32(aw.Radius)-d < 2.5 || aw.ToCenter {
		aw.Heading = math.TrueToMagnetic(math.Heading2LL(nav.FlightState.Position, aw.Center, nav.FlightState.NmPerLongitude),
			nav.FlightState.MagneticVariation)
		aw.TurnRate = StandardTurnRate
		aw.TurnDirection = av.TurnClosest
		aw.ToCenter = true
		return true
	}

	// Don't check IAS; we only care that we reach the heading and altitude
	// we wanted to do next.
	if nav.FlightState.Heading == aw.Heading && nav.FlightState.Altitude == aw.Altitude {
		if aw.NextMoveCounter == 0 {
			// We just finished. Clean up and Continue straight and level for a bit.
			aw.Dive = false
			aw.NextMoveCounter = nav.Rand.IntRange(5, 30)
		} else if aw.NextMoveCounter == 1 {
			// Pick a new thing.
			aw.ToCenter = false
			if nav.Rand.Float32() < .2 {
				// Do a 360
				aw.Start360(*nav)
			} else if nav.FlightState.Altitude > aw.AltRange[0]+2000 && nav.Rand.Float32() < .2 {
				// Dive.
				aw.Dive = true
				aw.Altitude = nav.Rand.Float32Range(aw.AltRange[0], aw.AltRange[0]+200)
			} else if nav.FlightState.Altitude+1000 < aw.AltRange[1] && nav.Rand.Float32() < .2 {
				// Climbing turn
				aw.Altitude = nav.Rand.Float32Range(aw.AltRange[1]-500, aw.AltRange[1])
				aw.Heading = math.MagneticHeading(nav.Rand.Float32Range(0, 360))
				aw.TurnDirection = util.Select(nav.Rand.Float32() < .5, av.TurnLeft, av.TurnRight)
			} else if nav.FlightState.Altitude < aw.AltRange[0]+1000 && nav.Rand.Float32() < .2 {
				// Descending turn
				aw.Altitude = nav.Rand.Float32Range(aw.AltRange[0], aw.AltRange[0]+500)
				aw.Heading = math.MagneticHeading(nav.Rand.Float32Range(0, 360))
				aw.TurnDirection = util.Select(nav.Rand.Float32() < .5, av.TurnLeft, av.TurnRight)
			} else if nav.Rand.Float32() < .2 {
				// Slow turn
				aw.Heading = math.MagneticHeading(nav.Rand.Float32Range(0, 360))
				aw.IAS = math.Lerp(.1, nav.Perf.Speed.Min, av.TASToIAS(nav.Perf.Speed.CruiseTAS, nav.FlightState.Altitude))
				aw.TurnDirection = util.Select(nav.Rand.Float32() < .5, av.TurnLeft, av.TurnRight)
			} else if nav.Rand.Float32() < .2 {
				// Slow, straight and level
				aw.IAS = math.Lerp(.1, nav.Perf.Speed.Min, av.TASToIAS(nav.Perf.Speed.CruiseTAS, nav.FlightState.Altitude))
				aw.NextMoveCounter = 20
			} else {
				// Straight and level and then we'll reconsider.
				aw.NextMoveCounter = 10
			}
		}
		// Tick
		aw.NextMoveCounter--
	}

	return true
}

func (aw *NavAirwork) Start360(nav Nav) {
	if nav.Rand.Intn(2) == 0 {
		aw.TurnDirection = av.TurnLeft
		aw.Heading = math.OffsetHeading(nav.FlightState.Heading, 1)
	} else {
		aw.TurnDirection = av.TurnRight
		aw.Heading = math.OffsetHeading(nav.FlightState.Heading, -1)
	}
	aw.TurnRate = StandardTurnRate
}

func (aw *NavAirwork) TargetHeading() (math.MagneticHeading, av.TurnDirection, float32) {
	return aw.Heading, aw.TurnDirection, aw.TurnRate
}

func (aw *NavAirwork) TargetAltitude() (float32, float32, bool) {
	return aw.Altitude, float32(util.Select(aw.Dive, 3000, 500)), false
}

func (aw *NavAirwork) TargetSpeed() (float32, float32, bool) {
	if aw.IAS == 0 {
		return 0, 0, false
	}
	return aw.IAS, 10, true
}

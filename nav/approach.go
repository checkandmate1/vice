// nav/approaches.go
// Copyright(c) 2022-2025 vice contributors, licensed under the GNU Public License, Version 3.
// SPDX: GPL-3.0-only

package nav

import (
	"log/slog"
	"slices"
	"time"

	av "github.com/mmp/vice/aviation"
	"github.com/mmp/vice/log"
	"github.com/mmp/vice/math"
	"github.com/mmp/vice/util"
	"github.com/mmp/vice/wx"
)

func (nav *Nav) ApproachHeading(callsign string, wxs wx.Sample, simTime Time) (heading math.MagneticHeading, turn av.TurnDirection) {
	// Baseline
	heading, turn = *nav.Heading.Assigned, av.TurnClosest

	ap := nav.Approach.Assigned
	hasLocalizer := ap.Type == av.ILSApproach || ap.Type == av.LocalizerApproach

	// Determine the course and line to intercept.
	var courseTrue math.TrueHeading
	var courseLine [2]math.Point2LL
	var interceptWaypoints []av.Waypoint // waypoints from intercept point forward
	if hasLocalizer {
		courseTrue = ap.RunwayHeading(nav.FlightState.NmPerLongitude)
		courseLine = ap.ExtendedCenterline(nav.FlightState.NmPerLongitude, nav.FlightState.MagneticVariation)
	} else if nav.Approach.InterceptState == TurningToJoin {
		// Already committed to a segment; use cached values.
		courseLine = nav.Approach.InterceptCourseLine
		courseTrue = math.Heading2LL(courseLine[0], courseLine[1], nav.FlightState.NmPerLongitude)
		interceptWaypoints = nav.Approach.InterceptWaypoints
	} else {
		// For non-ILS approaches, find the approach segment that the
		// aircraft's heading ray will cross.
		var ok bool
		courseTrue, courseLine, interceptWaypoints, ok = nav.findInterceptSegment(ap, wxs)
		if !ok {
			nav.approachOvershootRequestVectors()
			return
		}
	}

	switch nav.Approach.InterceptState {
	case InitialHeading:
		assignedMag, _ := nav.AssignedHeading() // use the deferred heading for the following
		assignedTrue := math.MagneticToTrue(assignedMag, nav.FlightState.MagneticVariation)
		if d := math.HeadingDifference(courseTrue, assignedTrue); d > 45 {
			// Too big an intercept angle; request vectors
			nav.approachOvershootRequestVectors()
			return
		}
		// If the aircraft is still turning to the assigned heading, don't
		// evaluate the intercept yet — shouldTurnToIntercept would simulate
		// a direct turn from the current physical heading to the localizer,
		// which isn't the path the aircraft is going to fly. Re-check on a
		// later tick once the aircraft is established on the assigned heading.
		if math.HeadingDifference(nav.FlightState.Heading, assignedMag) > 5 {
			return
		}
		hdgMag := math.TrueToMagnetic(courseTrue, nav.FlightState.MagneticVariation)
		switch nav.shouldTurnToIntercept(courseLine[0], hdgMag, av.TurnClosest, wxs) {
		case turnToInterceptWait:
			// Still too far; keep flying the assigned heading.

		case turnToInterceptTurn:
			if hasLocalizer && !nav.approachRecoveryFeasible(ap) {
				nav.approachOvershootRequestVectors()
				return
			}
			nav.Approach.InterceptState = TurningToJoin
			if !hasLocalizer {
				nav.Approach.InterceptCourseLine = courseLine
				nav.Approach.InterceptWaypoints = interceptWaypoints
			}
			nav.Heading = NavHeading{Assigned: &hdgMag}
			nav.DeferredNavHeading = nil
			nav.Waypoints = []av.Waypoint{nav.FlightState.ArrivalAirport}

		case turnToInterceptCorrectableOvershoot:
			if hasLocalizer && !nav.approachRecoveryFeasible(ap) {
				nav.approachOvershootRequestVectors()
				return
			}
			acftTrue := math.MagneticToTrue(nav.FlightState.Heading, nav.FlightState.MagneticVariation)
			signed := math.HeadingSignedTurn(acftTrue, courseTrue)
			offset := float32(20)
			if signed < 0 {
				offset = -20
			}
			recoveryTrue := math.OffsetHeading(courseTrue, offset)
			recoveryHdg := math.TrueToMagnetic(recoveryTrue, nav.FlightState.MagneticVariation)
			nav.Approach.InterceptState = TurningToJoin
			if !hasLocalizer {
				nav.Approach.InterceptCourseLine = courseLine
				nav.Approach.InterceptWaypoints = interceptWaypoints
			}
			nav.Heading = NavHeading{Assigned: &recoveryHdg}
			nav.DeferredNavHeading = nil
			nav.Waypoints = []av.Waypoint{nav.FlightState.ArrivalAirport}

		case turnToInterceptMajorOvershoot:
			nav.approachOvershootRequestVectors()
		}
		return

	case TurningToJoin:
		// we've turned to intercept. have we intercepted?
		if !nav.onCourseLine(courseLine, .2) {
			// Apply wind correction to track the approach course, not just
			// fly the course heading. Without this, strong crosswind would
			// blow the aircraft off the course.
			heading = nav.headingForTrack(*nav.Heading.Assigned, wxs)
			NavLog(callsign, simTime, NavLogApproach, "TurningToJoin: not on course, flying wind-corrected hdg %.0f (course hdg %.0f)", heading, *nav.Heading.Assigned)
			return
		}
		NavLog(callsign, simTime, NavLogApproach, "TurningToJoin->OnApproachCourse: established on approach course")

		// We're established on the approach course. Figure out which
		// fixes are still ahead and add them to the aircraft's waypoints.
		if hasLocalizer {
			apHeading := ap.RunwayHeading(nav.FlightState.NmPerLongitude)
			wps, idx := ap.FAFSegment(nav.FlightState.NmPerLongitude, nav.FlightState.MagneticVariation)
			acftTrue := math.MagneticToTrue(nav.FlightState.Heading, nav.FlightState.MagneticVariation)
			for idx > 0 {
				prev := wps[idx-1]
				hdg := math.Heading2LL(prev.Location, wps[idx].Location,
					nav.FlightState.NmPerLongitude)

				if math.HeadingDifference(hdg, apHeading) > 5 { // not on the final approach course
					break
				}

				acToWpHeading := math.Heading2LL(nav.FlightState.Position, wps[idx].Location,
					nav.FlightState.NmPerLongitude)
				acToPrevHeading := math.Heading2LL(nav.FlightState.Position, wps[idx-1].Location,
					nav.FlightState.NmPerLongitude)

				da := math.Mod(float32(acToWpHeading-acftTrue)+360, 360)
				db := math.Mod(float32(acToPrevHeading-acftTrue)+360, 360)
				if (da < 180 && db > 180) || (da > 180 && db < 180) {
					// prev and current are on different sides of the current
					// heading, so don't take the prev so we don't turn away
					// from where we should be going.
					break
				}
				idx--
			}
			nav.Waypoints = append(util.DuplicateSlice(wps[idx:]), nav.FlightState.ArrivalAirport)
		} else {
			nav.Waypoints = append(util.DuplicateSlice(interceptWaypoints),
				nav.FlightState.ArrivalAirport)
		}

		// Ignore the approach altitude constraints if the aircraft is only
		// intercepting but isn't cleared.
		if nav.Approach.Cleared {
			nav.clearAltitudeForApproach()
		}
		// As with the heading assignment above under the InitialHeading
		// case, do this immediately.
		nav.Heading = NavHeading{}
		nav.Approach.InterceptState = OnApproachCourse

		// If we have intercepted the approach course, we don't do procedure turns.
		nav.Approach.NoPT = true

		return
	}

	return
}

// findInterceptSegment finds the approach segment that the aircraft's
// ground track will cross. Returns the segment course, the two endpoints
// defining the line, the remaining waypoints from the intercept point
// forward, and whether a valid segment was found.
func (nav *Nav) findInterceptSegment(ap *av.Approach, wxs wx.Sample) (math.TrueHeading, [2]math.Point2LL, []av.Waypoint, bool) {
	nmPerLong := nav.FlightState.NmPerLongitude
	posNM := math.LL2NM(nav.FlightState.Position, nmPerLong)

	// Build a ray from the aircraft's position along its ground track
	// (heading + wind), extending far enough to cross any approach segment.
	hdgTrue := math.MagneticToTrue(nav.FlightState.Heading, nav.FlightState.MagneticVariation)
	TAS := nav.TAS(wxs.Temperature()) / 3600
	flightVec := math.Scale2f(math.SinCos(math.Radians(hdgTrue)), TAS)
	groundVec := math.Add2f(flightVec, wxs.WindVec())
	dir := math.Normalize2f(groundVec)
	rayEnd := math.Add2f(posNM, math.Scale2f(dir, 50)) // 50nm ray

	bestDist := float32(1e9)
	var bestCourse math.TrueHeading
	var bestLine [2]math.Point2LL
	var bestWaypoints []av.Waypoint
	found := false

	for _, route := range ap.Waypoints {
		for i := 0; i < len(route)-1; i++ {
			p0 := math.LL2NM(route[i].Location, nmPerLong)
			p1 := math.LL2NM(route[i+1].Location, nmPerLong)

			// Check if the aircraft's heading ray crosses this segment.
			pt, ok := math.SegmentSegmentIntersect(posNM, rayEnd, p0, p1)
			if !ok {
				continue
			}

			// Use the closest crossing point.
			dist := math.Distance2f(posNM, pt)
			if dist < bestDist {
				bestDist = dist
				bestCourse = math.Heading2LL(route[i].Location, route[i+1].Location, nmPerLong)
				bestLine = [2]math.Point2LL{route[i].Location, route[i+1].Location}
				bestWaypoints = route[i+1:]
				found = true
			}
		}
	}

	return bestCourse, bestLine, bestWaypoints, found
}

// approachRecoveryFeasible returns true if the aircraft's current position
// allows a turn back to the localizer: it must not be too close to the FAF
// and must be within the localizer capture cone (2° half-width).
func (nav *Nav) approachRecoveryFeasible(ap *av.Approach) bool {
	nmPerLong := nav.FlightState.NmPerLongitude
	magVar := nav.FlightState.MagneticVariation
	pos := nav.FlightState.Position

	// Check proximity to the FAF: need at least 2nm along the course.
	wps, fafIdx := ap.FAFSegment(nmPerLong, magVar)
	if wps != nil {
		fafDist := math.NMDistance2LLFast(pos, wps[fafIdx].Location, nmPerLong)
		if fafDist < 2 {
			return false
		}
	}

	// Check if within the localizer capture cone (2° half-width from threshold).
	cl := ap.ExtendedCenterline(nmPerLong, magVar)
	posNM := math.LL2NM(pos, nmPerLong)
	cl0NM := math.LL2NM(cl[0], nmPerLong)
	cl1NM := math.LL2NM(cl[1], nmPerLong)
	lateralOffset := math.Abs(math.SignedPointLineDistance(posNM, cl0NM, cl1NM))
	distFromThreshold := math.NMDistance2LLFast(pos, ap.Threshold, nmPerLong)
	coneHalfWidth := distFromThreshold * math.Tan(math.Radians(float32(2)))
	return lateralOffset <= coneHalfWidth
}

// approachOvershootRequestVectors cancels the approach and flags the
// pilot to request new vectors from ATC.
func (nav *Nav) approachOvershootRequestVectors() {
	nav.Approach.InterceptState = NotIntercepting
	nav.Approach.Cleared = false
	nav.Approach.RequestVectors = true
}

func (nav *Nav) getApproach(airport *av.Airport, id string, lg *log.Logger) (*av.Approach, error) {
	if id == "" {
		return nil, ErrInvalidApproach
	}

	for name, appr := range airport.Approaches {
		if name == id {
			return appr, nil
		}
	}
	return nil, ErrUnknownApproach
}

func (nav *Nav) ExpectApproach(airport *av.Airport, id string, runwayWaypoints map[string]av.WaypointArray,
	lahsoRunway string, lg *log.Logger) av.CommandIntent {
	ap, err := nav.getApproach(airport, id, lg)
	if err != nil {
		return av.MakeUnableIntent("unable. We don't know the {appr} approach.", id)
	}

	if id == nav.Approach.AssignedId && nav.Approach.Assigned != nil {
		nav.Approach.StandbyApproach = true
		return av.ApproachIntent{
			Type:         av.ApproachExpect,
			ApproachName: ap.FullName,
			LAHSORunway:  lahsoRunway,
		}
	}

	nav.Approach.Assigned = ap
	nav.Approach.AssignedId = id
	nav.Approach.ATPAVolume = airport.ATPAVolumes[ap.Runway]

	if waypoints := runwayWaypoints[ap.Runway]; len(waypoints) > 0 {
		if len(nav.Waypoints) == 0 {
			// Nothing left on our route; this shouldn't ever happen but
			// just in case patch the runway waypoints in there and hope it
			// works out.
			nav.Waypoints = append(util.DuplicateSlice(waypoints[1:]), nav.FlightState.ArrivalAirport)
		} else {
			// Try to splice the runway-specific waypoints in with the
			// aircraft's current waypoints...
			found := false
			for i, wp := range waypoints {
				navwp := nav.AssignedWaypoints()
				if idx := slices.IndexFunc(navwp, func(w av.Waypoint) bool { return w.Fix == wp.Fix }); idx != -1 {
					// This is a little messy: there are a handful of
					// modifiers we would like to carry over if they are
					// set though in general the waypoint from the approach
					// takes priority for things like altitude, speed, etc.
					nopt := navwp[idx].NoPT()
					humanHandoff := navwp[idx].HumanHandoff()
					tcpHandoff := navwp[idx].HandoffController()
					clearapp := navwp[idx].ClearApproach()

					// Keep the waypoints up to but not including the match.
					navwp = navwp[:idx]
					// Add the approach waypoints; take the matching one from there.
					navwp = append(navwp, waypoints[i:]...)
					// And add the destination airport again at the end.
					navwp = append(navwp, nav.FlightState.ArrivalAirport)

					navwp[idx].SetNoPT(nopt)
					navwp[idx].SetHumanHandoff(humanHandoff)
					navwp[idx].InitExtra().HandoffController = tcpHandoff
					navwp[idx].SetClearApproach(clearapp)

					// Update the deferred waypoints if present (as they're
					// what we got from AssignedWaypoints() above) and
					// otherwise the regular ones. Arguably we'd like to
					// defer the route change but don't have a way to do
					// that that preserves the current assigned heading, etc.
					if dh := nav.DeferredNavHeading; dh != nil && len(dh.Waypoints) > 0 {
						dh.Waypoints = navwp
					} else {
						nav.Waypoints = navwp
					}

					found = true
					break
				}
			}

			if !found {
				// Most likely they were told to expect one runway, then
				// given a different one, but after they passed the common
				// set of waypoints on the arrival.  We'll replace the
				// waypoints but leave them on their current heading; then
				// it's over to the controller to either vector them or
				// send them direct somewhere reasonable...
				lg.Info("aircraft waypoints don't match up with arrival runway waypoints. splicing...",
					slog.Any("aircraft", nav.Waypoints),
					slog.Any("runway", waypoints))
				nav.Waypoints = append(util.DuplicateSlice(waypoints), nav.FlightState.ArrivalAirport)

				hdg := nav.FlightState.Heading
				nav.Heading = NavHeading{Assigned: &hdg}
				nav.DeferredNavHeading = nil
			}
		}
	}

	return av.ApproachIntent{
		Type:         av.ApproachExpect,
		ApproachName: ap.FullName,
		LAHSORunway:  lahsoRunway,
	}
}

func (nav *Nav) InterceptApproach(airport string, lg *log.Logger) av.CommandIntent {
	if nav.Approach.AssignedId == "" {
		return av.MakeUnableIntent("unable. you never told us to expect an approach")
	}

	if _, onHeading := nav.AssignedHeading(); !onHeading {
		wps := nav.AssignedWaypoints()
		if len(wps) == 0 || !wps[0].OnApproach() {
			return av.MakeUnableIntent("unable. we have to be on a heading or direct to an approach fix to intercept")
		}
	}

	if intent := nav.prepareForApproach(false); intent != nil {
		return intent
	}

	ap := nav.Approach.Assigned
	if ap.Type == av.ILSApproach || ap.Type == av.LocalizerApproach {
		return av.ApproachIntent{
			Type:         av.ApproachIntercept,
			ApproachName: ap.FullName,
		}
	}
	return av.ApproachIntent{
		Type:         av.ApproachJoin,
		ApproachName: ap.FullName,
	}
}

func (nav *Nav) AtFixCleared(fix, id string, straightIn bool) av.CommandIntent {
	if nav.Approach.AssignedId == "" {
		return av.MakeUnableIntent("unable. you never told us to expect an approach")
	}

	ap := nav.Approach.Assigned
	if ap == nil {
		return av.MakeUnableIntent("unable. We were never told to expect an approach")
	}
	if id != "" && nav.Approach.AssignedId != id {
		return av.MakeUnableIntent("unable. We were told to expect the {appr} approach.", ap.FullName)
	}

	if !slices.ContainsFunc(nav.AssignedWaypoints(), func(wp av.Waypoint) bool { return wp.Fix == fix }) {
		return av.MakeUnableIntent("unable. {fix} is not in our route", fix)
	}
	nav.Approach.AtFixClearedRoute = nil
	for _, route := range ap.Waypoints {
		for i, wp := range route {
			if wp.Fix == fix {
				nav.Approach.AtFixClearedRoute = util.DuplicateSlice(route[i:])
			}
		}
	}

	if nav.Approach.AtFixClearedRoute == nil {
		return av.MakeUnableIntent("unable. {fix} is not on the {appr} approach", fix, ap.FullName)
	}
	if straightIn && len(nav.Approach.AtFixClearedRoute) > 0 {
		nav.Approach.AtFixClearedRoute[0].SetNoPT(true)
	}

	return av.ApproachIntent{
		Type:         av.ApproachAtFixCleared,
		ApproachName: ap.FullName,
		Fix:          fix,
		StraightIn:   straightIn,
	}
}

func (nav *Nav) AtFixIntercept(fix, airport string, lg *log.Logger) av.CommandIntent {
	if nav.Approach.AssignedId == "" {
		return av.MakeUnableIntent("unable. you never told us to expect an approach")
	}

	ap := nav.Approach.Assigned
	if ap == nil {
		return av.MakeUnableIntent("unable. We were never told to expect an approach")
	}

	if !slices.ContainsFunc(nav.AssignedWaypoints(), func(wp av.Waypoint) bool { return wp.Fix == fix }) {
		return av.MakeUnableIntent("unable. {fix} is not in our route", fix)
	}

	// Store the fix where the aircraft should intercept
	nav.Approach.AtFixInterceptFix = fix

	return av.ApproachIntent{
		Type:         av.ApproachAtFixIntercept,
		ApproachName: ap.FullName,
		Fix:          fix,
		HasLocalizer: ap.Type == av.ILSApproach || ap.Type == av.LocalizerApproach,
	}
}

func (nav *Nav) prepareForApproach(straightIn bool) av.CommandIntent {
	if nav.Approach.AssignedId == "" {
		return av.MakeUnableIntent("unable. you never told us to expect an approach")
	}

	ap := nav.Approach.Assigned

	// Charted visual is special in all sorts of ways
	if ap.Type == av.ChartedVisualApproach {
		return nav.prepareForChartedVisual()
	}

	directApproachFix := false
	_, assignedHeading := nav.AssignedHeading()
	if !assignedHeading {
		// See if any of the waypoints in our route connect to the approach. Prefer a route where
		// the fix has a procedure turn; some approaches have multiple transitions where a fix
		// appears both with and without a PT.
		navwps := nav.AssignedWaypoints()
		bestRoute, bestIdx, navIdx := func() ([]av.Waypoint, int, int) {
			for i, wp := range navwps {
				var candidate []av.Waypoint
				var candidateIdx int
				for _, route := range ap.Waypoints {
					idx := slices.IndexFunc(route, func(awp av.Waypoint) bool { return wp.Fix == awp.Fix })
					if idx == -1 {
						continue
					}
					if route[idx].ProcedureTurn() != nil {
						return route, idx, i
					}
					if candidate == nil {
						candidate, candidateIdx = route, idx
					}
				}
				if candidate != nil {
					return candidate, candidateIdx, i
				}
			}
			return nil, 0, 0
		}()

		if bestRoute != nil {
			directApproachFix = true
			navwps = append(navwps[:navIdx], bestRoute[bestIdx:]...)
			navwps = append(navwps, nav.FlightState.ArrivalAirport)
			if dh := nav.DeferredNavHeading; dh != nil && len(dh.Waypoints) > 0 {
				dh.Waypoints = navwps
			} else {
				nav.Waypoints = navwps
			}
		}
	}

	if directApproachFix {
		// The aircraft is going direct to an approach fix; clear any
		// OnApproachCourse state that DirectFix may have set (which
		// is used to gate altitude before clearance). Without this,
		// ClearedApproach incorrectly sets NoPT when it sees
		// OnApproachCourse, skipping the procedure turn.
		nav.Approach.InterceptState = NotIntercepting
	} else if assignedHeading {
		nav.Approach.InterceptState = InitialHeading
	} else {
		return av.MakeUnableIntent("unable. We need either direct or a heading to intercept")
	}
	// If the aircraft is on a heading, there's nothing more to do for
	// now; keep flying the heading and after we intercept we'll add
	// the rest of the waypoints to the aircraft's waypoints array.

	// No procedure turn if it intercepts via a heading or we're coming off a hold.
	nav.Approach.NoPT = straightIn || assignedHeading || nav.Heading.Hold != nil

	return nil
}

func (nav *Nav) prepareForChartedVisual() av.CommandIntent {
	// Airport PostDeserialize() checks that there is just a single set of
	// waypoints for charted visual approaches.
	wp := nav.Approach.Assigned.Waypoints[0]

	// First try to find the first (if any) waypoint along the approach
	// that is within 15 degrees of the aircraft's current heading.
	intercept := -1
	acftTrue := math.MagneticToTrue(nav.FlightState.Heading, nav.FlightState.MagneticVariation)
	for i := range wp {
		h := math.Heading2LL(nav.FlightState.Position, wp[i].Location,
			nav.FlightState.NmPerLongitude)

		if math.HeadingDifference(h, acftTrue) < 30 {
			intercept = i
			break
		}
	}

	// Also check for intercepting a segment of the approach. There are two
	// cases:
	// 1. If we found a waypoint intercept above, then we are only
	//    interested in the segment from that waypoint to the subsequent
	//    one; we will take that if we find it (so the aircraft can stay
	//    on its present heading) but will not take a later one (so that it
	//    gets on the approach sooner rather than later.)
	// 2. If no waypoint intercept is found, we will take the first
	//    intercept with an approach segment. This case should be unusual
	//    but may come into play when an aircraft is very close to the
	//    approach route and no waypoints are close to its current course.

	// Work in nm coordinates
	pac0 := math.LL2NM(nav.FlightState.Position, nav.FlightState.NmPerLongitude)
	// Find a second point along its current course (note: ignoring wind)
	hdg := math.MagneticToTrue(nav.FlightState.Heading, nav.FlightState.MagneticVariation)
	dir := math.SinCos(math.Radians(hdg))
	pac1 := math.Add2f(pac0, dir)

	checkSegment := func(i int) *av.Waypoint {
		if i+1 == len(wp) {
			return nil
		}
		pl0 := math.LL2NM(wp[i].Location, nav.FlightState.NmPerLongitude)
		pl1 := math.LL2NM(wp[i+1].Location, nav.FlightState.NmPerLongitude)

		if pi, ok := math.LineLineIntersect(pac0, pac1, pl0, pl1); ok {
			// We only want intersections along the segment from pl0 to pl1
			// and not along the infinite line they define, so this is a
			// hacky check to limit to that.
			if math.Extent2DFromPoints([][2]float32{pl0, pl1}).Inside(pi) {
				return &av.Waypoint{
					Fix:      "intercept",
					Location: math.NM2LL(pi, nav.FlightState.NmPerLongitude),
				}
			}
		}
		return nil
	}

	// wi will store the route the aircraft will fly if it is going to join
	// the approach.
	var wi []av.Waypoint

	if intercept == -1 { // check all of the segments
		for i := range wp {
			if w := checkSegment(i); w != nil {
				// Take the first one that works
				wi = append([]av.Waypoint{*w}, wp[i+1:]...)
				break
			}
		}
	} else {
		// Just check the segment after the waypoint we're considering
		if w := checkSegment(intercept); w != nil {
			wi = append([]av.Waypoint{*w}, wp[intercept+1:]...)
		} else {
			// No problem if it doesn't intersect that segment; just start
			// the route from that waypoint.
			wi = wp[intercept:]
		}
	}

	if wi == nil {
		return av.MakeUnableIntent("unable. We are not on course to intercept the approach")
	}

	// Update the route and go direct to the intercept point.
	nav.Waypoints = append(wi, nav.FlightState.ArrivalAirport)
	nav.Heading = NavHeading{}
	nav.DeferredNavHeading = nil
	return nil
}

func (nav *Nav) ClearedApproach(airport string, id string, straightIn bool, simTime Time) av.CommandIntent {
	ap := nav.Approach.Assigned
	if ap == nil {
		return av.MakeUnableIntent("unable. We haven't been told to expect an approach")
	}
	if id != "" && nav.Approach.AssignedId != id {
		return av.MakeUnableIntent("unable. We were told to expect the {appr} approach.", ap.FullName)
	}

	if intent := nav.prepareForApproach(straightIn); intent != nil {
		return intent
	}

	nav.Approach.Cleared = true
	nav.Approach.StandbyApproach = false
	if nav.Approach.PassedApproachFix {
		// We've already passed an approach fix, so allow it to start descending.
		nav.clearAltitudeForApproach()
	} else if nav.Approach.InterceptState == OnApproachCourse {
		// First intercepted then cleared or otherwise passed an
		// approach fix, so allow it to start descending.
		nav.clearAltitudeForApproach()
		// No procedure turn needed if we were vectored to intercept.
		nav.Approach.NoPT = true
	}
	// Cleared approach also cancels speed restrictions.
	nav.Speed = NavSpeed{}

	// Minimal delay for heading changes given an approach clearance.
	if dh := nav.DeferredNavHeading; dh != nil {
		dh.Time = simTime.Add(time.Duration((1 + 2*nav.Rand.Float32()) * float32(time.Second)))
	}

	nav.flyProcedureTurnIfNecessary()

	cancelHold := nav.Heading.Hold != nil
	if nav.Heading.Hold != nil {
		nav.Heading.Hold.Cancel = true
	}

	return av.ClearedApproachIntent{
		Approach:   ap.FullName,
		StraightIn: straightIn,
		CancelHold: cancelHold,
	}
}

// visualApproachRoute returns a route for a visual approach to
// the given runway. If a reference approach is available, visual geometry
// follows its route segments; otherwise it falls back to a synthetic 3nm
// straight-in final. Returns nil if the runway is not found or if the
// first waypoint is behind the aircraft (meaning it should go around instead).
func (nav *Nav) visualApproachRoute(runway string, followTraffic *math.Point2LL, referenceApproach *av.Approach) []av.Waypoint {
	if referenceApproach != nil && len(referenceApproach.Waypoints) > 0 {
		if wps := nav.visualApproachRouteFromApproach(runway, followTraffic, referenceApproach); wps != nil {
			return wps
		}
	}
	if synthetic := nav.visualApproachFromRunway(runway); synthetic != nil {
		return nav.visualApproachRouteFromApproach(runway, followTraffic, synthetic)
	}
	return nil
}

func (nav *Nav) visualApproachFromRunway(runway string) *av.Approach {
	rwy, ok := av.LookupRunway(nav.FlightState.ArrivalAirport.Fix, runway)
	if !ok {
		return nil
	}

	nmPerLong := nav.FlightState.NmPerLongitude
	rwyTrue := math.MagneticToTrue(rwy.Heading, nav.FlightState.MagneticVariation)
	threshold := math.Offset2LL(rwy.Threshold, rwyTrue, rwy.DisplacedThresholdDistance, nmPerLong)
	reciprocal := math.TrueHeading(math.NormalizeHeading(float32(rwyTrue) + 180))
	extendedFinal := math.Offset2LL(threshold, reciprocal, 25, nmPerLong)

	thresholdWp := av.Waypoint{
		Fix:      "_" + runway + "_THRESHOLD",
		Location: threshold,
	}
	thresholdWp.SetLand(true)
	thresholdWp.SetFlyOver(true)
	thresholdWp.SetAltitudeRestriction(av.MakeAtAltitudeRestriction(float32(rwy.Elevation + rwy.ThresholdCrossingHeight)))

	return &av.Approach{
		Id:        "VIS" + runway,
		FullName:  "Visual Approach Runway " + runway,
		Type:      av.VisualApproach,
		Runway:    runway,
		Threshold: threshold,
		Waypoints: []av.WaypointArray{{
			{Fix: "_" + runway + "_EXTENDED_FINAL", Location: extendedFinal},
			thresholdWp,
		}},
	}
}

type visualApproachPoint struct {
	route               []av.Waypoint
	segment             int
	segmentFraction     float32
	location            math.Point2LL
	distanceToThreshold float32
	lateralDistance     float32
	interceptDistance   float32
	finalPoint          bool
}

func (nav *Nav) projectOntoApproachRoutes(routes []av.WaypointArray, position math.Point2LL) (visualApproachPoint, bool) {
	nmPerLong := nav.FlightState.NmPerLongitude
	posNM := math.LL2NM(position, nmPerLong)

	best := visualApproachPoint{lateralDistance: 1e9}
	found := false

	for _, route := range routes {
		if len(route) < 2 {
			continue
		}
		tailDistance := visualRouteTailDistances(route, nmPerLong)
		for i := 0; i < len(route)-1; i++ {
			p0 := math.LL2NM(route[i].Location, nmPerLong)
			p1 := math.LL2NM(route[i+1].Location, nmPerLong)
			seg := math.Sub2f(p1, p0)
			segLen2 := math.Dot(seg, seg)
			if segLen2 == 0 {
				continue
			}

			t := math.Clamp(math.Dot(math.Sub2f(posNM, p0), seg)/segLen2, 0, 1)
			proj := math.Add2f(p0, math.Scale2f(seg, t))
			lateral := math.Distance2f(posNM, proj)
			distToThreshold := math.Distance2f(proj, p1) + tailDistance[i+1]

			if !found || lateral < best.lateralDistance ||
				(lateral == best.lateralDistance && distToThreshold < best.distanceToThreshold) {
				best = visualApproachPoint{
					route:               route,
					segment:             i,
					segmentFraction:     t,
					location:            math.NM2LL(proj, nmPerLong),
					distanceToThreshold: distToThreshold,
					lateralDistance:     lateral,
				}
				found = true
			}
		}
	}

	return best, found
}

func (nav *Nav) intersectApproachRoutes(routes []av.WaypointArray, heading math.MagneticHeading) (visualApproachPoint, bool) {
	nmPerLong := nav.FlightState.NmPerLongitude
	magVar := nav.FlightState.MagneticVariation
	acNM := math.LL2NM(nav.FlightState.Position, nmPerLong)
	headingDir := math.HeadingVector(math.MagneticToTrue(heading, magVar))

	best := visualApproachPoint{interceptDistance: 1e9}
	found := false

	for _, route := range routes {
		if len(route) < 2 {
			continue
		}
		tailDistance := visualRouteTailDistances(route, nmPerLong)
		for i := 0; i < len(route)-1; i++ {
			p0 := math.LL2NM(route[i].Location, nmPerLong)
			p1 := math.LL2NM(route[i+1].Location, nmPerLong)
			seg := math.Sub2f(p1, p0)
			segLen2 := math.Dot(seg, seg)
			if segLen2 == 0 {
				continue
			}
			intersect, ok := math.LineLineIntersect(acNM, math.Add2f(acNM, headingDir), p0, p1)
			if !ok {
				continue
			}

			alongHeading := math.Dot(math.Sub2f(intersect, acNM), headingDir)
			if alongHeading <= 0.1 || !visualPointOnSegment(intersect, p0, p1) {
				continue
			}

			distToThreshold := math.Distance2f(intersect, p1) + tailDistance[i+1]
			if !found || alongHeading < best.interceptDistance {
				best = visualApproachPoint{
					route:               route,
					segment:             i,
					segmentFraction:     math.Dot(math.Sub2f(intersect, p0), seg) / segLen2,
					location:            math.NM2LL(intersect, nmPerLong),
					distanceToThreshold: distToThreshold,
					interceptDistance:   alongHeading,
					lateralDistance:     0,
				}
				found = true
			}
		}
	}

	return best, found
}

func visualPointOnSegment(p, p0, p1 [2]float32) bool {
	seg := math.Sub2f(p1, p0)
	segLen2 := math.Dot(seg, seg)
	if segLen2 == 0 {
		return false
	}
	t := math.Dot(math.Sub2f(p, p0), seg) / segLen2
	const epsilon = 1e-4
	return t >= -epsilon && t <= 1+epsilon
}

func visualRouteTailDistances(route []av.Waypoint, nmPerLong float32) []float32 {
	dist := make([]float32, len(route))
	for i := len(route) - 2; i >= 0; i-- {
		dist[i] = dist[i+1] + math.NMDistance2LLFast(route[i].Location, route[i+1].Location, nmPerLong)
	}
	return dist
}

func visualRoutePointAtDistance(route []av.Waypoint, distanceToThreshold float32, nmPerLong float32) (visualApproachPoint, bool) {
	if len(route) < 2 {
		return visualApproachPoint{}, false
	}

	dist := float32(0)
	for i := len(route) - 1; i > 0; i-- {
		p1 := math.LL2NM(route[i].Location, nmPerLong)
		p0 := math.LL2NM(route[i-1].Location, nmPerLong)
		segLen := math.Distance2f(p0, p1)
		if segLen == 0 {
			continue
		}
		if dist+segLen >= distanceToThreshold {
			t := (distanceToThreshold - dist) / segLen
			locNM := math.Lerp2f(t, p1, p0)
			return visualApproachPoint{
				route:               route,
				segment:             i - 1,
				location:            math.NM2LL(locNM, nmPerLong),
				distanceToThreshold: distanceToThreshold,
				finalPoint:          true,
			}, true
		}
		dist += segLen
	}

	return visualApproachPoint{}, false
}

func (nav *Nav) visualApproachRouteFromApproach(runway string, followTraffic *math.Point2LL, referenceApproach *av.Approach) []av.Waypoint {
	rwy, ok := av.LookupRunway(nav.FlightState.ArrivalAirport.Fix, runway)
	if !ok {
		return nil
	}

	nmPerLong := nav.FlightState.NmPerLongitude
	magVar := nav.FlightState.MagneticVariation

	joinPoint := visualApproachPoint{}
	if followTraffic != nil {
		trafficPoint, ok := nav.projectOntoApproachRoutes(referenceApproach.Waypoints, *followTraffic)
		if !ok || trafficPoint.distanceToThreshold <= 0.5 {
			return nil
		}
		joinPoint = trafficPoint

		bearingToJoin := math.Heading2LL(nav.FlightState.Position, *followTraffic, nmPerLong)
		if math.HeadingDifference(bearingToJoin, math.MagneticToTrue(nav.FlightState.Heading, magVar)) > 120 {
			return nil
		}
	} else {
		heading := nav.FlightState.Heading
		if assignedHeading, ok := nav.AssignedHeading(); ok {
			heading = assignedHeading
		}

		intercept, hasIntercept := nav.intersectApproachRoutes(referenceApproach.Waypoints, heading)
		if hasIntercept && intercept.distanceToThreshold > 3 && intercept.distanceToThreshold <= 8 {
			joinPoint = intercept
		} else {
			projection, ok := nav.projectOntoApproachRoutes(referenceApproach.Waypoints, nav.FlightState.Position)
			if !ok || projection.distanceToThreshold <= 0.5 {
				return nil
			}
			headingDir := math.HeadingVector(math.MagneticToTrue(heading, magVar))
			thresholdDir := math.Normalize2f(math.Sub2f(
				math.LL2NM(referenceApproach.Threshold, nmPerLong),
				math.LL2NM(nav.FlightState.Position, nmPerLong)))
			if (hasIntercept && intercept.distanceToThreshold >= 0 && intercept.distanceToThreshold <= 3) ||
				math.Dot(headingDir, thresholdDir) >= 0 {
				// Join at the route-equivalent 3nm point if the aircraft
				// is already pointed generally toward the field or would
				// intercept inside the stabilized final segment.
				if finalPoint, ok := visualRoutePointAtDistance(projection.route, 3, nmPerLong); ok {
					joinPoint = finalPoint
				} else {
					joinPoint = projection
				}
			} else {
				bearingToProjection := math.Heading2LL(nav.FlightState.Position, projection.location, nmPerLong)
				if math.HeadingDifference(bearingToProjection, math.MagneticToTrue(nav.FlightState.Heading, magVar)) > 90 {
					return nil
				}
				joinPoint = projection
			}
		}
	}

	if len(joinPoint.route) < 2 {
		return nil
	}

	finalPoint, hasFinalPoint := visualRoutePointAtDistance(joinPoint.route, 3, nmPerLong)
	alt := rwy.Elevation + rwy.ThresholdCrossingHeight
	final3nmAlt := float32(rwy.Elevation) + 900

	var wps []av.Waypoint
	join := av.Waypoint{
		Fix:      "_" + runway + "_APPROACH_JOIN",
		Location: joinPoint.location,
	}
	if followTraffic != nil {
		join.Fix = "_" + runway + "_FOLLOW_TRAFFIC"
		join.Location = *followTraffic
	} else if joinPoint.lateralDistance == 0 {
		join.Fix = "_" + runway + "_INTERCEPT"
	} else {
		join.Fix = "_" + runway + "_PROJECTION"
	}
	join.SetOnApproach(true)
	if joinPoint.finalPoint {
		join.Fix = "_" + runway + "_3NM_FINAL"
		join.SetAltitudeRestriction(av.MakeAtAltitudeRestriction(final3nmAlt))
	}
	wps = append(wps, join)

	start := joinPoint.segment + 1
	if followTraffic != nil && joinPoint.segmentFraction <= 1e-4 &&
		math.NMDistance2LLFast(*followTraffic, joinPoint.route[joinPoint.segment].Location, nmPerLong) > 0.05 {
		start = joinPoint.segment
	}
	if hasFinalPoint && joinPoint.distanceToThreshold > 3.25 {
		if start < finalPoint.segment+1 {
			wps = append(wps, util.DuplicateSlice(joinPoint.route[start:finalPoint.segment+1])...)
		}
		finalWp := av.Waypoint{
			Fix:      "_" + runway + "_3NM_FINAL",
			Location: finalPoint.location,
		}
		finalWp.SetOnApproach(true)
		finalWp.SetAltitudeRestriction(av.MakeAtAltitudeRestriction(final3nmAlt))
		wps = append(wps, finalWp)
		start = finalPoint.segment + 1
	}

	if start < len(joinPoint.route) {
		wps = append(wps, util.DuplicateSlice(joinPoint.route[start:])...)
	}

	if len(wps) == 0 {
		return nil
	}
	return finalizeVisualApproachWaypoints(wps, alt)
}

func (nav *Nav) visualApproachRouteFollowingTraffic(runway string, trafficPosition math.Point2LL, trafficRoute av.WaypointArray) []av.Waypoint {
	rwy, ok := av.LookupRunway(nav.FlightState.ArrivalAirport.Fix, runway)
	if !ok || len(trafficRoute) == 0 {
		return nil
	}

	// trafficRoute is the traffic aircraft's Waypoints, which ends with
	// the destination airport appended after the threshold; drop the
	// airport so the threshold is the route's final point and
	// finalizeVisualApproachWaypoints flags it correctly.
	trafficRoute = trafficRoute[:len(trafficRoute)-1]

	nmPerLong := nav.FlightState.NmPerLongitude
	magVar := nav.FlightState.MagneticVariation
	bearingToTraffic := math.Heading2LL(nav.FlightState.Position, trafficPosition, nmPerLong)
	if math.HeadingDifference(bearingToTraffic, math.MagneticToTrue(nav.FlightState.Heading, magVar)) > 120 {
		return nil
	}
	if math.NMDistance2LLFast(trafficPosition, trafficRoute[len(trafficRoute)-1].Location, nmPerLong) <= 0.5 {
		return nil
	}

	join := av.Waypoint{
		Fix:      "_" + runway + "_FOLLOW_TRAFFIC",
		Location: trafficPosition,
	}
	join.SetOnApproach(true)

	wps := av.WaypointArray{join}
	wps = append(wps, util.DuplicateSlice(trafficRoute)...)
	return finalizeVisualApproachWaypoints(wps, rwy.Elevation+rwy.ThresholdCrossingHeight)
}

func finalizeVisualApproachWaypoints(wps av.WaypointArray, thresholdAltitude int) av.WaypointArray {
	last := &wps[len(wps)-1]
	last.SetOnApproach(true)
	last.SetLand(true)
	last.SetFlyOver(true)
	if last.AltitudeRestriction() == nil {
		last.SetAltitudeRestriction(av.MakeAtAltitudeRestriction(float32(thresholdAltitude)))
	}
	for i := range wps {
		wps[i].SetOnApproach(true)
	}
	return wps
}

// setVisualApproach populates nav.Approach with metadata for a visual
// approach to runway, using the given waypoints if provided.
func (nav *Nav) setVisualApproach(runway string, waypoints []av.Waypoint) bool {
	rwy, ok := av.LookupRunway(nav.FlightState.ArrivalAirport.Fix, runway)
	if !ok {
		return false
	}
	opp, ok := av.LookupOppositeRunway(nav.FlightState.ArrivalAirport.Fix, runway)
	if !ok {
		opp.Threshold = rwy.Threshold
	}

	nav.Approach.Assigned = &av.Approach{
		Id:                "VIS" + runway,
		FullName:          "Visual Approach Runway " + runway,
		Type:              av.VisualApproach,
		Runway:            runway,
		Threshold:         rwy.Threshold,
		OppositeThreshold: opp.Threshold,
	}
	if len(waypoints) > 0 {
		nav.Approach.Assigned.Waypoints = []av.WaypointArray{util.DuplicateSlice(waypoints)}
	}
	nav.Approach.AssignedId = "VIS" + runway
	nav.Approach.Cleared = true
	return true
}

// FollowTraffic describes a leader aircraft to sequence behind on a visual
// approach. Route (when non-empty) is the leader's own waypoints, used for
// tight in-trail spacing; Position alone is used as a join-point override
// on the reference approach.
type FollowTraffic struct {
	Position math.Point2LL
	Route    av.WaypointArray
}

// ClearedVisualApproach sets up the aircraft to fly a visual approach to the
// runway threshold. When follow is non-nil and has a Route, it first tries
// tight in-trail sequencing along the leader's route; if that's not
// geometrically viable, or follow has only a Position, it falls back to
// reference-approach geometry (using Position as a join-point override).
// When referenceApproach is nil or yields no viable route, it synthesizes a
// straight-in final from runway data. Returns (intent, true) on success, or
// (nil, false) if no viable route can be built (unknown runway or aircraft
// too close for a stable approach).
func (nav *Nav) ClearedVisualApproach(runway string, follow *FollowTraffic, referenceApproach *av.Approach, lahsoRunway string, simTime time.Time) av.CommandIntent {
	if follow != nil && len(follow.Route) > 0 {
		if wps := nav.visualApproachRouteFollowingTraffic(runway, follow.Position, follow.Route); wps != nil {
			return nav.clearedVisualApproach(runway, lahsoRunway, simTime, wps)
		}
	}

	var joinPos *math.Point2LL
	if follow != nil {
		joinPos = &follow.Position
	}
	wps := nav.visualApproachRoute(runway, joinPos, referenceApproach)
	if wps == nil {
		return av.MakeUnableIntent("unable, we don't know runway " + runway)
	}

	return nav.clearedVisualApproach(runway, lahsoRunway, simTime, wps)
}

func (nav *Nav) clearedVisualApproach(runway string, lahsoRunway string, simTime time.Time, wi []av.Waypoint) av.CommandIntent {
	// Cancel hold before clearing nav state.
	cancelHold := nav.Heading.Hold != nil
	if nav.Heading.Hold != nil {
		nav.Heading.Hold.Cancel = true
	}

	nav.Waypoints = append(wi, nav.FlightState.ArrivalAirport)
	nav.Heading = NavHeading{}
	nav.DeferredNavHeading = nil

	// Populate nav.Approach so downstream consumers (go-around,
	// spacing checks, landing bookkeeping, departure scheduling) have
	// the runway data they need.
	nav.setVisualApproach(runway, wi)

	// Mark as cleared and allow descent.
	nav.Altitude = NavAltitude{}
	nav.Speed = NavSpeed{}

	return av.ClearedApproachIntent{
		Approach:    "Visual Approach Runway " + runway,
		CancelHold:  cancelHold,
		LAHSORunway: lahsoRunway,
	}
}

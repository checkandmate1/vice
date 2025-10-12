package eram

import (
	"fmt"
	"strings"

	av "github.com/mmp/vice/aviation"
	"github.com/mmp/vice/math"
	"github.com/mmp/vice/panes"
	"github.com/mmp/vice/radar"
	"github.com/mmp/vice/renderer"
	"github.com/mmp/vice/sim"
	"github.com/mmp/vice/util"
)

// DatablockType enumerates the supported ERAM datablock formats. Only the
// general types are provided here; the specific contents are defined
// elsewhere.
type DatablockType int

const (
	// LimitedDatablock represents the two-line limited data block used for
	// untracked or unpaired targets.
	LimitedDatablock DatablockType = iota
	// FullDatablock represents the five line full data block.
	FullDatablock
	// EnhancedLimitedDatablock represents the optional enhanced limited data
	// block.  It behaves like LimitedDatablock with additional information.
	EnhancedLimitedDatablock
)

// datablock abstracts the different concrete datablock implementations.  A
// datablock knows how to render itself at a particular point relative to the
// leader line.
type datablock interface {
	draw(td *renderer.TextDrawBuilder, pt [2]float32, font *renderer.Font,
		sb *strings.Builder, brightness radar.Brightness,
		dir math.CardinalOrdinalDirection, halfSeconds int64)
}

// dbChar represents a single character in a datablock along with its colour and
// flashing state.
type dbChar struct {
	ch       rune
	color    renderer.RGB
	flashing bool
}

// --- Drawing helpers -----------------------------------------------------

// dbLine stores characters making up a single line of a datablock.  The slice
// length is capped to the maximum possible number of characters drawn on a
// line.
type dbLine struct {
	length int
	ch     [16]dbChar
}

// dbMakeLine flattens a number of datablock fields into a contiguous line.
func dbMakeLine(fields ...[]dbChar) dbLine {
	var l dbLine
	for _, f := range fields {
		for _, ch := range f {
			l.ch[l.length] = ch
			l.length++
		}
	}
	return l
}

// Len returns the number of active characters in the line.
func (l dbLine) Len() int {
	for i := l.length - 1; i >= 0; i-- {
		if l.ch[i].ch != 0 {
			return i + 1
		}
	}
	return 0
}

// dbChopTrailing removes trailing unset characters from the provided field.
func dbChopTrailing(f []dbChar) []dbChar {
	for i := len(f) - 1; i >= 0; i-- {
		if f[i].ch != 0 {
			return f[:i+1]
		}
	}
	return nil
}

// dbDrawLines renders the given datablock lines.  The leader line direction is
// used only to determine justification.
func dbDrawLines(lines []dbLine, td *renderer.TextDrawBuilder, pt [2]float32,
	font *renderer.Font, sb *strings.Builder, brightness radar.Brightness,
	dir math.CardinalOrdinalDirection, halfSeconds int64) {
	scale := float32(2) // 1.5 for default font
	if len(lines) >= 5 {
		if lines[3].ch[0].ch != rune('R') {
			scale = 2
		}
	}
	glyph := font.LookupGlyph(' ')
	fontWidth := glyph.AdvanceX * scale

	for i, line := range lines {
		// All lines start at the same position
		xOffset := float32(0)

		// Special case: line 3 (index 3) starts 1 character to the left
		if i == 2 || i == 3 {
			xOffset -= fontWidth
		}
		lineSpacing := 1.4
		sb.Reset()
		dbDrawLine(line, td, math.Add2f(pt, [2]float32{xOffset, 0}), font, sb,
			brightness, halfSeconds)
		pt[1] -= float32(font.Size) * float32(lineSpacing)
	}
}

// dbDrawLine renders a single datablock line.
func dbDrawLine(line dbLine, td *renderer.TextDrawBuilder, pt [2]float32,
	font *renderer.Font, sb *strings.Builder, brightness radar.Brightness,
	halfSeconds int64) {

	style := renderer.TextStyle{Font: font}

	flush := func() {
		if sb.Len() > 0 {
			pt = td.AddText(rewriteDelta(sb.String()), pt, style)
			sb.Reset()
		}
	}

	for i := range line.length {
		ch := line.ch[i]
		if ch.ch == 0 {
			sb.WriteByte(' ')
			continue
		}

		if ch.flashing && halfSeconds&1 == 1 { // TODO: adjust this value
			continue
		}

		c := ch.color
		if !c.Equals(style.Color) {
			flush()
			style.Color = c
		}
		sb.WriteRune(ch.ch)
	}
	flush()
}

// fieldEmpty reports whether the datablock field contains any characters.
func fieldEmpty(f []dbChar) bool {
	for _, ch := range f {
		if ch.ch != 0 {
			return false
		}
	}
	return true
}

// dbWriteText writes the provided text into the datablock field using the given
// colour. Any unused characters remain unset.
func dbWriteText(dst []dbChar, s string, c renderer.RGB, flashing bool) {
	for i, ch := range s {
		if i >= len(dst) {
			break
		}
		dst[i] = dbChar{ch: ch, color: c, flashing: flashing}
	}
}

func rewriteDelta(s string) string { return s }

type limitedDatablock struct {
	line0 [8]dbChar
	line1 [8]dbChar
	line2 [8]dbChar
}

func (db limitedDatablock) draw(td *renderer.TextDrawBuilder, pt [2]float32,
	font *renderer.Font, sb *strings.Builder, brightness radar.Brightness,
	dir math.CardinalOrdinalDirection, halfSeconds int64) {
	dir = math.CardinalOrdinalDirection(math.East) // Always east or west for LDBs (west not simulated)
	lines := []dbLine{
		dbMakeLine(dbChopTrailing(db.line0[:])),
		dbMakeLine(dbChopTrailing(db.line1[:])),
		dbMakeLine(dbChopTrailing(db.line2[:])),
	}
	pt[1] += float32(font.Size)
	dbDrawLines(lines, td, pt, font, sb, brightness, dir, halfSeconds)
}

type fullDatablock struct {
	line0 [16]dbChar
	line1 [16]dbChar
	// line 2
	vci   [2]dbChar
	line2 [16]dbChar
	// line3
	col1   [2]dbChar
	fieldD [8]dbChar
	fieldE [8]dbChar
	line4  [16]dbChar
}

func (db fullDatablock) draw(td *renderer.TextDrawBuilder, pt [2]float32,
	font *renderer.Font, sb *strings.Builder, brightness radar.Brightness,
	dir math.CardinalOrdinalDirection, halfSeconds int64) {

	lines := []dbLine{
		dbMakeLine(dbChopTrailing(db.line0[:])),
		dbMakeLine(dbChopTrailing(db.line1[:])),
		dbMakeLine(db.vci[:], dbChopTrailing(db.line2[:])),
		dbMakeLine(db.col1[:], dbChopTrailing(db.fieldD[:]), dbChopTrailing(db.fieldE[:])),
		dbMakeLine(dbChopTrailing(db.line4[:])),
	}
	pt[1] += float32(font.Size)
	dbDrawLines(lines, td, pt, font, sb, brightness, dir, halfSeconds)
}

func (ep *ERAMPane) getAllDatablocks(ctx *panes.Context, tracks []sim.Track) map[av.ADSBCallsign]datablock {
	ep.fdbArena.Reset()
	ep.ldbArena.Reset()

	dbs := make(map[av.ADSBCallsign]datablock)
	for _, trk := range tracks {
		state := ep.TrackState[trk.ADSBCallsign]
		if state == nil {
			continue
		}

		dbType := ep.datablockType(ctx, trk)
		ps := ep.currentPrefs()
		brite := util.Select(dbType == FullDatablock, ps.Brightness.FDB, ps.Brightness.LDB)
		color := brite.ScaleRGB(ERAMYellow)
		db := ep.getDatablock(ctx, trk, dbType, color)
		dbs[trk.ADSBCallsign] = db
	}
	return dbs
}

func (ep *ERAMPane) getDatablock(ctx *panes.Context, trk sim.Track, dbType DatablockType, color renderer.RGB) datablock {
	state := ep.TrackState[trk.ADSBCallsign]
	ps := ep.currentPrefs()
	switch dbType {
	case FullDatablock:
		db := ep.fdbArena.AllocClear()
		// DBLine 0 is point out
		dbWriteText(db.line1[:], trk.ADSBCallsign.String(), color, false) // also * if satcom
		vciBright := radar.Brightness(ps.Brightness.ONFREQ + ps.Brightness.Portal)
		vciColor := vciBright.ScaleRGB(renderer.RGB{0.01, 1, 0.05})
		dbWriteText(db.vci[:], util.Select(state.DisplayVCI, vci, ""), vciColor, false)
		dbWriteText(db.line2[:], ep.getAltitudeFormat(trk), color, false)
		// format line 3.
		// TODO: HIJK, RDOF, EMERG (what colors are these?) incoming handoff
		colColor := (ps.Brightness.FDB + ps.Brightness.Portal).ScaleRGB(ERAMYellow)
		dbWriteText(db.col1[:], util.Select(trk.FlightPlan.TrackingController == ctx.UserTCP, "", " R"), colColor, false)
		dbWriteText(db.fieldD[:], trk.FlightPlan.CID, color, false)
		if trk.FlightPlan.HandoffTrackController != "" {
            ctrl, ok := ctx.Client.State.Controllers[trk.FlightPlan.HandoffTrackController]
            var controller string
            if ok {
                controller = ctrl.ERAMID()
                if len(controller) == 2 {
                    controller = "-" + controller
                }
            } else {
                controller = trk.FlightPlan.HandoffTrackController
            }
            if len(controller) == 2 {
                controller = "-" + controller
            }
            a := util.Select(ep.dbAlternate, fmt.Sprintf("H%v", controller), fmt.Sprintf(" %v", int(state.track.Groundspeed)))
            dbWriteText(db.fieldE[:], a, color, true)
        } else if ctx.Client.State.SimTime.Before(state.OSectorEndTime) {
            ctrl, ok := ctx.Client.State.Controllers[trk.FlightPlan.TrackingController]
            var controller string
            if ok {
                controller = ctrl.ERAMID()
                if len(controller) == 2 {
                    controller = "-" + controller
                }
            } else {
                controller = trk.FlightPlan.TrackingController
            }
            a := util.Select(ep.dbAlternate, fmt.Sprintf("O%v", controller), fmt.Sprintf(" %v", int(state.track.Groundspeed)))
            dbWriteText(db.fieldE[:], a, color, false)
        } else {
			middle := " "
			airportCode, ok := ctx.Client.State.FacilityAdaptation.AirportCodes[trk.FlightPlan.ExitFix]
			if ok {
				middle = airportCode
			}
            dbWriteText(db.fieldE[:], fmt.Sprintf("%v%v", middle, int(state.track.Groundspeed)), color, false)
        }
		// Get line 4 (if applicable)
		if ps.Line4Type == Line4Destination {
			line4Color := (ps.Brightness.FDB - ps.Brightness.Line4).ScaleRGB(ERAMYellow)
			dbWriteText(db.line4[:], trk.FlightPlan.ArrivalAirport, line4Color, false)
		} else if ps.Line4Type == Line4Type {
			line4Color := (ps.Brightness.FDB - ps.Brightness.Line4).ScaleRGB(ERAMYellow)
			dbWriteText(db.line4[:], trk.FlightPlan.AircraftType, line4Color, false)
		}
		return db
	case EnhancedLimitedDatablock:
		return ep.ldbArena.AllocClear()
	case LimitedDatablock:
		db := ep.ldbArena.AllocClear()
		state := ep.TrackState[trk.ADSBCallsign]
		dbWriteText(db.line0[:], trk.ADSBCallsign.String(), color, false)
		dbWriteText(db.line1[:], fmt.Sprintf("%03d", int(state.track.TransponderAltitude+50)/100), color, false)
		return db
	default:
		return nil // should not happen
	}
}

func (ep *ERAMPane) getAltitudeFormat(track sim.Track) string {
	state := ep.TrackState[track.ADSBCallsign]
	currentAltitude := state.track.TransponderAltitude
	assignedAltitude := track.FlightPlan.AssignedAltitude
	// if assignedAltitude == 0 {
	// 	fmt.Println(track.ADSBCallsign, "has no assigned altitude")
	// }
	interimAltitude := track.FlightPlan.InterimAlt
	formatCurrent := av.FormatScopeAltitude(currentAltitude)
	formatAssigned := av.FormatScopeAltitude(assignedAltitude)
	formatInterim := av.FormatScopeAltitude(interimAltitude)
	if interimAltitude > 0 { // Interim alt takes precedence (i think) TODO: check this
		intType := getInterimAltitudeType(track)
		return fmt.Sprintf("%03v%s%03v", formatInterim, intType, formatCurrent)
	} else /* if assignedAltitude != -1 */ { // Eventually for block altitudes...
		switch {
		case formatCurrent == formatAssigned:
			return fmt.Sprintf("%vC", formatCurrent)
		case currentAltitude > float32(assignedAltitude) && assignedAltitude > -1: // TODO: Find actual font so that the up arrows draw
			middle := util.Select(state.Descending() || state.IsLevel(), downArrow, "+")
			return fmt.Sprintf("%v%v%v", formatAssigned, middle, formatCurrent)
		case currentAltitude < float32(assignedAltitude):
			middle := util.Select(state.Climbing() || state.IsLevel(), upArrow, "+")
			return fmt.Sprintf("%v%v%v", formatAssigned, middle, formatCurrent) // or maintaining

		}
	}
	return "" // This shouldn't happen?
}

func getInterimAltitudeType(track sim.Track) string {
	if track.FlightPlan.InterimAlt == -1 {
		return ""
	}
	interimType := track.FlightPlan.InterimType
	switch interimType {
	case radar.Normal:
		return "T"
	case radar.Procedure:
		return "P"
	case radar.Local:
		return "L"
	}
	return ""
}

func (ep *ERAMPane) drawDatablocks(tracks []sim.Track, dbs map[av.ADSBCallsign]datablock,
	ctx *panes.Context, transforms radar.ScopeTransformations, cb *renderer.CommandBuffer) {
	td := renderer.GetTextDrawBuilder()
	defer renderer.ReturnTextDrawBuilder(td)

	var ldbs, eldbs, fdbs []sim.Track
	for _, trk := range tracks {
		if !ep.datablockVisible(ctx, trk) {
			continue
		}
		switch ep.datablockType(ctx, trk) {
		case FullDatablock:
			fdbs = append(fdbs, trk)
		case EnhancedLimitedDatablock:
			eldbs = append(eldbs, trk)
		default:
			ldbs = append(ldbs, trk)
		}
	}

	var sb strings.Builder
	halfSeconds := ctx.Now.UnixMilli() / 500
	ps := ep.currentPrefs()

	draw := func(tracks []sim.Track) {
		for _, trk := range tracks {
			db := dbs[trk.ADSBCallsign]
			if db == nil {
				continue
			}
			state := ep.TrackState[trk.ADSBCallsign]
			if state == nil {
				continue
			}
			dbType := ep.datablockType(ctx, trk)
			var sz int
			if dbType == FullDatablock {
				sz = ps.FDBSize
			} else {
				sz = ps.LDBSize
			}
			font := ep.ERAMFont(sz)
			start := transforms.WindowFromLatLongP(state.track.Location)
			dir := ep.leaderLineDirection(ctx, trk)
			offset := datablockOffset(*dir)
			vector := ep.leaderLineVector(*dir)
			vector[0] += float32(offset[0]) * ctx.DrawPixelScale
			vector[1] += float32(offset[1]) * ctx.DrawPixelScale
			if dbType == EnhancedLimitedDatablock || dbType == LimitedDatablock {
				*dir = math.East // TODO: change to state eventually
				vector = ep.leaderLineVectorNoLength(*dir)
				offset[1] = -10
				vector[1] += float32(offset[1]) * ctx.DrawPixelScale
			}
			end := math.Add2f(start, math.Scale2f(vector, ctx.DrawPixelScale))
			brightness := ep.datablockBrightness(state)
			db.draw(td, end, font, &sb, brightness, *dir, halfSeconds)
		}
	}

	for _, blocks := range [][]sim.Track{ldbs, eldbs, fdbs} {
		draw(blocks)
	}

	transforms.LoadWindowViewingMatrices(cb)
	td.GenerateCommands(cb)
}

func datablockOffset(dir math.CardinalOrdinalDirection) [2]float32 {
	var offset [2]float32
	switch dir {
	case math.North:
		offset[0] = 5
		offset[1] = 40
	case math.NorthEast:
		offset[0] = 10
		offset[1] = 40
	case math.NorthWest:
		offset[0] = -80
		offset[1] = 25
	case math.East:
		offset[1] = 35
	case math.West:
		offset[0] = -80
		offset[1] = 25
	case math.SouthEast:
		offset[1] = 15
		offset[0] = 10
	case math.South:
		offset[0] = 4
		offset[1] = 16
	case math.SouthWest:
		offset[0] = -80
		offset[1] = 15
	}
	return offset
}

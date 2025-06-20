package eram

import (
	"strings"

	av "github.com/mmp/vice/pkg/aviation"
	"github.com/mmp/vice/pkg/math"
	"github.com/mmp/vice/pkg/panes"
	"github.com/mmp/vice/pkg/radar"
	"github.com/mmp/vice/pkg/renderer"
	"github.com/mmp/vice/pkg/sim"
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
		sb *strings.Builder, brightness radar.ScopeBrightness,
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
	font *renderer.Font, sb *strings.Builder, brightness radar.ScopeBrightness,
	dir math.CardinalOrdinalDirection, halfSeconds int64) {

	rightJustify := dir >= math.South
	glyph := font.LookupGlyph(' ')
	fontWidth := glyph.AdvanceX

	for _, line := range lines {
		xOffset := float32(4)
		if rightJustify {
			xOffset = -4 - float32(line.Len())*fontWidth
		}
		sb.Reset()
		dbDrawLine(line, td, math.Add2f(pt, [2]float32{xOffset, 0}), font, sb,
			brightness, halfSeconds)
		pt[1] -= float32(font.Size)
	}
}

// dbDrawLine renders a single datablock line.
func dbDrawLine(line dbLine, td *renderer.TextDrawBuilder, pt [2]float32,
	font *renderer.Font, sb *strings.Builder, brightness radar.ScopeBrightness,
	halfSeconds int64) {

	style := renderer.TextStyle{Font: font}

	flush := func() {
		if sb.Len() > 0 {
			pt = td.AddText(rewriteDelta(sb.String()), pt, style)
			sb.Reset()
		}
	}

	for i := 0; i < line.length; i++ {
		ch := line.ch[i]
		if ch.ch == 0 {
			sb.WriteByte(' ')
			continue
		}

		br := brightness
		if ch.flashing && halfSeconds&1 == 1 {
			br /= 3
		}

		c := br.ScaleRGB(ch.color)
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

// rewriteDelta is a stub used by dbDrawLine.  ERAM uses standard fonts so no
// rewriting is required.
func rewriteDelta(s string) string { return s }

///////////////////////////////////////////////////////////////////////////
// Concrete datablock types

// limitedDatablock holds the characters for a limited data block.  The exact
// field layout is intentionally omitted; callers populate the character arrays
// as appropriate.
type limitedDatablock struct {
	line0 [8]dbChar
	line1 [8]dbChar
	line2 [8]dbChar
}

func (db limitedDatablock) draw(td *renderer.TextDrawBuilder, pt [2]float32,
	font *renderer.Font, sb *strings.Builder, brightness radar.ScopeBrightness,
	dir math.CardinalOrdinalDirection, halfSeconds int64) {

	lines := []dbLine{
		dbMakeLine(dbChopTrailing(db.line0[:])),
		dbMakeLine(dbChopTrailing(db.line1[:])),
		dbMakeLine(dbChopTrailing(db.line2[:])),
	}
	pt[1] += float32(font.Size)
	dbDrawLines(lines, td, pt, font, sb, brightness, dir, halfSeconds)
}

// fullDatablock is used for the five line ERAM FDB.  As with limitedDatablock
// the exact field definitions are not specified here.
type fullDatablock struct {
	line0 [16]dbChar
	line1 [16]dbChar
	line2 [16]dbChar
	line3 [16]dbChar
	line4 [16]dbChar
}

func (db fullDatablock) draw(td *renderer.TextDrawBuilder, pt [2]float32,
	font *renderer.Font, sb *strings.Builder, brightness radar.ScopeBrightness,
	dir math.CardinalOrdinalDirection, halfSeconds int64) {

	lines := []dbLine{
		dbMakeLine(dbChopTrailing(db.line0[:])),
		dbMakeLine(dbChopTrailing(db.line1[:])),
		dbMakeLine(dbChopTrailing(db.line2[:])),
		dbMakeLine(dbChopTrailing(db.line3[:])),
		dbMakeLine(dbChopTrailing(db.line4[:])),
	}
	pt[1] += float32(font.Size)
	dbDrawLines(lines, td, pt, font, sb, brightness, dir, halfSeconds)
}

///////////////////////////////////////////////////////////////////////////
// High-level datablock helpers

// getAllDatablocks creates datablocks for the given tracks using the pane's
// object arenas. The caller is responsible for ensuring ep.TrackState has been
// initialized for each track.
func (ep *ERAMPane) getAllDatablocks(ctx *panes.Context, tracks []sim.Track) map[av.ADSBCallsign]datablock {
	ep.fdbArena.Reset()
	ep.ldbArena.Reset()

	dbs := make(map[av.ADSBCallsign]datablock)
	for _, trk := range tracks {
		state := ep.TrackState[trk.ADSBCallsign]
		if state == nil {
			continue
		}

		color := renderer.RGB{}
		brightness := radar.ScopeBrightness(0)
		if state.DatablockType == FullDatablock {
			brightness = ep.currentPrefs().Brightness.FDB
		} else {
			brightness = ep.currentPrefs().Brightness.LDB
		}
		dbs[trk.ADSBCallsign] = ep.getDatablock(ctx, trk, color, brightness)
	}
	return dbs
}

// getDatablock returns a datablock object for the given track. No datablock
// fields are populated here; callers are expected to do so elsewhere.
func (ep *ERAMPane) getDatablock(ctx *panes.Context, trk sim.Track, color renderer.RGB,
	brightness radar.ScopeBrightness) datablock {
	state := ep.TrackState[trk.ADSBCallsign]
	if state == nil {
		return nil
	}

	switch state.DatablockType {
	case FullDatablock:
		return ep.fdbArena.AllocClear()
	case LimitedDatablock, EnhancedLimitedDatablock:
		return ep.ldbArena.AllocClear()
	default:
		return nil
	}
}

// drawDatablocks draws the given map of datablocks. The map should have been
// created by getAllDatablocks. Leader lines and datablock characters are drawn
// using default fonts and colours.
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

	font := renderer.GetDefaultFont()
	var sb strings.Builder
	halfSeconds := ctx.Now.UnixMilli() / 500

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

			start := transforms.WindowFromLatLongP(state.track.Location)
			dir := ep.leaderLineDirection(ctx, trk)
			end := math.Add2f(start, math.Scale2f(ep.leaderLineVector(dir), ctx.DrawPixelScale))
			brightness := ep.datablockBrightness(state)
			db.draw(td, end, font, &sb, brightness, dir, halfSeconds)
		}
	}

	for _, set := range [][]sim.Track{ldbs, eldbs, fdbs} {
		draw(set)
	}

	transforms.LoadWindowViewingMatrices(cb)
	td.GenerateCommands(cb)
}

// wx/metar_test.go
package wx

import (
	"math"
	"testing"
)

func TestEffectiveVisualRangeSurface(t *testing.T) {
	// 3 SM surface vis, ground level → ~3 SM in NM.
	m := METAR{Raw: "KJFK 3SM BKN050"}
	got := m.EffectiveVisualRange(0)
	want := float32(3 * 0.8690)
	if math.Abs(float64(got-want)) > 0.1 {
		t.Errorf("EffectiveVisualRange(3SM, 0) = %.3f, want ~%.3f", got, want)
	}
}

func TestEffectiveVisualRangeAltitudeBonus(t *testing.T) {
	// At altitude the slant-path integral expands effective range beyond surface vis.
	m := METAR{Raw: "KJFK 3SM BKN080"}
	surface := m.EffectiveVisualRange(0)
	aloft := m.EffectiveVisualRange(5000)
	if aloft <= surface {
		t.Errorf("expected altitude bonus: surface=%.2f aloft=%.2f", surface, aloft)
	}
}

func TestEffectiveVisualRangeCap(t *testing.T) {
	// 10SM at high altitude should be capped at maxVisualRangeNM (25 NM).
	m := METAR{Raw: "KJFK 10SM CLR"}
	got := m.EffectiveVisualRange(20000)
	if got > maxVisualRangeNM {
		t.Errorf("EffectiveVisualRange exceeded cap: %.2f > %.2f", got, maxVisualRangeNM)
	}
	if got < maxVisualRangeNM-0.1 {
		t.Errorf("expected cap at %.2f NM for 10SM + high altitude, got %.2f", maxVisualRangeNM, got)
	}
}

func TestEffectiveVisualRangeObscurationPenalty(t *testing.T) {
	clear := METAR{Raw: "KJFK 5SM BKN050"}
	haze := METAR{Raw: "KJFK 5SM HZ BKN050"}
	if haze.EffectiveVisualRange(0) >= clear.EffectiveVisualRange(0) {
		t.Errorf("expected obscuration penalty: clear=%.2f haze=%.2f",
			clear.EffectiveVisualRange(0), haze.EffectiveVisualRange(0))
	}
}

func TestEffectiveVisualRangeUnparseable(t *testing.T) {
	// Missing visibility field → conservative cap.
	m := METAR{Raw: "KJFK BKN050"}
	if got := m.EffectiveVisualRange(0); got != maxVisualRangeNM {
		t.Errorf("unparseable visibility: got %.2f, want %.2f", got, maxVisualRangeNM)
	}
}
